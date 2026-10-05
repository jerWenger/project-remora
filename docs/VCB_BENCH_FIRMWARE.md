# VCB bench firmware: arming and thruster startup (reverse-engineered)

Source: `firmware/VCB_flash_20260805_211528_pre_tolerant_pwm.bin` (STM32H723, base 0x08000000), disassembled 2026-10-04.
No hardware was touched. Addresses are flash addresses. Confidence: **H** high (read directly from code), **M** medium (inferred), **L** low.

> **Caveat: the board may not be running this image.** The file name says "pre_tolerant_pwm", and in this image a
> pulse narrower than 1000 µs is rejected (see PWM validity below). The live board reports 990–995 µs as `valid=1` with
> `bad=0` (HARDWARE.md, 2026-10-04), so a later "tolerant" build is flashed. Its arming logic is probably the same
> (its strings match), but that is unverified (**M**). The cleanest check is a fresh flash dump and a byte diff.

## Key addresses

| What | Address |
|---|---|
| Bench main loop (inlined in `main`) | 0x8001e9e (loop top) … 0x8001fee |
| Thruster state machine object (6 states) | RAM 0x240002e8; `sm_init` 0x8002106, `sm_set_state` 0x80020fc, `sm_get_state` 0x8002144, `sm_run` 0x8002148 |
| `armed` | RAM 0x24000272 (.bss, 0 at boot) |
| `thruster_enable` / `charge` | RAM 0x24000296 / 0x24000294 |
| Arm hold timer start (0 = not started) | stack `[sp+0x24]` in main |
| E-stop / TSMS present | RAM 0x24000271 (PA9 == 1) / 0x24000270 (PA10 == 1) |
| Bus voltage (float V) | RAM 0x2400026c = (ADC − 32768)/32768 · 3.3 · 28.2727 |
| PWM capture callback | 0x8001448 (TIM5 → ch1 at 0x24000248, TIM2 → ch2 at 0x24000224), edge logic 0x8001294 |
| `safe_outputs()` (clears enable/charge/speeds, opens LSC+HSC, sends stop) | 0x8001020 |

## State machine table (registered at 0x8001b38–0x8001b7c; `sm_run` calls action, then transition, and stores the returned state)

| # | State | Action | Transition (returns next state) |
|---|---|---|---|
| 0 | FAULT | 0x8001108 → `safe_outputs()` every loop | 0x8000b46: **always returns 0** (latched) |
| 1 | IDLE | 0x8000b9c: LSC_CONTROL (PD5) and HSC_CONTROL (PD6) low | 0x8000bf0: `thruster_enable && charge && bus < 5.0 V` → print "THRUSTER IDLE: Charge button…", go to 2; otherwise stay 1 |
| 2 | STARTUP_LSC | none | 0x8000c38: on first pass, LSI (PD4) must read 1 or FAULT ("LSI pin is not SET"); then drive PD5 high; PD4 must go 0 within 1000 ms → 3, else FAULT ("LSI pin timeout") |
| 3 | STARTUP_WAIT | none | 0x8000bbc: 500 ms → 4 |
| 4 | STARTUP_HSC | none | 0x8000d08: HSI (PD3) must read 1 at start or FAULT; drive PD6 high; PD3 must go 0 within 1000 ms (else FAULT); 10 ms after closing, bus ≥ 40.0 V → 5, else FAULT |
| 5 | RUN | 0x8000ffc: send speeds (applied ‰ / 1000) for ch1 and ch2 | 0x8000b50: `armed && bus ≥ 35.0 V` → stay 5, else FAULT |

The boot state is **IDLE** (`sm_init(sm,1,6,0)` at 0x8001b34, then `sm_set_state(sm,1)` at 0x8001bbe). (**H**)
Only two places ever set the thruster state: boot (→1) and BENCH FAULT (→0, 0x8001fae). Nothing moves FAULT back to IDLE.
**FAULT can only be cleared by a reset or power cycle.** (**H**)

## PWM validity (this image, 1 tick ≈ 1 µs: TIM2/TIM5 prescaler 127; the observed 20070 µs period agrees)

- Each input is captured as a rising edge on CH3 and a falling edge on CH4 of the same timer (init at 0x80017cc).
- Period (rising to rising) must be in **[2000, 30000] µs** (`(p−2000) ≤ 28000`, 0x80012d2). If a rising edge arrives before the falling edge, the pulse counts as bad.
- Pulse width must be in **[1000, 2000] µs** (`(w−1000) ≤ 1000`, 0x800135c) and less than the period. Otherwise the pulse counts as bad and `valid` is cleared.
- `valid` is set after **3 consecutive good pulses** (counter at +0xb, 0x800137c).
- In the main loop (0x8000b14) a channel is valid only if `valid`, now − last good pulse ≤ 100 ms, and now − last edge ≤ 100 ms. (**H**)
- raw ‰ = width − 1000 (0 if invalid). applied ‰ = clamp(raw, 0, 200) (0x8000b38, 0x8001ee2). (**H**)
- In this image, 990–995 µs would be invalid and `bad` would count up. That does not match the live board, hence the caveat above.

## (a) Arming: what sets `armed` and prints BENCH ARMED (0x8001f7c → 0x8001c42 → 0x8001c9c)

Every loop (about 5 ms), while `armed == 0`:
1. "Stopped" means **both channels valid AND ch1 width ≤ 1050 µs AND ch2 width ≤ 1050 µs** (`cmp #0x41a`, 0x8001f8a). The test uses measured µs, not ‰. In this image that is a 1000–1050 µs window. (**H**)
2. **The thruster state must be IDLE (1)** (`sm_get_state == 1`, 0x8001c9e). (**H**)
3. If both hold, the hold timer starts. If either fails on any loop, the timer resets to 0.
4. After **≥ 1000 ms** of continuous stopped + IDLE: `armed = 1`, 0x24000000 = 0, and "BENCH ARMED: …" prints **once** (0x8001cb2–0x8001cc6). (**H**)

- No edge is needed: you don't have to go non-stopped → stopped. Stopped from boot works. (**H**)
- **E-stop and TSMS are not checked in software.** 0x24000271/0x24000270 are only written and printed (single xrefs, both in main). (**H**)
- So TSMS only matters physically: without coil power the contactors can't close, and the interlock then faults (see (c)). (**M**: inferred from the wiring role, not from the code)

## What disarms

- **PWM lost or invalid while armed** (either channel not valid; going above 1050 µs is fine once armed): `armed = 0`, thruster → FAULT, `safe_outputs()`, prints "BENCH FAULT: PWM lost or invalid" (0x8001fa0). (**H**)
- **Thruster state == FAULT while armed** (after `sm_run`): `armed = 0`, `safe_outputs()`, prints "BENCH DISARMED: thruster startup/run interlock fault" (0x8001fc8). (**H**)
- Either way the thruster ends in FAULT, so re-arming (which needs IDLE) is impossible until reset. **One arming attempt per boot.** (**H**)

## (b) FAULT → IDLE

There is no path. FAULT's transition always returns 0, and no code calls `sm_set_state(thruster,1)` after boot. The GitHub
"hold charge 1 s with valid RC" recovery is **not present** in this build. **Reset or power-cycle the VCB.** (**H**)

## (c) thruster_enable / charge → STARTUP → RUN

- `thruster_enable` and `charge` are both set to 1 every loop when `armed && both channels valid`, and cleared to 0 otherwise (0x8001c58–0x8001c68, 0x8001d8a). The bench `armed` flag drives them directly. (**H**)
- The thruster speeds are ch1 applied ‰ / 1000 → 0x24000278 and ch2 → 0x24000274. They are only sent in RUN. (**H**)
- So the moment it arms, IDLE moves to STARTUP_LSC, provided **bus < 5.0 V**. If the bus is already ≥ 5 V it stays armed in IDLE and never starts (no fault). (**H**)
- Then: LSC closes within 1 s → wait 500 ms → HSC closes within 1 s → bus ≥ 40 V 10 ms later → RUN. RUN holds while bus ≥ 35 V and armed. (**H**)
- Thresholds: 5.0 V (0x8000c04), 40.0 V (0x8000e74), 35.0 V (0x8000b80). (**H**)
- `lsc_closed` = PD4 reads 0 and `hsc_closed` = PD3 reads 0 (status print, 0x8001e1e). (**H**)

## Why the board is stuck in FAULT (observed 2026-10-04)

The earlier "BENCH DISARMED" message can only print if `armed` was 1, and `armed` is only set together with "BENCH ARMED".
So it **did arm** at that time, and the ARMED line was probably missed (it prints once). (**H** for this image) Then:

1. Armed → `thruster_enable` and `charge` set → STARTUP_LSC.
2. With TSMS absent the LSC couldn't close, giving the 1 s LSI timeout (or "LSI pin is not SET") → FAULT. (**M** on which of the two)
3. FAULT disarmed it, and FAULT is latched.

Applying TSMS later can't help, and good PWM can't re-arm it, because arming needs IDLE. **Power-cycle or reset.** (**H**)

## Operator procedure

1. Before powering the VCB: TSMS 24 V present, E-stop released (24 V present), HV bus **< 5 V**, and pack/precharge path ready so the bus will reach ≥ 40 V within 10 ms of HSC closing. Both contactors open (LSI and HSI read high → `lsc_closed=0`, `hsc_closed=0`).
2. Have both PWM inputs at idle before or at boot (Pixhawk disarmed at MIN/TRIM). This image needs 1000–1050 µs; the live tolerant build accepts 990 µs.
3. Power-cycle or reset the VCB. Check `thruster_state=1`.
4. Hold both channels stopped for ≥ 1 s. Expect "BENCH ARMED". Startup begins **immediately**: the LSC closes, then 500 ms later the HSC closes, then RUN (`thruster_state=5`). No separate enable or charge action is needed.
5. Throttle: ch1 = right, ch2 = left, (µs − 1000) ‰ capped at 200 ‰ (1200 µs).
6. After **any** fault (PWM dropout > 100 ms or 3 bad pulses while armed, contactor not closing within 1 s, a contactor already closed at start, bus < 40 V at HSC, bus < 35 V in RUN), it goes to FAULT and disarms. Power-cycle and start again at step 1.

## Unknowns

- What 0x24000000 does: it is 1 at boot (.data init 0x800ffa0), set to 1 by `safe_outputs` and BENCH FAULT, and 0 on arm; no reader was found.
- Whether the flashed "tolerant" build changed anything beyond the pulse-width window.
