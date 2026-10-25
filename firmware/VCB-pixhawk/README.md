# VCB firmware: Pixhawk PWM input, reversible thrusters

Best-effort re-creation of the VCB (Vehicle Control Board, STM32H723VG) firmware
that runs on the boat, built from the only source we have, plus reverse thrust.

**Version 0.3.2 (2026-10-25). Status: 0.3.0 was flashed on the boat on 2026-10-25 and
proved the console both ways (banner, headers, `help`), but its PWM capture never
qualified: sub-microsecond glitches on both CONT lines, about every 5 ms, were captured as
rise-and-fall at the same tick and counted as bad pulses. 0.3.1 fixes that (8 us hardware
input filter, rising edge on CH3 and falling on CH4 as in the deployed build, glitch pairs
discarded without touching the period, `chN_glitch` counters in the telemetry). 0.3.1 was
flashed the same day: both channels qualify (993 us / 20067 us, `bad` 0, glitch counters 0,
so the filter alone removes the glitches). Same day, through bench mode: first live precharge
and RUN (bus 53.8 V), TSMS proven as the contactor kill, both motors turned at 5 % one at a
time, which showed the ePropulsion addresses are the reverse of cttdev's comment: 0.3.2 swaps
them. Commissioning continues in `docs/VCB_COMMISSIONING.md`.** Read "Before flashing" and
"Bench test plan" below.

## Where it comes from

| Piece | Source |
|---|---|
| Board support, contactor start-up sequence, ePropulsion and MCB drivers | `github.com/cttdev/VCB`, commit `aa3dbf8` ("Initial commit", May 2026). That version reads an RC receiver (CRSF on USART2). |
| Behaviour of the deployed build | `docs/VCB_BENCH_FIRMWARE.md` (disassembly of `firmware/VCB_flash_20260805_211528_pre_tolerant_pwm.bin`) and the 2026-10-04 bench check in `docs/HARDWARE.md`. Its source is lost. |
| What its author told us (2026-10-17) | cttdev's VCB took an RC receiver on the **CONT** connector and translated it into ePropulsion frames. It was then reprogrammed to read Pixhawk PWM on the same CONT pins, "timer style", and to map 1100..1900 us to 0..20 %. Near the ends of that range a commanded 1900 us read as 1901 or 1905, fell outside the mapped range, became a value the ePropulsion converter does not accept, and **the converter dropped the motors at full throttle**. Their advice: pin the end points explicitly. |
| What this build adds | PWM input on the same two pins the deployed build uses, bidirectional mapping with 1500 us = stop, a thrust limit in both directions, fault recovery, the same CSV telemetry so the Jetson tools keep working, and (0.3.0) console commands so the Jetson can drive the motors directly for commissioning. |

We were told the deployed build differs from the GitHub version only by taking
PWM from the Pixhawk instead of radio input. The disassembly agrees: the
contactor state machine, messages and thresholds are the same. This build keeps
them unchanged (`Core/Src/main.cpp`, `thruster_update()`).

The "1100..1900 us to 0..20 %" description is from memory. The dump and the
2026-10-04 measurement both say (us - 1000) permille, clamped at 1200 us, and that is
what `docs/HARDWARE.md` records; 1100/1900 were most likely the Pixhawk's SERVO
MIN/MAX at the time. The part that matters is the end-point incident, and this build
is organised around it: see "End points are pinned" under "PWM contract".

`UPSTREAM.md` lists every file changed against `cttdev/VCB` and how to diff.

## Pins and ports (unchanged)

| Signal | Pin | Note |
|---|---|---|
| PWM ch1 (right motor) | PA3, CONT connector pin 2 | TIM5 input capture. Was USART2_RX (CRSF from the RC receiver on CONT). |
| PWM ch2 (left motor) | PA2, CONT connector pin 1 | TIM2 input capture. Was USART2_TX. |
| ePropulsion protocol converter | UART5 (RS-485, 38400 8N1) | frame `28 addr 03 2C dir value xor 29`; addr 1 = right motor, 2 = left (measured) |
| Console / telemetry | UART4 (PA0/PA1), 115200 8N1 | the STLINK-V3 VCP, `/dev/vcb` on the Jetson; also the command input, see "Console commands" |
| MCB "LARS", "REEL" | UART7, USART3 | only `idle\n` is sent, 10 Hz |
| LSC / HSC control | PD5 / PD6 | contactor coils |
| LSI / HSI sense | PD4 / PD3 | low = closed |
| E-stop / TSMS 24 V sense | PA9 / PA10 | read and reported; not acted on (see `REQUIRE_ESTOP_TSMS_TO_START`) |
| HV bus voltage | ADC1 IN3 differential | (30k+30k+2.2k)/2.2k divider |
| LEDs | CTL3 red, CTL2 orange, CTL1 green | see "LEDs" |

The inputs now have weak pull-downs, so an unplugged input reads a steady low
and goes invalid instead of picking up noise.

## PWM contract

| Pulse width | Meaning |
|---|---|
| 1475..1525 us (+-25 us deadband) | stop, raw = 0 |
| 1525 -> 2000 us | forward, raw 0 -> 1000 permille, straight line (2.1 permille per us) |
| 1525 -> 1000 us | reverse, raw 0 -> -1000 permille, same line |
| 2000..2300 us | full forward, raw = +1000, still `valid` |
| 700..1000 us | full reverse, raw = -1000, still `valid` |
| below 700 or above 2300 us | pulse rejected (`bad` counts up); the last good width stays in force; `valid` drops after 100 ms without a good pulse -> stop |
| anything | applied = clamp(raw, -`THRUST_LIMIT_PERMILLE`, +`THRUST_LIMIT_PERMILLE`), default +-200 = 20 % |
| anything | converter value byte = round(\|applied\| x 127 / 1000), direction byte from the sign; 0..0x7F by construction |

So 1550 us = 52, 1600 us = 157, 1620 us = 200, 1700 us = 368 (applied 200), 1900 us = 789
(applied 200), 1990 us = 978, 2000 us and beyond = 1000; mirror for reverse. At the default
limit the value byte never exceeds 25.

**End points are pinned, never rejected.** The deployed build's author reports that a
commanded 1900 us measured as 1905, fell outside its mapped range, produced a value the
ePropulsion converter does not accept, and the converter dropped the motors at full
throttle. Here three independent fences make that impossible, each sufficient on its own:
the capture accepts 700..2300 us (wider than anything a Pixhawk can emit, 800..2200 us,
plus the ~1 % timebase error), `width_to_permille()` returns exactly +-1000 for any width at
or past 1000/2000 us, and `EProp::sendOne()` takes integer permille, clamps to +-1000, converts
with integer arithmetic and clamps the byte to 0..0x7F again. The byte actually sent is in
the telemetry (`ch1_cmd`, `ch2_cmd`), so the bench can watch it at 990, 1010, 1990 and 2010 us.

- The deadband is wide because the MCU clocks from the internal HSI RC oscillator (cttdev's
  clock tree; the board's HSE crystal, if fitted, is not used). HSI is good to about 1 %,
  and the deployed build already measured a 20000 us Pixhawk frame as 20070 us, so expect a
  true 1500 us to read 1485..1515 us. The width in the telemetry is the measured one.
- A pulse is accepted when its width is 700..2300 us and its period 2..30 ms (33..500 Hz).
  ArduPilot's own limits are 800..2200 us, so a 2200 us output that reads 2222 us here is
  still accepted (as full forward), not rejected.
- A channel is `valid` after 3 consecutive accepted pulses; the first pulse after boot or a
  gap only establishes the period base, so that is 4 pulses, 80 ms at 50 Hz. A rejected
  pulse does not invalidate it; the last accepted width stays in force. It drops to invalid
  when no accepted pulse, or no edge at all, arrives for 100 ms. Invalid means raw =
  applied = 0.
- Glitches (a rise and a fall less than `PWM_MIN_WIDTH_US` apart) are not pulses: the timer
  input filter removes anything under 8 us, and whatever gets through is discarded whole,
  counted in `chN_glitch`, and leaves the width and period untouched. The boat's CONT lines
  carry such glitches at ~200/s (2026-10-25); the deployed build evidently filtered them too.
- `raw_permille` is the stick position (+-1000), `applied_permille` is what goes to the
  converter. The telemetry shows both, so the limit is visible on the bench.
- The limit is a clamp, not a scale: with the default, 1600 us and 2000 us both give 20 %.
  To use ArduPilot's full throttle range inside the limit, narrow `SERVOn_MIN/MAX`
  (see "Pixhawk parameters"). Raising the limit is one number in `Core/Inc/config.h`.

Both channels are independent (no mixing). Which physical motor each channel drives
was settled on the boat on 2026-10-25, one motor commanded at a time through bench mode:
ePropulsion **address 1 is the right (starboard) motor and address 2 the left**, the
opposite of cttdev's source comment. Since 0.3.2 ch1 (PA3, Pixhawk ThrottleRight) sends to
address 1 and ch2 (PA2, ThrottleLeft) to address 2 (`EPROP_ADDR_RIGHT/LEFT` in config.h).
0.3.0/0.3.1 had them swapped.

## Direction changes

When the sign of a channel's command flips, the converter is sent 0 for that channel until
`DIRECTION_CHANGE_STOP_MS` (default 200 ms) have passed since the last non-zero command of
the old direction; then the new direction goes out. A stick that rests in the deadband for
longer than that adds no delay, and returning to the old direction mid-hold resumes at once.
Whether the ePropulsion converter needs a stop between forward and reverse is unknown;
given that it is reported to drop the motors on input it does not like, 200 ms of zero per
reversal is cheap insurance. It is visible in `ch1_cmd`/`ch2_cmd` (25 -> 0 -> -25). Set the
knob to 0 to reverse directly, as cttdev's RC version did.

## Console commands and bench mode

Lines on the console (UART4, so `/dev/vcb` from the Jetson), newline terminated,
case-insensitive. Replies are free-text event lines in the same stream.

| Command | Effect |
|---|---|
| `bench on` | the console becomes the command source; the PWM inputs are ignored (still measured and reported). Only accepted while disarmed in IDLE or FAULT. |
| `bench R L` | thrust for the right (ch1) and left (ch2) motor in permille, `-1000..1000`, clamped to `+-BENCH_LIMIT_PERMILLE` (default 100 = 10 %) and then to the normal thrust limit. Must repeat at least every `BENCH_TIMEOUT_MS` (500 ms) or the command counts as lost: fault 1, exactly like losing PWM. |
| `bench off` | back to the PWM inputs. If the board is armed or running it first opens the contactors, sends stop and returns to IDLE (`DISARMED: bench off`), not a fault. |
| `stop` | fault 9: contactors open, thrusters stopped, disarmed, **latched** until `clear` or a reset. Works whatever the command source. A software stop, not a safety device. |
| `clear` | un-latches fault 9; the normal recovery then runs (valid neutral command for `FAULT_RECOVER_HOLD_MS`). |
| `help` | lists the commands. |

Bench mode changes nothing else: arming still needs a zero command for `ARM_HOLD_MS`,
the contactor sequence is the same, the thrust limit and the direction-change hold apply,
and the command stream must stay at zero until RUN. The status line reports the active
source in `cmd_source` (0 PWM, 1 bench). `tools/vcb_bench_drive.py` on the Jetson wraps the
whole sequence (on, arm, ramp, hold, ramp down, off) with its own cap and abort rules; it
is stage 3 of `docs/VCB_COMMISSIONING.md`. Line length is limited to 47 characters; longer
lines are discarded with a message.

## Arming, start-up, faults

Thruster states keep cttdev's numbering: 0 FAULT, 1 IDLE, 2 STARTUP_LSC,
3 STARTUP_WAIT, 4 STARTUP_HSC, 5 RUN.

1. Boot: IDLE, contactors open, "stop" sent to the converter.
2. **Arm**: both channels valid and within +-50 us of 1500 us for 1 s, while IDLE.
   Prints `ARMED: ...`. Warns if the E-stop or TSMS 24 V sense reads absent.
   Neutral stays required until RUN: if either channel leaves the +-50 us window while
   armed but the contactors are not closed yet (IDLE waiting for the bus, or during the
   start-up sequence) the board disarms, aborts the start-up and opens the contactors,
   and prints `DISARMED: throttle left neutral before RUN`. The first command ever sent in
   RUN is therefore zero; thrust only follows a stick moved after the contactors closed.
3. **Start-up** (unchanged from cttdev): needs bus < 5 V and neutral, then LSC energised
   and LSI must confirm within 1 s, wait 500 ms, HSC energised and HSI must confirm within
   1 s, 10 ms later bus must be >= 40 V, then RUN. If the bus is >= 5 V after arming it
   waits in IDLE and says so every 5 s.
4. **RUN**: both channels forwarded to the converter every 15 ms. Faults when the bus
   drops below 35 V, or a contactor reads open for 50 ms (E-stop / TSMS pulled), or PWM
   is lost on either channel.
5. **FAULT**: contactors opened, converter told to stop (and again every 100 ms), disarmed.
   `fault_code` in the status line says why:

   | code | reason |
   |---|---|
   | 1 | PWM lost or invalid |
   | 2 / 3 | LSC already closed at start / LSC did not close in 1 s |
   | 4 / 5 | HSC already closed at start / HSC did not close in 1 s |
   | 6 | bus < 40 V 10 ms after HSC closed |
   | 7 | bus < 35 V in RUN |
   | 8 | contactor opened in RUN (E-stop or TSMS pulled); latched, see below |
   | 9 | `stop` on the console; latched until `clear` or reset |

   With `REQUIRE_ESTOP_TSMS_TO_START` set, an absent E-stop/TSMS sense is not a fault:
   the board waits in IDLE and says so every 5 s.

6. **Recovery**: after both channels have been valid and neutral for 3 s the board goes
   back to IDLE and the arm sequence (step 2) runs again. Interlock faults (codes 2-7)
   are retried at most 3 times per boot, then the fault is latched until reset. PWM-loss
   faults recover without limit. Fault 8 is latched (`CONTACTOR_OPEN_FAULT_LATCHED`):
   releasing the E-stop must not re-close the contactors by itself. Fault 9 is latched
   until `clear`. Set
   `FAULT_RECOVER_HOLD_MS` to 0 for the deployed build's behaviour (everything latched
   until power cycle).

So, as on the deployed build, a Pixhawk that holds neutral (disarmed, or armed with
zero throttle) arms the VCB and closes the contactors by itself. Rover's own arming
is the operator's gate.

## Telemetry (UART4, 115200)

Same line kinds as the deployed build, so `scripts/vcb_logger.py`,
`tools/vcb_bench_check.py` and the GCS page need no change:

```
VCB pixhawk-pwm reversible 0.3.2; limit 200 permille both directions; neutral 1500 us; deadband 25 us; window 700..2300 us; dir-change stop 200 ms; console commands: help
mapping,ch1=CONT_pin2_PA3,ch2=CONT_pin1_PA2,motors=ch1_right:ch2_left,range_us=1000:2000,neutral_us=1500,deadband_us=25,limit_permille=200,window_us=700:2300,dir_stop_ms=200,cmd_full_scale=127,fw=0.3.2
pwm,ms,ch1_us,ch1_period_us,ch1_valid,ch1_raw_permille,ch1_applied_permille,ch1_ok,ch1_bad,ch1_ovr,ch2_us,...,ch2_ovr,ch1_cmd,ch2_cmd,ch1_glitch,ch2_glitch
status,ms,thruster_state,armed,estop_24v_present,tsms_24v_present,lsc_closed,hsc_closed,bus_mv,fault_code,interlock_faults,cmd_source
pwm,12345,1500,20000,1,0,0,617,0,0,1499,20001,1,0,0,617,0,0,0,0,41,40
status,12400,1,1,1,1,0,0,1200,0,0,0
```

- `pwm` at 20 Hz, `status` at 5 Hz, the `mapping` and header lines at boot and every 10 s.
- `raw_permille` / `applied_permille` are now signed. The GCS thrust bars show reverse as a
  negative number with an empty bar (no change made to `scripts/gcs.py`).
- `status` has three new trailing fields, `fault_code`, `interlock_faults` and
  `cmd_source` (0 PWM, 1 bench), and `pwm` has `ch1_cmd` and `ch2_cmd` (the signed value
  byte last sent to the converter for that channel's motor; 0 unless in RUN; 25 at the
  default limit; the direction hold shows as 0 between + and -) followed by `ch1_glitch`
  and `ch2_glitch` (glitch pairs discarded since boot). In bench mode
  `raw_permille`/`applied_permille` are the bench command while `chN_us`/`chN_valid` still
  show the measured PWM. The logger picks new fields up from the header line.
- Free-text event lines (`ARMED: ...`, `FAULT 3: ...`, `THRUSTER STARTUP_LSC: ...`) are as
  close to the deployed build's wording as the disassembly allows, without the `BENCH` prefix.

## LEDs

| LED | Meaning |
|---|---|
| red (CTL3) solid | FAULT |
| orange (CTL2) solid / fast blink / slow blink | IDLE / start-up sequence / RUN |
| green (CTL1) solid / slow blink | both PWM inputs valid / valid and armed |

## Pixhawk parameters

The deployed contract was forward-only with 990/990/1200. For this build:

```
SERVO9_FUNCTION,74     # ThrottleRight -> AUX1 -> VCB ch1 (unchanged, measured 2026-10-04)
SERVO10_FUNCTION,73    # ThrottleLeft  -> AUX2 -> VCB ch2
SERVO9_MIN,1380        # full Rover reverse = -200 permille = -20 % at the default limit
SERVO9_TRIM,1500       # stop; also the disarmed output, which arms the VCB
SERVO9_MAX,1620        # full Rover forward = +20 %  (25 us deadband + 95 us ramp)
SERVO10_MIN,1380
SERVO10_TRIM,1500
SERVO10_MAX,1620
```

`params/vcb-pixhawk.parm` holds these. With MIN/MAX at 1000/2000 instead, Rover's
throttle would spend 80 % of its range on the clamped plateau, as discussed in
`docs/HARDWARE.md`. Raise MIN/MAX together with `THRUST_LIMIT_PERMILLE`.
`MOT_SLEWRATE` (Rover) is the place for a throttle slew limit; this build has none.

**Do not apply these while the old firmware is flashed**: it reads 1500 us as 50 %
thrust, clamped to 20 %, on both motors, as soon as it arms.

## Build

STM32CubeIDE: import the folder as an existing project (`.project`, `.cproject`), build.
The `.ioc` was edited by hand to match the code (USART2 removed, TIM2/TIM5 input
capture on PA2/PA3). It was not opened in CubeMX here, so if CubeMX complains or
regenerates differently, set TIM2 and TIM5 as: internal clock, prescaler 127,
period 0xFFFFFFFF; TIM2 CH3 = input capture direct on TI3 (rising), CH4 = indirect
on TI3 (falling); TIM5 CH4 = direct on TI4 (rising), CH3 = indirect on TI4
(falling); both interrupts enabled; PA2 = TIM2_CH3 and PA3 = TIM5_CH4 with pull-down.
All application code is inside `USER CODE` blocks or in new files, so regeneration
keeps it.

Command line (any `arm-none-eabi-gcc` with newlib; CubeIDE's own works):

```
cd firmware/VCB-pixhawk
make                       # build/VCB-pixhawk.{elf,bin,hex,map}
make TOOLCHAIN=/opt/st/stm32cubeide/plugins/.../tools/bin/
```

On an Apple Silicon Mac the `/opt/gcc-arm-none-eabi-10-2020-q4-major` install is an Intel
binary and Homebrew's `arm-none-eabi-gcc` ships without newlib; use the Arm GNU Toolchain
`darwin-arm64` tarball from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
(14.2.Rel1 was used here) and pass its `bin/` as `TOOLCHAIN=`. On the Jetson,
`apt install gcc-arm-none-eabi` works too.

## Before flashing

1. **Dump the current flash again** and keep it next to the August dump:
   ```
   # on the Jetson (STLINK-V3 is on the Jetson, serial 003E001C3234511037333934)
   STM32_Programmer_CLI -c port=SWD mode=hotplug -u 0x08000000 0x100000 VCB_flash_$(date +%Y%m%d_%H%M%S).bin
   # or
   openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
     -c "init; halt; dump_image VCB_flash_$(date +%Y%m%d_%H%M%S).bin 0x08000000 0x100000; resume; exit"
   # (hotplug / halt+resume: reads the flash without resetting the running build, which keeps
   # printing its banner so you can also see what it calls itself)
   ```
   Diff it against `firmware/VCB_flash_20260805_211528_pre_tolerant_pwm.bin`; that
   settles open question 1 in `docs/HARDWARE.md` (which build runs today).
2. HV bus down, both contactors open, Pixhawk outputs at the **new** neutral
   (1500 us) or unplugged. With the old params (990 us idle) this build reads 990 us as
   full reverse: valid, but never neutral, so it never arms. Safe, but confusing.
3. Flash. The built image is committed as `firmware/VCB-pixhawk-0.3.1.bin`
   (sha256 `5cde2afaa82b94a7a8ddbb0515eed363c2a4f1c200fdaea2b164953e0a0f82b5`), so the Jetson
   checkout has it; `build/` is not committed. (`VCB-pixhawk-0.3.0.bin` is the first image
   that ran on the boat, kept for the record; its PWM capture does not qualify.)
   ```
   sudo openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program firmware/VCB-pixhawk-0.3.1.bin 0x08000000 verify reset exit"
   # or
   STM32_Programmer_CLI -c port=SWD -w firmware/VCB-pixhawk-0.3.1.bin 0x08000000 -v -rst
   ```
   (sudo, or the udev rule in the cheat sheet, for the STLINK's USB debug interface.)
4. `cat ~/vcb.json` or `vcb_logger.py --stdout`: the first line must be the
   `VCB pixhawk-pwm reversible` banner. If the console is silent, UART4 is not the
   VCP or the baud differs; the deployed build's console port was inferred, not read.
5. To go back: flash the dump from step 1 the same way.

## Bench test plan (dead bus first)

This is stage 0 of `docs/VCB_COMMISSIONING.md`, which continues from here with the
converter, contactors, pack and motors connected.

1. Apply `params/vcb-pixhawk.parm`. Confirm `status` shows `thruster_state=1`, `armed=0`.
2. Pixhawk disarmed: both channels should read ~1500 us, `valid=1`, `raw=applied=0`.
   After 1 s: `ARMED`, then (bus < 5 V) STARTUP_LSC. With TSMS absent expect
   `FAULT 3` (LSC timeout), three retries, then `FAULT latched`. That exercises the
   recovery path. With TSMS present expect `FAULT 6` (bus < 40 V) because the bus is dead.
3. `tools/vcb_bench_check.py --pwm 1500,1550,1600,1620,1700,1450,1400,1380,1300,1000,2000`
   with the bus dead. Expected `raw` (measured us may read ~0.3-1 % high, see the HSI note):
   0, 52, 157, 200, 368, -52, -157, -200, -368, -1000, 1000; `applied` clamps at +-200;
   `LEFT` must reach ch2 / `RIGHT` ch1 as before; 1000 and 2000 must stay `valid=1`.
   `cmd` stays 0 throughout: the bus is dead, so the VCB is not in RUN.
   Then the end points, the case that bit the deployed build:
   `--pwm 990,1000,1010,1990,2000,2010` -> `raw` -1000, -1000, -978, 978, 1000, 1000, all
   `valid=1`, `bad` not counting up. If the motor test lets you (ArduPilot may refuse values
   outside 800..2200), add 2250 and 2300: still valid, raw 1000; and 2350 or 650: `bad`
   counts up, `valid` drops to 0 after 100 ms. A servo tester on the CONT pins does the
   same job if the Pixhawk will not.
   Then move a stick while armed-but-IDLE: expect `DISARMED: throttle left neutral before RUN`.
4. Unplug one input: that channel invalid within 100 ms; if armed, `FAULT 1` and
   `DISARMED`. Plug back: valid after 3 pulses, IDLE 3 s later, ARMED 1 s after that.
5. With the pack on, props in water or removed: arm, confirm RUN, then a right-only
   forward command. **Confirm the right prop turns forward**, and that `ch1_cmd` reads 25
   at full forward (if the motor visibly runs at more like 25 % than 20 %, the converter's
   full scale is 100, not 127: set `EPROP_FULL_SCALE_VALUE` to 100). Then left, then
   reverse on each: `cmd` goes 25 -> 0 (200 ms) -> -25 and the motor must follow without
   the converter dropping it. Hold 2000 us and 1000 us for a while on each side: the motor
   must keep running at the limit, with `bad` not counting. Only then raise
   `THRUST_LIMIT_PERMILLE`.

## Knobs (`Core/Inc/config.h`)

`THRUST_LIMIT_PERMILLE`, `BENCH_LIMIT_PERMILLE`, `BENCH_TIMEOUT_MS`,
`DIRECTION_CHANGE_STOP_MS`, `EPROP_FULL_SCALE_VALUE`,
`PWM_DEADBAND_US`, `PWM_MIN_WIDTH_US` / `PWM_MAX_WIDTH_US`, `ARM_NEUTRAL_WINDOW_US`,
`ARM_HOLD_MS`, `FAULT_RECOVER_HOLD_MS`, `INTERLOCK_FAULT_MAX_RETRIES`,
`CONTACTOR_OPEN_FAULT_LATCHED`, `REQUIRE_ESTOP_TSMS_TO_START`, `THRUSTER_CMD_PERIOD_MS`,
the bus voltage thresholds, the telemetry periods.

## Known unknowns

- The deployed build's console port (UART4) and baud (115200) are inferred from the
  cttdev project and the fact that `/dev/vcb` at 115200 produced data; not read from the dump.
- What the ePropulsion converter does with a frame it does not like. The one data point is
  the deployed build's incident (out-of-range value -> motors dropped). This build never
  sends a value outside 0..0x7F, and passes through 0 on a direction change
  (`DIRECTION_CHANGE_STOP_MS`), in case a direct reversal gets the same treatment.
- The converter's full scale: cttdev's code sends 0x7F for 100 % and this build keeps that
  (`EPROP_FULL_SCALE_VALUE`). If it is really 100, the default 20 % limit is 25 % at the
  motor. Bench step 5 is where to notice.
- The E-stop and TSMS sense polarity, and whether the contactor coils really lose power
  when either is pulled. Fault code 8 assumes they do.
- Physical left/right: see "PWM contract".
- Whether the precharge path can take repeated start-up attempts (hence the retry limit).
- Where the CONT-line glitches come from. They are on both lines at once, roughly every
  5 ms, shorter than a microsecond. The 8 us filter makes them irrelevant to the capture;
  finding the source (cable, Pixhawk output stage, something on the board switching) is a
  scope job for a quiet day.
- The clock: `VCB.ioc` declares an HSE crystal on PH0/PH1 but cttdev's `SystemClock_Config`
  runs the PLL from HSI. Its frequency is unknown, so this build does not switch. Moving
  to HSE would make the microsecond timebase crystal-accurate and let the deadband shrink.
- Console output pacing: one telemetry line per 5 ms loop, so a 190-byte header line
  still blocks the loop ~17 ms once every 10 s (`_write` waits for the previous DMA).
  Harmless unless the ePropulsion converter has a command watchdog tighter than ~40 ms.
