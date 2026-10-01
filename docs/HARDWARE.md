# HighFieldBoat — hardware and firmware, what we actually know

Each fact is tagged with how we know it. Upgrade tags as the inventories run.

- **[src]** read in source code (cttdev GitHub, May 2026)
- **[bin]** read from strings in the deployed flash dump `firmware/VCB_flash_20260805_211528_pre_tolerant_pwm.bin`
- **[told]** secondhand description, not yet verified
- **[?]** unknown

## Topology

```
RadioMaster Boxer ─ELRS─► (no receiver yet)
                                     Pixhawk (ArduRover 4.7.0 [told])
GC01-02 GPS ──CAN1 / DroneCAN──►        │ PWM ch1, ch2
                                        ▼
                               VCB  (STM32H723VG)  ── UART5 ──► ePropulsion protocol converter ──► 2× Navy 6.0
                                 │   ├─ UART7 ──► MCB "LARS"  (STM32U535, DRV8245 ×2, 2.25 A limit)
                                 │   └─ UART3 ──► MCB "REEL"  (same board, 6 A limit, commented out)
                                 │   contactors LSC/HSC, E-stop + TSMS sense, HV bus ADC
                                 └── USB-CDC ──► Jetson Orin Nano  (CSV telemetry [bin])
Jetson ── USB ──► Pixhawk (/dev/ttyACM0, mavlink-router TCP 5760) ── Tailscale ──► QGC / laptop
Jetson ── FTDI RS485 ──► LiTime 48 V 100 Ah BMS (battery_litime.py)
Jetson ── Ethernet ──► IP camera 192.168.1.110 (MediaMTX)
```

## Flight controller
- Model **[?]** — get from `ap_inventory.py` boot banner / `lsusb`.
- Firmware ArduRover 4.7.0 **[told]**.
- GPS "GC01-02", DroneCAN on CAN1, `GPS_TYPE=9` **[told]**. Confirm part via QGC DroneCAN node list; it likely includes a compass.
- RC receiver: none installed **[told]**. Plan: ELRS diversity RX (RP3/RP4TD) on a full UART, CRSF.
- Power module / battery sense on the Pixhawk **[?]** — `POWER_STATUS` flags will tell.

## VCB (Vehicle Control Board) — the PWM → ePropulsion gateway

Two firmware generations exist:

### A. GitHub `cttdev/VCB` (May 2026) — RC-direct version **[src]**
- Reads **CRSF directly** from an RC receiver on USART2 (no Pixhawk in the loop).
- Thruster state machine: FAULT → IDLE → STARTUP_LSC → STARTUP_WAIT(500 ms) → STARTUP_HSC → RUN.
  Pre-charge sequence closes the low-side then high-side contactor, checks bus ≥ 40 V,
  requires "thruster enable" switch + "charge" button; any CRSF loss > 500 ms → FAULT;
  recovery needs charge button held 1 s.
- In RUN: `left = throttle − steer`, `right = throttle + steer`, both in [−1, 1], sent every loop (15 ms).
- Bus voltage ADC: differential 16-bit, scalar 28.27 (30k/30k/2.2k divider).
- Aux state machine drives the two MCBs (LARS / REEL) with text commands `forward0.50\n`, `reverse…`, `idle\n`, `ping\n`.

### B. Deployed "pre_tolerant_pwm" bench build (dumped 2026-08-05) **[bin]** — not on GitHub
Strings in the flash image describe it:
- `VCB independent-motor PWM bench test; hard limit 20 percent`
- `mapping,ch1=CONT_pin2_PA3,ch2=CONT_pin1_PA2,motors=ch1_right:ch2_left,range_us=1000:2000,clamp_above_us=1200`
- `BENCH ARMED: both PWM channels held stopped for 1 second`
- `BENCH DISARMED: thruster startup/run interlock fault`
- `BENCH FAULT: PWM lost or invalid`
- CSV telemetry: `pwm,ms,ch1_us,ch1_period_us,ch1_valid,ch1_raw_permille,ch1_applied_permille,ch1_ok,ch1_bad,ch1_ovr,ch2_…`
  and `status,ms,thruster_state,armed,estop_24v_present,tsms_24v_present,lsc_closed,hsc_closed,bus_mv`

Reading of the PWM contract from those strings (needs bench confirmation):
- Two **independent, forward-only** channels: ch1 = right motor, ch2 = left motor.
- Nominal input range 1000–2000 µs. Pulses above **1200 µs are clamped**, i.e. the
  usable band is ~1000–1200 µs = 0–20 % thrust; everything above 1200 µs = 20 %.
  This differs from the secondhand "1100–1900 → 0–20 %" **[told]** description.
- Arming: both channels at "stopped" for 1 s. PWM dropout or invalid period → FAULT.
- The ePropulsion frame **[src]**: `28 addr 03 2C dir value xor 29`, addr 1 = left, 2 = right,
  dir 1 fwd / 0 rev, value 0–127. So the hardware *can* reverse; the bench build just doesn't.

**Caveat:** the file name says *pre*-tolerant-pwm, meaning a later "tolerant PWM" build
was likely flashed after this dump. What runs today may differ. The USB-CDC CSV stream is
the quickest way to find out: `cat /dev/ttyACM1` on the Jetson will print `mapping,…` and
live `pwm,…` lines if the bench build (or a descendant) is running.

### Consequences for ArduPilot
- **No reverse** on the current firmware ⇒ a skid-steer boat cannot turn in place or stop
  quickly; Rover's steering authority is only differential forward thrust. Tuning still
  works (Rover handles `MOT_THR_MIN`), but expect large turn radii at low speed.
- **20 % thrust cap** ⇒ `CRUISE_THROTTLE` will sit near 100 % of the available band.
  Set `SERVO1/3_MIN=1000`, `SERVO1/3_MAX=1200` so ArduPilot's full range maps to the
  usable band rather than spending 80 % of its output range on a clamped plateau.
- The VCB's own startup interlock (charge button, contactors) must complete before
  ArduPilot output means anything. Needs a physical step or a change to the VCB.
- Both are **firmware policy, not hardware limits**. When the boat is closer to the
  water, re-evaluate whether to rebuild the VCB firmware from the GitHub version with
  PWM input, reverse, and a higher cap; the source for the bench build is lost.

## MCB (Motor Control Board) **[src]**
STM32U535, two DRV8245 brushed H-bridges, current via IPROPI ADC, 1 s power ramp,
over-current fault (2.25 A "LARS" variant, 6 A "REEL" variant), text command protocol
over UART4 (VCP) or USART1 (RS485). Not in the propulsion path; drives auxiliary
actuators (LARS = launch-and-recovery?, REEL).

## Battery
LiTime 48 V ComFlex, 16 cells, 100 Ah; RS485 at 19200 8N1 via FTDI on the Jetson;
decode verified 2026-08-03 **[src: scripts/battery_litime.py]**. Not visible to ArduPilot.

## Open questions, in order of importance
1. Which VCB build is running now, and does the CSV stream appear on the Jetson's USB?
2. Exact Pixhawk model and which UART is free for the ELRS receiver.
3. Pixhawk param dump: is `SERVOn_MAX` already 1200? `FRAME_CLASS`, `MOT_*`?
4. Is the GC01-02 a compass too, and is it the only compass?
5. How does the VCB charge/enable interlock get satisfied with no RC receiver present?
