# HighFieldBoat — hardware and firmware, what we actually know

Each fact is tagged with how we know it. Upgrade tags as the inventories run.

- **[src]** read in source code (cttdev GitHub, May 2026)
- **[bin]** read from strings in the deployed flash dump `firmware/VCB_flash_20260805_211528_pre_tolerant_pwm.bin`
- **[told]** secondhand description, not yet verified
- **[inv]** read from the vehicle by the 2026-10-04 inventories (`docs/inventory/`, `params/2026-10-04.parm`)
- **[?]** unknown

## Topology

```
RadioMaster Boxer ─ELRS─► (no receiver yet)
                                     Pixhawk 6X (ArduRover 4.7.0 [inv])
CG01-02 GPS ──CAN1 / DroneCAN──►        │ PWM AUX1 (ThrottleLeft), AUX2 (ThrottleRight) [inv]
                                        ▼
                               VCB  (STM32H723VG)  ── UART5 ──► ePropulsion protocol converter ──► 2× Navy 6.0
                                 │   ├─ UART7 ──► MCB "LARS"  (STM32U535, DRV8245 ×2, 2.25 A limit)
                                 │   └─ UART3 ──► MCB "REEL"  (same board, 6 A limit, commented out)
                                 │   contactors LSC/HSC, E-stop + TSMS sense, HV bus ADC
                                 └── USB-CDC ──► Jetson Orin Nano  (CSV telemetry [bin])
Jetson ── USB ──► Pixhawk (/dev/ttyACM0 = if00, mavlink-router TCP 5760) ── Tailscale ──► QGC / laptop
Jetson ── FTDI FT230X (/dev/ttyUSB0) RS485 ──► LiTime 48 V 100 Ah BMS (battery_litime.py)
Jetson ── Ethernet ──► IP camera 192.168.1.110 (MediaMTX)
```

## Flight controller
- **Holybro Pixhawk 6X**, IOMCU present, 3 IMUs, 2 baros **[inv]** (boot banner).
  USB `1209:5740`, serial `2A0026000E51333439333237`. `ttyACM0` = if00 = SERIAL0 MAVLink;
  `ttyACM1` = if02 = SERIAL8 MAVLink (second USB port, not the VCB).
- ArduRover **4.7.0 official** (`1511f271`), `FRAME_CLASS=2` boat **[inv]**.
- GPS: DroneCAN on CAN1, `GPS1_TYPE=9`, node 125, DGPS fix with 26 sats at the dock **[inv]**.
  The part number (CG01-02) is still **[told]**.
- **Compasses** **[inv 2026-10-04, after `COMPASS_ENABLE=1` + reboot]**:
  - `COMPASS_DEV_ID=97539`: DroneCAN, node 125. The GPS puck has a magnetometer. Use it as the primary.
  - `COMPASS_DEV_ID2=331777`: I2C 0x10, devtype 5 = the Pixhawk 6X's internal BMM150.
    It sits near the motor wiring, so plan `COMPASS_USE2=0`.
  - The original dump had `COMPASS_ENABLE=0` and `COMPASS_USE=0`, so the EKF had no heading and
    stayed in `CONST_POS_MODE` (no horizontal position). To do: `COMPASS_USE=1`, check
    `COMPASS_ORIENT` against the puck's arrow, then a large-vehicle mag cal.
- RC receiver: none (`RC_CHANNELS` count 0) **[inv]**. `RC_PROTOCOLS=1` (all). Free UARTs:
  SERIAL5 (TELEM3) and SERIAL6 (UART4) have protocol −1. TELEM1/2 are set to MAVLink 57600
  and their use is unknown. Plan: ELRS RX on TELEM3, `SERIAL5_PROTOCOL=23`.
- **No power module**: `POWER_STATUS` has no `BRICK_VALID`, and `Vservo=0` **[inv]**.
  `BATT_MONITOR=21` (the 6X default for an I2C power module), so `SENSOR_BATTERY` reads BAD and
  probably blocks arming. Set it to 0 until battery injection (tools/README option 2) exists.
- Outputs **[inv]**:
  - `SERVO9_FUNCTION=73` ThrottleLeft and `SERVO10_FUNCTION=74` ThrottleRight (AUX1/AUX2),
    1000/**1000**/2000 (MIN/TRIM/MAX). With TRIM = MIN the output is forward-only, which matches
    the VCB. Disarmed (`MOT_SAFE_DISARM=0`) they output TRIM = 1000 µs, which is the VCB's
    "stopped" value, so the VCB arming handshake should pass.
  - Leftovers: `SERVO1_FUNCTION=26` GroundSteering and `SERVO3_FUNCTION=70` Throttle on MAIN1/3,
    sitting at 1500 µs while disarmed. They're harmless only if nothing is plugged into MAIN1/3.
    If the VCB is on MAIN1, its right motor sees 1500 µs, which is above the 1200 clamp, so 20 %
    thrust as soon as the VCB arms. Check the wiring, then set both to 0.
  - Wiring to confirm: VCB ch1 (right) → AUX2, VCB ch2 (left) → AUX1.
  - MAX 2000 puts the whole 0–20 % band in the bottom fifth of the range. Set MAX=1200 once the
    bench test confirms the clamp.
- Safety posture **[inv]**: `FS_THR_ENABLE=0`, `FS_GCS_ENABLE=0`, `FENCE_ENABLE=0`,
  `ARMING_SKIPCHK=64` (skips the RC check), all `MODEn`=Manual, `MODE_CH=8`, safety switch
  disabled (`BRD_SAFETY_DEFLT=0`). Nothing stops the boat on a lost link. This needs fixing
  before it goes in the water.
- Logging: `LOG_BITMASK=65535`, `LOG_DISARMED=0`, SD card backend **[inv]**.
- Pre-arm: `PREARM_CHECK` reads BAD, but the 8 s sample caught none of the messages. Read them
  in QGC or wait 30 s with `ap_inventory.py`.

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
**Update 2026-10-04 [inv]:** `ttyACM1` is the Pixhawk's second USB interface, not the VCB.
No STM32 / VCB USB device shows in the Jetson's `lsusb`, so the VCB is either not cabled to
the Jetson or was unpowered at the time.

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
2026-10-04 **[inv]**: the FTDI is on `/dev/ttyUSB0`, but `battery-litime` has no installed unit
file and isn't running. `battery.json` is 61 days old, so the GCS battery card shows stale data.

## Jetson **[inv]**
Orin Nano Super devkit, L4T R36.4.7, Ubuntu 22.04.5, 25 W mode. Tailscale is still relayed via
DERP nyc, and it logs an iptables `--restore-mark` health warning (legacy iptables, harmless
for now). Ethernet 192.168.1.50 (camera LAN), Wi-Fi 10.31.128.214. Not installed: MOOS-IvP,
iArduRoverBridge, mavproxy. `pymavlink` was installed for the inventory.
`mavlink-routerd` (v4-16) isn't run by systemd. A shell loop
(`sleep 15 && while true; do mavlink-routerd -c ~/mav.conf >> ~/mav.log; sleep 5; done`,
probably from cron `@reboot` or rc.local) starts it on `/dev/ttyACM0` and appends to
`~/mav.log` without limit.

## Open questions, in order of importance
1. Which VCB build is running now? Its USB isn't visible on the Jetson; cable it, then `cat` the new ttyACM.
2. What, if anything, is plugged into MAIN1/MAIN3, and does the VCB go to AUX1/AUX2?
3. What exactly is the CG01-02 (label only says that)? Ask node 125 with `GetNodeInfo` over MAVLink-CAN.
4. What are the pre-arm failure messages (beyond the battery monitor)?
5. How does the VCB charge/enable interlock get satisfied with no RC receiver present?
6. What is on TELEM1/TELEM2 (both set to MAVLink)?
