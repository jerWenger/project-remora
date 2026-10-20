# VCB commissioning plan: from "motors wired" to "boat drives"

Ordered plan for bringing the Pixhawk → VCB → ePropulsion chain up with the motors
connected. Safety systems first, then the VCB's own arming, then motion from the least to
the most indirect command path: Jetson console, Pixhawk motor test, RC. **Props stay off
through stage 5.** The thrust limit stays at 20 % until the control system has been tested
and tuned under autonomy (stage 7); raising it is the last thing on the list.

Written for the `firmware/VCB-pixhawk` 0.3.0 build (1500 µs = stop, reverse allowed, 20 %
limit, console bench mode capped at 10 %). Each stage has a gate; do not start a stage until
the previous gate passed. Fill in **date / pass / fail / notes** in the tables, then copy
the outcome into the firmware README status line and `HARDWARE.md` §C.

Companions: `VCB_COMMISSIONING_CHECKLIST.pdf` is this plan as a printable with tick boxes,
sign-off lines and value fields (source: `VCB_COMMISSIONING_CHECKLIST.html`, rendered with
headless Chrome); `VCB_COMMISSIONING_CHEATSHEET.md` has every command in test order and is
the PDF's appendix.

## What you need on the bench

| Item | Why |
|---|---|
| Jetson with `vcb-logger` running, `vcb.json` updating, dashboard on :8080 | every pass/fail below is read from the VCB telemetry (`status` and `pwm` fields) |
| `tools/vcb_bench_drive.py` on the Jetson (stage 3) | sends `bench` commands on `/dev/vcb`, watches `vcb.json`, aborts on anything odd |
| Pixhawk on USB to the Jetson, `mavlink-router` on TCP 5760, QGC or MAVProxy (stage 4 on) | motor test commands, params with `tools/ap_apply.py` |
| Multimeter on the HV bus (contactor output side) and on the contactor coil feed | `bus_mv` is an uncalibrated ADC reading; the coil feed is how the kill chain is verified without closing anything |
| BMS reader (`battery_litime.py`, `battery.json`) | pack current while the motors turn |
| Optional: USB-RS485 adapter sniffing UART5 (38400 8N1) | to see the frames the converter receives |
| Optional: servo tester for the CONT pins (stage 4) | exact pulse widths without ArduPilot in the way. The CONT pins are 3.3 V logic with pull-downs; check the tester's output level before connecting a 5 V one |
| One person at the E-stop, nobody near the motors | the VCB arms and closes the contactors **by itself** when both inputs sit at zero for 1 s |

Abort rule for every stage: anything unexpected (a contactor closes when it should not, a
motor moves when it should not, `bus_mv` disagrees with the meter, a fault code you did not
provoke) → E-stop, then power-cycle the VCB and read the log before going on.

Telemetry fields used below (meanings in `firmware/VCB-pixhawk/README.md`):
`status.thruster_state` (1 IDLE, 2–4 start-up, 5 RUN, 0 FAULT), `status.armed`,
`status.lsc_closed` / `hsc_closed`, `status.bus_mv`, `status.fault_code`,
`status.estop_24v_present` / `tsms_24v_present`, `status.cmd_source` (0 PWM, 1 bench),
`pwm.chN_us`, `chN_valid`, `chN_raw_permille`, `chN_applied_permille`, `chN_bad`, and
`chN_cmd` (the signed byte last sent to the converter; ch1 = right motor, ch2 = left).

## Stage 0: firmware and dead-bus checks (firmware README, "Before flashing" and "Bench test plan")

Pack disconnected. Can be done now.

1. Fresh flash dump of whatever is on the VCB, kept next to the August dump. Settles which build runs today.
2. Flash 0.3.0. Banner `VCB pixhawk-pwm reversible 0.3.0` on `/dev/vcb`. If silent: UART4/115200 was inferred, not read from the dump; try other baud rates before anything else.
3. `help` typed on `/dev/vcb` (`echo help > /dev/vcb`) → the command list comes back. That proves the console input works before stage 3 depends on it.
4. Apply `params/vcb-pixhawk.parm` (SERVO9/10 at 1380/1500/1620). Only after step 2: the old build reads 1500 µs as 20 % thrust on both motors.
5. Pixhawk safety switch off (or `BRD_SAFETY_DEFLT=0`): with the switch engaged there is no PWM and the VCB never arms.
6. Dead-bus contract check with `tools/vcb_bench_check.py`: the ±permille curve, the end points 990/1000/1010/1990/2000/2010 µs (all `valid=1`, `bad` not counting), left/right channel mapping, `DISARMED: throttle left neutral before RUN` when a stick moves early, PWM loss → `FAULT 1` → recovery.
7. Note the timebase offset: a commanded 1500 µs should read 1485–1515. Outside that, the HSI clock is further off than assumed and `PWM_DEADBAND_US` needs attention before stage 4.

**Gate 0:** every item passed; console input proven; measured neutral inside the deadband.

| date | pass/fail | notes |
|---|---|---|
| | | |

## Stage 1: safety: every kill path, with the contactors open

Goal: prove each way of removing power before anything can close a contactor. Pack
connected, VCB powered, Pixhawk **unplugged from the CONT port** (or outputs invalid), so
the VCB cannot arm: `thruster_state` 1, `armed` 0 throughout. If the VCB arms during this
stage, stop and find out why.

| Step | Do | Watch | Pass |
|---|---|---|---|
| 1.1 | E-stop pressed / released, three times | `estop_24v_present` follows; meter on the contactor coil feed | the coil feed loses 24 V when pressed. Record which way the sense reads (polarity is an open question) |
| 1.2 | TSMS pulled / inserted, three times | `tsms_24v_present`; coil feed | same |
| 1.3 | Main battery disconnect / BMS off | `bus_mv` at the contactor input side (meter), BMS reader | the HV side is dead at the contactor input |
| 1.4 | With the E-stop **pressed**, plug the Pixhawk in at neutral so the VCB arms and tries to start | `ARMED`, `STARTUP_LSC`, then `FAULT 3` (LSI timeout) within 1 s; `lsc_closed` stays 0 | a contactor cannot close with the E-stop pressed. Three retries, then `FAULT latched`. Unplug the Pixhawk, reset the VCB |
| 1.5 | Same with TSMS pulled | same | same |
| 1.6 | `echo stop > /dev/vcb` while IDLE | `FAULT 9`, `FAULT latched: stop command`; `echo clear > /dev/vcb` un-latches | the software stop works end to end (it is a convenience, not a safety device) |
| 1.7 | Walk the kill chain physically: which devices lose power when the E-stop is pressed (converter? motors? only the coils?) | by inspection and meter | written down. Stage 3.8 tests it with a motor turning |

**Gate 1:** both hardware kills remove coil power and block a start-up; the software stop
works; sense polarity recorded.

| date | pass/fail | estop sense polarity | tsms sense polarity | what loses power on E-stop | notes |
|---|---|---|---|---|---|
| | | | | | |

## Stage 2: VCB arm, start-up, disarm, zero throttle

Goal: the VCB closes the contactors in the right order, the bus comes up, the VCB reaches
RUN with `cmd` 0/0, every stop path stops, and it recovers where it should. Pack connected.
Motors can stay connected: the command is zero throughout.

Power-up order matters: Pixhawk first (its disarmed output is 1500 µs, which is what arms the
VCB), then TSMS present and E-stop released, then VCB power. The VCB arms 1 s after boot and
starts the sequence at once if the bus reads < 5 V. The E-stop person is ready at VCB
power-on.

| Step | Do | Watch | Pass |
|---|---|---|---|
| 2.1 | VCB power on, hands off | `ARMED`, then `thruster_state` 1 → 2 → 3 → 4 → 5 within ~2 s; `lsc_closed` then `hsc_closed` = 1; `bus_mv` rises to the pack voltage | RUN reached, `cmd` 0/0, no fault |
| 2.2 | Compare `bus_mv` with the meter at the contactor output | `status.bus_mv` | agree within 1 V. If not, the 28.27 divider scalar in `config.h` is wrong for this board; fix before stage 3, the 40 V / 35 V thresholds depend on it |
| 2.3 | Stay in RUN 10 min | `fault_code` 0, `cmd` 0/0, contactor coils and precharge resistor temperature by hand | nothing warms up unexpectedly |
| 2.4 | **E-stop in RUN** (the dynamic half of stage 1) | `FAULT 8` within ~50 ms, `lsc_closed` / `hsc_closed` → 0, `FAULT latched`, `armed` 0 | contactors open **and stay open after the E-stop is released** (fault 8 is latched by design) |
| 2.5 | VCB reset, 2.1 again, then TSMS pulled in RUN | as 2.4 | as 2.4 |
| 2.6 | VCB reset, 2.1 again, then pull the CONT cable in RUN | `FAULT 1` within 100 ms, contactors open, `DISARMED` | stop. Plug back at neutral: `FAULT cleared` after 3 s, `ARMED` 1 s later, start-up, RUN: recovers without a power cycle |
| 2.7 | Reboot the Pixhawk while in RUN | outputs vanish → `FAULT 1`; on return at 1500 µs → recovery as in 2.6 | stop, then recover |
| 2.8 | Move a Pixhawk output off neutral while the VCB is armed but not yet in RUN (Pixhawk armed in Manual, stick nudged, during the 1.5 s start-up) | `DISARMED: throttle left neutral before RUN`, start-up aborted | the first command in RUN is always zero |
| 2.9 | `stop` on the console in RUN; then `clear` | `FAULT 9`, contactors open; after `clear` and 3 s neutral, recovery | the software stop and its recovery work with the bus live |
| 2.10 | VCB reset, pack disconnected (bus dead) | `FAULT 6` (bus < 40 V after HSC), up to 3 retries, then `FAULT latched` | the retry limit works; the precharge resistor is not hot after 4 attempts |
| 2.11 | Bench-mode arming: Pixhawk unplugged, `tools/vcb_bench_drive.py --right 0 --left 0 --seconds 5` | `cmd_source` 1, `ARMED`, start-up, RUN with `cmd` 0/0 for 5 s, then `bench off` → `DISARMED: bench off`, contactors open, IDLE. Run it again and Ctrl-C mid-way: same clean exit | the console command path arms and disarms cleanly, nothing turns |
| 2.12 | Three clean power cycles in a row with the pack on | RUN each time | start-up is repeatable |

**Gate 2:** RUN reached 3/3, bus reading trusted, E-stop and TSMS each open the contactors
with fault 8, PWM loss and Pixhawk reboot stop and recover, bench-mode arm/disarm clean.

| date | pass/fail | bus_mv vs meter | notes |
|---|---|---|---|
| | | | |

## Stage 3: Jetson drives the motors directly, 5 %, props off

Goal: the motors turn, in both directions, from the simplest command path there is: the
Jetson writing `bench R L` lines to `/dev/vcb`, no Pixhawk in the loop (unplug it from
CONT, or leave it; bench mode ignores the PWM inputs). 50 permille = 5 % = converter byte
±6. The firmware caps bench commands at 10 % whatever the tool asks for.

`tools/vcb_bench_drive.py` runs each row: `bench on`, zero until RUN, 1 s ramp, hold, ramp
down, `bench off`. It aborts to zero and `bench off` on Ctrl-C, on any fault, or if the VCB
leaves RUN. Watch the dashboard thruster card and the BMS current alongside.

| Step | Command | Watch | Pass |
|---|---|---|---|
| 3.1 | `--right 50 --left 0 --seconds 5` | `ch1_cmd` = 6, **which** motor turns, **which way** it would push water | the right motor, forward (would push water aft). If the left one turns: the converter addresses are swapped relative to cttdev's comment; note it, we swap in firmware (`eprop.cpp`), not in ArduPilot. If it turns backwards: note it, we invert the direction byte in firmware |
| 3.2 | `--right 0 --left 50 --seconds 5` | `ch2_cmd` = 6 | the left motor, forward |
| 3.3 | `--right -50 --left 0`, then `--right 0 --left -50` | `cmd` = −6 | each motor reverses |
| 3.4 | `--right 50 --left 50 --seconds 10` | both `cmd` 6, BMS current | both run together, no fault |
| 3.5 | `--right 50 --left 0 --seconds 30` | steady, `bad` counters not moving | 30 s without the converter dropping the motor |
| 3.6 | `--right 100 --left 100 --seconds 5` | `cmd` 13 (10 %, the bench cap) | runs; then `--right 300 --left 0` must be refused by the tool, and `echo "bench 300 0"` by hand must print `BENCH: clamped to 100 0` |
| 3.7 | During a `--seconds 30` run: kill the tool with `kill -9` (not Ctrl-C) | no `bench` lines for 500 ms → `FAULT 1`, `DISARMED: bench command lost`, motors stop, contactors open | a dead Jetson process stops the motors |
| 3.8 | During a run: **E-stop** | `FAULT 8`, motor stops | the motor must stop or coast, not keep running on the converter's last command. This is the answer to stage 1.7: whether opening the contactors removes motor power, or only the stop frame does |
| 3.9 | During a run: `stop` from a second terminal | `FAULT 9`, motor stops | software stop with a motor turning |
| 3.10 | Reversal through zero: `--right 50 --left 0 --seconds 3 --flip` (3 s at +5 %, then −5 % commanded directly) | `ch1_cmd` 6 → 0 (200 ms) → −6; the motor reverses without the converter dropping it | reversal works at low thrust (repeated at 20 % in 4.6). Same on the left |

**Gate 3:** motor identity and direction known (firmware corrected and re-flashed if not),
both directions on both motors, both together, 30 s steady, every stop path with a motor
turning stops it.

| date | pass/fail | right = ch1? | forward = forward? | 3.7 stop | 3.8 what stops the motor | notes |
|---|---|---|---|---|---|---|
| | | | | | | |

## Stage 4: Pixhawk motor test, props off

Goal: the Pixhawk's PWM path commands the right thing at the 20 % limit, and the three
things the end-point incident taught us hold under power. Pixhawk on CONT, `bench off`.

Command source: ArduPilot motor test (`MAV_CMD_DO_MOTOR_TEST`, motor 3 = ThrottleLeft,
4 = ThrottleRight, type 1 = PWM µs, timeout in s), one motor at a time, sent by
`tools/vcb_bench_check.py --live --side right|left --pwm <us> --hold <s>`. The tool requires
the VCB in RUN with no fault, asks for a typed props-off confirmation, prints the VCB's
measured µs, raw/applied permille and `cmd` bytes for each step, and returns the output to
1500 µs if the VCB leaves RUN. A servo tester on the CONT pins gives exact widths. Every
command is 2 s unless stated; start every step from RUN with `cmd` 0/0. The commands for
every row are in `VCB_COMMISSIONING_CHEATSHEET.md`.

| Step | Command | Watch | Pass |
|---|---|---|---|
| 4.1 | Right (motor 4), 1550 µs | `ch1_raw` ≈ 52, `ch1_cmd` 7, the **right** motor turns forward | AUX1 → ch1 → right motor, as measured dead-bus. If the left one turns: swap `SERVO9/10_FUNCTION` (73↔74) and repeat |
| 4.2 | Left (motor 3), 1550 µs; then both at 1450 µs | `cmd` 7 / −7 | left forward, each reverses |
| 4.3 | Right, 1620 µs (the limit), 10 s | `ch1_cmd` 25, BMS current | steady, no fault |
| 4.4 | Right, 2000 µs, 30 s; then 1000 µs, 30 s | `ch1_raw` ±1000, `ch1_applied` ±200, `ch1_cmd` ±25, `ch1_bad` not counting | **the end-point case**: the motor keeps running at the limit for the whole 30 s. Repeat on the left |
| 4.5 | Servo tester if available: 2200, 2300, 800, 700 µs | still `valid`, raw ±1000; 2350 or 650: `bad` counts, `valid` → 0 after 100 ms, `FAULT 1` | the acceptance window is where the README says |
| 4.6 | Right 1620 µs then immediately 1380 µs (two motor tests back to back, or the tester) | `ch1_cmd` 25 → 0 (200 ms) → −25 | reversal at 20 % without a dropout. Repeat on the left |
| 4.7 | Optional: `DIRECTION_CHANGE_STOP_MS` 0, rebuild, repeat 4.6 | same | tells us whether the hold is needed. If the converter drops the motor without it, keep the default and write that down |
| 4.8 | Scale check at 1620 µs | ePropulsion display throttle % or power, BMS current, versus the rating | 25/127 = 20 % of scale. If the motor behaves like 25 %, the converter's full scale is 100: `EPROP_FULL_SCALE_VALUE` 100, rebuild |
| 4.9 | ArduPilot disarm while the VCB is in RUN | output goes to TRIM 1500 µs: the VCB **stays in RUN with cmd 0**, contactors closed | by design (same as the deployed build). Decide whether that is acceptable; the alternatives are a Rover-side `MOT_SAFE_DISARM` change or a VCB-side timeout, neither implemented |

**Gate 4:** PWM mapping confirmed, 30 s at each end point without a dropout on each side,
reversal at 20 % without a dropout, full scale known, 4.9 decided.

| date | pass/fail | mapping ok? | end points ok? | reversal ok? | full scale 127/100 | 4.9 decision | notes |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

## Stage 5: RC commands, props off

Needs the RC receiver installed (ELRS XR3, ordered 2026-10-13) and RC calibrated. Pixhawk
on CONT, `params/vcb-pixhawk.parm` applied, `MOT_SLEWRATE` set (start at 100: full range
in 1 s; the VCB has no slew limit of its own).

| Step | Do | Watch | Pass |
|---|---|---|---|
| 5.1 | Disarmed: `SERVO_OUTPUT_RAW` AUX1/AUX2 | 1500 both; VCB arms and reaches RUN with `cmd` 0/0 | as in stage 2 |
| 5.2 | Arm in Manual, throttle forward a little | both `cmd` positive, both motors forward | both motors follow the stick |
| 5.3 | Throttle back | both negative | both reverse, through the 200 ms zero |
| 5.4 | Steering right, throttle centred | right motor negative, left positive (or right less than left with throttle) | the pair that would swing the bow right. If mirrored, fix on the Rover side (`SERVO9/10_FUNCTION` or `SERVOn_REVERSED`), not in the VCB |
| 5.5 | Full stick forward, 10 s | `SERVO_OUTPUT_RAW` 1620, `cmd` 25 | Rover's full range lands on the 20 % band, nothing above it |
| 5.6 | Stick slam forward then back | `cmd` ramps at the slew rate, reversal through zero | no dropout |
| 5.7 | RC transmitter off while armed and moving | Rover RC failsafe (hold / disarm per `FS_ACTION`) → outputs to neutral → `cmd` 0 | the VCB sees neutral, not garbage |
| 5.8 | Disarm, arm again | `cmd` 0 while disarmed (stage 4.9), commands resume on arm | no VCB action needed between runs |
| 5.9 | 30 min of gentle stick use | `bad` counters 0, `fault_code` 0, contactor and precharge temperature | clean |

**Gate 5:** boat responds correctly to the sticks in all four directions at the 20 % limit,
RC failsafe ends at neutral, 30 min clean.

| date | pass/fail | notes |
|---|---|---|
| | | |

## Stage 6: props on, in the water, 20 % limit

Boat tied at the dock, lines fore and aft, then open water. Same 20 % limit.

1. Repeat 5.2–5.4 with the props on: lines go taut the right way, the bow swings the right way.
2. Dock drive 30 min: watch pack current and voltage sag (`bus_mv` must stay well above 35 V, the RUN cut-off) and the `bad` counters.
3. Open water in Manual, then Rover tuning (`CRUISE_SPEED`, `CRUISE_THROTTLE`, steering and speed controllers), then Guided over MAVLink, then MOOS through `iArduRoverBridge`. Expect `CRUISE_THROTTLE` near the top of the band: at 20 % the usable range is narrow.
4. Autonomy accepted when the boat holds a line and a speed under Rover/MOOS control without the operator touching the sticks for a full run.

**Gate 6:** the control system is tested and tuned under autonomy at 20 %. Only then stage 7.

| date | pass/fail | notes |
|---|---|---|
| | | |

## Stage 7: raising the limit (after gate 6, not before)

One number in `config.h` (`THRUST_LIMIT_PERMILLE`) plus the matching Pixhawk range, rebuilt
and reflashed each step. Keep ArduPilot's full stick travel on the usable band:
`SERVOn_MAX = 1525 + limit × 0.475`, `SERVOn_MIN = 1475 − limit × 0.475`. Rover's tuning
changes with the band, so expect to retune at each step.

| Limit | SERVO MIN / MAX | Do |
|---|---|---|
| 200 (today) | 1380 / 1620 | stages 3–6 |
| 400 | 1285 / 1715 | repeat 4.3–4.6 (props off), then a dock drive, then retune |
| 700 | 1143 / 1857 | same; watch pack current against the BMS limit and the contactor rating |
| 1000 | 1000 / 2000 | same; the `cmd` byte now reaches 127 |

At each step the end-point hold (4.4) is the test that matters: it is where the deployed
build failed.

| limit | date | pass/fail | notes |
|---|---|---|---|
| | | | |

## Open questions this plan answers

| Question (firmware README / HARDWARE.md) | Answered at |
|---|---|
| Which build is on the VCB today | 0.1 |
| Console port and baud; does console input work | 0.2, 0.3 |
| HSI timebase offset, deadband adequate | 0.7 |
| E-stop / TSMS sense polarity; do the coils really lose power | 1.1, 1.2, 1.4, 1.5 |
| What loses power on E-stop; what actually stops a turning motor | 1.7, 3.8 |
| `bus_mv` calibration | 2.2 |
| Can the precharge path take repeated attempts | 2.10 |
| Physical left/right and forward direction | 3.1–3.3 |
| Converter's reaction to a direction change; is the 200 ms hold needed | 3.10, 4.6, 4.7 |
| Converter full scale 127 or 100 | 4.8 |
| Is "VCB stays in RUN when Rover disarms" acceptable | 4.9 |
