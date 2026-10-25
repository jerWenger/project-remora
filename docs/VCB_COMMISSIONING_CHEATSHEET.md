# VCB commissioning cheat sheet: every command, in test order

Companion to `VCB_COMMISSIONING.md` (the plan) and `VCB_COMMISSIONING_CHECKLIST.pdf` (the
printable). Step numbers match the plan. **M** = on the Mac in `~/moos/2.733`; **J** = on the
Jetson, in the repo checkout there (`~/project-remora`; adjust if it lives elsewhere).
Props off through stage 5. Nothing here raises the 20 % limit before stage 7.

## Before every session (J)

```sh
ssh boat@highfieldboat.tailcacf9.ts.net            # or: mosh ... -- tmux new -A -s boat
cd ~/project-remora
deploy/update.sh                                   # fast-forward to origin/main, re-install, restart services
systemctl status vcb-logger --no-pager | head -3   # must be active; it writes ~/vcb.json
python3 -c 'import serial, pymavlink'              # bench tools need both (pip3 install --user pyserial pymavlink)
```

Open three windows and keep them open all day:

```sh
# 1. raw VCB console: every line the VCB prints (ARMED, FAULT n, THRUSTER ..., BENCH ...)
tail -f "$(ls -t ~/vcblogs/*.csv | head -1)"

# 2. parsed fields: status (thruster_state, armed, lsc/hsc_closed, bus_mv, fault_code, cmd_source)
#    and pwm (chN_us, chN_valid, chN_raw_permille, chN_applied_permille, chN_bad, chN_cmd)
watch -n0.5 cat ~/vcb.json

# 3. a shell for sending console commands (the logger keeps reading; we only write)
stty -F /dev/vcb 115200 raw -echo clocal
vcb() { printf '%s\n' "$*" > /dev/vcb; }
vcb help
```

Dashboard: `http://highfieldboat.tailcacf9.ts.net:8080/` (thruster card = Pixhawk outputs + VCB).

Reset the VCB without touching power (needed after every latched fault 8, or `vcb clear` for fault 9):

```sh
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "init; reset run; exit"
# or: STM32_Programmer_CLI -c port=SWD -rst
```

## Stage 0: firmware and dead-bus checks

```sh
# 0.1  dump what is on the board today (J) without resetting it, compare with the August dump
#      (mode=hotplug / halt+resume: the VCB keeps running; the August dump is in the checkout)
tail -3 "$(ls -t ~/vcblogs/*.csv | head -1)"      # first: what the running build says it is (old build: mapping,...clamp_above_us=1200)
sudo openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "init; halt; dump_image $HOME/VCB_flash_$(date +%Y%m%d_%H%M%S).bin 0x08000000 0x100000; resume; exit"
sudo chown boat:boat ~/VCB_flash_*.bin
#   or: STM32_Programmer_CLI -c port=SWD mode=hotplug -u 0x08000000 0x100000 ~/VCB_flash_$(date +%Y%m%d_%H%M%S).bin
cmp -l ~/VCB_flash_<new>.bin firmware/VCB_flash_20260805_211528_pre_tolerant_pwm.bin | wc -l   # 0 = same build as August
cp ~/VCB_flash_<new>.bin firmware/ && git add firmware/VCB_flash_<new>.bin                   # keep it; commit with the results

# 0.2  flash the committed binary (J); it is in the checkout, no copying needed
sha256sum firmware/VCB-pixhawk-0.3.2.bin           # 9bb6c56c40c25aeb7549df6619035518c5c62ffedaf4c457380190989496883d
sudo openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program firmware/VCB-pixhawk-0.3.2.bin 0x08000000 verify reset exit"
#   or: STM32_Programmer_CLI -c port=SWD -w firmware/VCB-pixhawk-0.3.2.bin 0x08000000 -v -rst
#   (sudo until the STLINK udev rule below is in; the VCP /dev/vcb already has one)
#   window 1 must show:  VCB pixhawk-pwm reversible 0.3.2; ...
#   once only, so openocd works as boat from then on:
#   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3754", MODE="0666"' | sudo tee /etc/udev/rules.d/60-stlink.rules
#   sudo udevadm control --reload && sudo udevadm trigger
#   to go back:          STM32_Programmer_CLI -c port=SWD -w ~/VCB_flash_<new>.bin 0x08000000 -v -rst
#   to rebuild after a config change (M): cd ~/moos/2.733/firmware/VCB-pixhawk && make TOOLCHAIN=$TC/bin/
#     (TC = Arm GNU Toolchain 14.2 darwin-arm64 dir), cp build/VCB-pixhawk.bin ../VCB-pixhawk-<ver>.bin, commit, push, update.sh
#     or on the Jetson: sudo apt install gcc-arm-none-eabi && make   (then flash build/VCB-pixhawk.bin)

# 0.3  console input works
vcb help                      # window 1: CONSOLE: bench on | bench off | bench R L ... | stop | clear | help

# 0.4  Pixhawk params for this build (only AFTER 0.2)
python3 tools/ap_apply.py params/vcb-pixhawk.parm --dry-run
python3 tools/ap_apply.py params/vcb-pixhawk.parm

# 0.5  safety switch off, otherwise there is no PWM at all (reboot the Pixhawk after)
printf 'BRD_SAFETY_DEFLT,0\n' > /tmp/safety.parm && python3 tools/ap_apply.py /tmp/safety.parm

# 0.6  dead-bus contract (pack disconnected; the VCB will sit in FAULT 3/6 latched, that is fine)
python3 tools/vcb_bench_check.py --pwm 1500,1550,1600,1620,1700,1450,1400,1380,1300,1000,2000
#   expect raw 0,52,157,200,368,-52,-157,-200,-368,-1000,1000; applied clamps at +-200; LEFT->ch2, RIGHT->ch1
python3 tools/vcb_bench_check.py --pwm 990,1000,1010,1990,2000,2010
#   expect raw -1000,-1000,-978,978,1000,1000; all valid=1; bad not counting
python3 tools/vcb_bench_check.py --pwm 2250,2300,2350,650 --hold 3      # only if ArduPilot accepts them
#   expect 2250/2300 valid raw 1000; 2350/650 bad counts, valid 0 after 100 ms

# 0.7  timebase: in the 1500 step above, ch1_us/ch2_us must read 1485..1515
```

## Stage 1: safety, contactors open (Pixhawk unplugged from CONT)

```sh
# 1.1-1.3  physical: E-stop x3, TSMS x3, battery disconnect. Watch window 2:
#          status.estop_24v_present / tsms_24v_present / bus_mv ; meter on the coil feed
# 1.4-1.5  E-stop pressed (then TSMS pulled), plug the Pixhawk in at neutral:
#          window 1: ARMED -> THRUSTER STARTUP_LSC ... -> FAULT 3 ... x3 -> FAULT latched. Then unplug, reset:
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "init; reset run; exit"
# 1.6  software stop / clear
vcb stop                      # FAULT 9 ..., FAULT latched: stop command
vcb clear                     # CLEAR: stop un-latched ...
# 1.7  by inspection: what loses power on E-stop (converter? motors? coils only?) -> checklist
```

## Stage 2: VCB arm, start-up, disarm, zero throttle (pack on)

Power-up order: Pixhawk (disarmed, outputs 1500) → TSMS in, E-stop released → VCB power.

```sh
# 2.1  hands off; window 1: ARMED ... STARTUP_LSC ... STARTUP_WAIT ... STARTUP_HSC ... RUN ; window 2: state 5, cmd 0/0
# 2.2  meter at the contactor output vs status.bus_mv (within 1 V)
# 2.3  10 min in RUN, fault_code stays 0
# 2.4  E-stop in RUN -> FAULT 8, lsc/hsc_closed 0, "FAULT latched". Release: must stay open. Reset:
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "init; reset run; exit"
# 2.5  same with TSMS
# 2.6  pull CONT in RUN -> FAULT 1 within 100 ms ; plug back -> "FAULT cleared" (3 s) -> ARMED (1 s) -> RUN
# 2.7  reboot the Pixhawk in RUN (QGC: Reboot Vehicle) -> FAULT 1 -> recovery
# 2.8  arm the Pixhawk in Manual and nudge throttle during the 1.5 s start-up -> "DISARMED: throttle left neutral before RUN"
# 2.9  software stop with the bus live, then recovery
vcb stop                      # FAULT 9, contactors open
vcb clear                     # then 3 s neutral -> FAULT cleared -> ARMED -> RUN
# 2.10 pack disconnected, VCB reset -> FAULT 6 x3 -> FAULT latched; precharge resistor cool
# 2.11 bench-mode arm/disarm with nothing turning (Pixhawk unplugged from CONT)
python3 tools/vcb_bench_drive.py --right 0 --left 0 --seconds 5
#      window 1: BENCH: on ... ARMED ... RUN ... DISARMED: bench off ... back to IDLE ... BENCH: off
python3 tools/vcb_bench_drive.py --right 0 --left 0 --seconds 30     # then Ctrl-C mid-way: same clean exit
# 2.12 three power cycles, RUN each time
```

## Stage 3: Jetson drives the motors at 5 %, props off

Pixhawk unplugged from CONT or left in: bench mode ignores the PWM inputs. 50 permille = 5 % = byte ±6.

```sh
# 3.1  right forward: which motor, which way?
python3 tools/vcb_bench_drive.py --right 50 --left 0 --seconds 5
# 3.2  left forward
python3 tools/vcb_bench_drive.py --right 0 --left 50 --seconds 5
# 3.3  reverse, each
python3 tools/vcb_bench_drive.py --right -50 --left 0 --seconds 5
python3 tools/vcb_bench_drive.py --right 0 --left -50 --seconds 5
# 3.4  both
python3 tools/vcb_bench_drive.py --right 50 --left 50 --seconds 10
# 3.5  30 s steady
python3 tools/vcb_bench_drive.py --right 50 --left 0 --seconds 30
# 3.6  the caps: 10 % runs; 30 % is refused by the tool and clamped by the firmware
python3 tools/vcb_bench_drive.py --right 100 --left 100 --seconds 5           # cmd 13/13
python3 tools/vcb_bench_drive.py --right 300 --left 0 --seconds 5             # "refusing: |permille| above --max 100"
vcb bench on; vcb bench 300 0; vcb bench off                                  # window 1: BENCH: clamped to 100 0 (limit +-100)
# 3.7  dead Jetson process: start a 30 s run, then from another shell
python3 tools/vcb_bench_drive.py --right 50 --left 0 --seconds 30
kill -9 $(pgrep -f vcb_bench_drive.py)       # <= 500 ms later: FAULT 1 "bench command lost", motor stops
vcb bench off                                # tidy up; the VCB recovers to IDLE on its own after 3 s of neutral
# 3.8  E-stop during a run -> FAULT 8, motor stops. Note what actually stopped it. Reset the VCB afterwards.
# 3.9  software stop during a run (second shell)
vcb stop                                     # FAULT 9, motor stops
vcb clear
# 3.10 direct reversal through the 200 ms zero hold (cmd 6 -> 0 -> -6), each side
python3 tools/vcb_bench_drive.py --right 50 --left 0 --seconds 3 --flip
python3 tools/vcb_bench_drive.py --right 0 --left 50 --seconds 3 --flip
```

If 3.1 shows the wrong motor or the wrong direction: it is fixed in `firmware/VCB-pixhawk/Core/Src/eprop.cpp`
(addresses or the direction byte), rebuilt and re-flashed with the stage 0 commands, then 3.1–3.3 again.

## Stage 4: Pixhawk motor test, props off (Pixhawk on CONT, `vcb bench off`)

`--live` requires the VCB in RUN with no fault and asks for a typed confirmation; it prints
measured µs, raw/applied permille and `cmd` for each step.

```sh
# 4.1  right forward (AUX1 -> ch1 -> right motor?)
python3 tools/vcb_bench_check.py --live --side right --pwm 1550 --hold 2        # cmd 7
# 4.2  left forward, then each in reverse
python3 tools/vcb_bench_check.py --live --side left  --pwm 1550 --hold 2
python3 tools/vcb_bench_check.py --live --side right --pwm 1450 --hold 2        # cmd -7
python3 tools/vcb_bench_check.py --live --side left  --pwm 1450 --hold 2
# 4.3  the limit, 10 s
python3 tools/vcb_bench_check.py --live --side right --pwm 1620 --hold 10       # cmd 25
# 4.4  END POINTS, 30 s each, both sides: raw +-1000, applied +-200, cmd +-25, bad not counting, motor keeps running
python3 tools/vcb_bench_check.py --live --side right --pwm 2000 --hold 30
python3 tools/vcb_bench_check.py --live --side right --pwm 1000 --hold 30
python3 tools/vcb_bench_check.py --live --side left  --pwm 2000 --hold 30
python3 tools/vcb_bench_check.py --live --side left  --pwm 1000 --hold 30
# 4.5  acceptance window (servo tester on CONT if ArduPilot refuses): 2200,2300,800,700 valid; 2350/650 -> bad, FAULT 1
python3 tools/vcb_bench_check.py --live --side right --pwm 2200,2300,800,700 --hold 3
# 4.6  direct reversal at 20 %: cmd 25 -> 0 (200 ms) -> -25, no dropout, both sides
python3 tools/vcb_bench_check.py --live --side right --pwm 1620,1380 --hold 3
python3 tools/vcb_bench_check.py --live --side left  --pwm 1620,1380 --hold 3
# 4.7  (optional) is the hold needed? build without it, re-flash (stage 0.2), repeat 4.6, then put it back
sed -i '' 's/^#define DIRECTION_CHANGE_STOP_MS .*/#define DIRECTION_CHANGE_STOP_MS     0/' Core/Inc/config.h      # (M)
sed -i '' 's/^#define DIRECTION_CHANGE_STOP_MS .*/#define DIRECTION_CHANGE_STOP_MS     200/' Core/Inc/config.h    # (M) revert
#      (watch window 2: chN_glitch counts the discarded glitch pairs; chN_bad must stay 0 on a clean signal)
# 4.8  scale: at 1620 us the converter sees 25. If the motor behaves like 25 %, full scale is 100:
sed -i '' 's/^#define EPROP_FULL_SCALE_VALUE .*/#define EPROP_FULL_SCALE_VALUE       100/' Core/Inc/config.h      # (M), rebuild, re-flash
# 4.9  QGC: disarm while the VCB is in RUN -> outputs 1500, VCB stays RUN with cmd 0. Decide and write it down.
```

## Stage 5: RC, props off (receiver installed)

```sh
python3 tools/ap_inventory.py | grep -i -E "rc|receiver"            # receiver bound?
#   QGC: Radio calibration; Flight modes: Manual on the mode switch; arm switch if used
printf 'MOT_SLEWRATE,100\n' > /tmp/slew.parm && python3 tools/ap_apply.py /tmp/slew.parm
# 5.1-5.9  sticks; watch the dashboard thruster card (SERVO_OUTPUT_RAW 1380..1620) and window 2 (cmd)
#   5.7: transmitter off while armed -> FS action -> outputs 1500 -> cmd 0
python3 tools/ap_logs.py --latest 1                                  # keep the log of the session
```

## Stage 6: props on, water, 20 % (nothing new to type; Rover tuning in QGC, logs with ap_logs.py)

## Stage 7: raising the limit (after gate 6 only)

One config line, rebuild, re-flash (stage 0.2), then the matching Pixhawk range:

```sh
# 400 permille
sed -i '' 's/^#define THRUST_LIMIT_PERMILLE .*/#define THRUST_LIMIT_PERMILLE        400/' Core/Inc/config.h   # (M)
printf 'SERVO9_MIN,1285\nSERVO9_MAX,1715\nSERVO10_MIN,1285\nSERVO10_MAX,1715\n' > /tmp/limit.parm && python3 tools/ap_apply.py /tmp/limit.parm   # (J)
# 700 permille
sed -i '' 's/^#define THRUST_LIMIT_PERMILLE .*/#define THRUST_LIMIT_PERMILLE        700/' Core/Inc/config.h
printf 'SERVO9_MIN,1143\nSERVO9_MAX,1857\nSERVO10_MIN,1143\nSERVO10_MAX,1857\n' > /tmp/limit.parm && python3 tools/ap_apply.py /tmp/limit.parm
# 1000 permille
sed -i '' 's/^#define THRUST_LIMIT_PERMILLE .*/#define THRUST_LIMIT_PERMILLE        1000/' Core/Inc/config.h
printf 'SERVO9_MIN,1000\nSERVO9_MAX,2000\nSERVO10_MIN,1000\nSERVO10_MAX,2000\n' > /tmp/limit.parm && python3 tools/ap_apply.py /tmp/limit.parm
# at each step: 4.3, 4.4, 4.6 again (props off), then a dock drive, then retune
```

## Fault codes (window 1 prints `FAULT n: reason`)

| n | meaning | clears by |
|---|---|---|
| 1 | command lost (PWM invalid, or no `bench R L` for 500 ms) | valid neutral command for 3 s |
| 2–7 | contactor / bus interlock during start-up or RUN | 3 s neutral, max 3 retries per boot, then reset |
| 8 | contactor opened in RUN (E-stop / TSMS) | reset only |
| 9 | `stop` on the console | `vcb clear`, then 3 s neutral |
