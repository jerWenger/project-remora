# tools/ — discovery scripts

Goal: shrink the list of unknowns about the HighFieldBoat stack, then tune it from a
versioned param file. Everything here is read-only against the vehicle except
`ap_apply.py` (writes params, after a diff + confirm + backup) and `ap_logs.py erase`.

| Script | Runs on | What it answers |
|---|---|---|
| `ap_inventory.py` | laptop (over tailnet) or Jetson | What autopilot board/firmware, which sensors are present and healthy, GPS state, is an RC receiver bound, is a power module wired, which IMUs/baros stream, full param dump + curated table |
| `ap_params.py` | anywhere | Diff two param dumps; grep params by prefix |
| `ap_apply.py` | laptop or Jetson | Apply a desired-state `.parm` (e.g. `params/boat.parm`): reads only the named params, prints current → desired, asks y/N, backs up the old values to `params/backup_<UTC>.parm`, writes + verifies each, lists the ones that need a reboot. Refuses if armed |
| `ap_logs.py` | laptop or Jetson | `list` / `get <id>… \| --latest N \| --all` / `erase --yes-really` dataflash logs over MAVLink into `logs/ap/<UTC>_<id>.BIN`; re-requests dropped chunks, skips logs already downloaded |
| `vcb_bench_check.py` | Jetson | Dead-bus check of Pixhawk → VCB wiring and the µs → ‰ curve via ArduPilot motor test; refuses unless the VCB reports bus < 5 V and contactors open |
| `jetson_inventory.sh` | Jetson | L4T/kernel, Tailscale relay state, USB/serial device IDs for a udev rule, who holds the serial port, service states, BMS file freshness, temps, installed Python/MOOS software |

## Setup (laptop)

```sh
python3 -m venv ~/.venv/mav && . ~/.venv/mav/bin/activate
pip install pymavlink
```

## Run from the laptop (Starlink → Tailscale → mavlink-router TCP 5760)

```sh
mkdir -p params
tools/ap_inventory.py --params params/$(date +%F).parm --json params/$(date +%F).json
```

QGC can stay connected; the router's TCP server accepts several clients. The link
is relayed through DERP, so a full param download can take a minute or two; the
script re-requests dropped indices by itself.

## Run on the Jetson

```sh
scp tools/*.py tools/*.sh boat@highfieldboat.tailcacf9.ts.net:~/tools/
ssh boat@highfieldboat.tailcacf9.ts.net
bash ~/tools/jetson_inventory.sh | tee ~/jetson_$(date +%F).txt
python3 ~/tools/ap_inventory.py --conn tcp:127.0.0.1:5760 --params ~/$(date +%F).parm
```

Then copy the outputs back into `params/` here. Commit a dump after every tuning
session and `ap_params.py diff` them; the param file is the source of truth.

## Tuning session workflow

```sh
tools/ap_inventory.py --params params/$(date +%F).parm     # 1. dump what's on the vehicle
$EDITOR params/boat.parm                                    # 2. change the desired state
tools/ap_apply.py params/boat.parm --dry-run                # 3. review current -> desired
tools/ap_apply.py params/boat.parm                          # 4. y/N, backup, write, verify
                                                            #    reboot if it lists [reboot] params
tools/ap_inventory.py --params params/$(date +%F)_after.parm   # 5. dump again
tools/ap_params.py diff params/$(date +%F).parm params/$(date +%F)_after.parm   # 6. only what you meant?
git add params/ && git commit                               # 7. boat.parm + dumps are the record
```

Undo: `tools/ap_apply.py params/backup_<UTC>.parm` (a backup is a normal `.parm`).
Proposals sit commented out in `boat.parm`; uncomment one group at a time after the
physical check written next to it.

Logs after each run (much faster on the Jetson, `--conn tcp:127.0.0.1:5760`, then scp):

```sh
tools/ap_logs.py list
tools/ap_logs.py get --latest 1          # -> logs/ap/2026-10-04_16-02-29_7.BIN (gitignored)
```

Open the `.BIN` in https://plot.ardupilot.org (drag and drop) or `MAVExplorer.py`.
Logs only exist while armed unless `LOG_DISARMED=1`. The autopilot pauses logging
during a download. Over the DERP-relayed tailnet expect ~10–30 KiB/s, so a few MB
per minute of driving takes a few minutes.

## What to look for in the output

- **Boot banner**: lines like `ArduRover V4.7.0`, the board name (`CubeOrange`,
  `Pixhawk6C`, …), `IMU0: …`, `Baro…`, `GPS 1: detected …`, `RCOut: PWM:1-…`.
  This is the authoritative hardware list.
- **AUTOPILOT_VERSION vendor/product** should match the Pixhawk's `lsusb` line in
  the Jetson inventory — those two IDs go into `systemd/99-pixhawk.rules`.
- **POWER_STATUS flags** without `BRICK_VALID` confirms no power module on the
  Pixhawk, so ArduPilot has no battery voltage of its own (see Battery below).
- **RC_CHANNELS count = 0** = no receiver (expected right now).
- **SYS_STATUS** `BAD` lines and the pre-arm `STATUSTEXT`s are the to-do list.
- **Key params**: `FRAME_CLASS` (2 = boat), `SERVO1/3_FUNCTION` (73/74 = skid
  steer L/R), `SERVOn_MIN/TRIM/MAX`, `MOT_*`, `GPS1_TYPE` (9 = DroneCAN),
  `CAN_P1_DRIVER`/`CAN_D1_PROTOCOL`, `ARMING_CHECK`, `FS_*`, `BATT_MONITOR`.

## Battery (BMS is on the Jetson, not the Pixhawk)

The LiTime pack talks RS485 to the Jetson (`scripts/battery_litime.py`). Nothing
feeds that to ArduPilot today. Three ways forward, cheapest first:

1. **GUI + MOOS only (now).** The BMS reader keeps writing `battery.json`; the web
   GUI reads it and a tiny MOOS app publishes `BATTERY_*`. ArduPilot battery
   failsafe stays off (`BATT_MONITOR=0`, rely on MOOS/GUI + fence/GCS failsafes).
2. **Inject into ArduPilot via Lua (`BATT_MONITOR=29` Scripting).** A Jetson
   script sends the pack voltage/current/SoC as a MAVLink message a Lua script
   on the Pixhawk reads and pushes into the scripting battery backend. Gives
   real `BATT_FS_*` failsafes with no wiring change. ~50 lines of Lua + a
   pymavlink sender.
3. **Hardware power module** on POWER1 — the most robust, but a wiring job.

Start with 1, add 2 once the GUI works. Confirm `SCR_ENABLE` and free flash in
the param dump before committing to option 2.

## Next: remove the biggest unknowns in order

1. Run both inventories; save outputs under `params/` and `docs/`.
2. From the banner: pin down the exact flight controller and GPS, write them into
   a `docs/HARDWARE.md`.
3. From USB IDs: install the udev rule so the Pixhawk is always `/dev/pixhawk`,
   and point `mav.conf` at it.
4. Add a `mavlink-router.service` with a second UDP endpoint
   (`127.0.0.1:14551`) for `iArduRoverBridge` and a third (`14552`) for the GUI.
5. Spec an RC receiver matching the transmitter protocol and `RC_PROTOCOLS`.
