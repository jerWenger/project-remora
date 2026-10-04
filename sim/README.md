# sim/ — laptop SITL with the boat's endpoint layout

One command runs an ArduRover boat in SITL (`motorboat-skid` physics) that
exposes the **same MAVLink endpoints as the Jetson's mavlink-router**
(`config/mav.conf.new`). The dashboard, iArduRoverBridge, QGC and
`tools/ap_inventory.py` can then be pointed at the laptop instead of the boat.

```
sim/run_sitl.sh                 # instance 0: 5760 / 14551 / 14552
sim/run_sitl.sh -I 5 --wipe     # instance 5: 5810 / 14601 / 14602, fresh eeprom
.venv/bin/python sim/smoke_test.py -I 5
```

Ctrl-C stops SITL and the fake-battery writer.

## Files

| File | What |
|---|---|
| `run_sitl.sh` | starts `ardurover` directly (no MAVProxy, no mavlink-routerd needed) |
| `boat-sitl.parm` | param overlay: real-boat behaviour params on top of the SITL defaults |
| `smoke_test.py` | end-to-end check; refuses to command anything that isn't SITL |
| `run/` (gitignored) | `I<n>/eeprom.bin`, `I<n>/logs/` (dataflash), `state/` (fake dashboard JSON), `ap-build/` (native SITL build) |

## Options

| Option | Meaning |
|---|---|
| `-I N` | instance; **every** port shifts by `10*N`, including the two UDP ports, so several sims can run side by side |
| `--speedup X` | sim speedup (default 1) |
| `--wipe` | fresh eeprom. Needed after editing `boat-sitl.parm`: params saved in `run/I<n>/eeprom.bin` beat the defaults files |
| `--home lat,lon,alt,hdg` | start point. Default `42.3577,-71.0872,0,0`, open water ~80 m south of the MIT Sailing Pavilion dock |
| `--build` | build a native ArduRover SITL into `run/ap-build` (see below) |
| `ARDUPILOT_DIR=` | ArduPilot checkout (default `~/dev/ardupilot`) |
| `ARDUROVER=` | explicit SITL binary |

### Binary / Apple Silicon

`~/dev/ardupilot/build/sitl/bin/ardurover` is an **x86_64** binary and this Mac
has no Rosetta, so it won't run. `run_sitl.sh --build` builds a native arm64 one
into `sim/run/ap-build` and the script prefers it. Two macOS quirks are handled:

- Xcode 26's linker rejects the packed `AP_FWVersion` struct ("pointer not aligned");
  the build passes `LDFLAGS=-Wl,-no_fixup_chains`.
- `WAFLOCK=.lock-waf_darwin_hfb_sim` keeps waf from overwriting the checkout's own
  `.lock-waf_darwin_build` (otherwise a plain `./waf` in the checkout would build
  into `sim/run/ap-build`).

Param layering is the same as `sim_vehicle.py -f motorboat-skid`
(`Tools/autotest/pysim/vehicleinfo.py`): `rover.parm`, `motorboat.parm`,
`rover-skid.parm`, then `sim/boat-sitl.parm`.

## Endpoints (identical to the boat, +10·I in sim)

| Port | Boat (mavlink-router on Jetson) | Sim (`run_sitl.sh`) | Consumer | Connection string |
|---|---|---|---|---|
| TCP 5760 | `TcpServerPort` | SERIAL0 `tcp:0` (server; does not wait for a client) | QGC, `ap_inventory.py` | `tcp:127.0.0.1:5760` |
| UDP 14551 | `[UdpEndpoint moos]` → 127.0.0.1 | SERIAL1 `udpclient:127.0.0.1:14551` | iArduRoverBridge | `udpin 0.0.0.0:14551` |
| UDP 14552 | `[UdpEndpoint gui]` → 127.0.0.1 | SERIAL2 `udpclient:127.0.0.1:14552` | dashboard `gcs.py`, `smoke_test.py` | `udpin:0.0.0.0:14552` |
| UDP 5501 | – | SITL RC input | RC emulation | 8–16 little-endian uint16 PWM values per packet |

All are MAVLink2 (`SERIAL0..2_PROTOCOL 2`). In both cases the vehicle sends
first and the UDP consumer listens (`udpin`), then replies to the sender.

One difference: on the boat all three consumers share **one** Pixhawk link through
the router, so a stream rate requested by one of them applies to all. In SITL each
port is its own MAVLink channel with its own rates. `boat-sitl.parm` therefore sets
`MAV2_*` (14551) and `MAV3_*` (14552) to 2–5 Hz so the UDP consumers get data
without asking. iArduRoverBridge asks for its own rates anyway (MAVSDK `set_rate_*`).

## Pointing things at the sim

**QGC:** Application Settings → Comm Links → TCP, host `127.0.0.1`, port `5760`
(+10·I). Disable QGC's UDP 14550 autoconnect if you like; nothing uses 14550 here.

**Dashboard:** it listens on `udpin:0.0.0.0:14552`, exactly as on the Jetson. Run it
locally with the fake battery/VCB state:

```
GCS_STATE_DIR=sim/run/state .venv/bin/python scripts/gcs.py --port 8080
```

`run_sitl.sh` rewrites `sim/run/state/battery.json` every 2 s (same fields as
`scripts/battery_litime.py`, plus `"sim": true`). No `vcb.json` is generated; drop one
into `sim/run/state/` by hand if the dashboard needs it. Only one process can bind
14552: stop the dashboard before running `smoke_test.py`, or vice versa. For
instance N>0 the dashboard needs to listen on 14552+10·N (if it has a port option).

**ap_inventory.py:** keep sim param dumps out of `params/`:

```
.venv/bin/python tools/ap_inventory.py --conn tcp:127.0.0.1:5760 \
    --params sim/run/I0/inventory.parm --json sim/run/I0/inventory.json
```

**iArduRoverBridge** (`~/moos/mCDR-Plume-Tracking/src/iArduRoverBridge`): it builds a
MAVSDK URL from three config params, `mavlink_protocol + "://" + mavlink_host + ":" +
mavlink_port` (a single `mavlink_connection_url` can't be written in a .moos file
because `//` starts a comment). Every mission plug in that repo
(`missions/*/plugs/ardurover/plug_iArduRoverBridge.moos`) currently has

```
mavlink_protocol = udpin
mavlink_host     = 0.0.0.0
mavlink_port     = 14550
```

i.e. `udpin://0.0.0.0:14550`, the old SITL/MAVProxy default. **That doesn't match the
boat (14551) or this sim.** To use the sim, and the boat, set `mavlink_port = 14551`
(14551+10·I for instance I). Nothing else changes: the protocol (`udpin`) and host are
already right. The bridge sends `SET_POSITION_TARGET_GLOBAL_INT` velocity commands
and only acts in GUIDED, so arm and switch to GUIDED from QGC (or let its RC/DEPLOY
logic do it).

## smoke_test.py

```
.venv/bin/python sim/smoke_test.py -I 5
```

It checks: heartbeat on the UDP port, `MAV_TYPE_SURFACE_BOAT`, **SITL guard**,
unrequested streams, GPS 3D fix, EKF absolute horizontal position, a TCP connection
open at the same time, GUIDED, force-arm, a position target 50 m west (upriver), boat
closes to within 30 m of it, HOLD, disarm, TCP still alive. Each check prints PASS/FAIL;
the exit status is 0 only if all pass.

**Guard:** before sending any command it reads `SIM_SPEEDUP`, a parameter only SITL
builds have. With no answer it prints FAIL and exits 2 without sending anything, so
running it on the Jetson (where 14552 is the real boat) can't move the boat. Keep it.

## How the sim differs from the boat

| | Boat | Sim |
|---|---|---|
| Thruster outputs | `SERVO9` ThrottleLeft, `SERVO10` ThrottleRight (AUX1/2), 1000/1000/2000 | `SERVO1` ThrottleLeft, `SERVO3` ThrottleRight, 1500/1500/2000 |
| Why | VCB is wired to AUX1/2 | SITL physics (`libraries/SITL/SIM_Sailboat.cpp`, `MOTORLEFT_SERVO_CH 0` / `MOTORRIGHT_SERVO_CH 2`) reads outputs 1 and 3, with 1000 = full reverse, 1500 = stop, 2000 = full forward |
| Forward-only | VCB bench firmware; `MIN = TRIM = 1000` | `MIN = TRIM = 1500`. Same trick: negative thrust maps onto an empty band, so the motors never reverse. The smoke test confirms armed outputs never go below 1500 |
| 20 % VCB cap (clamp above 1200 µs) | yes | **not modelled** (see below) |
| Hull / thrust | RHIB + 2× Navy 6.0, unknown curve | generic SITL boat: thrust ∝ throttle, drag ∝ v², ~7 m/s at 50 % throttle. `CRUISE_*` are the boat's (unlearned defaults), so the speed PID corrects a large error |
| Compass | not yet in use (`COMPASS_USE=0` on the dump) | simulated compass, EKF has yaw |
| Battery | none visible to ArduPilot (`BATT_MONITOR=21`, no module) | SITL analog battery (`BATT_MONITOR 4`) |
| RC | no receiver | SITL simulated receiver, all channels 1500 (healthy) |
| Environment | real | `motorboat.parm` adds wind 3 m/s and waves |
| Firmware | ArduRover 4.7.0 (1511f271) | 4.7.0-dev master (09c81414). Some names differ: the build uses `ARMING_CHECK`, so `ARMING_SKIPCHK` from the dump is skipped |
| Heartbeat compid | – | SITL heartbeats show compid 0 to pymavlink; commands addressed to compid 1 work |

Copied from the real dump (`params/2026-10-04.parm`): `CRUISE_*`, `ATC_*`, `PSC_*`,
`WP_*`, `TURN_RADIUS`, `LOIT_*`, `MOT_*` behaviour, `FS_*`, `FENCE_*`, `ARMING_*`,
`MODE_CH`/`MODEn`/`INITIAL_MODE`, RC ranges and options, `LOG_*`. Not copied:
anything hardware-specific (`INS_*_ID`, `COMPASS_*`, `CAN_*`, `GPS1_TYPE`, `BRD_*`,
`SERIAL*`, `BATT_MONITOR`, baro IDs, calibration offsets, `EK3_SRC*`). The failsafes
are copied **as they are on the boat**, i.e. off. Flip them in `boat-sitl.parm` (then
`--wipe`) to rehearse the planned config.

### Emulating the VCB 20 % cap

Options, none exact (20 % of the SITL motor isn't 20 % of an ePropulsion Navy 6.0 anyway):

1. **`SERVO1_MAX = SERVO3_MAX = 1600`** (20 % of the 1500–2000 forward band). ArduPilot
   knows about the limit and scales its whole output range into it. This is the sim
   twin of the *planned* boat config (`SERVO9/10_MAX=1200`, HARDWARE.md). The best
   choice for tuning.
2. **`MOT_THR_MAX`**: ArduPilot **clamps it to 30..100** (`AP_MotorsUGV.cpp`), so
   `MOT_THR_MAX=20` really means 30. It also caps only the throttle demand; steering
   mix can still push one motor above it. Rough at best.
3. **Unaware clamp** (the *current* boat: ArduPilot thinks it has 100 %, the VCB clips
   at 1200 µs): no param does this. A small Lua script (`SCR_ENABLE 1`, script in
   `sim/run/I<n>/scripts/`) could read `SRV_Channels:get_output_pwm(73/74)` and
   re-emit `min(pwm, 1600)` with `SRV_Channels:set_output_pwm_chan_timeout`, or patch
   `SIM_Sailboat.cpp` to clip the normalised input at 0.2. Use this to see the integrator
   windup and sluggish turns the real config will have.

### RC without a radio

- `RC_CHANNELS_OVERRIDE` from sysid 255 (`MAV_GCS_SYSID`), e.g.
  `m.mav.rc_channels_override_send(1, 1, 1500, 0, 1700, 0, 0, 0, 0, 0)`. Resend faster
  than `RC_OVERRIDE_TIME` (3 s).
- UDP packets to `127.0.0.1:5501+10·I`: 8 or 16 little-endian uint16 PWM values.
- `SIM_RC_FAIL 1` (no pulses) or `2` (neutral, throttle 950) to rehearse radio loss.
