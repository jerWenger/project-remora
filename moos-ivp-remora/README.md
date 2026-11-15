# moos-ivp-remora

The MOOS-IvP side of Project Remora, laid out like
[moos-ivp-extend](https://github.com/pavlab-mit/moos-ivp-extend): `src/` for our own
apps and behaviors, `missions/` for runnable missions, `bin/` and `lib/` for build output.
It lives inside the boat repo so one checkout on the Jetson carries the services, the
ArduPilot tools and the autonomy.

```
moos-ivp-remora/
  CMakeLists.txt         finds moos-ivp and MAVSDK, builds src/
  build.sh, clean.sh     ./build.sh [-jN] [--debug|--release]
  src/
    iArduRoverBridge/    MOOS <-> ArduRover over MAVLink (copied from mCDR-Plume-Tracking)
    iContactUDP/         position fixes over UDP (the phone app in ../ios) -> NODE_REPORT
  missions/
    remora/              example mission: survey box off the pavilion, return, station-keep
    joust/               leg run with COLREGS avoidance against a skiff (simulated, or a phone)
  bin/ lib/ build/       generated, not tracked
  scripts/               developer scripts (on PATH after the ansible run)
```

## Build

Needs a built `moos-ivp` tree and, for `iArduRoverBridge`, the MAVSDK C++ library.

| Where | moos-ivp | MAVSDK | Build |
|---|---|---|---|
| Laptop (macOS) | `~/moos/moos-ivp` (found as `../../moos-ivp`) | `brew install mavsdk` (3.x) | `./build.sh` |
| Jetson | `/home/boat/moos-ivp` | built from source | both done by [`../ansible`](../ansible/README.md) |

`CMakeLists.txt` looks for moos-ivp in `$MOOS_IVP_ROOT`, then `../moos-ivp`, `../../moos-ivp`,
`../../../moos-ivp`, `~/moos-ivp`, `~/moos/moos-ivp`. Without MAVSDK the tree still configures
and builds; it just skips the bridge and prints a warning.

Put the outputs on your path (the playbook does this on the Jetson):

```sh
PATH=$PATH:~/moos/2.733/moos-ivp-remora/bin:~/moos/2.733/moos-ivp-remora/scripts
IVP_BEHAVIOR_DIRS=$IVP_BEHAVIOR_DIRS:~/moos/2.733/moos-ivp-remora/lib
```

## iArduRoverBridge

Copied unchanged from `mCDR-Plume-Tracking/src/iArduRoverBridge` at commit `7e47576`
(2026-07-10). Adapt it here from now on; this copy is the one that goes on the boat.

What it does: subscribes to `DESIRED_HEADING` and `DESIRED_SPEED` from the helm and sends
`SET_POSITION_TARGET_GLOBAL_INT` velocity targets to ArduRover, which closes the heading and
speed loops itself (so no `pMarinePIDV22` on the boat). It publishes `NAV_LAT/LONG/X/Y/HEADING/SPEED`,
GPS, IMU, battery and health variables, plus `ARDUROVER_MODE` and `ARDUROVER_ARMED`. Run
`iArduRoverBridge -i` for the full interface.

Safety behaviour, which the mission relies on:

- It **never arms** or switches to GUIDED by itself. It only sends velocity targets while
  `DEPLOY=true`, the autopilot reports **armed** and **GUIDED**, and the helm's setpoints are
  less than 5 s old. Arming and GUIDED come from QGC or the RC transmitter.
- `ALLSTOP=true` asks the autopilot to disarm. `MOOS_MANUAL_OVERRIDE=true` asks for HOLD.
- At zero desired speed it commands the vehicle's current heading, so station-keeping does not
  pivot the boat.

Connection: `mavlink_protocol`, `mavlink_host`, `mavlink_port` are joined into a MAVSDK URL
(`udpin://0.0.0.0:14551`). On the boat, mavlink-router's `[UdpEndpoint moos]` sends to
127.0.0.1:14551 ([`config/mav.conf.new`](../config/mav.conf.new)). The laptop SITL
([`sim/run_sitl.sh`](../sim/README.md)) sends to 14551 + 10·I for instance I.

## The `remora` mission

One vehicle plus a shoreside. The vehicle drives a 40 m × 40 m box south of the MIT Sailing
Pavilion dock at 1 m/s, returns to its start and station-keeps. `BHV_OpRegionV24` fences it
to the basin. Origin 42.358436, -71.087448; the SITL home (42.3577, -71.0872) is at about
x=20, y=-82.

Three ways to run it, all from `missions/remora/`:

**1. Pure MOOS simulation** (`uSimMarineV22` + `pMarinePIDV22`, no ArduPilot). Time warp is fine.

```sh
./launch.sh --sim 10          # DEPLOY button in pMarineViewer, or:
uPokeDB targs/targ_shoreside.moos DEPLOY_ALL=true
```

**2. ArduRover SITL on the laptop.** The real control chain, minus the hardware. Time warp
must stay 1.

```sh
sim/run_sitl.sh -I 5                      # from the repo root, in one terminal
cd moos-ivp-remora/missions/remora
./launch.sh --sitl_instance=5             # bridge listens on 14601
.venv/bin/python sim/smoke_test.py -I 5   # (optional) proves the SITL link
```

Then arm and select GUIDED, either from QGC (TCP 127.0.0.1:5810) or with
`.venv/bin/python sim/sitl_arm_guided.py -I 5` from the repo root, and press DEPLOY. Until
then the bridge appcast says `Autonomy Status: INACTIVE, Reason: NOT_ARMED NOT_IN_GUIDED`. The bridge
appcast (uMAC, or pMarineViewer's appcast pane) shows `ARDUROVER_MODE`, armed state and the
commands sent.

**3. The boat.** Vehicle community on the Jetson, shoreside on the laptop, both over Tailscale.

```sh
# Jetson (ssh boat@highfieldboat.tailcacf9.ts.net)
cd ~/project-remora/moos-ivp-remora/missions/remora
./launch_vehicle.sh --ip=100.83.34.69 --shore=<laptop tailscale IP>

# Laptop
./launch_shoreside.sh --ip=<laptop tailscale IP>
```

`--ip` is what `pShare`/`pHostInfo` advertise, so it must be the address the other side can
reach. `uFldNodeBroker` registers the vehicle with the shoreside; `uFldShoreBroker` bridges
`DEPLOY_ALL`, `RETURN_ALL`, `STATION_KEEP_ALL`, `ALLSTOP_ALL` and `MOOS_MANUAL_OVERRIDE_ALL`
back down. Arm and GUIDED still come from QGC or the RC. **The lost-link failsafes are off
(see the top-level README); do not deploy on the water until they are set.**

### Patterns, op region and the battery return

`--pattern=box` (default) is the 40 m square at 1 m/s inside the pavlab basin polygon.
`--pattern=fig8` is a smooth figure 8 with no corners: a lemniscate `--fig8_size` m long
(default 120) and half that wide, centred on `--fig8_center` (default `0,-100`, the middle
of the 2026-11-15 tuning area), long axis `--fig8_rot` degrees clockwise from north (default
90, east-west), `--laps` times (default 2) at 1.5 m/s. Its tightest radius is about a third
of the half-length (20 m at the default size), which this boat turns without reversing a
motor. The op region for fig8 is the pattern's bounding box plus `--op_margin` (default 30 m).
The op region only arms once the boat has been inside it, so deploying at the dock is fine.

The boat returns and station-keeps on its own when the bridge posts `BATTERY_OK=false`:
the pack at or under `--return_volts` (default 50.0) or `--return_pct` (default 20) for 10 s,
or the BMS data gone stale for 10 s. The return latches, so a voltage that recovers once the
load comes off does not send the boat back out. Where there is no BMS file (SITL, the laptop)
`BATTERY_OK` is never posted and nothing triggers; to test it in simulation:
`uPokeDB targs/targ_remora.moos BATTERY_OK=false`.

```sh
./launch.sh --sim --pattern=fig8 10                       # simulation
./launch_vehicle.sh --pattern=fig8 --return_volts=51.0 --ip=100.83.34.69 --shore=<laptop>   # boat
```

Launcher options worth knowing (`--help` lists all):

| Option | Script | Meaning |
|---|---|---|
| `--sim` | both | uSimMarineV22 instead of iArduRoverBridge |
| `--sitl_instance=N` / `--mav_port=P` | both | bridge listens on 14551+10N, or on P |
| `--debug` | both | bridge writes `DBG_iArduRoverBridge_*.log` |
| `--speed=`, `--max_spd=` | both | survey speed, helm speed domain (default 1.0, 2.0 m/s: the VCB caps thrust at 20 %) |
| `--start_pos=`, `--return_pos=` | both | sim start, return point |
| `--nogui` | launch.sh, shoreside | no pMarineViewer |
| `--just_make` | all | only write `targs/` |
| `--ip=`, `--shore=`, `--mport=`, `--pshare=` | vehicle/shoreside | network |

Ports: shoreside MOOSDB 9000, pShare 9200; vehicle MOOSDB 9001, pShare 9201. Logs go to
`missions/remora/logs/`; `./clean.sh` removes them and `targs/`.

## Known issues with moos-ivp main (two workarounds in this mission)

Both found while testing against moos-ivp `68d862fbd` (main, 2026-09-18). Remove the
workarounds once upstream changes, and pin `moos_ivp_version` in
`ansible/group_vars/all.yml` to a commit you have tested.

**1. `park_on_allstop` now defaults to true** (pHelmIvP, Aug 2026). The helm hands control
back on any all-stop and waits for `MOOS_MANUAL_OVERRIDE=false` again. When the survey
completes there is one helm iteration with no active behavior before `MODE` flips to
RETURNING, and that single `NothingToDo` parked the helm, so the boat never returned.
`plugs/shared/plug_pHelmIvP.moos` sets `park_on_allstop = false`.

**2. Last `BHV_Waypoint` loses its points.**

When NAV_X/NAV_Y are already in the helm's info buffer at startup (uSimMarineV22 or the bridge
publishing), the **last `BHV_Waypoint` block in the .bhv file** ends up with an empty
waypoint list: the helm posts `BHV_WARNING <name>: No waypts given` on its first iteration
and, when that behavior later becomes active, `IVPHELM_ALLSTOP NothingToDo`. Which block is
hit depends only on file order; a block with `points = start` or `points = empty` is skipped.
Without a simulator (no NAV in the buffer) it does not happen. The stock
`ivp/missions/s1_alpha/alpha.bhv` shows it too (its `waypt_return` is last).

Minimal repro: MOOSDB + pHelmIvP + uSimMarineV22 + pLogger, `alpha.bhv`, poke
`MOOS_MANUAL_OVERRIDE=false`, then `aloggrep <alog> BHV_WARNING`.

Workaround used here: `plugs/bhvs/plug_waypt_placeholder.bhv`, a never-active
`BHV_Waypoint` included last in `meta_vehicle.bhv`.

## Adding apps and behaviors

```sh
cd src
~/moos/moos-ivp/scripts/GenMOOSApp_AppCasting pFoo p     # new appcasting app
~/moos/moos-ivp/scripts/GenBehavior Foo                  # new behavior -> lib_behaviors-foo
```

Add the directory to `src/CMakeLists.txt`, `./build.sh`, and reference it from a mission.
Behaviors land in `lib/`, which must be on `IVP_BEHAVIOR_DIRS`.
