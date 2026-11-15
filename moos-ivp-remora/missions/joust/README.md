# joust

The boat runs one leg back and forth (`BHV_LegRun`) while another vessel crosses it, and keeps
clear by the COLREGS (`BHV_AvdColregsV22`, one behavior spawned per contact). Modelled on the
pavlab `ufld_joust` mission, cut down to one leg per vehicle and no joust manager. The other
vessel is either a simulated skiff or a phone on the real skiff.

App config plugs are shared with [`../remora`](../remora) (the launchers call
`nsplug --path=../remora`); only the contact manager, the avoidance behavior and the phone
receiver live here.

| File | What |
|---|---|
| `meta_vehicle.moos`, `meta_vehicle.bhv` | vehicle community: the remora set plus `pContactMgrV20`; leg run, COLREGS avoid, return, station-keep, op region, battery return |
| `meta_shoreside.moos` | the remora shoreside plus `iContactUDP` (phone fixes in, `NODE_REPORT` out) |
| `plugs/` | `pContactMgrV20`, `iContactUDP`, the avoidance behavior |
| `launch.sh` | one machine: boat + simulated skiff + shoreside |
| `launch_vehicle.sh`, `launch_shoreside.sh` | one community each; the boat's launcher runs on the Jetson |

## Run

**Simulation** (boat and skiff both simulated; the skiff does not avoid):

```sh
./launch.sh --sim 10        # then DEPLOY in pMarineViewer
```

**The boat against a phone on the skiff.** Shoreside on the laptop, vehicle on the Jetson, the
phone on the tailnet running [`ios/RemoraContact`](../../../ios/RemoraContact) and sending to the
laptop's Tailscale address, UDP port 9300:

```sh
# Laptop
./launch_shoreside.sh --ip=<laptop tailscale IP>
# Jetson
./launch_vehicle.sh --ip=100.83.34.69 --shore=<laptop tailscale IP>
```

The skiff appears in pMarineViewer as soon as the phone sends; the `iContactUDP` appcast shows
its fix rate, accuracy and latency. Arm, DEPLOY, then GUIDED, as in the remora mission.

**The phone path without a phone**: `./launch.sh --sim --no_skiff`, then send fixes to
127.0.0.1:9300, one line per datagram:

```
NAME=skiff,LAT=42.3575364,LON=-71.0874480,SPD=1.20,HDG=180.0,TIME=1794772478.31,ACC=4.0
```

## Options

`launch_vehicle.sh --help` lists all. The ones for this mission:

| Option | Default | Meaning |
|---|---|---|
| `--leg=x1,y1:x2,y2` | `50,-100:-50,-100` | the leg's end points, metres from the datum |
| `--turn_rad=` | 18 | turn radius at each end, m |
| `--speed=` | 1.5 | m/s |
| `--avoid=` | true | false = hold the leg whatever the contact does |
| `--op_margin=` | 30 | op region = the leg, plus two turn radii, plus this, m |
| `--return_volts=`, `--return_pct=` | 50.0, 20 | return and station-keep below either (see ../remora) |

Avoidance distances are in `plugs/plug_avd_colregs.bhv`, sized for this boat: it starts
weighing a contact at 80 m and aims for 15 to 30 m at the closest point.

## Checked so far (simulation only, 2026-11-15)

- Simulated skiff crossing the leg at right angles, not avoiding: 10 encounters, closest
  approach 36 m, no behavior errors, boat stayed inside the op region.
- Fixes sent over UDP in the app's format instead of the simulated skiff: the boat treated it
  as a contact the same way, closest approach 29 m.

Not yet done: a real phone, the real boat, more than one contact, head-on and overtaking
geometry. The simulator is a rough stand-in for the RHIB.
