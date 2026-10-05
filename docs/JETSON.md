# Jetson runbook

The Jetson runs everything from a git checkout at `/home/boat/project-remora`
(public repo, cloned over HTTPS, so the boat needs no GitHub keys). Runtime state stays in
`/home/boat`: `mav.conf`, `battery.json`, `vcb.json`, `mavlogs/`, `vcblogs/`, plus the
`gcs.py`, `battery_litime.py` and `vcb_logger.py` symlinks into `project-remora/scripts/`.

| Unit | What | Device |
|---|---|---|
| `mavlink-router` | Pixhawk → TCP 5760 (QGC), UDP 14551 (MOOS), UDP 14552 (dashboard) | `/dev/pixhawk` (`99-pixhawk.rules`) |
| `boat-gcs` | dashboard on :8080 | |
| `mediamtx` | camera RTSP → WebRTC/HLS (`~/mediamtx/cam.yml`, not managed here) | |
| `battery-litime` | BMS → `battery.json` | `/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0FF8LR-if00-port0` |
| `vcb-logger` | VCB CSV → `vcb.json` + `vcblogs/vcb_*.csv` (read-only) | `/dev/vcb` (`99-vcb.rules`), else autodetect |

## Dashboard and ports

`http://highfieldboat.tailcacf9.ts.net:8080/` is the "Boat GCS" dashboard, served by
`scripts/gcs.py` (plain Python `http.server`, no framework; the HTML is a string inside the
script). It shows the live camera with PTZ controls, and cards for the vehicle, battery, VCB
and host health. It is monitor-only: apart from stream-rate requests it sends nothing to the
autopilot (no heartbeat, arm, mode or parameter commands).

| Route | Method | Purpose |
|---|---|---|
| `/`, `/index*` | GET | dashboard HTML |
| `/api/vehicle` | GET | ArduPilot telemetry from mavlink-router UDP 14552 |
| `/api/battery` | GET | battery state (`battery.json`, written by `battery_litime.py`) |
| `/api/vcb` | GET | VCB state (`vcb.json`, written by `vcb_logger.py`) |
| `/api/health` | GET | service states, disk, temperatures, load |
| `/api/ptz` | POST | camera pan/tilt/zoom |

`gcs.py` options: `--port`/`GCS_PORT`, `--mavlink`/`GCS_MAVLINK`, `--state-dir`/`GCS_STATE_DIR`.

Video is not served by `gcs.py`. MediaMTX pulls RTSP from the IP camera at `192.168.1.110` and
republishes it; the dashboard embeds the WebRTC player.

| Port | Service |
|---|---|
| 22 | SSH |
| 5760 | `mavlink-routerd` TCP server (QGroundControl, tools) |
| 8080 | dashboard (`gcs.py`) |
| 8554 / 8888 / 8889 / 1935 | MediaMTX RTSP / HLS / WebRTC / RTMP |
| 9997 | MediaMTX API (localhost only) |
| UDP 14551, 14552 | `mavlink-routerd` to the MOOS bridge and the dashboard (localhost only) |

`mavlink-routerd` and the `mediamtx` binary are installed on the Jetson separately; this repo
holds only their configs.

## Logs

Logs go to the journal (persistent, 500 MB cap). `/etc/logrotate.d/boat` rotates the old
`~/*.log` files and deletes VCB CSVs untouched for 30 days.

## First-time deploy

```sh
ssh boat@highfieldboat.tailcacf9.ts.net       # or: mosh boat@highfieldboat.tailcacf9.ts.net
git clone https://github.com/jerWenger/project-remora.git ~/project-remora
cd ~/project-remora
pip3 install --user pymavlink                 # dashboard vehicle card + tools (optional for gcs.py)
deploy/install.sh --dry-run                   # read what it will do
sudo deploy/install.sh
```

## Updating

Push from the laptop, then on the Jetson:

```sh
~/project-remora/deploy/update.sh             # fetch, list incoming commits, fast-forward, sudo install.sh --restart
```

It refuses if the checkout has local edits (fix things on the laptop and push instead) and
restarts the services so the symlinked scripts reload. Restarting `mavlink-router` drops QGC and
MOOS for a few seconds, so don't update while armed. `--no-restart` installs without restarting.

`install.sh` is idempotent. It backs up every file it replaces as `<file>.bak.<timestamp>`.
It installs the udev rules, `mav.conf`, the symlinks, the `mavlogs`/`vcblogs` dirs, the five
units (enabled; `mavlink-router` only once the old loop is gone), logrotate and journald configs. It starts stopped services, but restarts
running ones only with `--restart`.

## Remove the old router loop (once)

Before this repo, a shell loop (`sleep 15 && while true; do mavlink-routerd ...; done`)
started the router from cron `@reboot` or `rc.local`. `install.sh` greps crontabs,
`/etc/rc.local`, `~/.bashrc`, `/etc/systemd` and `/etc/cron.d` for it and prints where it
lives. Until it's removed, `install.sh` neither enables nor starts `mavlink-router.service`
(enabled, it would also start whenever the Pixhawk re-enumerates, e.g. after a reboot from
QGC, and two routers would share the Pixhawk port).

```sh
crontab -e                          # or: sudo crontab -e / sudo nano /etc/rc.local
sudo pkill -f 'while true; do .*[m]avlink-routerd'; sudo pkill -x mavlink-routerd
sudo systemctl enable --now mavlink-router
```

## Verify

```sh
systemctl status mavlink-router boat-gcs mediamtx battery-litime vcb-logger
ls -l /dev/pixhawk /dev/vcb /dev/serial/by-id/
ss -tulpn | grep -E '5760|14551|14552'      # routerd on 5760; 14551/14552 show once consumers bind
cat ~/vcb.json; echo; cat ~/battery.json
watch -n1 cat ~/vcb.json
journalctl -u vcb-logger -f                  # or any unit; -b for this boot, -b -1 for the last one
```

## mosh + tmux (sessions that survive Starlink drops)

```sh
sudo apt install mosh tmux                   # on the Jetson
brew install mosh                            # on the laptop
mosh boat@highfieldboat.tailcacf9.ts.net -- tmux new -A -s boat
```

mosh logs in over ssh, then switches to UDP 60000–61000. Tailscale carries that UDP over
the tailnet (DERP relay included), so no extra port opening is needed. tmux keeps your
shells if the mosh session dies: `Ctrl-b d` detaches, and running the same command again
reattaches.

## Troubleshooting

**Can't reach the boat.** Use the DNS name, not the raw tailnet IP. The link is relayed through
Tailscale's DERP server, and `tailscale ping` by IP or an ICMP ping can fail while the name works.

**Pixhawk re-enumeration** (reboot from QGC, param reboot, cable bump). The USB device goes
away and comes back. `BindsTo=dev-pixhawk.device` stops the router, and
`WantedBy=dev-pixhawk.device` starts it again. If it doesn't come back:
`ls -l /dev/pixhawk /dev/serial/by-id/`, `udevadm info -q property -n /dev/ttyACM0 | grep ID_`
(expect `1209:5740`, interface `00`), `journalctl -u mavlink-router -b`,
`sudo systemctl restart mavlink-router`. In bootloader mode the board shows different IDs
for a few seconds. That's normal.

**VCB not appearing** (`vcb.json` says "waiting for VCB port"). It has never been seen on
the Jetson. Check the cable and VCB power, then `lsusb` and `sudo dmesg -w` while you plug
it in. The logger also picks up any `/dev/serial/by-id` entry that isn't the Pixhawk or the
FTDI, so a missing `/dev/vcb` alone doesn't stop it. Fix `99-vcb.rules` with the real IDs
(steps are in the file), then re-run `install.sh`. "open, no data yet" means the port opened
but the firmware is silent. To watch it by hand, stop the service so it doesn't split the
data, then:
`sudo systemctl stop vcb-logger; python3 ~/vcb_logger.py --stdout --port /dev/ttyACMn --out-dir /tmp --state /tmp/vcb.json`.
If `ModemManager` is installed it may probe new ACM ports. The rule sets
`ID_MM_DEVICE_IGNORE`. Otherwise run `sudo systemctl disable --now ModemManager`.

**BMS "no response"** (`battery.json`). "adapter disconnected" means the by-id path is
missing: `ls -l /dev/serial/by-id/`. A different FTDI serial needs a fix in
`systemd/battery-litime.service`. "no BMS response" means the adapter is fine but the pack
didn't answer. Things to check:
- RS485 A/B are swapped. This is the most common cause, so try swapping them.
- The pack is asleep or its BMS is off.
- The link must be 19200 8N1.
- GND is not shared.

To test by hand:
`sudo systemctl stop battery-litime; python3 ~/battery_litime.py --port /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DU0FF8LR-if00-port0`.
