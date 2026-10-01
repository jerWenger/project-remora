# 2.733 — HighFieldBoat

Boat-side code and configuration for the HighFieldBoat RHIB: Jetson services,
ArduPilot discovery tools and param dumps, hardware notes. Started from a copy of
the Jetson `HighFieldBoat` taken on 2026-09-29.

Related code lives elsewhere and is **not** in this repo:
- PCB firmware (VCB, MCB): https://github.com/cttdev (see `docs/HARDWARE.md`)
- MOOS-IvP missions and `iArduRoverBridge`: `~/moos/mCDR-Plume-Tracking`

Not tracked here: `logs/`, `web/` (captures from the Jetson) and `config/cam.yml`
(holds the camera password; use `config/cam.yml.example`).

Start with [`docs/HARDWARE.md`](docs/HARDWARE.md) and [`tools/README.md`](tools/README.md).

## Original Jetson snapshot notes

- **Host:** `highfieldboat.tailcacf9.ts.net` / `100.83.34.69` (tailnet, owner `chaca@`)
- **Hardware:** NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
- **OS:** Ubuntu 22.04 / L4T, kernel `5.15.148-tegra` (built 2025-09-18)
- **Login:** user `boat`, source dir `/home/boat`

## The website

`http://100.83.34.69:8080/` — **"Boat GCS"** dashboard, served by `scripts/gcs.py`
(plain Python `BaseHTTPServer`, no framework; HTML is a string literal inside the
script). Run by the `boat-gcs.service` unit. Dark-themed single page: live camera
pane on the left, PTZ controls and battery card on the right.

Routes:
| Route | Method | Purpose |
|---|---|---|
| `/`, `/index*` | GET | dashboard HTML |
| `/api/battery` | GET | battery JSON (reads `battery.json`) |
| `/api/ptz` | POST | camera pan/tilt/zoom commands |

Video is *not* served by `gcs.py`. A separate **MediaMTX** instance
(`mediamtx.service`, config in `config/cam.yml`) pulls RTSP from an IP camera at
`192.168.1.110` and republishes it; the dashboard embeds the WebRTC player.

Other listening ports on the Jetson:

| Port | Service |
|---|---|
| 8080 | Boat GCS dashboard (`gcs.py`) |
| 8554 | MediaMTX RTSP |
| 8888 | MediaMTX HLS (low-latency) |
| 8889 | MediaMTX WebRTC |
| 1935 | MediaMTX RTMP |
| 9997 | MediaMTX API (localhost only) |
| 5760 | `mavlink-routerd` TCP server |
| 22 | SSH |

## Contents

```
scripts/    gcs.py                 dashboard server + PTZ + battery poll
            battery_litime.py      LiTime BMS reader (the one wired to systemd)
            battery_rs485.py       RS485 BMS variant
            battery_teensy.py      Teensy-based battery variant
            bms_probe.py           BMS discovery/probe utility
systemd/    boat-gcs.service       runs gcs.py as user boat
            mediamtx.service       runs mediamtx with cam.yml
config/     mav.conf               mavlink-router: /dev/ttyACM0 @115200 -> TCP 5760
            cam.yml                MediaMTX: RTSP pull from cam, WebRTC/HLS/RTMP out
firmware/   VCB_flash_*.bin        1 MB VCB flash dump, "pre_tolerant_pwm" (2026-08-05)
logs/       mav.log                MAVLink router log (~1.3 MB, still live)
            battery.json           last battery state
            batt.log, gcs.log, mtx.log
web/        dashboard-rendered.html  live capture of the served page
```

## Notes / current state

- `battery.json` reads `{"online": false, "msg": "no BMS response"}` — the BMS was
  not responding at copy time. Three different battery reader scripts exist
  (litime / rs485 / teensy), suggesting the interface was still being worked out;
  `battery-litime.service` is the variant that got a unit file.
- Both MediaMTX camera paths (`cam`, `camhd`) show `ready: false` — the camera at
  `192.168.1.110` was not reachable, and there are no local `/dev/video*` devices.
  The boat had just rebooted (uptime ~1 min).
- `cam.yml` contains the camera's RTSP credentials in cleartext. The password is
  the MD5 of "123456" — a factory default.
- **Not copied** (upstream, not unique): `~/mavlink-router/` (49 MB git clone) and
  the `mediamtx` binary (62 MB) — only their config files were taken.
- Tailscale reaches this node over the **DERP relay "nyc"**, not a direct path.
  `tailscale ping` by raw IP can fail while the DNS name works; ICMP ping fails.
