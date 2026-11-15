#!/usr/bin/env python3
"""LiTime 48V ComFlex battery telemetry -> dashboard.
FTDI USB-RS485 adapter on the Jetson, LiTime-proprietary protocol @ 19200 8N1.
Polls every second (--interval), writes /home/boat/battery.json (GCS dashboard /api/battery) and
appends every reading to /home/boat/battlogs/batt_YYYYMMDD.csv (one file per local day, ~3 MB),
so runs can be matched to energy use afterwards: tools/batt_report.py integrates Ah/Wh and, given
a dataflash log, tabulates W, Wh/km and range per speed. Times are the Jetson clock (UTC column
and unix seconds); the report lines them up with the log's GPS time.

Verified 2026-08-03: V/I/SOC/temp/cells decode confirmed against a live pack
(52.96V, 0A, 48%, 21C, 16 cells @3.28V, 100Ah full).
"""
import argparse, csv, glob, json, os, time
try:
    import serial
except ImportError:
    raise SystemExit("pip install pyserial")

OUT = "/home/boat/battery.json"
LOG_DIR = "/home/boat/battlogs"
CSV_COLS = ["utc", "unix", "online", "voltage", "current", "power", "soc", "soh", "temp",
            "cell_min", "cell_max", "cell_spread_mv", "cells"]
QUERY = bytes([0x00, 0x00, 0x04, 0x01, 0x13, 0x55, 0xAA, 0x17])

def u16(b, i): return b[i] | (b[i+1] << 8)
def u32(b, i): return b[i] | (b[i+1] << 8) | (b[i+2] << 16) | (b[i+3] << 24)
def s32(v):    return v - 0x100000000 if v >= 0x80000000 else v
def s16(v):    return v - 0x10000 if v >= 0x8000 else v

def find_port():
    for pat in ("/dev/ttyUSB*", "/dev/cu.usbserial*"):
        m = glob.glob(pat)
        if m: return m[0]
    return None

def write_atomic(d):
    tmp = OUT + ".tmp"
    with open(tmp, "w") as f: json.dump(d, f)
    os.replace(tmp, OUT)

def log_csv(d, log_dir):
    """Append one reading; never let a logging problem stop the dashboard feed."""
    if not log_dir:
        return
    try:
        os.makedirs(log_dir, exist_ok=True)
        now = time.time()
        fn = os.path.join(log_dir, time.strftime("batt_%Y%m%d.csv", time.localtime(now)))
        new = not os.path.exists(fn)
        cells = d.get("cells") or []
        row = {
            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)),
            "unix": f"{now:.1f}",
            "online": int(bool(d.get("online"))),
            **{k: d.get(k, "") for k in ("voltage", "current", "power", "soc", "soh", "temp")},
            "cell_min": min(cells) if cells else "",
            "cell_max": max(cells) if cells else "",
            "cell_spread_mv": round((max(cells) - min(cells)) * 1000) if cells else "",
            "cells": " ".join(f"{c:.3f}" for c in cells),
        }
        with open(fn, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_COLS)
            if new:
                w.writeheader()
            w.writerow(row)
    except OSError as e:
        print(f"battery csv: {e}", flush=True)

def publish(d, log_dir):
    write_atomic(d)
    log_csv(d, log_dir)

def parse(b):
    volt = u32(b, 8) / 1000.0
    curr = s32(u32(b, 48)) / 1000.0
    soc  = u16(b, 90)
    soh  = u16(b, 92)
    temps = [s16(u16(b, o)) for o in (52, 54, 56, 58, 60)]
    temp = max(t for t in temps if -40 < t < 120) if any(-40 < t < 120 for t in temps) else None
    full_ah = u16(b, 64) / 100.0
    cells = [round(u16(b, 16 + 2*c) / 1000.0, 3) for c in range(16) if u16(b, 16 + 2*c)]
    return {
        "online": True,
        "voltage": round(volt, 2),
        "current": round(curr, 2),
        "power": round(volt * curr, 0),
        "soc": soc, "soh": soh, "temp": temp,
        "full_ah": full_ah, "cells": cells,
    }

def read_response(s):
    s.reset_input_buffer()
    s.write(QUERY); s.flush()
    time.sleep(0.15)                      # ~105 bytes @19200 ≈ 55ms; margin
    n = s.in_waiting
    return s.read(n) if n else b""

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--interval", type=float, default=1.0, help="s between polls (default 1)")
    ap.add_argument("--log-dir", default=LOG_DIR, help="daily CSV history ('' to disable)")
    a = ap.parse_args()
    port = a.port or find_port()
    if not port: raise SystemExit("no FTDI adapter (ls /dev/ttyUSB*)")
    print(f"LiTime BMS reader on {port} @ {a.baud} -> {OUT}, history in {a.log_dir or '(off)'}")
    while True:
        try:
            with serial.Serial(port, a.baud, bytesize=8, parity="N", stopbits=1, timeout=0.5) as s:
                fails = 0
                while True:
                    resp = read_response(s)
                    if len(resp) >= 100 and resp[5:7] == b"\x55\xaa":
                        publish(parse(resp), a.log_dir); fails = 0
                    else:
                        fails += 1
                        if fails >= 3:
                            publish({"online": False, "msg": "no BMS response"}, a.log_dir)
                    time.sleep(max(0.2, a.interval - 0.15))   # the read itself takes 0.15 s
        except serial.SerialException:
            publish({"online": False, "msg": "adapter disconnected"}, a.log_dir)
            time.sleep(3)

if __name__ == "__main__":
    main()
