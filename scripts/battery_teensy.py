#!/usr/bin/env python3
"""Jetson-side bridge: read battery JSON lines from the Teensy over USB serial,
write /home/boat/battery.json for the GCS dashboard (/api/battery).

Contract: the Teensy prints ONE JSON object per line, e.g.
  {"soc":87,"voltage":52.6,"current":-4.3,"temp":24.5,"cells":[3.28,...]}
This bridge validates it, stamps online/power, and writes it atomically.

Run: python3 battery_teensy.py [--port /dev/ttyACM1] [--baud 115200]
Auto-detects the Teensy port if --port omitted.
"""
import argparse
import glob
import json
import os
import time

try:
    import serial
except ImportError:
    raise SystemExit("pip install pyserial")

OUT = "/home/boat/battery.json"
KEYS = ("soc", "voltage", "current", "temp")


def find_port():
    # Teensy shows as ttyACM*; skip ttyACM0 (Pixhawk). Prefer highest ACM, then USB.
    acm = sorted(glob.glob("/dev/ttyACM*"))
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    cands = [p for p in acm if p != "/dev/ttyACM0"] + usb + acm
    return cands[0] if cands else None


def write_atomic(d):
    tmp = OUT + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f)
    os.replace(tmp, OUT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args()
    port = a.port or find_port()
    if not port:
        raise SystemExit("no serial port found (ls /dev/ttyACM* /dev/ttyUSB*)")
    print(f"reading Teensy on {port} @ {a.baud} -> {OUT}")

    last = 0
    while True:
        try:
            with serial.Serial(port, a.baud, timeout=2) as s:
                while True:
                    line = s.readline().decode("ascii", "ignore").strip()
                    if not line or line[0] != "{":
                        # stale-data watchdog
                        if time.time() - last > 8:
                            write_atomic({"online": False, "msg": "no battery data"})
                        continue
                    try:
                        d = json.loads(line)
                    except ValueError:
                        continue
                    if not any(k in d for k in KEYS):
                        continue
                    v, i = d.get("voltage"), d.get("current")
                    if v is not None and i is not None:
                        d["power"] = round(v * i, 0)
                    d["online"] = True
                    write_atomic(d)
                    last = time.time()
        except serial.SerialException:
            write_atomic({"online": False, "msg": "Teensy disconnected"})
            time.sleep(3)          # port vanished; retry


if __name__ == "__main__":
    main()
