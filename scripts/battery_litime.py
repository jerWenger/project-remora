#!/usr/bin/env python3
"""LiTime 48V ComFlex battery telemetry -> dashboard.
FTDI USB-RS485 adapter on the Jetson, LiTime-proprietary protocol @ 19200 8N1.
Polls every 2s, writes /home/boat/battery.json (GCS dashboard /api/battery).

Verified 2026-08-03: V/I/SOC/temp/cells decode confirmed against a live pack
(52.96V, 0A, 48%, 21C, 16 cells @3.28V, 100Ah full).
"""
import argparse, glob, json, os, time
try:
    import serial
except ImportError:
    raise SystemExit("pip install pyserial")

OUT = "/home/boat/battery.json"
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
    a = ap.parse_args()
    port = a.port or find_port()
    if not port: raise SystemExit("no FTDI adapter (ls /dev/ttyUSB*)")
    print(f"LiTime BMS reader on {port} @ {a.baud} -> {OUT}")
    while True:
        try:
            with serial.Serial(port, a.baud, bytesize=8, parity="N", stopbits=1, timeout=0.5) as s:
                fails = 0
                while True:
                    resp = read_response(s)
                    if len(resp) >= 100 and resp[5:7] == b"\x55\xaa":
                        write_atomic(parse(resp)); fails = 0
                    else:
                        fails += 1
                        if fails >= 3:
                            write_atomic({"online": False, "msg": "no BMS response"})
                    time.sleep(2)
        except serial.SerialException:
            write_atomic({"online": False, "msg": "adapter disconnected"})
            time.sleep(3)

if __name__ == "__main__":
    main()
