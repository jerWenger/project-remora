#!/usr/bin/env python3
"""LiTime 48V/51.2V 100Ah Smart ComFlex battery telemetry over RS485 (Modbus-RTU).

Two modes:
  scan   : sweep slave addresses + function codes, dump raw registers + best-guess
           decode. Use FIRST to confirm wiring/address/register alignment.
  daemon : poll one confirmed address every 2 s, write /home/boat/battery.json
           (the GCS dashboard /api/battery reads this file).

Protocol (verified vs 48V 15S/16S BMS Modbus doc): 9600 8N1, FC 0x03/0x04,
read from register 1. reg3=V/100  reg4=I/10 signed  reg5=SOC%  reg25..28,35=temp/10
reg9..24=cell V/1000. Address: dial 1-15 or soft-default 170 (0xAA).
Full reference: Obsidian KEY-NOTES (Battery section).
"""
import argparse
import json
import struct
import sys
import time

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    sys.exit("pip install pymodbus pyserial")

OUT = "/home/boat/battery.json"
ADDR_SWEEP = [170] + list(range(1, 16))   # 0xAA first, then dial codes 1-15


def s16(v):
    return v - 0x10000 if v >= 0x8000 else v


def read_block(client, addr, base, count, fc):
    """Return list of registers or None. fc: 'h'=holding(0x03) 'i'=input(0x04)."""
    fn = client.read_holding_registers if fc == "h" else client.read_input_registers
    for kw in ("device_id", "slave"):          # pymodbus renamed slave->device_id
        try:
            r = fn(address=base, count=count, **{kw: addr})
            if not r.isError():
                return r.registers
        except TypeError:
            continue
        except Exception:
            return None
    return None


def decode(regs, base):
    """regs indexed from `base` (doc register number of regs[0]). Returns dict."""
    def reg(n):  # doc register n -> value, or None
        i = n - base
        return regs[i] if 0 <= i < len(regs) else None
    v, i, soc = reg(3), reg(4), reg(5)
    ncell = reg(8) or 0
    cells = [reg(9 + k) / 1000 for k in range(min(ncell, 16)) if reg(9 + k) is not None]
    temps = [s16(reg(t)) / 10 for t in (25, 26, 27, 28, 35) if reg(t) is not None]
    volt = v / 100 if v is not None else None
    curr = s16(i) / 10 if i is not None else None
    return {
        "voltage": round(volt, 2) if volt else None,
        "current": round(curr, 1) if curr is not None else None,
        "power": round(volt * curr, 0) if volt and curr is not None else None,
        "soc": reg(5), "soh": reg(6), "cycles": reg(7),
        "temp": round(max(temps), 1) if temps else None,
        "cells": [round(c, 3) for c in cells],
    }


def plausible(d):
    """Sanity gate to auto-pick the right address/base during scan."""
    return (d["voltage"] and 40 <= d["voltage"] <= 60
            and d["soc"] is not None and 0 <= d["soc"] <= 100)


def scan(client):
    print("Scanning addresses", ADDR_SWEEP, "function codes 0x03/0x04, base 0 and 1...")
    hits = []
    for addr in ADDR_SWEEP:
        for fc in ("h", "i"):
            regs = read_block(client, addr, 0, 40, fc)
            if regs is None:
                continue
            print(f"\n[addr={addr} fc={'0x03' if fc=='h' else '0x04'}] raw[0:40]={regs}")
            for base in (0, 1):        # doc register-numbering ambiguity
                d = decode(regs, base)
                tag = "  <-- PLAUSIBLE" if plausible(d) else ""
                print(f"   base={base}: V={d['voltage']} I={d['current']} "
                      f"SOC={d['soc']} T={d['temp']} cells={len(d['cells'])}{tag}")
                if plausible(d):
                    hits.append((addr, fc, base, d))
    print("\n=== result ===")
    if hits:
        addr, fc, base, d = hits[0]
        print(f"USE: --addr {addr} --fc {'0x03' if fc=='h' else '0x04'} --base {base}")
        print("decoded:", json.dumps(d))
    else:
        print("No plausible battery response. Checklist:")
        print(" - wiring: RS485 A<->A, B<->B, GND. Swap A/B if silent.")
        print(" - right RJ45 jack (RS485/inverter port, NOT the Victron/CAN jack).")
        print(" - protocol enabled: select a LiTime/inverter protocol in the LiTime app.")
        print(" - adapter present: ls /dev/ttyUSB*  (in dialout group).")
    return hits


def daemon(client, addr, base, fc):
    print(f"daemon: addr={addr} base={base} fc={fc} -> {OUT}")
    while True:
        regs = read_block(client, addr, 0, 40, fc)
        if regs:
            d = decode(regs, base)
            d["online"] = True
        else:
            d = {"online": False, "msg": "RS485 read failed"}
        d["ts"] = None
        try:
            with open(OUT, "w") as f:
                json.dump(d, f)
        except Exception as e:
            print("write err:", e)
        time.sleep(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["scan", "daemon"])
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--addr", type=int, default=170)
    ap.add_argument("--base", type=int, default=0, choices=[0, 1])
    ap.add_argument("--fc", default="0x03", choices=["0x03", "0x04"])
    a = ap.parse_args()
    client = ModbusSerialClient(port=a.port, baudrate=a.baud,
                                bytesize=8, parity="N", stopbits=1, timeout=1)
    if not client.connect():
        sys.exit(f"cannot open {a.port} (ls /dev/ttyUSB*, dialout group?)")
    fc = "h" if a.fc == "0x03" else "i"
    try:
        if a.mode == "scan":
            scan(client)
        else:
            daemon(client, a.addr, a.base, fc)
    finally:
        client.close()


if __name__ == "__main__":
    main()
