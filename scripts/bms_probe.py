#!/usr/bin/env python3
"""Multi-protocol RS485 BMS prober for the FTDI USB-RS485 adapter.
Sends LiTime-proprietary + Modbus + PACE-paceic + JBD + Daly + JK frames across
bauds; prints any reply as hex, and decodes a LiTime reply if it comes.

Wiring: black=GND -> adapter GND, the two data wires -> adapter A(D+) and B(D-).
If everything is silent, physically SWAP A and B and re-run.

Run:  python3 bms_probe.py            (auto-detects the adapter)
      python3 bms_probe.py --port /dev/cu.usbserial-XXXX
"""
import argparse, glob, sys, time
try:
    import serial
except ImportError:
    sys.exit("pip install pyserial")

BAUDS = [9600, 115200, 19200, 57600, 38400, 4800]

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def modbus(addr, start=0, count=20):
    f = bytes([addr, 3, start >> 8, start & 0xFF, count >> 8, count & 0xFF])
    c = crc16(f)
    return f + bytes([c & 0xFF, c >> 8])

FRAMES = [
    ("LITIME", bytes([0x00,0x00,0x04,0x01,0x13,0x55,0xAA,0x17])),
    ("MODBUS-1",   modbus(1)),
    ("MODBUS-170", modbus(170)),
    ("MODBUS-0",   modbus(0)),
    ("PACEIC", b"~25014642E00201FD30\r"),
    ("JBD",  bytes([0xDD,0xA5,0x03,0x00,0xFF,0xFD,0x77])),
    ("DALY", bytes([0xA5,0x40,0x90,0x08,0,0,0,0,0,0,0,0,0x7D])),
    ("JK",   bytes([0x4E,0x57,0x00,0x13,0,0,0,0,0x06,0x03,0,0,0,0,0,0,0x68,0,0,0x01,0x29])),
]

def u16(b, i): return b[i] | (b[i+1] << 8)
def u32(b, i): return b[i] | (b[i+1] << 8) | (b[i+2] << 16) | (b[i+3] << 24)
def s32(v): return v - 0x100000000 if v >= 0x80000000 else v

def parse_litime(b):
    if len(b) < 100: return
    v = u32(b, 8) / 1000.0
    i = s32(u32(b, 48)) / 1000.0
    soc = u16(b, 90)
    t = max((u16(b, 52) if u16(b,52) < 0x8000 else u16(b,52)-0x10000),
            (u16(b, 54) if u16(b,54) < 0x8000 else u16(b,54)-0x10000))
    print(f"   >>> LITIME DECODE  V={v:.2f}  I={i:.2f}  SOC={soc}%  T={t}")

def find_port():
    for pat in ("/dev/cu.usbserial*", "/dev/tty.usbserial*", "/dev/ttyUSB*", "/dev/cu.usbmodem*"):
        m = glob.glob(pat)
        if m: return m[0]
    return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    a = ap.parse_args()
    port = a.port or find_port()
    if not port: sys.exit("no adapter found (ls /dev/cu.usbserial* /dev/ttyUSB*)")
    print(f"probing on {port}\n")
    any_reply = False
    for baud in BAUDS:
        print(f"== baud {baud} ==")
        try:
            s = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0.4)
        except Exception as e:
            print("  open failed:", e); continue
        hit = False
        for name, frame in FRAMES:
            s.reset_input_buffer()
            s.write(frame); s.flush()
            time.sleep(0.05)
            resp = s.read(256)
            if resp:
                hit = any_reply = True
                print(f"  {name:<11} -> {resp.hex(' ')}")
                if name == "LITIME": parse_litime(resp)
            time.sleep(0.02)
        if not hit: print("  (silent)")
        s.close()
    print("\ndone." if any_reply else "\nALL SILENT -> physically swap A and B on the adapter, re-run.")

if __name__ == "__main__":
    main()
