#!/usr/bin/env python3
"""Why doesn't ArduPilot see the serial GPS? Read the config, watch the GPS messages,
then tap the raw bytes on the GPS port through ArduPilot's serial passthrough.

Runs on the Jetson (it needs the Pixhawk's second USB port, /dev/ttyACM1).

    python3 ~/tools/gps_probe.py                      # config + GPS_RAW_INT only, no writes
    python3 ~/tools/gps_probe.py --probe              # also tap every SERIALn with protocol GPS
    python3 ~/tools/gps_probe.py --probe --port 4     # only SERIAL4 (the GPS2 connector)
    python3 ~/tools/gps_probe.py --probe --bauds 115200 460800

Stage 1 prints the GPS slots (GPS1_TYPE/GPS2_TYPE), which SERIALn ports carry the GPS
protocol, their bauds, the EKF yaw source and SERIAL_PASS*, with a plain-English
reading of each. Stage 2 watches GPS_RAW_INT / GPS2_RAW for a few seconds.
Stage 3 (--probe) sets SERIAL_PASS1=8 (USB port 2 = ttyACM1 here), SERIAL_PASS2=n,
SERIAL_PASSTIMO=0, opens ttyACM1 at each baud, and says what the bytes look like:
NMEA, Unicore ASCII, Unicore/NovAtel binary, u-blox UBX, other text, noise, or
nothing. ArduPilot copies the USB baud onto the tapped port while passthrough is on,
so each baud is tried by reopening ttyACM1. SERIAL_PASS* are restored afterwards
(also on Ctrl-C). Refuses to probe while armed: ArduPilot disables passthrough when
armed anyway. Reboot the autopilot after a probe so the GPS driver gets its port back
cleanly.

Requires: pip install pymavlink pyserial
"""
import argparse
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ap_apply import connect, is_armed, read_named, write_one  # noqa: E402

try:
    import serial
except ImportError:
    serial = None

DEFAULT_CONN = "tcp:127.0.0.1:5760"
DEFAULT_TTY = "/dev/ttyACM1"          # Pixhawk 6X USB interface 02 = SERIAL8
PASS_PORT = 8
DEFAULT_BAUDS = (115200, 230400, 460800, 921600, 57600, 38400, 9600)
PROTO_GPS = 5
PROTO_MAVLINK = (1, 2)

# ArduPilot 4.7 GPS_TYPE values that matter here
GPS_TYPES = {
    0: "None", 1: "AUTO", 2: "uBlox", 5: "NMEA", 6: "SiRF", 7: "HIL", 8: "SwiftNav",
    9: "DroneCAN", 10: "SBF", 11: "GSOF", 13: "ERB", 14: "MAV", 15: "NOVA", 16: "HemisphereNMEA",
    17: "uBlox-MovingBaseline-Base", 18: "uBlox-MovingBaseline-Rover", 19: "MSP", 20: "AllyStar",
    21: "ExternalAHRS", 22: "DroneCAN-MovingBaseline-Base", 23: "DroneCAN-MovingBaseline-Rover",
    24: "UnicoreNMEA", 25: "UnicoreMovingBaselineNMEA", 26: "SBF-DualAntenna",
}
SERIAL_NEEDED = {1, 2, 5, 6, 8, 10, 11, 13, 15, 16, 17, 18, 20, 24, 25, 26}
NMEA_TYPES = {1, 5, 16, 24, 25}
PORT_NAMES = {0: "USB", 1: "TELEM1", 2: "TELEM2", 3: "GPS1", 4: "GPS2", 5: "TELEM3",
              6: "UART4", 7: "DEBUG", 8: "USB2"}
YAW_SRC = {0: "None", 1: "Compass", 2: "GPS", 3: "GPS with compass fallback", 6: "ExternalNav", 8: "GSF"}
FIX = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}

PARAMS = ["GPS1_TYPE", "GPS2_TYPE", "GPS_AUTO_CONFIG", "GPS_AUTO_SWITCH", "GPS1_GNSS_MODE",
          "GPS1_RATE_MS", "EK3_SRC1_YAW", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY",
          "SERIAL_PASS1", "SERIAL_PASS2", "SERIAL_PASSTIMO", "BRD_SER1_RTSCTS", "BRD_SER2_RTSCTS"]
for _n in range(9):
    PARAMS += [f"SERIAL{_n}_PROTOCOL", f"SERIAL{_n}_BAUD"]

NMEA_RE = re.compile(rb"\$(G[PNLABQ]|UNI|PQ|PS)[A-Z]{3,5},")
UNICORE_ASCII_RE = re.compile(rb"#[A-Z]{4,12}A?,")


def ival(p, name, default=None):
    return int(round(p[name][0])) if name in p else default


def baud_hz(v):
    """SERIALn_BAUD is in units of 100 (57 = 57600, 115 = 115200) but accepts full values too."""
    v = int(round(v))
    return v if v > 2000 else {1: 1200, 2: 2400, 4: 4800, 9: 9600, 19: 19200, 38: 38400, 57: 57600,
                                111: 111100, 115: 115200, 230: 230400, 256: 256000, 460: 460800,
                                500: 500000, 921: 921600, 1500: 1500000, 2000: 2000000}.get(v, v * 100)


def show_config(p):
    print("\n== Autopilot GPS configuration ==")
    gps_ports = []
    for n in range(9):
        proto, baud = ival(p, f"SERIAL{n}_PROTOCOL"), p.get(f"SERIAL{n}_BAUD", (None,))[0]
        if proto is None:
            continue
        tag = {PROTO_GPS: "GPS", 1: "MAVLink1", 2: "MAVLink2", -1: "disabled", 23: "RCIN"}.get(proto, f"proto {proto}")
        mark = "  <-- GPS driver scans this port" if proto == PROTO_GPS else ""
        print(f"  SERIAL{n} ({PORT_NAMES.get(n, '?'):6s}) {tag:9s} {baud_hz(baud):>7d} baud{mark}")
        if proto == PROTO_GPS:
            gps_ports.append(n)

    print()
    slots = []
    for i, name in enumerate(("GPS1_TYPE", "GPS2_TYPE"), 1):
        t = ival(p, name)
        if t is None:
            continue
        slots.append(t)
        print(f"  {name} = {t} ({GPS_TYPES.get(t, 'unknown')})")
    yaw = ival(p, "EK3_SRC1_YAW")
    print(f"  EK3_SRC1_YAW = {yaw} ({YAW_SRC.get(yaw, '?')})   GPS_AUTO_CONFIG = {ival(p, 'GPS_AUTO_CONFIG')}")
    print(f"  SERIAL_PASS1 = {ival(p, 'SERIAL_PASS1')}  SERIAL_PASS2 = {ival(p, 'SERIAL_PASS2')}  "
          f"SERIAL_PASSTIMO = {ival(p, 'SERIAL_PASSTIMO')}")

    print("\n  Reading:")
    serial_slots = [t for t in slots if t in SERIAL_NEEDED]
    if not gps_ports:
        print("  - No SERIALn has protocol 5 (GPS). A serial receiver cannot be found. Set SERIALn_PROTOCOL=5 on its port and reboot.")
    if not serial_slots:
        print("  - Neither GPS slot is a serial type, so the serial ports are never scanned. Set GPS1_TYPE for the receiver "
              "(e.g. 24 UnicoreNMEA, 25 UnicoreMovingBaselineNMEA, 1 AUTO) and reboot.")
    for i, t in enumerate(slots, 1):
        if t == 0:
            continue
        if t == 9 or t in (22, 23):
            print(f"  - GPS{i} is DroneCAN. It only appears when the CAN puck is connected and answering; "
                  f"if the puck is unplugged, EKF3 waits for it forever ('waiting for GPS config data'). Set GPS{i}_TYPE=0 to isolate the serial receiver.")
        elif t in NMEA_TYPES:
            print(f"  - GPS{i} expects NMEA text from the receiver. Unicore binary (AA 44 ..) will not be parsed.")
        elif t == 15:
            print(f"  - GPS{i} = NOVA expects NovAtel/Unicore binary (AA 44 12). NMEA text will not be parsed.")
        if t in (24, 25) and ival(p, "GPS_AUTO_CONFIG", 1) == 0:
            print(f"  - GPS{i} Unicore driver needs GPS_AUTO_CONFIG=1: it sends the CONFIG/GPGGA/AGRICA commands itself.")
    if len(gps_ports) > 1:
        print(f"  - {len(gps_ports)} ports carry the GPS protocol ({', '.join(PORT_NAMES[n] for n in gps_ports)}). "
              "The GPS driver cycles through each at each baud, so first detection can take 30-60 s after boot.")
    if yaw in (2, 3) and not any(t in (17, 18, 22, 23, 25, 26) for t in slots):
        print("  - EK3_SRC1_YAW wants GPS yaw but no GPS slot is a dual-antenna/moving-baseline type, so there will never be a yaw.")
    if ival(p, "SERIAL_PASS2", -1) != -1:
        print("  - SERIAL_PASS2 is set: passthrough is ON and the GPS driver does not own that port. Set SERIAL_PASS2=-1 and reboot.")
    return gps_ports


def watch_gps(m, seconds):
    print(f"\n== GPS_RAW_INT / GPS2_RAW for {seconds} s ==")
    seen, t0 = {}, time.time()
    while time.time() - t0 < seconds:
        msg = m.recv_match(type=("GPS_RAW_INT", "GPS2_RAW"), blocking=True, timeout=1)
        if msg and msg.get_srcSystem() == m.target_system:
            seen[msg.get_type()] = msg
    if not seen:
        print("  no GPS_RAW_INT at all (stream not requested? QGC usually requests it; try again)")
    for k, msg in seen.items():
        ft = msg.fix_type
        print(f"  {k:12s} fix={ft} ({FIX.get(ft, '?')}) sats={msg.satellites_visible} "
              f"lat={msg.lat / 1e7:.6f} lon={msg.lon / 1e7:.6f}")
        if ft == 0:
            print("    -> the driver has not detected a receiver on any GPS port.")
    return seen


def classify(buf):
    if not buf:
        return "NOTHING", "no bytes at all"
    nmea = NMEA_RE.findall(buf)
    if nmea:
        names = sorted({b"$" + a for a in set(re.findall(rb"\$([A-Z]{5,8}),", buf))})
        return "NMEA", "sentences: " + " ".join(n.decode() for n in names[:12])
    if UNICORE_ASCII_RE.search(buf):
        names = sorted(set(re.findall(rb"#([A-Z]{4,12}),", buf)))
        return "UNICORE-ASCII", "messages: " + " ".join(n.decode() for n in names[:12])
    if b"\xaa\x44\xb5" in buf or b"\xaa\x44\x12" in buf:
        kind = "Unicore" if b"\xaa\x44\xb5" in buf else "NovAtel-compatible"
        return "BINARY", f"{kind} binary headers (receiver is in binary mode; NMEA driver cannot use this)"
    if b"\xb5\x62" in buf:
        return "UBX", "u-blox UBX frames (this is not a Unicore?)"
    printable = sum(32 <= b < 127 or b in (10, 13) for b in buf) / len(buf)
    if printable > 0.9:
        return "TEXT", "readable text but not NMEA: " + buf[:60].decode("ascii", "replace").replace("\n", "\\n")
    if printable > 0.5:
        return "MIXED", "part readable: baud is probably close but wrong, or a mixed stream"
    return "NOISE", f"{len(buf)} bytes of garbage: wrong baud (or TX/RX swapped with a noisy line)"


def tap(tty, baud, seconds):
    with serial.Serial(tty, baud, timeout=0.2) as s:
        s.reset_input_buffer()
        time.sleep(1.2)            # give ArduPilot's 1 Hz passthru task time to push the new baud
        s.reset_input_buffer()
        buf, t0 = b"", time.time()
        while time.time() - t0 < seconds:
            buf += s.read(4096)
    return buf


def probe_port(m, p, port, tty, bauds, seconds):
    print(f"\n== Tapping SERIAL{port} ({PORT_NAMES.get(port, '?')}) via passthrough on {tty} ==")
    orig = {n: p[n] for n in ("SERIAL_PASS1", "SERIAL_PASS2", "SERIAL_PASSTIMO") if n in p}
    want = {"SERIAL_PASS1": PASS_PORT, "SERIAL_PASS2": port, "SERIAL_PASSTIMO": 0}
    results = {}
    try:
        for n, v in want.items():
            got = write_one(m, n, v, orig[n][1])
            if got is None or int(round(got)) != v:
                raise SystemExit(f"could not set {n}={v} (vehicle says {got})")
        print(f"  passthrough SERIAL{PASS_PORT} <-> SERIAL{port} enabled; waiting for it to start")
        time.sleep(2.5)
        for b in bauds:
            try:
                buf = tap(tty, b, seconds)
            except (serial.SerialException, OSError) as e:
                print(f"  {b:>7d}: cannot open {tty}: {e}")
                print("          (is the second USB interface there? ls /dev/serial/by-id/ should show ...-if02; is the router or vcb-logger holding it?)")
                break
            kind, detail = classify(buf)
            results[b] = kind
            print(f"  {b:>7d}: {kind:14s} {len(buf):6d} B  {detail}")
            if kind in ("NMEA", "UNICORE-ASCII", "BINARY", "UBX"):
                break
    finally:
        print("  restoring SERIAL_PASS* ...", end=" ", flush=True)
        # SERIAL_PASS2 first: that is the one that keeps the GPS driver off its port
        ok = all(write_one(m, n, orig[n][0], orig[n][1], retries=6, timeout=3.0) is not None
                 for n in ("SERIAL_PASS2", "SERIAL_PASS1", "SERIAL_PASSTIMO") if n in orig)
        print("ok" if ok else "FAILED")
        if not ok:
            raise SystemExit("param writes are not being acknowledged. In QGC set SERIAL_PASS2=-1 and "
                             "SERIAL_PASS1=0, reboot, then run again with --port <n> for one port only.")
    return results


def verdict(port, results, p, baud_cfg):
    kinds = set(results.values())
    good = next((b for b, k in results.items() if k in ("NMEA", "UNICORE-ASCII", "BINARY", "UBX")), None)
    print(f"\n  Verdict for SERIAL{port}:")
    if good:
        k = results[good]
        if good != baud_cfg:
            print(f"  - The receiver talks at {good}, but SERIAL{port}_BAUD is {baud_cfg}. Set SERIAL{port}_BAUD={good // 100 if good >= 9600 else good} and reboot.")
        else:
            print(f"  - Baud {good} matches SERIAL{port}_BAUD. Bytes are arriving and framed correctly.")
        slot_types = [ival(p, n) for n in ("GPS1_TYPE", "GPS2_TYPE") if n in p]
        if k == "BINARY" and not any(t == 15 for t in slot_types):
            print("  - Receiver is in binary mode but no GPS slot is NOVA (15) and the Unicore NMEA drivers need NMEA. "
                  "Either set the UM982 back to NMEA output (per the ArduSimple guide, 'unlog' then GPGGA/GPRMC/etc.) "
                  "or let GPS_AUTO_CONFIG=1 with GPS1_TYPE=24/25 reconfigure it, which only works if the driver can "
                  "first see *something* it recognises.")
        if k in ("NMEA", "UNICORE-ASCII") and not any(t in NMEA_TYPES or t == 1 for t in slot_types):
            print("  - Receiver sends NMEA but no GPS slot is an NMEA type. Set GPS1_TYPE=24 (UnicoreNMEA) or 25 (moving baseline) and reboot.")
        if k in ("NMEA", "UNICORE-ASCII") and any(t in NMEA_TYPES or t == 1 for t in slot_types):
            print("  - Config and data agree. If GPS_RAW_INT still shows fix 0 after a reboot, the GPS driver may be stuck on a "
                  "different GPS port or slot: set every other SERIALn_PROTOCOL=5 port to -1 and GPS2_TYPE=0, reboot, retry.")
    elif kinds <= {"NOTHING"}:
        print("  - Not one byte at any baud: nothing is reaching this port. Check the receiver has power (LED), "
              "receiver TX -> Pixhawk RX (pin 3 on the 6-pin), and that the cable is on the UM982 COM port the guide configured, "
              "not another COM. Also confirm the cable really is in this connector and not the other GPS/TELEM socket.")
    elif "NOISE" in kinds or "MIXED" in kinds:
        print("  - Garbage at every baud tried: the receiver is at a baud not in the list (try --bauds with 19200 300 1500000) "
              "or the signal is inverted/levels wrong (the simpleRTK3B UART is 3.3 V, fine for a Pixhawk).")
    else:
        print("  - Readable text that isn't NMEA. Paste the line above; the receiver may be in a config/echo mode.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default=DEFAULT_CONN, help="pymavlink connection (router TCP on the Jetson)")
    ap.add_argument("--baud", type=int, default=115200, help="baud for --conn if it is a serial device")
    ap.add_argument("--probe", action="store_true", help="tap the GPS port(s) through passthrough (writes SERIAL_PASS*)")
    ap.add_argument("--port", type=int, action="append", help="SERIALn to tap (default: every port with protocol 5)")
    ap.add_argument("--tty", default=DEFAULT_TTY, help=f"the Pixhawk's SERIAL8 USB device here (default {DEFAULT_TTY})")
    ap.add_argument("--bauds", type=int, nargs="+", default=list(DEFAULT_BAUDS))
    ap.add_argument("--seconds", type=float, default=3.0, help="listen time per baud")
    ap.add_argument("--watch", type=float, default=6.0, help="seconds to watch GPS_RAW_INT")
    a = ap.parse_args()

    if a.probe and serial is None:
        raise SystemExit("pip install pyserial (needed for --probe)")

    m, hb = connect(a.conn, a.baud)
    armed = is_armed(hb)
    print(f"sysid={m.target_system} armed={armed}")
    p = read_named(m, PARAMS)
    gps_ports = show_config(p)
    seen = watch_gps(m, a.watch)
    if any(msg.fix_type > 0 for msg in seen.values()):
        print("\nA receiver IS detected. The remaining problems are fix/yaw/config, not wiring; --probe is not needed.")
        if not a.probe:
            return 0

    if not a.probe:
        print("\nRun again with --probe to look at the raw bytes on the GPS port(s).")
        return 0
    if armed:
        raise SystemExit("armed: ArduPilot disables serial passthrough while armed; disarm first")
    if ival(p, "SERIAL_PASS2", -1) != -1:
        raise SystemExit("SERIAL_PASS2 is already set (leftover from an earlier run?). Set it to -1 in QGC, "
                         "reboot, then run again; otherwise the restore at the end would re-save the leftover.")
    ports = a.port or gps_ports
    if not ports:
        raise SystemExit("no port to tap: no SERIALn has protocol 5 and --port not given")
    if not os.path.exists(a.tty):
        raise SystemExit(f"{a.tty} missing: the Pixhawk's second USB interface (-if02) is not enumerated here; "
                         "use --tty, or check ls -l /dev/serial/by-id/")
    for port in ports:
        cfg = baud_hz(p.get(f"SERIAL{port}_BAUD", (115200,))[0])
        bauds = [cfg] + [b for b in a.bauds if b != cfg]       # configured baud first
        res = probe_port(m, p, port, a.tty, bauds, a.seconds)
        verdict(port, res, p, cfg)
    print("\nReboot the autopilot now so the GPS driver re-opens its port cleanly.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(1)
