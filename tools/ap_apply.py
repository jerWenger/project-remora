#!/usr/bin/env python3
"""Apply a desired-state .parm file to the autopilot: diff, confirm, back up, write, verify.

    ./ap_apply.py ../params/boat.parm --dry-run           # show what would change
    ./ap_apply.py ../params/boat.parm                     # diff, ask y/N, back up, write
    ./ap_apply.py ../params/boat.parm --yes --conn tcp:127.0.0.1:5760   # on the Jetson
    ./ap_apply.py ../params/backup_20261004_120000.parm   # undo: a backup is a .parm too

Only the params named in the file are read (one PARAM_REQUEST_READ each, retried),
so it is quick over the DERP-relayed tailnet. --full downloads all ~940 instead.
Before writing, the current values of everything it will change go to
params/backup_<UTC timestamp>.parm (--backup to override). Each PARAM_SET is
verified against the echoed PARAM_VALUE and retried. Refuses if the vehicle is
armed unless --allow-armed.
Requires: pip install pymavlink
"""
import argparse
import os
import struct
import sys
import time

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mav
except ImportError:
    raise SystemExit("pip install pymavlink")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ap_params import load  # noqa: E402  (same "NAME,value" / "NAME value" parser)

DEFAULT_CONN = "tcp:highfieldboat.tailcacf9.ts.net:5760"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Heuristic, deliberately conservative: params ArduPilot only reads at boot (drivers,
# ports, buses, sensor enable). Not exhaustive -- when in doubt, reboot.
REBOOT_PREFIXES = (
    "SERIAL", "CAN_", "GPS_TYPE", "GPS1_TYPE", "GPS2_TYPE", "GPS_AUTO_CONFIG",
    "COMPASS_ENABLE", "COMPASS_TYPEMASK", "COMPASS_EXTERNAL", "BRD_", "INS_ENABLE_MASK",
    "INS_GYRO_FILTER", "INS_ACCEL_FILTER", "FRAME_CLASS", "SCR_ENABLE", "SCR_HEAP_SIZE",
    "LOG_BACKEND_TYPE", "LOG_FILE_BUFSIZE", "BATT_MONITOR", "BATT2_MONITOR",
    "EK3_ENABLE", "EK2_ENABLE", "AHRS_EKF_TYPE", "RC_PROTOCOLS", "NET_", "DDS_",
    "BARO_PROBE", "BARO_EXT_BUS", "RNGFND1_TYPE", "OSD_TYPE", "NTF_LED_TYPES",
)
INT_TYPES = {mav.MAV_PARAM_TYPE_UINT8, mav.MAV_PARAM_TYPE_INT8, mav.MAV_PARAM_TYPE_UINT16,
             mav.MAV_PARAM_TYPE_INT16, mav.MAV_PARAM_TYPE_UINT32, mav.MAV_PARAM_TYPE_INT32}


def needs_reboot(name):
    return name.startswith(REBOOT_PREFIXES)


def f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]


def same(cur, want, ptype):
    """cur arrives as a float32; ints round-trip exactly below 2**24."""
    if ptype in INT_TYPES:
        return int(round(cur)) == int(round(want))
    return abs(f32(cur) - f32(want)) <= max(1e-6, 1e-5 * abs(want))


def fmt(v):
    return f"{v:g}"


def connect(conn, baud):
    print(f"connecting to {conn} ...", flush=True)
    m = mavutil.mavlink_connection(conn, baud=baud, source_system=250, source_component=191)
    if hasattr(m, "handle_eof"):  # pymavlink would otherwise spin printing "EOF on TCP socket"
        def eof():
            raise SystemExit("link closed by peer (router restarted? SITL died?)")
        m.handle_eof = eof
    # through the router other GCS heartbeats arrive too; wait for the autopilot's
    t0 = time.time()
    while time.time() - t0 < 20:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.autopilot != mav.MAV_AUTOPILOT_INVALID and hb.type != mav.MAV_TYPE_GCS:
            m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            return m, hb
    raise SystemExit("no autopilot heartbeat in 20 s (router down? wrong host? Pixhawk unplugged?)")


def is_armed(hb):
    return bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)


def read_named(m, names, retries=5, timeout=1.5):
    """PARAM_REQUEST_READ each name; re-send the unanswered ones. -> {name: (value, type)}"""
    got, pending = {}, list(names)
    for attempt in range(retries):
        if not pending:
            break
        if attempt:
            print(f"  re-requesting {len(pending)} unanswered (try {attempt + 1}/{retries})")
        for n in pending:
            m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
            time.sleep(0.02)
        deadline = time.time() + timeout + 0.05 * len(pending)
        while time.time() < deadline and pending:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg and msg.param_id in pending:
                got[msg.param_id] = (msg.param_value, msg.param_type)
                pending.remove(msg.param_id)
    return got


def read_all(m, timeout=240):
    m.param_fetch_all()
    got, seen, total, t0, last = {}, set(), None, time.time(), time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        now = time.time()
        if msg:
            total, last = msg.param_count, now
            got[msg.param_id] = (msg.param_value, msg.param_type)
            seen.add(msg.param_index)
            print(f"  {len(got)}/{total}", end="\r", flush=True)
            if len(got) >= total:
                break
        elif total and now - last > 5:
            for i in [i for i in range(total) if i not in seen][:200]:
                m.mav.param_request_read_send(m.target_system, m.target_component, b"", i)
                time.sleep(0.01)
            last = now
    print()
    return got


def write_one(m, name, value, ptype, retries=3, timeout=2.0):
    """PARAM_SET, then wait for the PARAM_VALUE echo. -> final value or None"""
    if ptype in INT_TYPES:
        value = float(int(round(value)))
    last = None
    for _ in range(retries):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), value, ptype)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg and msg.param_id == name:
                last = msg.param_value
                if same(last, value, ptype):
                    return last
    return last


def write_backup(path, cur, names, src):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(f"# backup by ap_apply.py {time.strftime('%Y-%m-%d %H:%M:%S')} before applying {src}\n")
        f.write(f"# undo: tools/ap_apply.py {path}\n")
        for n in names:
            f.write(f"{n},{cur[n][0]:.9g}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parm", help="desired-state .parm file (NAME,value or NAME value)")
    ap.add_argument("--conn", default=DEFAULT_CONN, help="pymavlink connection string")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dry-run", action="store_true", help="print the diff and exit")
    ap.add_argument("--yes", action="store_true", help="don't ask for confirmation")
    ap.add_argument("--full", action="store_true", help="download every param instead of reading by name")
    ap.add_argument("--backup", metavar="FILE", help="backup path (default params/backup_<UTC>.parm)")
    ap.add_argument("--allow-armed", action="store_true", help="write even if the vehicle is armed")
    a = ap.parse_args()

    want = load(a.parm)
    if not want:
        raise SystemExit(f"no params in {a.parm}")
    m, hb = connect(a.conn, a.baud)
    armed = is_armed(hb)
    print(f"sysid={m.target_system} mode={mavutil.mode_string_v10(hb)} armed={armed}")
    if armed and not a.allow_armed and not a.dry_run:
        raise SystemExit("vehicle is ARMED: refusing to write params (--allow-armed to override)")

    print(f"reading {len(want)} params named in {a.parm} ...")
    cur = read_all(m) if a.full else read_named(m, list(want))
    missing = [n for n in want if n not in cur]
    changes = [n for n in want if n in cur and not same(cur[n][0], want[n], cur[n][1])]

    print(f"\n{'NAME':18s} {'CURRENT':>12s}    DESIRED")
    for n in want:
        if n in missing:
            print(f"{n:18s} {'??':>12s}    {fmt(want[n]):10s} NOT ON VEHICLE (typo? needs reboot/enable first?)")
        elif n in changes:
            print(f"{n:18s} {fmt(cur[n][0]):>12s} -> {fmt(want[n]):10s}" + ("  [reboot]" if needs_reboot(n) else ""))
        else:
            print(f"{n:18s} {fmt(cur[n][0]):>12s}    (unchanged)")
    print(f"\n{len(changes)} to change, {len(want) - len(changes) - len(missing)} unchanged, {len(missing)} not found")
    if not changes:
        print("no changes")
        return 1 if missing else 0
    if a.dry_run:
        return 0
    if not a.yes:
        if input(f"write {len(changes)} params to sysid {m.target_system}? [y/N] ").strip().lower() != "y":
            print("aborted")
            return 1

    # re-check armed state right before writing
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3) or hb
    if is_armed(hb) and hb.get_srcSystem() == m.target_system and not a.allow_armed:
        raise SystemExit("vehicle armed meanwhile: refusing")

    backup = a.backup or os.path.join(REPO, "params", time.strftime("backup_%Y%m%d_%H%M%S.parm", time.gmtime()))
    write_backup(backup, cur, changes, a.parm)
    print(f"backup of current values -> {backup}")

    failed = []
    for n in changes:
        got = write_one(m, n, want[n], cur[n][1])
        ok = got is not None and same(got, want[n], cur[n][1])
        print(f"  {'OK  ' if ok else 'FAIL'} {n:18s} -> {fmt(want[n])}" + ("" if ok else f"  (vehicle reports {got})"))
        if not ok:
            failed.append(n)

    reboot = [n for n in changes if needs_reboot(n) and n not in failed]
    live_out = [n for n in changes if n.startswith("SERVO") and n.endswith("_FUNCTION") and n not in failed]
    if reboot:
        print(f"\nREBOOT the autopilot for these to take effect (heuristic list): {' '.join(reboot)}")
    if live_out:
        print(f"note: output functions change LIVE, the pin switches immediately: {' '.join(live_out)}")
    if failed:
        print(f"\n{len(failed)} FAILED: {' '.join(failed)}  (out of range? read-only? link?)")
        return 2
    print(f"\n{len(changes)} params written and verified. Dump again and ap_params.py diff to confirm.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(1)
