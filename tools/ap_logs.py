#!/usr/bin/env python3
"""List / download / erase ArduPilot dataflash logs over MAVLink (LOG_REQUEST_*).

    ./ap_logs.py list
    ./ap_logs.py get --latest 1                     # newest log -> logs/ap/<UTC time>_<id>.BIN
    ./ap_logs.py get 12 13                          # by id
    ./ap_logs.py get --all --out /tmp/aplogs
    ./ap_logs.py --conn tcp:127.0.0.1:5760 list     # on the Jetson (much faster than the tailnet)
    ./ap_logs.py erase --yes-really                 # wipe the SD card logs (disarmed only)

Built for the slow DERP-relayed link: received 90-byte chunks are tracked, holes are
re-requested after a stall, partial downloads go to <name>.part and finished files
whose size matches LOG_ENTRY are skipped. The vehicle pauses logging while a
download is in progress; LOG_REQUEST_END at the end resumes it.
Open the .BIN in https://plot.ardupilot.org or MAVExplorer.py.
Requires: pip install pymavlink
"""
import argparse
import glob
import os
import sys
import time

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mav
except ImportError:
    raise SystemExit("pip install pymavlink")

DEFAULT_CONN = "tcp:highfieldboat.tailcacf9.ts.net:5760"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHUNK = 90  # LOG_DATA payload
MERGE_GAP = 64  # chunks: re-request across received runs shorter than this


def connect(conn, baud):
    print(f"connecting to {conn} ...", flush=True)
    m = mavutil.mavlink_connection(conn, baud=baud, source_system=250, source_component=191)
    if hasattr(m, "handle_eof"):  # pymavlink would otherwise spin printing "EOF on TCP socket"
        def eof():
            raise SystemExit("link closed by peer (router restarted? SITL died?)")
        m.handle_eof = eof
    t0 = time.time()
    while time.time() - t0 < 20:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.autopilot != mav.MAV_AUTOPILOT_INVALID and hb.type != mav.MAV_TYPE_GCS:
            m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            return m, hb
    raise SystemExit("no autopilot heartbeat in 20 s (router down? wrong host? Pixhawk unplugged?)")


def armed(hb):
    return bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)


def list_logs(m, retries=5, timeout=3):
    """-> {id: LOG_ENTRY}. Re-requests ids that went missing on the link."""
    entries, num, last_id = {}, None, None
    m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)
    for attempt in range(retries + 1):
        last = time.time()
        while time.time() - last < timeout:
            e = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=0.5)
            if not e:
                continue
            last = time.time()
            num, last_id = e.num_logs, e.last_log_num
            if e.num_logs == 0:
                return {}
            entries[e.id] = e
            if len(entries) >= num:
                return entries
        if num is None:
            m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)
            continue
        # ids are normally contiguous and end at last_log_num
        missing = [i for i in range(last_id - num + 1, last_id + 1) if i not in entries]
        print(f"  have {len(entries)}/{num} entries, re-requesting {len(missing)}")
        for i in missing:
            m.mav.log_request_list_send(m.target_system, m.target_component, i, i)
            time.sleep(0.05)
    if num is None:
        raise SystemExit("no LOG_ENTRY reply (no SD card? logging disabled?)")
    return entries


def log_name(e):
    t = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime(e.time_utc)) if e.time_utc else "notime"
    return f"{t}_{e.id}.BIN"


def missing_ranges(have, nchunks):
    """[(first_chunk, n_chunks)] of chunks not yet received."""
    out, start = [], None
    for i in range(nchunks):
        if i not in have and start is None:
            start = i
        elif i in have and start is not None:
            out.append((start, i - start))
            start = None
    if start is not None:
        out.append((start, nchunks - start))
    return out


def download(m, e, path, stall=2.0, max_stalls=30):
    size = e.size
    nchunks = (size + CHUNK - 1) // CHUNK
    buf, have = bytearray(size), set()
    t0, stalls, shown = time.time(), 0, 0
    while True:
        holes = missing_ranges(have, nchunks)
        if not holes:
            break
        # first pass: one hole = the whole file. Later passes: span the first hole plus any
        # holes close behind it (re-sending a few chunks we have beats a round trip per hole)
        first, n = holes[0]
        want = n
        for h0, hn in holes[1:]:
            if h0 - (first + n) > MERGE_GAP:
                break
            n, want = h0 + hn - first, want + hn
        m.mav.log_request_data_send(m.target_system, m.target_component, e.id, first * CHUNK, n * CHUNK)
        last, progressed, left = time.time(), False, want
        while time.time() - last < stall:
            d = m.recv_match(type="LOG_DATA", blocking=True, timeout=0.5)
            if not d or d.id != e.id:
                continue
            if d.count == 0 or d.ofs >= size:
                break  # past the end
            last = time.time()
            idx = d.ofs // CHUNK
            if d.ofs % CHUNK == 0 and idx not in have:
                cnt = min(d.count, size - d.ofs)
                buf[d.ofs:d.ofs + cnt] = bytes(d.data[:cnt])
                have.add(idx)
                progressed = True
                left -= first <= idx < first + n
            if left <= 0 or idx >= first + n - 1:
                break  # span filled, or the vehicle streamed past its end (rest was dropped)
            if time.time() - shown > 1:
                shown = time.time()
                got = len(have) * CHUNK
                rate = got / max(1e-3, shown - t0)
                print(f"  {min(got, size)/1024:8.0f}/{size/1024:.0f} KiB  {rate/1024:6.1f} KiB/s"
                      f"  holes={len(missing_ranges(have, nchunks))}", end="\r", flush=True)
        stalls = 0 if progressed else stalls + 1
        if stalls >= max_stalls:
            print(f"\n  giving up on log {e.id}: no data for {stalls} requests")
            with open(path + ".part", "wb") as f:
                f.write(buf)
            return False
    with open(path + ".part", "wb") as f:
        f.write(buf)
    os.replace(path + ".part", path)
    dt = time.time() - t0
    print(f"  {size/1024:8.0f} KiB in {dt:.0f} s ({size/1024/max(dt, 1e-3):.1f} KiB/s) -> {path}" + " " * 20)
    return True


def print_table(entries):
    print(f"{'ID':>4s} {'SIZE':>10s}  UTC")
    for i in sorted(entries):
        e = entries[i]
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(e.time_utc)) if e.time_utc else "(no time)"
        print(f"{e.id:4d} {e.size:10d}  {t}")
    print(f"{len(entries)} logs, {sum(e.size for e in entries.values())/1e6:.1f} MB total")


def cmd_get(m, a):
    entries = list_logs(m)
    if not entries:
        print("no logs on the vehicle")
        return 0
    if a.all:
        ids = sorted(entries)
    elif a.latest:
        ids = sorted(entries)[-a.latest:]
    else:
        ids = a.ids
    bad = [i for i in ids if i not in entries]
    if bad:
        raise SystemExit(f"no such log id(s): {bad}; have {sorted(entries)}")
    os.makedirs(a.out, exist_ok=True)
    failed = []
    for i in ids:
        e = entries[i]
        path = os.path.join(a.out, log_name(e))
        # LOG_ENTRY time is the file's mtime and can shift by a second, so match id + size
        done = [p for p in glob.glob(os.path.join(a.out, f"*_{i}.BIN")) if os.path.getsize(p) == e.size]
        if done:
            print(f"log {i}: already have {done[0]}")
            continue
        if e.size == 0:
            print(f"log {i}: empty, skipped")
            continue
        print(f"log {i}: {e.size/1024:.0f} KiB")
        if not download(m, e, path):
            failed.append(i)
    m.mav.log_request_end_send(m.target_system, m.target_component)
    if failed:
        print(f"FAILED: {failed} (re-run to retry; .part files hold what arrived)")
        return 2
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default=DEFAULT_CONN, help="pymavlink connection string")
    ap.add_argument("--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="table of id, size, UTC time")
    g = sub.add_parser("get", help="download logs")
    g.add_argument("ids", nargs="*", type=int)
    g.add_argument("--latest", type=int, metavar="N", help="newest N logs")
    g.add_argument("--all", action="store_true")
    g.add_argument("--out", default=os.path.join(REPO, "logs", "ap"), help="output dir (default logs/ap)")
    er = sub.add_parser("erase", help="erase ALL logs on the vehicle")
    er.add_argument("--yes-really", action="store_true")
    a = ap.parse_args()
    if a.cmd == "get" and not (a.ids or a.latest or a.all):
        ap.error("get: give log ids, --latest N or --all")
    if a.cmd == "erase" and not a.yes_really:
        ap.error("erase deletes every log on the SD card; add --yes-really")

    m, hb = connect(a.conn, a.baud)
    if a.cmd == "list":
        print_table(list_logs(m))
        m.mav.log_request_end_send(m.target_system, m.target_component)
        return 0
    if a.cmd == "get":
        return cmd_get(m, a)
    if armed(hb):
        raise SystemExit("vehicle is ARMED: refusing to erase logs")
    m.mav.log_erase_send(m.target_system, m.target_component)
    print("LOG_ERASE sent; the vehicle erases in the background (can take a while on a big SD card)")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(1)
