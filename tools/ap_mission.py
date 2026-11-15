#!/usr/bin/env python3
"""Build a box, figure-8 or tuning waypoint mission for Rover Auto; write a QGC file, optionally upload.

    .venv/bin/python tools/ap_mission.py box  --size 50                  # write missions/box.waypoints
    .venv/bin/python tools/ap_mission.py fig8 --size 80 --laps 2 --upload
    .venv/bin/python tools/ap_mission.py box --center 42.35755,-71.08765 --rotate 20 --upload

Shapes are centred on --center (default: the middle of the 2026-11-12 driving area in the
basin) and rotated --rotate degrees clockwise from north.
  box:  square, side --size m, corners in clockwise order starting at the north-west corner.
  fig8: lemniscate --size m long (along the rotated N axis) and size/2 wide, 16 points.
        Lobe radius ~size/4, so keep size >= 4 x the turn radius (~9 m at 1.5 m/s): >= 60 m.
  tune: one run that exercises every loop (see shape_tune): a chamfered rectangle --size m long
        and size/2 wide, one lap per entry of --speeds (DO_CHANGE_SPEED at the start of each
        lap, so every speed is held on two opposite long legs: wind and current cancel, and the
        battery log gives W at each speed); all corners 45 deg, under WP_PIVOT_ANGLE. Then at
        --track-speed: a zigzag of three 30 deg heading steps down the centreline, one ~135 deg
        corner (the pivot test) and a last straight. Default 120 m: ~1.7 km, ~18 min.
--laps N repeats the pattern with DO_JUMP. After the last waypoint Rover holds (MIS_DONE_BEHAVE 0).
Speed comes from WP_SPEED. Item 0 is home (the vehicle overwrites it).
The .waypoints file loads in QGC (Plan > Open) if you prefer to upload from there.
Requires: pymavlink
"""
import argparse
import math
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CENTER = (42.35755, -71.08765)  # middle of the 2026-11-12 driving area
NAV_WAYPOINT, DO_JUMP, DO_CHANGE_SPEED = 16, 177, 178


def offset(lat0, lon0, north, east):
    return (lat0 + north / 111320.0,
            lon0 + east / (111320.0 * math.cos(math.radians(lat0))))


def shape(kind, size):
    """Points (north, east) in metres before rotation."""
    if kind == "box":
        h = size / 2
        return [(h, -h), (h, h), (-h, h), (-h, -h)]
    # lemniscate of Gerono, long axis north: n = a sin t, e = a sin t cos t
    a = size / 2
    return [(a * math.sin(t), a * math.sin(t) * math.cos(t))
            for t in (2 * math.pi * k / 16 + math.pi / 2 for k in range(16))]


def shape_tune(size, speeds, track_speed):
    """-> [("wp", n, e) | ("speed", m/s)], long axis north before rotation.

    Rectangle L x L/2 with the corners cut (chamfer L/12 per side), driven clockwise from the
    north-going leg on the west side. A DO command placed after a waypoint runs when the
    vehicle starts towards that waypoint, so each lap's speed is set on the way to R2 (the
    end of the first long leg)."""
    L, W, c = size, size / 2, size / 12
    a, b = L / 2, W / 2
    rect = [(-a + c, -b), (a - c, -b), (a, -b + c), (a, b - c),      # R1 R2 R3 R4
            (a - c, b), (-a + c, b), (-a, b - c), (-a, -b + c)]      # R5 R6 R7 R8
    seq = [("wp",) + rect[0]]
    for v in speeds:
        seq.append(("wp",) + rect[1])
        seq.append(("speed", v))
        seq += [("wp",) + p for p in rect[2:]] + [("wp",) + rect[0]]
    # tracking part: up the west leg again, half a turn onto the centreline, zigzag south
    seq.append(("wp",) + rect[1])
    seq.append(("speed", track_speed))
    seq += [("wp", a, -b + c), ("wp", a, -c), ("wp", a - c, 0.0)]
    dx, dy = L / 4, L / 15
    x = a - c
    for k in range(3):                       # three 30 deg heading steps
        x -= dx
        seq.append(("wp", x, dy if k % 2 == 0 else 0.0))
    seq.append(("wp", x + dx, b))            # ~135 deg corner: the pivot
    seq.append(("wp", a - c, b))             # last straight; mission ends here
    return seq


def build(kind, size, center, rotate, laps, speeds=(), track_speed=1.5):
    r = math.radians(rotate)

    def geo(n, e):
        nr, er = n * math.cos(r) - e * math.sin(r), n * math.sin(r) + e * math.cos(r)
        return offset(center[0], center[1], nr, er)

    items = [(0, NAV_WAYPOINT, 0, 0, center[0], center[1])]  # home placeholder
    if kind == "tune":
        for it in shape_tune(size, speeds, track_speed):
            if it[0] == "speed":
                items.append((len(items), DO_CHANGE_SPEED, 1, it[1], 0, 0))  # p1 1 = ground speed
            else:
                items.append((len(items), NAV_WAYPOINT, 0, 0) + geo(it[1], it[2]))
        return items
    pts = [geo(n, e) for n, e in shape(kind, size)]
    pts.append(pts[0])  # close the loop
    for lat, lon in pts:
        items.append((len(items), NAV_WAYPOINT, 0, 0, lat, lon))
    if laps > 1:
        items.append((len(items), DO_JUMP, 1, laps - 1, 0, 0))  # jump to seq 1, repeat laps-1 times
    return items


def write_wpl(items, path):
    with open(path, "w") as f:
        f.write("QGC WPL 110\n")
        for seq, cmd, p1, p2, lat, lon in items:
            cur = 1 if seq == 0 else 0
            frame = 0 if seq == 0 else 3
            f.write(f"{seq}\t{cur}\t{frame}\t{cmd}\t{p1}\t{p2}\t0\t0\t{lat:.8f}\t{lon:.8f}\t0\t1\n")


def upload(items, conn):
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mav
    m = mavutil.mavlink_connection(conn, source_system=250, source_component=190)
    t0 = time.time()
    while True:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.autopilot != mav.MAV_AUTOPILOT_INVALID and hb.type != mav.MAV_TYPE_GCS:
            break
        if time.time() - t0 > 20:
            raise SystemExit("no autopilot heartbeat")
    ts, tc = hb.get_srcSystem(), hb.get_srcComponent()
    print(f"autopilot {ts}/{tc}, {'ARMED' if hb.base_mode & 128 else 'disarmed'}; uploading {len(items)} items")
    sent_at = 0
    m.mav.mission_count_send(ts, tc, len(items), 0)
    t0 = time.time()
    while time.time() - t0 < 60:
        msg = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=2)
        if msg is None:
            if time.time() - sent_at > 3:
                m.mav.mission_count_send(ts, tc, len(items), 0)
                sent_at = time.time()
            continue
        if msg.get_srcSystem() != ts:
            continue
        if msg.get_type() == "MISSION_ACK":
            if msg.type == 0:
                print("mission accepted")
                return
            raise SystemExit(f"mission rejected, MAV_MISSION_RESULT {msg.type}")
        seq, cmd, p1, p2, lat, lon = items[msg.seq]
        frame = 0 if seq == 0 else 3  # GLOBAL / GLOBAL_RELATIVE_ALT
        m.mav.mission_item_int_send(ts, tc, seq, frame, cmd, 0, 1, p1, p2, 0, 0,
                                    int(lat * 1e7), int(lon * 1e7), 0, 0)
        sent_at = time.time()
    raise SystemExit("upload timed out (relayed link?) -- load the .waypoints file in QGC instead")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=["box", "fig8", "tune"])
    ap.add_argument("--size", type=float, default=None, help="m (default box 50, fig8 80, tune 120)")
    ap.add_argument("--center", default=None, help="lat,lon (default the 11-12 driving area)")
    ap.add_argument("--rotate", type=float, default=0.0, help="deg clockwise from north")
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--speeds", default="1.0,1.5,2.0,2.5", help="tune: m/s per rectangle lap")
    ap.add_argument("--track-speed", type=float, default=1.5, help="tune: m/s for the zigzag and pivot part")
    ap.add_argument("--out", default=None, help="QGC .waypoints path (default missions/<kind>.waypoints)")
    ap.add_argument("--upload", action="store_true")
    ap.add_argument("--conn", default="tcp:highfieldboat.tailcacf9.ts.net:5760")
    a = ap.parse_args()
    size = a.size or {"box": 50.0, "fig8": 80.0, "tune": 120.0}[a.kind]
    center = tuple(float(v) for v in a.center.split(",")) if a.center else DEFAULT_CENTER
    speeds = [float(v) for v in a.speeds.split(",")]
    items = build(a.kind, size, center, a.rotate, a.laps, speeds, a.track_speed)
    out = a.out or os.path.join(REPO, "missions", f"{a.kind}.waypoints")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    write_wpl(items, out)
    nav = [i for i in items[1:] if i[1] == NAV_WAYPOINT]
    kx = 111320 * math.cos(math.radians(center[0]))
    vec = [((nav[k][4] - nav[k - 1][4]) * 111320, (nav[k][5] - nav[k - 1][5]) * kx) for k in range(1, len(nav))]
    legs = sum(math.hypot(*v) for v in vec)
    print(f"{a.kind} {size:.0f} m at {center[0]:.6f},{center[1]:.6f}, rot {a.rotate:.0f}, {len(nav)} wps, "
          f"{legs:.0f} m per lap x {a.laps} -> {out}")
    if a.kind == "tune":
        turn = [abs((math.degrees(math.atan2(v[1], v[0]) - math.atan2(u[1], u[0])) + 180) % 360 - 180)
                for u, v in zip(vec, vec[1:])]
        sharp = [f"{t:.0f}" for t in turn if t > 59]
        lap = sum(math.hypot(*v) for v in vec[:8])
        secs = sum(lap / v for v in speeds) + (legs - lap * len(speeds)) / a.track_speed
        print(f"  laps at {a.speeds} m/s ({lap:.0f} m each), then tracking at {a.track_speed} m/s; ~{secs / 60:.0f} min; "
              f"turns: {len(turn) - len(sharp)} of <= {max(t for t in turn if t <= 59):.0f} deg, "
              f"{len(sharp)} sharp ({', '.join(sharp)} deg: pivots if WP_PIVOT_ANGLE is below that)")
    if a.upload:
        upload(items, a.conn)


if __name__ == "__main__":
    sys.exit(main())
