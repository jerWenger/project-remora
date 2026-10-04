#!/usr/bin/env python3
"""smoke_test.py -- end-to-end check of the HighFieldBoat SITL (sim/run_sitl.sh).

Listens where the dashboard listens (udpin 0.0.0.0:14552+10*I), checks
heartbeat / GPS / EKF, opens the QGC-style TCP port (5760+10*I) at the same
time, then -- ONLY after proving the vehicle is SITL -- switches to GUIDED,
force-arms, sends a SET_POSITION_TARGET_GLOBAL_INT ~50 m away, confirms the
boat closes on it, then HOLD + disarm.

SAFETY: before sending any command it reads SIM_SPEEDUP. That parameter only
exists in SITL builds; a real Pixhawk does not have it. No answer => no
commands, exit status 2. Do not remove this guard.

Stop the dashboard (or anything else bound to the UDP port) first: only one
process can listen on it.

Usage: .venv/bin/python sim/smoke_test.py [-I N]
"""
import argparse
import math
import socket
import sys
import time

from pymavlink import mavutil

MODE_HOLD = 4
MODE_GUIDED = 15
FORCE_MAGIC = 21196          # param2 of COMPONENT_ARM_DISARM: skip checks (SITL only)

EKF_POS_HORIZ_ABS = mavutil.mavlink.EKF_POS_HORIZ_ABS
EKF_CONST_POS_MODE = mavutil.mavlink.EKF_CONST_POS_MODE

results = []


def report(name, ok, detail=""):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""), flush=True)
    return ok


def info(msg):
    print(f"       {msg}", flush=True)


def wait_msg(m, types, timeout, cond=None):
    """Return the first message of `types` from the autopilot satisfying cond, or None."""
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type=types, blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_srcSystem() != m.target_system or msg.get_srcComponent() != m.target_component:
            continue
        if cond is None or cond(msg):
            return msg
    return None


def wait_autopilot_heartbeat(m, timeout):
    """Wait for a HEARTBEAT from an autopilot (not a GCS / peripheral) and target it."""
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb is None or hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            continue
        m.target_system = hb.get_srcSystem()
        m.target_component = hb.get_srcComponent() or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        return hb
    return None


def read_param(m, name, timeout=2.0, tries=3):
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        msg = wait_msg(m, "PARAM_VALUE", timeout,
                       lambda p: p.param_id.rstrip("\x00") == name)
        if msg is not None:
            return msg.param_value
    return None


def set_mode(m, mode, timeout=5.0):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 0, 0, 0, 0, 0)
    return wait_msg(m, "HEARTBEAT", timeout, lambda h: h.custom_mode == mode) is not None


def arm(m, armed, timeout=8.0):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1 if armed else 0, FORCE_MAGIC, 0, 0, 0, 0, 0)
    want = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return wait_msg(m, "HEARTBEAT", timeout,
                    lambda h: bool(h.base_mode & want) == armed) is not None


def request_interval(m, msg_id, hz):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msg_id, 1e6 / hz, 0, 0, 0, 0, 0)


def offset(lat, lon, north_m, east_m):
    dlat = north_m / 6378137.0
    dlon = east_m / (6378137.0 * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def dist_m(lat1, lon1, lat2, lon2):
    dn = math.radians(lat2 - lat1) * 6378137.0
    de = math.radians(lon2 - lon1) * 6378137.0 * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def send_target(m, lat, lon):
    # position-only mask: ignore velocity, accel, yaw, yaw rate
    mask = 0b0000110111111000
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
        int(lat * 1e7), int(lon * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-I", "--instance", type=int, default=0, help="SITL instance (ports +10*I)")
    ap.add_argument("--udp-port", type=int, help="override UDP listen port (default 14552+10*I)")
    ap.add_argument("--tcp-port", type=int, help="override TCP port (default 5760+10*I)")
    ap.add_argument("--distance", type=float, default=50.0, help="guided target distance, m")
    ap.add_argument("--bearing", type=float, default=270.0,
                    help="guided target bearing, deg (default 270 = upriver/west, stays on the water)")
    args = ap.parse_args()

    udp_port = args.udp_port or 14552 + 10 * args.instance
    tcp_port = args.tcp_port or 5760 + 10 * args.instance
    print(f"smoke_test: udpin:0.0.0.0:{udp_port}  tcp:127.0.0.1:{tcp_port}")

    # ---- 1. heartbeat on the dashboard UDP port --------------------------
    m = mavutil.mavlink_connection(f"udpin:0.0.0.0:{udp_port}", source_system=253, source_component=190)
    hb = wait_autopilot_heartbeat(m, 15)
    if not report("heartbeat on UDP", hb is not None,
                  hb and f"sysid {m.target_system} type {hb.type} autopilot {hb.autopilot}"):
        return finish()
    report("vehicle is a boat (MAV_TYPE_SURFACE_BOAT)", hb.type == mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
           f"type {hb.type}")

    # ---- 2. SITL guard ----------------------------------------------------
    speedup = read_param(m, "SIM_SPEEDUP")
    if not report("vehicle is SITL (SIM_SPEEDUP readable)", speedup is not None,
                  f"SIM_SPEEDUP={speedup}" if speedup is not None else
                  "no SIM_SPEEDUP -- this may be a REAL vehicle; refusing to send any command"):
        m.close()
        finish()
        sys.exit(2)

    # ---- 3. streams arrive without being requested (MAV3_* rates) --------
    got = wait_msg(m, "GLOBAL_POSITION_INT", 5)
    report("default streams on UDP port (no request)", got is not None,
           "" if got else "set MAV3_* rates in boat-sitl.parm; requesting explicitly")
    for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5),
                       (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
                       (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
                       (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 2)):
        if not got:
            request_interval(m, msg_id, hz)

    # ---- 4. GPS fix ---------------------------------------------------------
    gps = wait_msg(m, "GPS_RAW_INT", 60, lambda g: g.fix_type >= 3)
    report("GPS 3D fix", gps is not None, gps and f"fix_type {gps.fix_type}, {gps.satellites_visible} sats")

    # ---- 5. EKF horizontal position ----------------------------------------
    ekf = wait_msg(m, "EKF_STATUS_REPORT", 90,
                   lambda e: (e.flags & EKF_POS_HORIZ_ABS) and not (e.flags & EKF_CONST_POS_MODE))
    report("EKF absolute horizontal position", ekf is not None, ekf and f"flags 0x{ekf.flags:x}")

    # ---- 6. TCP port accepts a connection at the same time ----------------
    tcp = None
    try:
        tcp = mavutil.mavlink_connection(f"tcp:127.0.0.1:{tcp_port}", source_system=252,
                                         source_component=190, autoreconnect=False)
        thb = wait_autopilot_heartbeat(tcp, 10)
        report("TCP connection alongside UDP", thb is not None, f"tcp:127.0.0.1:{tcp_port}")
    except (OSError, socket.error) as e:
        report("TCP connection alongside UDP", False, str(e))
        tcp = None

    if gps is None or ekf is None:
        info("no position -- skipping guided test")
        return finish(m, tcp)

    # ---- 7. GUIDED + force arm ---------------------------------------------
    report("mode GUIDED", set_mode(m, MODE_GUIDED))
    if not report("armed (force, SITL only)", arm(m, True)):
        return finish(m, tcp)

    # ---- 8. position target ~50 m away ------------------------------------
    pos = wait_msg(m, "GLOBAL_POSITION_INT", 5)
    lat0, lon0 = pos.lat / 1e7, pos.lon / 1e7
    b = math.radians(args.bearing)
    tlat, tlon = offset(lat0, lon0, args.distance * math.cos(b), args.distance * math.sin(b))
    d0 = dist_m(lat0, lon0, tlat, tlon)
    info(f"start {lat0:.6f},{lon0:.6f} -> target {tlat:.6f},{tlon:.6f} ({d0:.1f} m)")
    best = d0
    min_servo = 2000
    t_end = time.time() + 90
    last_send = 0
    last_print = 0
    while time.time() < t_end:
        now = time.time()
        if now - last_send > 1.0:
            send_target(m, tlat, tlon)
            last_send = now
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "SERVO_OUTPUT_RAW"], blocking=True, timeout=0.5)
        if msg is None or msg.get_srcSystem() != m.target_system:
            continue
        if msg.get_type() == "SERVO_OUTPUT_RAW":
            if m.motors_armed():
                min_servo = min(min_servo, msg.servo1_raw, msg.servo3_raw)
            continue
        d = dist_m(msg.lat / 1e7, msg.lon / 1e7, tlat, tlon)
        best = min(best, d)
        if now - last_print > 5:
            spd = math.hypot(msg.vx, msg.vy) / 100
            info(f"dist {d:5.1f} m  speed {spd:4.2f} m/s")
            last_print = now
        if d < 5:
            break
        if tcp is not None:
            tcp.recv_match(blocking=False)   # keep the TCP side drained
    report("boat moved toward target", best < d0 - 20,
           f"closest {best:.1f} m of {d0:.1f} m")
    info(f"lowest armed SERVO1/3 output {min_servo} us (forward-only => >= 1500)")

    # ---- 9. HOLD + disarm -----------------------------------------------------
    report("mode HOLD", set_mode(m, MODE_HOLD))
    report("disarmed", arm(m, False))

    if tcp is not None:
        thb = wait_autopilot_heartbeat(tcp, 5)
        report("TCP link still alive at end", thb is not None)
    return finish(m, tcp)


def finish(*conns):
    for c in conns:
        if c is not None:
            try:
                c.close()
            except Exception:
                pass
    n_fail = sum(1 for _, ok in results if not ok)
    print(f"\n{len(results) - n_fail}/{len(results)} checks passed -> {'PASS' if n_fail == 0 else 'FAIL'}")
    return n_fail


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
