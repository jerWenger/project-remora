#!/usr/bin/env python3
"""Dead-bus check of the Pixhawk -> VCB PWM contract. Run ON the Jetson:

    python3 tools/vcb_bench_check.py                       # default steps 1000..1300 us
    python3 tools/vcb_bench_check.py --pwm 1000,1100,1200 --hold 2

Uses ArduPilot motor test (MAV_CMD_DO_MOTOR_TEST, PWM mode) to drive ThrottleLeft and
ThrottleRight one at a time, and reads what the VCB says it measured and would apply
(vcb.json from vcb_logger.py). Answers: which VCB channel each Pixhawk output reaches,
the us -> permille curve, and whether the 1200 us clamp is real.

SAFETY: refuses to start, and aborts mid-test, unless the VCB reports the HV bus dead
(< --max-bus-v) and both contactors open. With the bus dead nothing can turn. Motor test
arms the Pixhawk briefly, so pre-arm checks must pass (a temporary ARMING_SKIPCHK compass
bit is fine for this; clear it afterwards).
"""
import argparse
import json
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    raise SystemExit("pip install pymavlink")

MOTOR_TEST_THROTTLE_LEFT, MOTOR_TEST_THROTTLE_RIGHT = 3, 4
MOTOR_TEST_THROTTLE_PWM = 1


def read_vcb(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def bus_safe(v, max_bus_v):
    """(ok, reason). Safe = fresh VCB data, bus below max_bus_v, both contactors open."""
    if not v or not v.get("online"):
        return False, "VCB offline (%s)" % (v or {}).get("msg", "no vcb.json")
    if time.time() - (v.get("ts") or 0) > 3:
        return False, "VCB data stale"
    st = v.get("status") or {}
    if "bus_mv" not in st:
        return False, "no bus voltage in VCB status"
    if st["bus_mv"] / 1000.0 >= max_bus_v:
        return False, "HV bus live: %.1f V" % (st["bus_mv"] / 1000.0)
    if st.get("lsc_closed") or st.get("hsc_closed"):
        return False, "contactor closed (LSC=%s HSC=%s)" % (st.get("lsc_closed"), st.get("hsc_closed"))
    return True, "bus %.1f V, contactors open" % (st["bus_mv"] / 1000.0)


def servo_out(m, ch, timeout=1.0):
    """Latest SERVO_OUTPUT_RAW value for output ch (1-16), or None."""
    end, val = time.time() + timeout, None
    while time.time() < end:
        s = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.2)
        if s is None:
            continue
        if ch <= 8 and getattr(s, "port", 0) == 0:
            val = getattr(s, "servo%d_raw" % ch)
        elif ch > 8:
            v = getattr(s, "servo%d_raw" % ch, None)       # MAVLink2 extension fields
            if v is None and getattr(s, "port", 0) == 1:
                v = getattr(s, "servo%d_raw" % (ch - 8))
            if v is not None:
                val = v
    return val


def motor_test(m, seq, pwm, timeout_s):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                            seq, MOTOR_TEST_THROTTLE_PWM, pwm, timeout_s, 0, 0, 0)
    end = time.time() + 3
    while time.time() < end:
        a = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=0.5)
        if a is None:
            continue
        if a.get_type() == "STATUSTEXT":
            print("   autopilot: %s" % a.text)
        elif a.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            return a.result
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default="tcp:127.0.0.1:5760")
    ap.add_argument("--vcb", default="/home/boat/vcb.json")
    ap.add_argument("--pwm", default="1000,1050,1100,1150,1200,1300", help="comma list of us")
    ap.add_argument("--hold", type=float, default=2.0, help="seconds per step")
    ap.add_argument("--max-bus-v", type=float, default=5.0)
    ap.add_argument("--left-ch", type=int, default=9, help="Pixhawk output of ThrottleLeft (SITL: 1)")
    ap.add_argument("--right-ch", type=int, default=10, help="Pixhawk output of ThrottleRight (SITL: 3)")
    a = ap.parse_args()
    steps = [int(x) for x in a.pwm.split(",")]

    ok, why = bus_safe(read_vcb(a.vcb), a.max_bus_v)
    print("safety: %s" % why)
    if not ok:
        sys.exit("refusing to run: %s" % why)

    m = mavutil.mavlink_connection(a.conn, source_system=254, source_component=190)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        sys.exit("no heartbeat on %s" % a.conn)
    if m.target_component == 0:
        m.target_component = 1
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        sys.exit("vehicle is ARMED: refusing")

    rows = []
    for name, seq, ch in (("LEFT", MOTOR_TEST_THROTTLE_LEFT, a.left_ch),
                          ("RIGHT", MOTOR_TEST_THROTTLE_RIGHT, a.right_ch)):
        for pwm in steps:
            ok, why = bus_safe(read_vcb(a.vcb), a.max_bus_v)
            if not ok:
                sys.exit("ABORT: %s" % why)       # motor test times out on its own
            res = motor_test(m, seq, pwm, a.hold + 3)   # outlives the step so steps chain in one test
            if res != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                sys.exit("motor test %s %d us rejected (result=%s): check pre-arm messages" % (name, pwm, res))
            time.sleep(a.hold * 0.6)
            out = servo_out(m, ch)
            v = read_vcb(a.vcb) or {}
            p = v.get("pwm") or {}
            rows.append((name, pwm, out,
                         p.get("ch1_us"), p.get("ch1_applied_permille"),
                         p.get("ch2_us"), p.get("ch2_applied_permille")))
            time.sleep(max(0.0, a.hold * 0.4))
        time.sleep(a.hold + 4)                    # let the test time out and disarm

    print("\n%-6s %6s %7s | %8s %8s | %8s %8s" % ("test", "cmd", "PX out", "ch1 us", "ch1 o/oo", "ch2 us", "ch2 o/oo"))
    for r in rows:
        print("%-6s %6d %7s | %8s %8s | %8s %8s" % tuple("-" if x is None else x for x in r))

    # Which VCB channel followed each side: the one whose measured us tracks the command.
    def follower(side):
        pts = [r for r in rows if r[0] == side and r[3] is not None and r[5] is not None]
        if len(pts) < 2 or max(r[1] for r in pts) - min(r[1] for r in pts) < 50:
            return None
        span = max(r[1] for r in pts) - min(r[1] for r in pts)
        moved = [ch for ch, i in (("ch1", 3), ("ch2", 5))
                 if max(r[i] for r in pts) - min(r[i] for r in pts) > span / 2]
        return moved[0] if len(moved) == 1 else None
    fl, fr = follower("LEFT"), follower("RIGHT")
    print("\nLEFT (ThrottleLeft) reached VCB %s; RIGHT (ThrottleRight) reached VCB %s" % (fl, fr))
    if (fl, fr) == ("ch2", "ch1"):
        print("OK: matches the VCB mapping (ch1 = right motor, ch2 = left motor)")
    elif (fl, fr) == ("ch1", "ch2"):
        print("SWAPPED: swap SERVO9/SERVO10 FUNCTION (73<->74) or the two PWM leads")
    else:
        print("UNCLEAR: inspect the table")


if __name__ == "__main__":
    main()
