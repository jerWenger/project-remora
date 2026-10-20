#!/usr/bin/env python3
"""Dead-bus check of the Pixhawk -> VCB PWM contract. Run ON the Jetson:

    python3 tools/vcb_bench_check.py                       # default steps 1000..1300 us
    python3 tools/vcb_bench_check.py --pwm 1000,1100,1200 --hold 2

Uses ArduPilot motor test (MAV_CMD_DO_MOTOR_TEST, PWM mode) to drive ThrottleLeft and
ThrottleRight one at a time, and reads what the VCB says it measured and would apply
(vcb.json from vcb_logger.py). Answers: which VCB channel each Pixhawk output reaches,
the us -> permille curve, and where the clamp is.

Deployed forward-only build: (us - 1000) o/oo clamped at 1200 us = 200 o/oo; the default
steps above cover it. firmware/VCB-pixhawk (reversible, 1500 us = stop): use the step list
in its README, e.g. --pwm 1500,1550,1600,1620,1700,1450,1400,1380,1300,1000,2000 and the
end-point cases 990,1010,1990,2010; it also reports ch1_cmd/ch2_cmd, the signed value byte
the VCB last sent to the ePropulsion converter (0 unless the VCB is in RUN).

--live (stage 4 of docs/VCB_COMMISSIONING.md, PROPS OFF): the opposite precondition. The VCB
must be in RUN with no fault, and the motors will turn. Asks for a typed confirmation; aborts
to 1500 us if the VCB leaves RUN or faults. --side right|left|both picks the motor:

    python3 tools/vcb_bench_check.py --live --side right --pwm 1550 --hold 2
    python3 tools/vcb_bench_check.py --live --side right --pwm 2000 --hold 30   # end-point hold
    python3 tools/vcb_bench_check.py --live --side right --pwm 1620,1380 --hold 3   # direct reversal

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


def live_safe(v):
    """(ok, reason) for --live: fresh VCB data, VCB in RUN, no fault."""
    if not v or not v.get("online"):
        return False, "VCB offline (%s)" % (v or {}).get("msg", "no vcb.json")
    if time.time() - (v.get("ts") or 0) > 3:
        return False, "VCB data stale"
    st = v.get("status") or {}
    if st.get("fault_code"):
        return False, "VCB fault %s" % st.get("fault_code")
    if st.get("thruster_state") != 5:
        return False, "VCB not in RUN (thruster_state %s)" % st.get("thruster_state")
    return True, "VCB in RUN, bus %.1f V, no fault" % ((st.get("bus_mv") or 0) / 1000.0)


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
    ap.add_argument("--left-ch", type=int, default=10, help="Pixhawk output of ThrottleLeft (SITL: 1); display only")
    ap.add_argument("--right-ch", type=int, default=9, help="Pixhawk output of ThrottleRight (SITL: 3); display only")
    ap.add_argument("--side", choices=("left", "right", "both"), default="both", help="which motor(s) to command")
    ap.add_argument("--live", action="store_true", help="bus LIVE, motors will turn: require VCB in RUN instead of a dead bus")
    ap.add_argument("--yes", action="store_true", help="with --live: skip the typed props-off confirmation")
    a = ap.parse_args()
    steps = [int(x) for x in a.pwm.split(",")]

    check = live_safe if a.live else (lambda v: bus_safe(v, a.max_bus_v))
    ok, why = check(read_vcb(a.vcb))
    print("safety (%s): %s" % ("LIVE, motors will turn" if a.live else "dead bus", why))
    if not ok:
        sys.exit("refusing to run: %s" % why)
    if a.live and not a.yes:
        if input("Props OFF and everyone clear of the motors? type yes: ").strip().lower() != "yes":
            sys.exit("aborted")

    m = mavutil.mavlink_connection(a.conn, source_system=254, source_component=190)
    # Through mavlink-router we also see QGC's heartbeats (QGC flags itself "armed"):
    # only accept the autopilot's own, and target that system.
    hb, end = None, time.time() + 10
    while time.time() < end:
        h = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if h and h.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID \
                and h.type != mavutil.mavlink.MAV_TYPE_GCS:
            hb = h
            break
    if hb is None:
        sys.exit("no autopilot heartbeat on %s" % a.conn)
    m.target_system = hb.get_srcSystem()
    m.target_component = hb.get_srcComponent() or 1
    print("autopilot: sysid %d compid %d type %d base_mode 0x%02x" % (
        m.target_system, hb.get_srcComponent(), hb.type, hb.base_mode))
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        sys.exit("vehicle is ARMED (per that heartbeat): refusing")

    rows = []
    sides = [s for s in (("LEFT", MOTOR_TEST_THROTTLE_LEFT, a.left_ch),
                         ("RIGHT", MOTOR_TEST_THROTTLE_RIGHT, a.right_ch))
             if a.side == "both" or s[0].lower() == a.side]
    for name, seq, ch in sides:
        for pwm in steps:
            ok, why = check(read_vcb(a.vcb))
            if not ok:
                if a.live:
                    motor_test(m, seq, 1500, 1)       # back to neutral now, not at the timeout
                sys.exit("ABORT: %s" % why)       # motor test times out on its own
            res = motor_test(m, seq, pwm, a.hold + 3)   # outlives the step so steps chain in one test
            if res != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                sys.exit("motor test %s %d us rejected (result=%s): check pre-arm messages" % (name, pwm, res))
            time.sleep(a.hold * 0.6)
            out = servo_out(m, ch)
            v = read_vcb(a.vcb) or {}
            p = v.get("pwm") or {}
            rows.append((name, pwm, out,
                         p.get("ch1_us"), p.get("ch1_applied_permille"), p.get("ch1_valid"),
                         p.get("ch2_us"), p.get("ch2_applied_permille"), p.get("ch2_valid"),
                         p.get("ch1_raw_permille"), p.get("ch2_raw_permille"),
                         p.get("ch1_cmd"), p.get("ch2_cmd")))   # raw/cmd: VCB-pixhawk build only
            time.sleep(max(0.0, a.hold * 0.4))
        time.sleep(a.hold + 4)                    # let the test time out and disarm

    print("\n%-6s %6s %7s | %7s %8s %5s | %7s %8s %5s | %7s %7s %7s %7s" % (
        "test", "us", "PX out", "ch1 us", "ch1 o/oo", "valid", "ch2 us", "ch2 o/oo", "valid",
        "ch1 raw", "ch2 raw", "ch1 cmd", "ch2 cmd"))
    for r in rows:
        print("%-6s %6d %7s | %7s %8s %5s | %7s %8s %5s | %7s %7s %7s %7s" % tuple("-" if x is None else x for x in r))

    # Which VCB channel followed each side: the one whose measured us tracks the command.
    def follower(side):
        pts = [r for r in rows if r[0] == side and r[3] is not None and r[6] is not None]
        if len(pts) < 2 or max(r[1] for r in pts) - min(r[1] for r in pts) < 50:
            return None
        span = max(r[1] for r in pts) - min(r[1] for r in pts)
        moved = [ch for ch, i in (("ch1", 3), ("ch2", 6))
                 if max(r[i] for r in pts) - min(r[i] for r in pts) > span / 2]
        return moved[0] if len(moved) == 1 else None
    fl, fr = follower("LEFT"), follower("RIGHT")
    print("\nLEFT (ThrottleLeft) reached VCB %s; RIGHT (ThrottleRight) reached VCB %s" % (fl, fr))
    if (fl, fr) == ("ch2", "ch1"):
        print("OK: matches the VCB mapping (ch1 = right motor, ch2 = left motor)")
    elif (fl, fr) == ("ch1", "ch2"):
        print("SWAPPED: swap the ThrottleLeft/Right functions (73<->74) of the two outputs, or the PWM leads")
    else:
        print("UNCLEAR: inspect the table")


if __name__ == "__main__":
    main()
