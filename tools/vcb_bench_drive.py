#!/usr/bin/env python3
"""Spin the thrusters from the Jetson through the VCB console ('bench' mode), no Pixhawk
involved. Run ON the Jetson, PROPS OFF. Needs firmware/VCB-pixhawk >= 0.3.0 and the
vcb-logger service (this tool only writes to the port and watches vcb.json).

    python3 tools/vcb_bench_drive.py --right 50 --left 0 --seconds 5    # right motor, 5 % forward
    python3 tools/vcb_bench_drive.py --right -50 --left -50 --seconds 5 # both, 5 % reverse
    python3 tools/vcb_bench_drive.py --right 0 --left 0 --seconds 5     # arm, hold zero in RUN, disarm
    python3 tools/vcb_bench_drive.py --right 50 --left 0 --seconds 3 --flip  # +5 % 3 s, then -5 % 3 s directly
    python3 tools/vcb_bench_drive.py --stop      # 'stop': fault 9, contactors open, latched
    python3 tools/vcb_bench_drive.py --clear     # un-latch after 'stop'
    python3 tools/vcb_bench_drive.py --off       # 'bench off' (back to the PWM inputs)

Sequence of a run: 'bench on' -> 'bench 0 0' at 10 Hz until the VCB arms and reaches RUN
(it closes the contactors by itself, 1 s after the command stream is at zero) -> ramp to the
target over --ramp s -> hold --seconds -> (with --flip: jump to the opposite sign, hold again)
-> ramp down -> zero for 1 s -> 'bench off'.
Aborts (zero, then 'bench off') on Ctrl-C, on any fault code, if the VCB leaves RUN, or if
vcb.json goes stale. --max (default 100 permille = 10 %) is this tool's cap; the firmware
has the same cap (BENCH_LIMIT_PERMILLE) and the usual THRUST_LIMIT_PERMILLE above it.
"""
import argparse
import json
import signal
import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit("pip install pyserial")

RUN, FAULT = 5, 0


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class Vcb:
    def __init__(self, a):
        self.a = a
        self.ser = serial.Serial(a.port, a.baud, timeout=0)
        self.sent = 0

    def send(self, line):
        self.ser.write((line + "\n").encode("ascii"))
        self.ser.flush()
        self.sent += 1

    def state(self, fresh_s=2.0):
        """(status, pwm, last_event) or (None, None, why) when the logger data is unusable."""
        v = read_json(self.a.vcb)
        if not v or not v.get("online"):
            return None, None, "VCB offline (%s)" % (v or {}).get("msg", "no vcb.json")
        if time.time() - (v.get("ts") or 0) > fresh_s:
            return None, None, "vcb.json stale"
        return v.get("status") or {}, v.get("pwm") or {}, v.get("last_event")

    def wait_for(self, pred, timeout, keepalive=None, label=""):
        """Poll until pred(status, pwm) or timeout; send keepalive at 10 Hz meanwhile."""
        end, last_print = time.time() + timeout, None
        while time.time() < end:
            if keepalive is not None:
                self.send(keepalive)
            st, pw, ev = self.state()
            if st is None:
                return None, ev
            if st.get("fault_code"):
                return None, "fault %s (%s)" % (st.get("fault_code"), ev)
            key = (st.get("thruster_state"), st.get("armed"), st.get("cmd_source"))
            if key != last_print:
                print("  %s state=%s armed=%s source=%s bus=%.1fV  %s" % (
                    label, key[0], key[1], key[2], (st.get("bus_mv") or 0) / 1000.0, ev or ""))
                last_print = key
            if pred(st, pw):
                return st, None
            time.sleep(0.1)
        return None, "timeout waiting for %s" % label


def battery_line(path):
    b = read_json(path)
    if not b:
        return ""
    parts = ["%s=%s" % (k, v) for k, v in b.items()
             if isinstance(v, (int, float)) and any(s in k.lower() for s in ("current", "amp", "volt", "soc"))]
    return " ".join(parts[:4])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/vcb")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--vcb", default="/home/boat/vcb.json")
    ap.add_argument("--battery", default="/home/boat/battery.json")
    ap.add_argument("--right", type=int, default=0, help="right motor (VCB ch1) permille, -max..max")
    ap.add_argument("--left", type=int, default=0, help="left motor (VCB ch2) permille")
    ap.add_argument("--seconds", type=float, default=5.0, help="hold time at the target")
    ap.add_argument("--ramp", type=float, default=1.0, help="ramp up / down time, s")
    ap.add_argument("--max", type=int, default=100, help="refuse |permille| above this (firmware caps at BENCH_LIMIT_PERMILLE too)")
    ap.add_argument("--flip", action="store_true", help="after the hold, command the opposite sign directly for another --seconds (reversal test)")
    ap.add_argument("--yes", action="store_true", help="skip the props-off confirmation")
    ap.add_argument("--stop", action="store_true", help="send 'stop' and exit")
    ap.add_argument("--clear", action="store_true", help="send 'clear' and exit")
    ap.add_argument("--off", action="store_true", help="send 'bench off' and exit")
    a = ap.parse_args()

    vcb = Vcb(a)
    if a.stop or a.clear or a.off:
        cmd = "stop" if a.stop else "clear" if a.clear else "bench off"
        vcb.send(cmd)
        time.sleep(0.5)
        st, pw, ev = vcb.state()
        print("sent '%s'; VCB: %s" % (cmd, ev if st is None else "state=%s armed=%s fault=%s source=%s  %s" % (
            st.get("thruster_state"), st.get("armed"), st.get("fault_code"), st.get("cmd_source"), ev or "")))
        return 0

    if abs(a.right) > a.max or abs(a.left) > a.max:
        sys.exit("refusing: |permille| above --max %d" % a.max)
    if a.right == 0 and a.left == 0:
        print("zero run: arm, hold 0/0 in RUN for %.1fs, 'bench off' (nothing should turn)" % a.seconds)
    if not a.yes:
        if input("Props OFF and everyone clear of the motors? type yes: ").strip().lower() != "yes":
            sys.exit("aborted")

    st, pw, ev = vcb.state()
    if st is None:
        sys.exit("cannot see the VCB: %s (is vcb-logger running?)" % ev)
    print("VCB: state=%s armed=%s fault=%s source=%s bus=%.1fV  %s" % (
        st.get("thruster_state"), st.get("armed"), st.get("fault_code"), st.get("cmd_source"),
        (st.get("bus_mv") or 0) / 1000.0, ev or ""))
    if "cmd_source" not in st:
        sys.exit("this VCB firmware has no bench mode (no cmd_source in the status line)")

    aborted = {"why": None}

    def abort(why):
        aborted["why"] = why
        for _ in range(5):
            vcb.send("bench 0 0")
            time.sleep(0.05)
        vcb.send("bench off")

    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

    try:
        vcb.send("bench on")
        st, err = vcb.wait_for(lambda s, p: s.get("cmd_source") == 1, 2.0, label="bench on")
        if st is None:
            sys.exit("VCB did not enter bench mode: %s (it refuses while armed or running)" % err)

        print("arming: zero command stream until RUN (contactors close by themselves)")
        st, err = vcb.wait_for(lambda s, p: s.get("thruster_state") == RUN and s.get("armed"), 20.0,
                               keepalive="bench 0 0", label="start-up")
        if st is None:
            abort(err)
            sys.exit("not in RUN: %s" % err)

        def drive(r, l):
            vcb.send("bench %d %d" % (r, l))
            st, pw, ev = vcb.state()
            if st is None:
                raise RuntimeError(ev)
            if st.get("fault_code") or st.get("thruster_state") != RUN:
                raise RuntimeError("VCB left RUN: state=%s fault=%s %s" % (st.get("thruster_state"), st.get("fault_code"), ev))
            return st, pw

        period, last_print = 0.1, 0.0
        t0 = time.time()
        hold2 = a.seconds if a.flip else 0.0
        total = a.ramp + a.seconds + hold2 + a.ramp
        print("driving: right %d left %d permille for %.1fs (ramp %.1fs each way)%s" % (
            a.right, a.left, a.seconds, a.ramp, "; then the opposite sign for %.1fs" % hold2 if a.flip else ""))
        while True:
            t = time.time() - t0
            if t >= total:
                break
            if t < a.ramp:
                k = t / a.ramp if a.ramp > 0 else 1.0
            elif t < a.ramp + a.seconds:
                k = 1.0
            elif t < a.ramp + a.seconds + hold2:
                k = -1.0                                   # --flip: direct reversal, the VCB inserts its zero hold
            else:
                k = max(0.0, (total - t) / a.ramp) if a.ramp > 0 else 0.0
                if a.flip:
                    k = -k
            st, pw = drive(int(round(a.right * k)), int(round(a.left * k)))
            if t - last_print >= 0.5:
                last_print = t
                print("  t=%4.1fs cmd=%s/%s bus=%.1fV raw=%s/%s %s" % (
                    t, pw.get("ch1_cmd"), pw.get("ch2_cmd"), (st.get("bus_mv") or 0) / 1000.0,
                    pw.get("ch1_raw_permille"), pw.get("ch2_raw_permille"), battery_line(a.battery)))
            time.sleep(period)
        for _ in range(10):
            drive(0, 0)
            time.sleep(0.1)
        vcb.send("bench off")
        print("done: zero sent, 'bench off' sent (contactors opened, VCB back to IDLE on PWM)")
        return 0
    except KeyboardInterrupt:
        abort("Ctrl-C")
    except RuntimeError as e:
        abort(str(e))
    print("ABORTED: %s (zero + 'bench off' sent)" % aborted["why"])
    return 1


if __name__ == "__main__":
    sys.exit(main())
