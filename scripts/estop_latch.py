#!/usr/bin/env python3
"""RC8 e-stop -> latched HV off at the VCB. Runs on the Jetson as estop-latch.service.

Layer 2 of the RC e-stop. Layer 1 is ArduPilot's own motor emergency stop on RC8
(RC8_OPTION 31): AUX1/AUX2 held at 1500 us within a frame, independent of the Jetson.
This service adds: while RC8 reads ENGAGED, send 'stop' to the VCB console until the VCB
reports fault 9 (contactors open, latched until 'clear'). Clearing RC8 does not restart
anything; run `estop_latch.py --clear` (alias vcbclear), which refuses while RC8 is engaged.

A software chain (receiver -> Pixhawk -> mavlink-router -> this -> USB -> VCB), not a
safety device: if any link is down, layer 1 still stops the motors but HV stays live.
No RC data (transmitter off, link lost) -> no action; ArduPilot's RC failsafe owns that.

    python3 estop_latch.py              # the service
    python3 estop_latch.py --clear      # un-latch the VCB once RC8 is clear
    python3 estop_latch.py --status     # one line from estop.json
    python3 estop_latch.py --selftest

Never sends a HEARTBEAT (that would mask the autopilot's GCS failsafe). Only MAVLink sent:
SET_MESSAGE_INTERVAL for RC_CHANNELS while it is missing.
"""
import argparse
import json
import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

ESTOP_CH = 8
ENGAGED_BELOW = 1300      # RC8 raw 987 = motor emergency stop (params/vcb-pixhawk.parm)
CLEAR_ABOVE = 1700        # RC8 raw 2011 = cleared
RC_FRESH_S = 1.0          # RC_CHANNELS older than this -> unknown, no action
STOP_RESEND_S = 0.5
FAULT_STOP = 9
MAV_SYSID, MAV_COMPID = 254, 193
RC_CHANNELS_ID = 65
STATE_PATH = "/home/boat/estop.json"
VCB_JSON = "/home/boat/vcb.json"


def classify(raw):
    if raw is None or raw <= 800 or raw >= 2300:
        return "unknown"
    if raw < ENGAGED_BELOW:
        return "engaged"
    if raw > CLEAR_ABOVE:
        return "clear"
    return "unknown"


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def write_atomic(path, d):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f)
    os.replace(tmp, path)


def vcb_fault(path, fresh_s=2.0):
    """VCB fault_code from vcb.json, or None if vcb.json is missing/stale/offline."""
    v = read_json(path)
    if not v or not v.get("online") or time.time() - (v.get("ts") or 0) > fresh_s:
        return None
    return (v.get("status") or {}).get("fault_code")


def should_send_stop(state, fault, last_sent, now):
    """Level-triggered: while engaged, (re)send until the VCB shows fault 9."""
    if state != "engaged" or fault == FAULT_STOP:
        return False
    return now - last_sent >= STOP_RESEND_S


class Vcb:
    def __init__(self, port, baud):
        self.port, self.baud, self.ser = port, baud, None

    def send(self, line):
        import serial
        try:
            if self.ser is None:
                self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.ser.write((line + "\n").encode("ascii"))
            self.ser.flush()
            return True
        except Exception as e:
            print(f"VCB write failed ({self.port}): {e}", file=sys.stderr, flush=True)
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            return False


def run(a):
    from pymavlink import mavutil
    vcb = Vcb(a.port, a.baud)
    rc_raw, rc_t, last_sent, last_req, last_write = None, 0.0, 0.0, 0.0, 0.0
    prev_state, target, stops = None, None, 0
    conn = None
    print(f"estop-latch: RC{ESTOP_CH} engaged <{ENGAGED_BELOW} us -> 'stop' to {a.port}", flush=True)
    while True:
        if conn is None:
            try:
                conn = mavutil.mavlink_connection(a.conn, source_system=MAV_SYSID,
                                                  source_component=MAV_COMPID, autoreconnect=True)
            except Exception as e:
                print(f"MAVLink connect failed: {e}", file=sys.stderr, flush=True)
                time.sleep(2)
                continue
        try:
            m = conn.recv_match(type=["RC_CHANNELS", "HEARTBEAT"], blocking=True, timeout=0.2)
        except Exception as e:
            print(f"MAVLink read failed: {e}", file=sys.stderr, flush=True)
            conn = None
            time.sleep(1)
            continue
        now = time.time()
        if m is not None:
            if m.get_type() == "HEARTBEAT":
                if m.type != 6 and m.autopilot != 8:          # not a GCS / companion
                    target = (m.get_srcSystem(), m.get_srcComponent())
            elif target is None or m.get_srcSystem() == target[0]:
                raw = getattr(m, f"chan{ESTOP_CH}_raw", 0)
                if m.chancount > 0 and raw:
                    rc_raw, rc_t = raw, now
        # ask for RC_CHANNELS at 10 Hz while it is missing (QGC may not have asked)
        if target and now - rc_t > 1.0 and now - last_req > 3.0:
            last_req = now
            conn.mav.command_long_send(target[0], target[1], 511, 0,
                                       RC_CHANNELS_ID, 100000, 0, 0, 0, 0, 0)
        state = classify(rc_raw) if now - rc_t <= RC_FRESH_S else "unknown"
        fault = vcb_fault(a.vcb)
        if state != prev_state:
            print(f"RC{ESTOP_CH} {state} ({rc_raw} us); VCB fault {fault}", flush=True)
            prev_state = state
        if should_send_stop(state, fault, last_sent, now):
            last_sent = now
            if vcb.send("stop"):
                stops += 1
                print(f"sent 'stop' (VCB fault was {fault})", flush=True)
        if now - last_write >= 0.2:
            last_write = now
            try:
                write_atomic(a.state, {"ts": now, "estop": state, "rc_us": rc_raw,
                                       "rc_age_s": None if not rc_t else round(now - rc_t, 2),
                                       "vcb_fault": fault, "latched": fault == FAULT_STOP,
                                       "stops_sent": stops, "last_stop_ts": last_sent or None})
            except OSError as e:
                print(f"state write failed: {e}", file=sys.stderr, flush=True)


def clear(a):
    s = read_json(a.state)
    if not s or time.time() - (s.get("ts") or 0) > 2:
        sys.exit("refusing: estop-latch not running (no fresh estop.json); "
                 "systemctl status estop-latch")
    if s["estop"] != "clear":
        sys.exit(f"refusing: RC{ESTOP_CH} is {s['estop']} ({s.get('rc_us')} us). "
                 f"Throw the e-stop to clear first.")
    if not Vcb(a.port, a.baud).send("clear"):
        sys.exit("VCB write failed")
    print("sent 'clear'. The VCB re-arms after ~3 s of neutral PWM, then the motor "
          "controllers need ~8 s to boot before they answer.")


def status(a):
    s = read_json(a.state)
    if not s:
        sys.exit("no estop.json: is estop-latch running?")
    age = time.time() - (s.get("ts") or 0)
    print(f"RC{ESTOP_CH} {s['estop'].upper()} ({s.get('rc_us')} us) · VCB fault {s.get('vcb_fault')}"
          f" · HV latched off: {'YES' if s.get('latched') else 'no'} · stops sent {s.get('stops_sent')}"
          f"{'' if age < 2 else f' · STALE {age:.0f} s'}")


def selftest():
    assert classify(987) == "engaged" and classify(2011) == "clear"
    assert classify(0) == "unknown" and classify(None) == "unknown" and classify(1500) == "unknown"
    assert should_send_stop("engaged", 0, 0, 10)
    assert should_send_stop("engaged", None, 0, 10)          # vcb.json stale: send anyway
    assert not should_send_stop("engaged", 9, 0, 10)
    assert not should_send_stop("engaged", 0, 9.8, 10)       # rate limit
    assert not should_send_stop("clear", 0, 0, 10)
    assert not should_send_stop("unknown", 0, 0, 10)
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--conn", default="tcp:127.0.0.1:5760")
    ap.add_argument("--port", default="/dev/vcb")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--vcb", default=VCB_JSON)
    ap.add_argument("--state", default=STATE_PATH)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--clear", action="store_true", help="send 'clear' if RC8 is clear")
    g.add_argument("--status", action="store_true")
    g.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
    elif a.clear:
        clear(a)
    elif a.status:
        status(a)
    else:
        run(a)


if __name__ == "__main__":
    main()
