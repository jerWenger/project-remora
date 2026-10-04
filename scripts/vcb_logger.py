#!/usr/bin/env python3
"""VCB USB-CDC telemetry logger (read-only toward the VCB).
Reads the VCB's CSV stream (mapping/status/pwm lines + free-text events), writes
every raw line with a host timestamp to /home/boat/vcblogs/vcb_<ts>.csv (one file
per connection) and keeps /home/boat/vcb.json current (atomic, <=5 Hz).

Port: /dev/vcb (99-vcb.rules), else the first /dev/serial/by-id entry that isn't
the Pixhawk or the FTDI. Never writes to the port.
Self-test: vcb_logger.py --self-test
"""
import argparse, glob, json, os, sys, time
try:
    import serial
except ImportError:
    raise SystemExit("pip install pyserial")

STATE = "/home/boat/vcb.json"
OUT_DIR = "/home/boat/vcblogs"
SILENT_S = 3.0          # no line for this long -> online=false
STATE_PERIOD = 0.2      # vcb.json at most 5 Hz
RETRY_S = 2.0

# Field order from the deployed bench build's flash strings (docs/HARDWARE.md, VCB B).
CH = ["us", "period_us", "valid", "raw_permille", "applied_permille", "ok", "bad", "ovr"]
DEFAULT_FIELDS = {
    "pwm": ["ms"] + [f"ch{c}_{f}" for c in (1, 2) for f in CH],
    "status": ["ms", "thruster_state", "armed", "estop_24v_present",
               "tsms_24v_present", "lsc_closed", "hsc_closed", "bus_mv"],
}
SKIP_IDS = ("ArduPilot", "Pixhawk", "FTDI")


def num(s):
    s = s.strip()
    for f in (int, float):
        try: return f(s)
        except ValueError: pass
    return s

def is_num(s):
    return not isinstance(num(s), str)


class Parser:
    """Line -> state dict. Header lines (non-numeric fields) override the default order."""
    def __init__(self):
        self.fields = {k: list(v) for k, v in DEFAULT_FIELDS.items()}
        self.state = {"mapping": None, "status": None, "pwm": None, "last_event": None}

    def feed(self, line):
        line = line.strip()
        if not line: return None
        kind, _, rest = line.partition(",")
        if kind == "mapping" and rest:
            self.state["mapping"] = line; return "mapping"
        if kind in self.fields and rest:
            vals = rest.split(",")
            if not any(is_num(v) for v in vals):          # header line
                self.fields[kind] = [v.strip() for v in vals]; return "header"
            names = self.fields[kind]
            d = {}
            for i, v in enumerate(vals):
                d[names[i] if i < len(names) else f"extra{i}"] = num(v)
            self.state[kind] = d; return kind
        self.state["last_event"] = line
        return "event"


def find_port(want):
    if want: return want if os.path.exists(want) else None
    if os.path.exists("/dev/vcb"): return "/dev/vcb"
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if not any(k in os.path.basename(p) for k in SKIP_IDS): return p
    return None

def write_atomic(path, d):
    tmp = path + ".tmp"
    with open(tmp, "w") as f: json.dump(d, f)
    os.replace(tmp, path)


class Logger:
    def __init__(self, a):
        self.a, self.p = a, Parser()
        self.port, self.ts, self.online, self.msg = "", 0.0, False, "starting"
        self.raw, self.last_write = None, 0.0

    def publish(self, force=False):
        now = time.time()
        if not force and now - self.last_write < STATE_PERIOD: return
        self.last_write = now
        d = {"ts": self.ts, "online": self.online, "port": self.port}
        d.update(self.p.state); d["msg"] = self.msg
        try: write_atomic(self.a.state, d)
        except OSError as e: print(f"state write failed: {e}", file=sys.stderr)

    def set_offline(self, msg):
        changed = self.online or msg != self.msg
        self.online, self.msg = False, msg
        if changed: print(msg, file=sys.stderr)
        self.publish(force=changed)

    def line(self, text):
        now = time.time()
        self.ts = now
        if self.raw is None:
            os.makedirs(self.a.out_dir, exist_ok=True)
            fn = os.path.join(self.a.out_dir, time.strftime("vcb_%Y%m%d_%H%M%S.csv"))
            self.raw = open(fn, "a", buffering=1)
            print(f"logging {self.port} -> {fn}", file=sys.stderr)
        self.raw.write(f"{now:.3f},{text}\n")
        if self.a.stdout: print(text, flush=True)
        self.p.feed(text)
        was = self.online
        self.online, self.msg = True, "ok"
        self.publish(force=not was)

    def session(self, s):
        buf = b""
        while True:
            chunk = s.read(s.in_waiting or 1)       # timeout 0.5 s; read only, never write
            if chunk:
                buf += chunk
                *lines, buf = buf.split(b"\n")
                for l in lines:
                    t = l.decode("ascii", "replace").strip("\r\x00 ")
                    if t: self.line(t)
                if len(buf) > 4096: buf = b""       # garbage without newlines
            if self.ts and time.time() - self.ts > SILENT_S:
                self.set_offline(f"no data from {self.port} for >{SILENT_S:.0f} s")
            elif not self.ts:
                self.set_offline(f"{self.port} open, no data yet")
            else:
                self.publish()                       # flush the latest throttled state

    def run(self):
        while True:
            port = find_port(self.a.port)
            if not port:
                self.port = self.a.port or ""
                self.set_offline(f"waiting for {self.a.port}" if self.a.port else
                                 "waiting for VCB port (/dev/vcb missing, none in /dev/serial/by-id)")
                time.sleep(RETRY_S); continue
            self.port, self.ts = port, 0.0
            try:
                with serial.Serial(port, self.a.baud, timeout=0.5) as s:
                    self.session(s)
            except (serial.SerialException, OSError) as e:
                self.set_offline(f"{port}: {e}")
            finally:
                if self.raw: self.raw.close(); self.raw = None
            time.sleep(RETRY_S)


# Sample lines from the flash strings (docs/HARDWARE.md); values made up.
SAMPLE = [
    "VCB independent-motor PWM bench test; hard limit 20 percent",
    "mapping,ch1=CONT_pin2_PA3,ch2=CONT_pin1_PA2,motors=ch1_right:ch2_left,range_us=1000:2000,clamp_above_us=1200",
    "status,1200,IDLE,0,1,1,0,0,51234",
    "pwm,1210,1000,20000,1,0,0,55,0,0,1002,20001,1,2,0,56,0,0",
    "BENCH ARMED: both PWM channels held stopped for 1 second",
    "pwm,ms,ch1_us,ch1_valid,ch1_raw_permille,ch1_applied_permille,ch2_us,ch2_valid,ch2_raw_permille,ch2_applied_permille,new_field",
    "pwm,2000,1150,1,150,150,1300,1,300,200,7,99",
    "status,2100,4",
    "BENCH FAULT: PWM lost or invalid",
    "pwm,,garbage,1.5",
]

def self_test():
    p = Parser()
    kinds = [p.feed(l) for l in SAMPLE]
    st = p.state
    assert kinds == ["event", "mapping", "status", "pwm", "event", "header", "pwm", "status", "event", "pwm"], kinds
    assert st["mapping"].startswith("mapping,ch1=")
    assert st["last_event"] == "BENCH FAULT: PWM lost or invalid"
    assert st["status"] == {"ms": 2100, "thruster_state": 4}, st["status"]
    p2 = Parser(); [p2.feed(l) for l in SAMPLE[:4]]
    s, w = p2.state["status"], p2.state["pwm"]
    assert s["thruster_state"] == "IDLE" and s["bus_mv"] == 51234 and s["estop_24v_present"] == 1, s
    assert w["ch1_us"] == 1000 and w["ch2_us"] == 1002 and w["ch2_ovr"] == 0 and len(w) == 17, w
    p3 = Parser(); [p3.feed(l) for l in SAMPLE[:7]]
    w = p3.state["pwm"]
    assert w["ch1_applied_permille"] == 150 and w["ch2_applied_permille"] == 200, w
    assert w["new_field"] == 7 and w["extra10"] == 99, w
    assert st["pwm"] == {"ms": "", "ch1_us": "garbage", "ch1_valid": 1.5}, st["pwm"]
    print(json.dumps(st, indent=1))
    print("self-test OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=None, help="default: /dev/vcb, else autodetect")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB-CDC")
    ap.add_argument("--out-dir", default=OUT_DIR)
    ap.add_argument("--state", default=STATE)
    ap.add_argument("--stdout", action="store_true", help="echo raw lines")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test: return self_test()
    print(f"VCB logger -> {a.state}, raw lines -> {a.out_dir}", file=sys.stderr)
    try: Logger(a).run()
    except KeyboardInterrupt: pass

if __name__ == "__main__":
    main()
