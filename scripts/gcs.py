#!/usr/bin/env python3
"""Boat GCS dashboard: WebRTC camera + ONVIF PTZ + battery + vehicle monitor.

Stdlib only, plus optional pymavlink for the vehicle card. Serves on :8080.
PTZ is proxied to the camera's ONVIF service.

MONITOR ONLY: the only things ever sent to the autopilot are data-stream /
message-interval requests, and only when telemetry is missing. No HEARTBEAT is
sent (that would mask the autopilot's GCS failsafe), and no arm/mode/param
commands exist here.

Config (CLI arg overrides env var overrides default):
  --port      GCS_PORT        8080
  --mavlink   GCS_MAVLINK     udpin:0.0.0.0:14552  (e.g. tcp:127.0.0.1:5760)
  --state-dir GCS_STATE_DIR   /home/boat           (battery.json, vcb.json)
"""
import argparse
import collections
import glob
import json
import math
import os
import shutil
import subprocess
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ.setdefault("MAVLINK20", "1")  # decode MAVLink2 (servo9..16 extensions)
try:
    from pymavlink import mavutil
except Exception:  # pymavlink is optional
    mavutil = None

CAM = "192.168.1.110"
PTZ_URL = f"http://{CAM}/onvif/ptz_service"
PROFILE = "MainStream"
SOAP_HDR = {"Content-Type": "application/soap+xml"}

NS = ('xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
      'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" '
      'xmlns:tt="http://www.onvif.org/ver10/schema"')

STATE_DIR = os.environ.get("GCS_STATE_DIR", "/home/boat")
MAVLINK_URL = os.environ.get("GCS_MAVLINK", "udpin:0.0.0.0:14552")
MAV_SYSID = 254          # our source ids for stream requests (QGC usually uses 255)
MAV_COMPID = 191         # MAV_COMP_ID_ONBOARD_COMPUTER
STREAM_HZ = 4            # rate used if we have to request streams
SERVICES = ["boat-gcs", "mediamtx", "mavlink-router", "battery-litime", "vcb-logger"]
STALE_S = 10.0


def _soap(body: str) -> str:
    return f'<s:Envelope {NS}><s:Body>{body}</s:Body></s:Envelope>'


def onvif(body: str) -> bool:
    data = _soap(body).encode()
    req = urllib.request.Request(PTZ_URL, data=data, headers=SOAP_HDR, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=4) as r:
            return r.status == 200
    except Exception:
        return False


def ptz_move(pan: float, tilt: float, zoom: float) -> bool:
    return onvif(
        f'<tptz:ContinuousMove><tptz:ProfileToken>{PROFILE}</tptz:ProfileToken>'
        f'<tptz:Velocity><tt:PanTilt x="{pan}" y="{tilt}"/><tt:Zoom x="{zoom}"/>'
        f'</tptz:Velocity></tptz:ContinuousMove>')


def ptz_stop() -> bool:
    return onvif(
        f'<tptz:Stop><tptz:ProfileToken>{PROFILE}</tptz:ProfileToken>'
        f'<tptz:PanTilt>true</tptz:PanTilt><tptz:Zoom>true</tptz:Zoom></tptz:Stop>')


# --------------------------------------------------------------------------
# State files (battery.json, vcb.json)
# --------------------------------------------------------------------------

def read_state_json(name, missing):
    """Read STATE_DIR/name, add age_s from file mtime. Returns a dict."""
    path = os.path.join(STATE_DIR, name)
    try:
        st = os.stat(path)
        with open(path, "rb") as f:
            d = json.loads(f.read() or b"{}")
        if not isinstance(d, dict):
            d = {"online": False, "msg": f"{name}: not a JSON object"}
        d["age_s"] = round(max(0.0, time.time() - st.st_mtime), 1)
        return d
    except FileNotFoundError:
        return dict(missing)
    except Exception as e:
        return {"online": False, "msg": f"{name}: {e.__class__.__name__}: {e}"}


# --------------------------------------------------------------------------
# MAVLink decoding tables (hardcoded so they work without pymavlink too)
# --------------------------------------------------------------------------

ROVER_MODES = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER",
               6: "FOLLOW", 7: "SIMPLE", 8: "DOCK", 9: "CIRCLE", 10: "AUTO",
               11: "RTL", 12: "SMART_RTL", 15: "GUIDED", 16: "INITIALISING"}
MAV_STATE = ["UNINIT", "BOOT", "CALIBRATING", "STANDBY", "ACTIVE", "CRITICAL",
             "EMERGENCY", "POWEROFF", "FLIGHT_TERMINATION"]
GPS_FIX = ["NO_GPS", "NO_FIX", "2D_FIX", "3D_FIX", "DGPS", "RTK_FLOAT",
           "RTK_FIXED", "STATIC", "PPP"]
SEVERITY = ["EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING", "NOTICE",
            "INFO", "DEBUG"]
EKF_FLAGS = [(1, "ATTITUDE"), (2, "VELOCITY_HORIZ"), (4, "VELOCITY_VERT"),
             (8, "POS_HORIZ_REL"), (16, "POS_HORIZ_ABS"), (32, "POS_VERT_ABS"),
             (64, "POS_VERT_AGL"), (128, "CONST_POS_MODE"),
             (256, "PRED_POS_HORIZ_REL"), (512, "PRED_POS_HORIZ_ABS"),
             (1024, "UNINITIALIZED"), (32768, "GPS_GLITCHING")]
SENSORS = [(1 << 0, "GYRO"), (1 << 1, "ACCEL"), (1 << 2, "MAG"),
           (1 << 3, "BARO"), (1 << 4, "DIFF_PRESSURE"), (1 << 5, "GPS"),
           (1 << 6, "OPTICAL_FLOW"), (1 << 7, "VISION_POSITION"),
           (1 << 8, "LASER_POSITION"), (1 << 9, "EXT_GROUND_TRUTH"),
           (1 << 10, "RATE_CONTROL"), (1 << 11, "ATTITUDE_STAB"),
           (1 << 12, "YAW_POSITION"), (1 << 13, "Z_ALT_CONTROL"),
           (1 << 14, "XY_POS_CONTROL"), (1 << 15, "MOTOR_OUTPUTS"),
           (1 << 16, "RC_RECEIVER"), (1 << 17, "GYRO2"), (1 << 18, "ACCEL2"),
           (1 << 19, "MAG2"), (1 << 20, "GEOFENCE"), (1 << 21, "AHRS"),
           (1 << 22, "TERRAIN"), (1 << 23, "REVERSE_MOTOR"), (1 << 24, "LOGGING"),
           (1 << 25, "BATTERY"), (1 << 26, "PROXIMITY"), (1 << 27, "SATCOM"),
           (1 << 28, "PREARM_CHECK"), (1 << 29, "OBSTACLE_AVOIDANCE"),
           (1 << 30, "PROPULSION")]
POWER_FLAGS = [(1, "BRICK_VALID"), (2, "SERVO_VALID"), (4, "USB_CONNECTED"),
               (8, "PERIPH_OVERCURRENT"), (16, "PERIPH_HIPOWER_OVERCURRENT"),
               (32, "CHANGED")]

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_DATA_STREAM_ALL = 0
MAV_CMD_SET_MESSAGE_INTERVAL = 511
# Messages the dashboard needs: name -> MAVLink id (for SET_MESSAGE_INTERVAL).
WANTED = {"GPS_RAW_INT": 24, "ATTITUDE": 30, "GLOBAL_POSITION_INT": 33,
          "SERVO_OUTPUT_RAW": 36, "VFR_HUD": 74, "SYS_STATUS": 1,
          "POWER_STATUS": 125, "EKF_STATUS_REPORT": 193}


def _idx(table, i):
    return table[i] if isinstance(i, int) and 0 <= i < len(table) else str(i)


def _bits(value, table):
    return [name for bit, name in table if value & bit]


def _mode_name(hb_type, custom_mode):
    mapping = None
    if mavutil is not None:
        try:
            mapping = mavutil.mode_mapping_bynumber(hb_type)
        except Exception:
            mapping = None
    if mapping is None and hb_type in (10, 11):  # GROUND_ROVER, SURFACE_BOAT
        mapping = ROVER_MODES
    if mapping and custom_mode in mapping:
        return mapping[custom_mode]
    return f"MODE{custom_mode}"


def _r(x, n=1):
    return None if x is None else round(x, n)


# --------------------------------------------------------------------------
# MAVLink monitor thread
# --------------------------------------------------------------------------

class MavMonitor:
    def __init__(self, url):
        self.url = url
        self.lock = threading.Lock()
        self.s = {}                       # section name -> dict with host time "t"
        self.statustext = collections.deque(maxlen=30)
        self.seen = {}                    # msg type -> host time last seen (target only)
        self.target = None                # (sysid, compid) of the autopilot
        self.error = None
        self.connected_since = None
        self.last_rx = None
        self.rx_count = 0
        self.stream_requests = 0
        self.last_request = None
        self.servo_port1_t = 0.0
        self.unanswered = 0
        self.last_missing = []
        self.target_since = 0.0

    def start(self):
        if mavutil is None:
            return
        threading.Thread(target=self._run, name="mavlink", daemon=True).start()

    # ---- connection loop ------------------------------------------------
    def _run(self):
        while True:
            conn = None
            try:
                conn = mavutil.mavlink_connection(
                    self.url, source_system=MAV_SYSID, source_component=MAV_COMPID,
                    autoreconnect=True)
                with self.lock:
                    self.error = None
                    self.connected_since = time.time()
                self._loop(conn)
            except Exception as e:
                with self.lock:
                    self.error = f"{e.__class__.__name__}: {e}"
            finally:
                try:
                    if conn is not None:
                        conn.close()
                except Exception:
                    pass
            time.sleep(3)

    def _loop(self, conn):
        opened = time.time()
        next_check = time.time() + 3
        while True:
            m = conn.recv_match(blocking=True, timeout=1)
            now = time.time()
            if m is not None:
                self._handle(m, now)
            if now >= next_check:
                next_check = now + 2
                self._maybe_request(conn, now)
                # Nothing at all for 20 s: reopen (helps tcp; harmless for udpin).
                last = self.last_rx or opened
                if now - last > 20 and now - opened > 20:
                    raise ConnectionError("no MAVLink data for 20 s, reconnecting")

    def _maybe_request(self, conn, now):
        """Request streams only if wanted telemetry is missing; at most every 10 s."""
        with self.lock:
            tgt = self.target
            hb = self.s.get("heartbeat")
            if tgt is None or hb is None or now - hb["t"] > 5:
                return
            if now - self.target_since < 10:
                return  # grace period: autopilot may still be booting / streams starting
            missing = [n for n in WANTED if now - self.seen.get(n, 0) > 5]
            if not missing:
                self.unanswered = 0
                return
            # every 10 s at first; back off to 60 s if the autopilot never sends some
            # message (e.g. a vehicle without POWER_STATUS) so we don't spam it.
            gap = 10 if self.unanswered < 3 else 60
            if self.last_request and now - self.last_request < gap:
                return
            self.last_request = now
            self.stream_requests += 1
            self.unanswered += 1
            self.last_missing = missing
        sysid, compid = tgt
        conn.mav.request_data_stream_send(sysid, compid, MAV_DATA_STREAM_ALL, STREAM_HZ, 1)
        for name in missing:
            conn.mav.command_long_send(sysid, compid, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       WANTED[name], 1e6 / 2, 0, 0, 0, 0, 0)

    # ---- message handling -----------------------------------------------
    def _handle(self, m, now):
        mt = m.get_type()
        if mt == "BAD_DATA":
            return
        src = (m.get_srcSystem(), m.get_srcComponent())
        with self.lock:
            if mt == "HEARTBEAT":
                if m.type == MAV_TYPE_GCS or m.autopilot == MAV_AUTOPILOT_INVALID:
                    return  # other GCSes / companions forwarded by mavlink-router
                if self.target is None or self.target[0] == src[0]:
                    if self.target is None or now - self.last_rx > 10:
                        self.target_since = now  # (re)acquired
                    self.target = src
                else:
                    return
            elif self.target is None or src[0] != self.target[0]:
                return
            self.last_rx = now
            self.rx_count += 1
            self.seen[mt] = now
            h = getattr(self, "_h_" + mt, None)
            if h is not None:
                h(m, now)

    def _h_HEARTBEAT(self, m, now):
        self.s["heartbeat"] = {
            "t": now, "mode": _mode_name(m.type, m.custom_mode),
            "custom_mode": m.custom_mode, "armed": bool(m.base_mode & 128),
            "system_status": _idx(MAV_STATE, m.system_status),
            "mav_type": m.type, "sysid": self.target[0], "compid": self.target[1]}

    def _h_GPS_RAW_INT(self, m, now):
        self.s["gps"] = {
            "t": now, "fix_type": m.fix_type, "fix": _idx(GPS_FIX, m.fix_type),
            "sats": None if m.satellites_visible == 255 else m.satellites_visible,
            "hdop": None if m.eph == 65535 else round(m.eph / 100, 2),
            "lat": m.lat / 1e7, "lon": m.lon / 1e7}

    def _h_GLOBAL_POSITION_INT(self, m, now):
        self.s["position"] = {
            "t": now, "lat": m.lat / 1e7, "lon": m.lon / 1e7,
            "hdg_deg": None if m.hdg == 65535 else round(m.hdg / 100, 1)}

    def _h_EKF_STATUS_REPORT(self, m, now):
        flags = _bits(m.flags, EKF_FLAGS)
        self.s["ekf"] = {
            "t": now, "flags_raw": m.flags, "flags": flags,
            "position_ok": "POS_HORIZ_ABS" in flags and "CONST_POS_MODE" not in flags,
            "variances": {"velocity": _r(m.velocity_variance, 2),
                          "pos_horiz": _r(m.pos_horiz_variance, 2),
                          "pos_vert": _r(m.pos_vert_variance, 2),
                          "compass": _r(m.compass_variance, 2)}}

    def _h_ATTITUDE(self, m, now):
        self.s["attitude"] = {
            "t": now, "roll_deg": round(math.degrees(m.roll), 1),
            "pitch_deg": round(math.degrees(m.pitch), 1),
            "yaw_deg": round(math.degrees(m.yaw) % 360, 1)}

    def _h_VFR_HUD(self, m, now):
        self.s["vfr"] = {"t": now, "groundspeed": round(m.groundspeed, 2),
                         "heading_deg": m.heading, "throttle": m.throttle}

    def _h_SERVO_OUTPUT_RAW(self, m, now):
        sv = self.s.setdefault("servo", {"ch": {}, "src9_16": None})
        sv["t"] = now
        if m.port == 0:
            for i in range(1, 9):
                sv["ch"][str(i)] = getattr(m, f"servo{i}_raw")
            # MAVLink2 extensions servo9..16 (0 when absent / MAVLink1). Use them
            # unless a port=1 message has been seen recently.
            if now - self.servo_port1_t > 5:
                for i in range(9, 17):
                    sv["ch"][str(i)] = getattr(m, f"servo{i}_raw", 0)
                sv["src9_16"] = "ext"
        elif m.port == 1:
            self.servo_port1_t = now
            for i in range(1, 9):
                sv["ch"][str(i + 8)] = getattr(m, f"servo{i}_raw")
            sv["src9_16"] = "port1"

    def _h_SYS_STATUS(self, m, now):
        p, e, h = (m.onboard_control_sensors_present, m.onboard_control_sensors_enabled,
                   m.onboard_control_sensors_health)
        sensors = [{"name": n, "enabled": bool(e & b), "healthy": bool(h & b)}
                   for b, n in SENSORS if p & b]
        self.s["sys_status"] = {
            "t": now, "sensors": sensors,
            "unhealthy": [n for b, n in SENSORS if p & b and e & b and not h & b],
            "load_pct": round(m.load / 10, 1),
            "battery_v": None if m.voltage_battery in (0, 65535) else round(m.voltage_battery / 1000, 2),
            "masks": {"present": p, "enabled": e, "health": h}}

    def _h_POWER_STATUS(self, m, now):
        self.s["power"] = {"t": now, "vcc_v": round(m.Vcc / 1000, 2),
                           "vservo_v": round(m.Vservo / 1000, 2),
                           "flags": _bits(m.flags, POWER_FLAGS)}

    def _h_STATUSTEXT(self, m, now):
        text = m.text
        if isinstance(text, bytes):
            text = text.decode("utf-8", "replace")
        text = text.rstrip("\x00")
        chunk_id = getattr(m, "id", 0)
        chunk_seq = getattr(m, "chunk_seq", 0)
        if chunk_id and chunk_seq and self.statustext and self.statustext[-1].get("id") == chunk_id:
            self.statustext[-1]["text"] += text
            return
        last = self.statustext[-1] if self.statustext else None
        if last and last["text"] == text and last["severity"] == m.severity:
            last["n"] += 1          # collapse repeats (e.g. "Field Elevation Set")
            last["t"] = round(now, 3)
            return
        self.statustext.append({"t": round(now, 3), "severity": m.severity, "n": 1,
                                "sev": _idx(SEVERITY, m.severity), "text": text,
                                "id": chunk_id})

    # ---- snapshot for /api/vehicle --------------------------------------
    def snapshot(self):
        now = time.time()
        if mavutil is None:
            return {"pymavlink": False, "msg": "pymavlink not installed",
                    "connection": self.url, "link": {"connected": False}}
        with self.lock:
            hb = self.s.get("heartbeat")
            hb_age = None if hb is None else round(now - hb["t"], 1)
            out = {
                "pymavlink": True, "connection": self.url, "ts": round(now, 3),
                "link": {
                    "connected": hb_age is not None and hb_age < 5,
                    "heartbeat_age_s": hb_age,
                    "last_msg_age_s": None if self.last_rx is None else round(now - self.last_rx, 1),
                    "connection": self.url,
                    "target": None if self.target is None else list(self.target),
                    "error": self.error, "msgs_rx": self.rx_count,
                    "stream_requests": self.stream_requests,
                    "last_requested": list(self.last_missing),
                    "last_stream_request_age_s": None if self.last_request is None
                    else round(now - self.last_request, 1)},
            }
            for k in ("heartbeat", "gps", "position", "ekf", "attitude", "vfr",
                      "servo", "sys_status", "power"):
                sec = self.s.get(k)
                if sec is None:
                    out[k] = None
                    continue
                sec = json.loads(json.dumps(sec))
                sec["age_s"] = round(now - sec.pop("t"), 1)
                out[k] = sec
            out["statustext"] = [
                {"t": st["t"], "age_s": round(now - st["t"], 1), "severity": st["severity"],
                 "sev": st["sev"], "text": st["text"], "n": st["n"]}
                for st in self.statustext]
        hd = None
        if out.get("vfr"):
            hd = out["vfr"]["heading_deg"]
        elif out.get("attitude"):
            hd = out["attitude"]["yaw_deg"]
        out["heading_deg"] = hd
        sv = out.get("servo")
        out["thrusters"] = None if not sv else {
            "left_us": sv["ch"].get("9"), "right_us": sv["ch"].get("10"),
            "src": sv.get("src9_16"), "age_s": sv["age_s"]}
        return out


# --------------------------------------------------------------------------
# Host health (/api/health), cached
# --------------------------------------------------------------------------

_health_lock = threading.Lock()
_health_cache = {"t": 0.0, "data": None}


def _service_states():
    states = {s: "unknown" for s in SERVICES}
    try:
        r = subprocess.run(["systemctl", "is-active"] + SERVICES, capture_output=True,
                           text=True, timeout=2)
        lines = r.stdout.split()
        if len(lines) == len(SERVICES):
            states = dict(zip(SERVICES, lines))
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        pass
    # mavlink-routerd has historically been started by a shell loop, not systemd.
    if states.get("mavlink-router") != "active":
        try:
            r = subprocess.run(["pgrep", "-x", "mavlink-routerd"], capture_output=True,
                               timeout=2)
            if r.returncode == 0:
                states["mavlink-router"] = "running (no unit)"
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
            pass
    return states


def _temps():
    out = []
    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        try:
            with open(os.path.join(zone, "temp")) as f:
                milli = int(f.read().strip())
            try:
                with open(os.path.join(zone, "type")) as f:
                    name = f.read().strip()
            except OSError:
                name = os.path.basename(zone)
            if -40000 < milli < 150000:
                out.append({"name": name, "c": round(milli / 1000, 1)})
        except (OSError, ValueError):
            continue
    return out


def _uptime_s():
    try:
        with open("/proc/uptime") as f:
            return round(float(f.read().split()[0]))
    except (OSError, ValueError, IndexError):
        pass
    try:  # macOS
        r = subprocess.run(["sysctl", "-n", "kern.boottime"], capture_output=True,
                           text=True, timeout=1)
        sec = int(r.stdout.split("sec =")[1].split(",")[0])
        return round(time.time() - sec)
    except Exception:
        return None


def health():
    with _health_lock:
        now = time.time()
        if _health_cache["data"] is not None and now - _health_cache["t"] < 5:
            return _health_cache["data"]
        try:
            du = shutil.disk_usage(STATE_DIR)
            disk = {"path": STATE_DIR, "total_gb": round(du.total / 1e9, 1),
                    "free_gb": round(du.free / 1e9, 1),
                    "used_pct": round(100 * du.used / du.total, 1)}
        except OSError as e:
            disk = {"path": STATE_DIR, "error": str(e)}
        try:
            load = [round(x, 2) for x in os.getloadavg()]
        except OSError:
            load = None
        data = {"ts": round(now, 3), "services": _service_states(), "disk": disk,
                "temps": _temps(), "uptime_s": _uptime_s(), "load": load,
                "gcs_uptime_s": round(now - START_TIME)}
        _health_cache.update(t=now, data=data)
        return data


START_TIME = time.time()
MAV = MavMonitor(MAVLINK_URL)


PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Boat GCS</title><style>
:root{--bg:#0d1117;--panel:#161b22;--line:#30363d;--txt:#e6edf3;--accent:#2f81f7;--ok:#3fb950;--warn:#d29922;--bad:#f85149;--mut:#8b949e}
*{box-sizing:border-box}body{margin:0;font:14px/1.4 system-ui,sans-serif;background:var(--bg);color:var(--txt)}
header{padding:10px 16px;border-bottom:1px solid var(--line);display:flex;align-items:center;gap:12px;flex-wrap:wrap}
header h1{font-size:15px;margin:0;font-weight:600}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--warn);flex:0 0 auto}.dot.on{background:var(--ok)}.dot.off{background:var(--bad)}.dot.unk{background:#484f58}
.wrap{display:flex;gap:16px;padding:16px;flex-wrap:wrap;align-items:flex-start}
.main{flex:1 1 640px;min-width:0;display:flex;flex-direction:column;gap:16px}
.video iframe{width:100%;aspect-ratio:16/9;border:1px solid var(--line);border-radius:8px;background:#000;display:block}
.pair{display:flex;flex-wrap:wrap;gap:16px}.pair>.card{flex:1 1 300px}
.side{flex:0 0 260px;display:flex;flex-direction:column;gap:16px;min-width:0}
@media (max-width:620px){.side,.main{flex:1 1 100%}.wrap{padding:10px;gap:10px}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px;min-width:0;overflow-wrap:anywhere}
.card h2{margin:0 0 10px;font-size:12px;text-transform:uppercase;letter-spacing:.5px;color:var(--mut);display:flex;justify-content:space-between;gap:8px}
.card h2 .age{text-transform:none;letter-spacing:0;font-weight:400}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:200px;margin:auto}
.dpad button,.zoom button{background:#21262d;border:1px solid var(--line);color:var(--txt);border-radius:6px;
  padding:14px 0;font-size:18px;cursor:pointer;user-select:none;touch-action:none}
.dpad button:active,.zoom button:active{background:var(--accent);border-color:var(--accent)}
.dpad .sp{visibility:hidden}
.zoom{display:flex;gap:8px;margin-top:10px}.zoom button{flex:1}
.speed{margin-top:12px;font-size:12px;color:var(--mut)}.speed input{width:100%}
.row{display:flex;justify-content:space-between;gap:8px;overflow-wrap:anywhere;padding:4px 0;border-bottom:1px solid #21262d}
.row:last-child{border:0}.v{font-variant-numeric:tabular-nums;font-weight:600;text-align:right}
.batt .big{font-size:28px;font-weight:700}.muted{color:var(--mut)}
.stale{opacity:.45}
.badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:12px;font-weight:700;background:#21262d;border:1px solid var(--line)}
.badge.armed{background:#5a1e1e;border-color:var(--bad);color:#ffd7d5}.badge.disarmed{background:#12361f;border-color:var(--ok);color:#aff5b4}
.mode{font-size:22px;font-weight:700;margin-right:8px}
.alert{margin:8px 0;padding:6px 8px;border-radius:6px;background:#3d1214;border:1px solid var(--bad);color:#ffd7d5;font-weight:600}
.alert.warn{background:#3a2a0b;border-color:var(--warn);color:#f8e3a1}
.nodata{color:var(--mut);font-style:italic;padding:4px 0}
.thr{margin:6px 0 12px}.thr .lbl{display:flex;justify-content:space-between;font-size:12px;color:var(--mut)}
.bar{position:relative;height:14px;background:#0d1117;border:1px solid var(--line);border-radius:4px;overflow:hidden;margin-top:3px}
.bar i{position:absolute;left:0;top:0;bottom:0;background:var(--accent)}.bar.vcb i{background:var(--ok)}.bar.clamp i{background:var(--warn)}
.bar span{position:absolute;right:6px;top:-1px;font-size:11px;font-variant-numeric:tabular-nums}
.msgs{max-height:260px;overflow-y:auto;font:12px/1.35 ui-monospace,SFMono-Regular,Menlo,monospace}
.msgs div{padding:2px 0;border-bottom:1px solid #21262d;word-break:break-word}
.msgs .t{color:var(--mut);margin-right:6px}
.s0,.s1,.s2,.s3{color:#ff7b72}.s4{color:#e3b341}.s5{color:#79c0ff}.s6{color:var(--txt)}.s7{color:var(--mut)}
.svc{display:flex;align-items:center;gap:8px;padding:3px 0}.svc .n{flex:1}
.small{font-size:12px}.chips span{display:inline-block;font-size:11px;padding:1px 6px;margin:2px 2px 0 0;border-radius:8px;background:#21262d;border:1px solid var(--line)}
.chips span.bad{border-color:var(--bad);color:#ffd7d5}.chips span.good{border-color:#238636}
</style></head><body>
<header><span class=dot id=camdot></span><h1>Boat Ground Station</h1>
<span class=muted id=status>connecting…</span>
<span style="flex:1"></span>
<span class=dot id=hdrlink></span><span id=hdrveh class=muted>vehicle: --</span></header>
<div class=wrap>
 <div class=main>
  <div class=video><iframe id=cam allow="autoplay" src=""></iframe></div>
  <div class=pair>
   <div class=card id=vehcard><h2>Vehicle <span class=age id=vehage></span></h2><div id=veh><div class=nodata>no data</div></div></div>
   <div class=card id=thrcard><h2>Thrusters <span class=age id=thrage></span></h2><div id=thr><div class=nodata>no data</div></div></div>
  </div>
   <div class=card><h2>Messages <span class=age id=msgcount></span></h2><div class=msgs id=msgs><div class=nodata>no messages</div></div></div>
 </div>
 <div class=side>
    <div class=card><h2>Camera PTZ</h2>
      <div class=dpad>
        <span class=sp></span><button data-p=0 data-t=1>▲</button><span class=sp></span>
        <button data-p=-1 data-t=0>◀</button><button data-stop=1>■</button><button data-p=1 data-t=0>▶</button>
        <span class=sp></span><button data-p=0 data-t=-1>▼</button><span class=sp></span>
      </div>
      <div class=zoom><button data-z=1>Zoom +</button><button data-z=-1>Zoom −</button></div>
      <div class=speed>Speed <input type=range min=0.1 max=1 step=0.1 value=0.4 id=spd></div>
    </div>
    <div class="card batt" id=battcard><h2>Battery <span class=age id=battage></span></h2>
      <div id=battbody>
      <div class=big id=soc>-- %</div>
      <div class=row><span>Voltage</span><span class=v id=volt>--</span></div>
      <div class=row><span>Current</span><span class=v id=curr>--</span></div>
      <div class=row><span>Power</span><span class=v id=pwr>--</span></div>
      <div class=row><span>Temp</span><span class=v id=temp>--</span></div>
      </div>
      <div class=muted id=bstat style=margin-top:8px>BLE not connected</div>
    </div>
    <div class=card><h2>Health <span class=age id=hlthage></span></h2><div id=hlth><div class=nodata>no data</div></div></div>
 </div>
</div>
<script>
const host=location.hostname;
document.getElementById('cam').src=`http://${host}:8889/cam/`;
const spd=document.getElementById('spd');
async function ptz(body){try{await fetch('/api/ptz',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});}catch(e){}}
function bind(btn){
  const stop=()=>ptz({action:'stop'});
  const start=()=>{const s=parseFloat(spd.value);
    if(btn.dataset.stop){stop();return;}
    const pan=(+btn.dataset.p||0)*s, tilt=(+btn.dataset.t||0)*s, zoom=(+btn.dataset.z||0)*s;
    ptz({action:'move',pan,tilt,zoom});};
  btn.addEventListener('mousedown',start);btn.addEventListener('touchstart',e=>{e.preventDefault();start();});
  btn.addEventListener('mouseup',stop);btn.addEventListener('mouseleave',stop);
  btn.addEventListener('touchend',e=>{e.preventDefault();stop();});
}
document.querySelectorAll('.dpad button,.zoom button').forEach(bind);

// ---- tiny DOM helpers (textContent only: STATUSTEXT etc. is never parsed as HTML)
const $=id=>document.getElementById(id);
function el(tag,cls,text){const e=document.createElement(tag);if(cls)e.className=cls;if(text!=null)e.textContent=text;return e;}
function row(k,v,cls){const r=el('div','row');r.append(el('span',null,k));const s=el('span','v'+(cls?' '+cls:''),v);r.append(s);return r;}
function nodata(msg){return el('div','nodata',msg||'no data');}
function fmt(x,n,u){return (x==null||isNaN(x))?'--':(+x).toFixed(n)+(u||'');}
function ago(s){if(s==null)return '';if(s<60)return s.toFixed(s<10?1:0)+' s';if(s<3600)return Math.round(s/60)+' min';if(s<172800)return (s/3600).toFixed(1)+' h';return Math.round(s/86400)+' d';}
const STALE=10;
function setStale(card,age){card.classList.toggle('stale',age!=null&&age>STALE);}

// ---- battery (existing behaviour + staleness)
async function pollBatt(){try{const r=await fetch('/api/battery');const b=await r.json();
  const bs=$('bstat'),age=b.age_s,stale=age!=null&&age>STALE;
  $('battage').textContent=age==null?'':(stale?'stale '+ago(age):ago(age));
  $('battbody').classList.toggle('stale',stale||!b.online);
  if(b.online){$('soc').textContent=(b.soc??'--')+' %';
    $('volt').textContent=(b.voltage??'--')+' V';
    $('curr').textContent=(b.current??'--')+' A';
    $('pwr').textContent=(b.power??'--')+' W';
    $('temp').textContent=(b.temp??'--')+' °C';
    bs.textContent=stale?'stale: last update '+ago(age)+' ago':'BMS connected';bs.className=stale?'muted':'';}
  else{bs.textContent=(b.msg||'BMS not connected')+(stale?' (stale '+ago(age)+')':'');bs.className='muted';}
}catch(e){$('bstat').textContent='dashboard unreachable';}}
setInterval(pollBatt,2000);pollBatt();

// ---- vehicle + thrusters (vehicle and VCB polled together at 1 Hz)
let VEH=null,VCB=null;
function renderVehicle(){
  const box=$('veh'),v=VEH;box.replaceChildren();
  const hl=$('hdrlink'),hv=$('hdrveh');
  if(!v){box.append(nodata('dashboard unreachable'));hl.className='dot off';hv.textContent='vehicle: --';return;}
  if(!v.pymavlink){box.append(nodata(v.msg||'pymavlink not installed'));hl.className='dot unk';hv.textContent='vehicle: pymavlink not installed';$('vehage').textContent='';return;}
  const L=v.link||{},hb=v.heartbeat;
  hl.className='dot '+(L.connected?'on':'off');
  $('vehage').textContent=L.heartbeat_age_s==null?'':'hb '+ago(L.heartbeat_age_s);
  setStale($('vehcard'),L.connected?0:L.heartbeat_age_s);
  if(!hb){box.append(nodata('no heartbeat on '+v.connection+(L.error?' — '+L.error:'')));
    hv.textContent='vehicle: no link';return;}
  hv.textContent=`${hb.mode} · ${hb.armed?'ARMED':'disarmed'} · hb ${ago(L.heartbeat_age_s)}`;
  const top=el('div');top.style.marginBottom='6px';
  top.append(el('span','mode',hb.mode),el('span','badge '+(hb.armed?'armed':'disarmed'),hb.armed?'ARMED':'DISARMED'));
  box.append(top);
  if(!L.connected)box.append(el('div','alert','LINK LOST — last heartbeat '+ago(L.heartbeat_age_s)+' ago'));
  box.append(row('Status',hb.system_status));
  const e=v.ekf;
  if(e&&!e.position_ok)box.append(el('div','alert',e.flags.includes('CONST_POS_MODE')?'EKF: no position (CONST_POS_MODE)':'EKF: no absolute horizontal position'));
  const g=v.gps;
  if(g){box.append(row('GPS',`${g.fix} · ${g.sats??'--'} sats · HDOP ${fmt(g.hdop,2)}`,g.fix_type<3?'s3':''));}
  else box.append(row('GPS','no data','muted'));
  const p=v.position||g;
  box.append(row('Position',p&&(p.lat||p.lon)?`${p.lat.toFixed(6)}, ${p.lon.toFixed(6)}`:'--'));
  box.append(row('Heading',fmt(v.heading_deg,0,'°')));
  box.append(row('Speed',v.vfr?`${fmt(v.vfr.groundspeed,2)} m/s (${fmt(v.vfr.groundspeed*1.94384,1)} kn)`:'--'));
  if(v.attitude)box.append(row('Roll / pitch',`${fmt(v.attitude.roll_deg,1)}° / ${fmt(v.attitude.pitch_deg,1)}°`));
  if(e){const c=el('div','chips small');c.style.margin='6px 0';
    c.append(el('span',null,'EKF'));
    e.flags.forEach(f=>c.append(el('span',f==='CONST_POS_MODE'||f==='GPS_GLITCHING'||f==='UNINITIALIZED'?'bad':'good',f)));
    box.append(c);}
  else box.append(row('EKF','no data','muted'));
  const s=v.sys_status;
  if(s){if(s.unhealthy.length){const a=el('div','alert warn','Unhealthy: '+s.unhealthy.join(', '));box.append(a);}
    else box.append(row('Sensors','all enabled healthy','s'));}
  if(v.power)box.append(row('Pixhawk Vcc',fmt(v.power.vcc_v,2,' V')));
}
function bar(label,frac,text,cls){
  const w=el('div','thr');const lb=el('div','lbl');lb.append(el('span',null,label));w.append(lb);
  const b=el('div','bar'+(cls?' '+cls:''));const i=el('i');i.style.width=(frac==null?0:Math.max(0,Math.min(1,frac))*100)+'%';
  b.append(i,el('span',null,text));w.append(b);return w;}
function usBar(side,ch,us){ // Pixhawk 1000-1200 us usable band = 0-20 % thrust
  if(us==null||us===0)return bar(`${side} · Pixhawk SERVO${ch}`,null,us===0?'0 (output off)':'no data');
  const clamp=us>1200;
  return bar(`${side} · Pixhawk SERVO${ch}`,(us-1000)/200,`${us} µs${clamp?' (>1200 clamped)':''}`,clamp?'clamp':'');}
function vcbBar(side,ch,pw){
  const lab=`${side} · VCB ch${ch} applied`;
  if(!pw||pw[`ch${ch}_applied_permille`]==null)return bar(lab,null,'no data','vcb');
  const a=pw[`ch${ch}_applied_permille`],valid=pw[`ch${ch}_valid`];
  return bar(lab,a/200,`${a}‰ (${(a/10).toFixed(1)} %)${valid===false||valid===0?' INVALID PWM':''} · in ${pw[`ch${ch}_us`]??'--'} µs`,'vcb');}
function renderThrusters(){
  const box=$('thr');box.replaceChildren();
  const t=VEH&&VEH.thrusters,c=VCB,online=c&&c.online,stale=c&&c.age_s!=null&&c.age_s>STALE;
  const pw=online&&!stale?c.pwm:null;
  $('thrage').textContent=c&&c.age_s!=null?'VCB '+(stale?'stale ':'')+ago(c.age_s):'';
  box.append(usBar('LEFT',9,t?t.left_us:null),vcbBar('LEFT',2,pw));
  box.append(usBar('RIGHT',10,t?t.right_us:null),vcbBar('RIGHT',1,pw));
  const st=c&&c.status;
  if(!c||!online){box.append(el('div','alert warn','VCB offline'+(c&&c.msg?': '+c.msg:'')));}
  else if(stale){box.append(el('div','alert warn','VCB data stale ('+ago(c.age_s)+')'));}
  const d=el('div');d.classList.toggle('stale',!online||stale);
  if(st){
    const yn=(x,good,bad)=>x==null?'--':(x?good:bad);
    d.append(row('Thruster state',String(st.thruster_state??'--')));
    d.append(row('VCB armed',yn(st.armed,'ARMED','disarmed'),st.armed?'s3':''));
    d.append(row('E-stop 24V',yn(st.estop_24v_present,'present (OK)','ABSENT (e-stop?)'),st.estop_24v_present?'':'s3'));
    d.append(row('TSMS 24V',yn(st.tsms_24v_present,'present','absent')));
    d.append(row('Contactors',`LSC ${yn(st.lsc_closed,'closed','open')} · HSC ${yn(st.hsc_closed,'closed','open')}`));
    d.append(row('Bus',st.bus_mv==null?'--':(st.bus_mv/1000).toFixed(1)+' V'));
  }else if(c&&online)d.append(nodata('no VCB status line yet'));
  if(c&&c.last_event)d.append(row('Last event',c.last_event));
  if(c&&c.mapping)d.append(el('div','muted small',c.mapping));
  box.append(d);
  if(t&&VEH.servo){const ch=VEH.servo.ch,o=[];
    for(const k of ['1','3'])if(ch[k])o.push(`MAIN${k} ${ch[k]}`);
    if(o.length)box.append(el('div','muted small','Other outputs: '+o.join(' · ')));}
}
let lastMsgKey='';
function renderMsgs(){
  const box=$('msgs'),list=(VEH&&VEH.statustext)||[];
  $('msgcount').textContent=list.length?list.length+' (last 30)':'';
  const key=list.length?list[list.length-1].t+'|'+list.length+'|'+list[list.length-1].text.length+'|'+list[list.length-1].n:'';
  if(key===lastMsgKey&&list.length)return;lastMsgKey=key;
  box.replaceChildren();
  if(!list.length){box.append(nodata(VEH&&VEH.pymavlink===false?'pymavlink not installed':'no messages'));return;}
  for(let i=list.length-1;i>=0;i--){const m=list[i];const d=el('div','s'+m.severity);
    d.append(el('span','t',new Date(m.t*1000).toLocaleTimeString()+' '+m.sev));d.append(document.createTextNode(m.text));if(m.n>1)d.append(el('span','t',' ×'+m.n));box.append(d);}
}
async function pollVeh(){
  try{const [a,b]=await Promise.all([fetch('/api/vehicle'),fetch('/api/vcb')]);VEH=await a.json();VCB=await b.json();}
  catch(e){VEH=null;VCB=null;}
  try{renderVehicle();}catch(e){console.error(e);}
  try{renderThrusters();}catch(e){console.error(e);}
  try{renderMsgs();}catch(e){console.error(e);}
}
setInterval(pollVeh,1000);pollVeh();

// ---- health (5 s)
async function pollHealth(){const box=$('hlth');
  try{const r=await fetch('/api/health');const h=await r.json();box.replaceChildren();
    for(const [n,s] of Object.entries(h.services||{})){const d=el('div','svc');
      const ok=s==='active'||s.startsWith('running');
      d.append(el('span','dot '+(ok?'on':(s==='unknown'?'unk':'off'))),el('span','n',n),el('span','muted small',s));box.append(d);}
    const k=h.disk||{};
    box.append(row('Disk',k.error?k.error:`${fmt(k.used_pct,0,'%')} used · ${fmt(k.free_gb,1)} GB free`,k.used_pct>90?'s3':''));
    if(h.temps&&h.temps.length){const mx=Math.max(...h.temps.map(t=>t.c));
      box.append(row('Temp (max)',fmt(mx,1,' °C'),mx>80?'s3':''));
      const c=el('div','chips small');h.temps.forEach(t=>c.append(el('span',null,`${t.name.replace(/-thermal$/,'')} ${t.c}°`)));box.append(c);}
    else box.append(row('Temps','n/a','muted'));
    box.append(row('Load',h.load?h.load.join(' '):'--'));
    box.append(row('Uptime',h.uptime_s==null?'--':ago(h.uptime_s)));
    $('hlthage').textContent='gcs up '+ago(h.gcs_uptime_s);
  }catch(e){box.replaceChildren(nodata('dashboard unreachable'));}}
setInterval(pollHealth,5000);pollHealth();

document.getElementById('status').textContent='live';
document.getElementById('camdot').classList.add('on');
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        if ctype.startswith("application/json"):
            self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj):
        self._send(200, "application/json", json.dumps(obj).encode())

    def do_GET(self):
        path = urllib.parse.urlsplit(self.path).path
        if path == "/" or path.startswith("/index"):
            self._send(200, "text/html; charset=utf-8", PAGE.encode())
        elif path == "/api/battery":
            self._json(read_state_json("battery.json",
                                       {"online": False, "msg": "RS485 not connected"}))
        elif path == "/api/vcb":
            self._json(read_state_json("vcb.json", {"online": False, "msg": "no vcb.json"}))
        elif path == "/api/vehicle":
            self._json(MAV.snapshot())
        elif path == "/api/health":
            self._json(health())
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self):
        if self.path != "/api/ptz":
            self._send(404, "text/plain", b"not found")
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            req = {}
        if req.get("action") == "move":
            ok = ptz_move(req.get("pan", 0), req.get("tilt", 0), req.get("zoom", 0))
        else:
            ok = ptz_stop()
        self._send(200, "application/json", json.dumps({"ok": ok}).encode())


def main():
    global STATE_DIR, MAV
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=int(os.environ.get("GCS_PORT", 8080)))
    ap.add_argument("--mavlink", default=MAVLINK_URL,
                    help="pymavlink connection string (env GCS_MAVLINK)")
    ap.add_argument("--state-dir", default=STATE_DIR, help="dir with battery.json, vcb.json")
    a = ap.parse_args()
    STATE_DIR = a.state_dir
    MAV = MavMonitor(a.mavlink)
    MAV.start()
    print(f"GCS on :{a.port}  mavlink={a.mavlink}"
          f"{'' if mavutil else ' (pymavlink not installed)'}  state={STATE_DIR}")
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), H)
    srv.daemon_threads = True
    srv.serve_forever()


if __name__ == "__main__":
    main()
