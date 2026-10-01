#!/usr/bin/env python3
"""ArduPilot discovery: what is this autopilot, what does it see, how is it set up.

Run from a laptop over Tailscale or on the Jetson itself. Connects through
mavlink-router (TCP 5760 accepts multiple clients, so QGC can stay connected).

    ./ap_inventory.py                                  # laptop -> boat over tailnet
    ./ap_inventory.py --conn tcp:127.0.0.1:5760        # on the Jetson
    ./ap_inventory.py --conn /dev/ttyACM0 --baud 115200  # direct serial (stop the router first)
    ./ap_inventory.py --params params/boat.parm --json out.json

What it collects
  - HEARTBEAT: sysid/compid, vehicle type, mode, armed
  - AUTOPILOT_VERSION: firmware version, git hash, board id, USB vendor/product
  - Boot banner (MAV_CMD_DO_SEND_BANNER): board name, IMU/baro/compass/GPS
    detection lines, RCOut config -- the single best "what hardware is this" source
  - SYS_STATUS sensor bitmask: present / enabled / healthy per sensor
  - POWER_STATUS: Vcc, Vservo, power-module flags (tells us if a PM is wired)
  - GPS_RAW_INT / GPS2_RAW: fix type, sats, HDOP
  - RC_CHANNELS: channel count + RSSI (is a receiver bound?)
  - BATTERY_STATUS, EKF_STATUS_REPORT, VIBRATION, SERVO_OUTPUT_RAW
  - Message census: which message IDs arrived and at what rate
  - Full parameter download, written as a .parm file, plus a curated table
Requires: pip install pymavlink
"""
import argparse
import json
import sys
import time
from collections import Counter, defaultdict

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mav
except ImportError:
    raise SystemExit("pip install pymavlink")

DEFAULT_CONN = "tcp:highfieldboat.tailcacf9.ts.net:5760"

# Parameter prefixes worth printing after the dump. Order = print order.
KEY_PARAM_PREFIXES = [
    "BRD_TYPE", "BRD_SAFETY", "SYSID_THISMAV",
    "FRAME_CLASS", "FRAME_TYPE",
    "SERVO1_", "SERVO2_", "SERVO3_", "SERVO4_",
    "MOT_", "CRUISE_", "ATC_", "WP_", "TURN_",
    "GPS", "CAN_", "COMPASS_USE", "COMPASS_TYPEMASK", "COMPASS_ENABLE", "COMPASS_DEV_ID",
    "INS_USE", "INS_ENABLE_MASK", "INS_GYR_ID", "INS_ACC_ID", "EK3_ENABLE", "EK3_IMU_MASK",
    "AHRS_EKF_TYPE", "BARO_PROBE", "BARO_PRIMARY",
    "ARMING_", "FS_", "FENCE_",
    "BATT_MONITOR", "BATT_FS_", "BATT_LOW_", "BATT_CRT_", "BATT_CAPACITY",
    "BATT2_MONITOR", "SCR_ENABLE",
    "SERIAL0_", "SERIAL1_", "SERIAL2_", "SERIAL3_", "SERIAL4_", "SERIAL5_",
    "RC_PROTOCOLS", "RSSI_TYPE", "RC1_", "RC3_", "MODE_CH", "MODE1", "MODE2", "MODE3", "MODE4", "MODE5", "MODE6",
    "LOG_BITMASK", "LOG_DISARMED",
]

# Messages we actively ask for at a modest rate during the sample window.
WANTED_MSGS = {
    "SYS_STATUS": 1, "POWER_STATUS": 1, "GPS_RAW_INT": 2, "GPS2_RAW": 2,
    "RC_CHANNELS": 2, "BATTERY_STATUS": 1, "EKF_STATUS_REPORT": 1,
    "VIBRATION": 1, "SERVO_OUTPUT_RAW": 2, "SCALED_IMU": 2, "SCALED_IMU2": 2,
    "SCALED_IMU3": 2, "SCALED_PRESSURE": 1, "SCALED_PRESSURE2": 1,
    "SCALED_PRESSURE3": 1, "GLOBAL_POSITION_INT": 2, "ATTITUDE": 2,
    "VFR_HUD": 2, "MEMINFO": 1, "HWSTATUS": 1,
}


def hr(title):
    print(f"\n=== {title} " + "=" * max(0, 60 - len(title)))


def enum_name(enum, value, strip=""):
    e = mav.enums.get(enum, {}).get(value)
    name = e.name if e else f"{value}"
    return name.replace(strip, "", 1) if strip and name.startswith(strip) else name


def decode_bitmask(enum, value, strip=""):
    out = []
    for bit, entry in mav.enums.get(enum, {}).items():
        if bit and (value & bit) == bit and bit & (bit - 1) == 0:
            out.append(entry.name.replace(strip, "", 1))
    return out


def fw_version(v):
    major, minor, patch, typ = (v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF
    types = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "official"}
    return f"{major}.{minor}.{patch} ({types.get(typ, typ)})"


def git_hash(b):
    return bytes(b).rstrip(b"\x00").decode(errors="replace")


def connect(conn, baud):
    print(f"connecting to {conn} ...", flush=True)
    m = mavutil.mavlink_connection(conn, baud=baud, source_system=250, source_component=191)
    hb = m.wait_heartbeat(timeout=20)
    if hb is None:
        raise SystemExit("no heartbeat in 20 s (router down? wrong host? Pixhawk unplugged?)")
    return m, hb


def request_message(m, msg_id):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mav.MAV_CMD_REQUEST_MESSAGE, 0, msg_id, 0, 0, 0, 0, 0, 0)


def set_interval(m, msg_id, hz):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


def section_heartbeat(m, hb, report):
    hr("HEARTBEAT")
    mode = mavutil.mode_string_v10(hb)
    armed = bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"sysid={m.target_system} compid={m.target_component}")
    print(f"type={enum_name('MAV_TYPE', hb.type, 'MAV_TYPE_')} "
          f"autopilot={enum_name('MAV_AUTOPILOT', hb.autopilot, 'MAV_AUTOPILOT_')}")
    print(f"mode={mode} armed={armed} system_status={enum_name('MAV_STATE', hb.system_status, 'MAV_STATE_')}")
    report["heartbeat"] = dict(sysid=m.target_system, compid=m.target_component, type=hb.type,
                               mode=mode, armed=armed)


def section_version(m, report):
    hr("AUTOPILOT_VERSION")
    for _ in range(3):
        request_message(m, mav.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        v = m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=3)
        if v:
            break
    if not v:
        print("no AUTOPILOT_VERSION reply")
        return
    caps = decode_bitmask("MAV_PROTOCOL_CAPABILITY", v.capabilities, "MAV_PROTOCOL_CAPABILITY_")
    info = dict(
        flight_sw=fw_version(v.flight_sw_version),
        flight_git=git_hash(v.flight_custom_version),
        os_git=git_hash(v.os_custom_version),
        middleware_git=git_hash(v.middleware_custom_version),
        board_version=v.board_version,
        vendor_id=f"0x{v.vendor_id:04x}", product_id=f"0x{v.product_id:04x}",
        uid=f"0x{v.uid:016x}",
        capabilities=caps,
    )
    for k, val in info.items():
        print(f"{k:16s} {val}")
    print("  (board_version is the APJ board id; vendor/product match `lsusb` on the Jetson)")
    report["version"] = info


def section_banner(m, report):
    hr("BOOT BANNER (STATUSTEXT)")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mav.MAV_CMD_DO_SEND_BANNER, 0, 0, 0, 0, 0, 0, 0, 0)
    lines, t0 = [], time.time()
    while time.time() - t0 < 4:
        s = m.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if s:
            txt = s.text.rstrip("\x00")
            sev = enum_name("MAV_SEVERITY", s.severity, "MAV_SEVERITY_")
            lines.append(dict(severity=sev, text=txt))
            print(f"[{sev:8s}] {txt}")
    if not lines:
        print("(no banner lines received; MAV_CMD_DO_SEND_BANNER may be unsupported on this build)")
    report["banner"] = lines


def section_sample(m, seconds, report):
    hr(f"LIVE SAMPLE ({seconds} s)")
    for name, hz in WANTED_MSGS.items():
        msg_id = getattr(mav, f"MAVLINK_MSG_ID_{name}", None)
        if msg_id is not None:
            set_interval(m, msg_id, hz)
    latest, counts, t0 = {}, Counter(), time.time()
    statustext = []
    while time.time() - t0 < seconds:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "BAD_DATA":
            continue
        counts[t] += 1
        latest[t] = msg
        if t == "STATUSTEXT":
            statustext.append(msg.text.rstrip("\x00"))
    elapsed = time.time() - t0

    # --- SYS_STATUS sensors
    s = latest.get("SYS_STATUS")
    if s:
        print("\n-- SYS_STATUS sensors (present / enabled / healthy)")
        rows = []
        for bit, e in sorted(mav.enums["MAV_SYS_STATUS_SENSOR"].items()):
            if not bit or bit & (bit - 1):
                continue
            pres = bool(s.onboard_control_sensors_present & bit)
            if not pres:
                continue
            en = bool(s.onboard_control_sensors_enabled & bit)
            ok = bool(s.onboard_control_sensors_health & bit)
            name = e.name.replace("MAV_SYS_STATUS_", "")
            flag = "OK " if ok else "BAD"
            print(f"  {flag} {'en ' if en else 'dis'} {name}")
            rows.append(dict(sensor=name, enabled=en, healthy=ok))
        print(f"  load={s.load/10:.0f}%  Vbatt={s.voltage_battery/1000:.2f} V  "
              f"I={s.current_battery/100 if s.current_battery!=-1 else 'n/a'} A  "
              f"remaining={s.battery_remaining}%  drop_rate={s.drop_rate_comm/100:.1f}%")
        report["sensors"] = rows

    p = latest.get("POWER_STATUS")
    if p:
        flags = decode_bitmask("MAV_POWER_STATUS", p.flags, "MAV_POWER_")
        print(f"\n-- POWER_STATUS Vcc={p.Vcc/1000:.2f} V  Vservo={p.Vservo/1000:.2f} V  flags={flags}")
        print("   (no BRICK_VALID => no power module on POWER1/2; battery sense is elsewhere)")
        report["power"] = dict(Vcc=p.Vcc, Vservo=p.Vservo, flags=flags)

    for gname in ("GPS_RAW_INT", "GPS2_RAW"):
        g = latest.get(gname)
        if g:
            fix = enum_name("GPS_FIX_TYPE", g.fix_type, "GPS_FIX_TYPE_")
            print(f"\n-- {gname} fix={fix} sats={g.satellites_visible} hdop={g.eph/100:.2f} "
                  f"lat={g.lat/1e7:.6f} lon={g.lon/1e7:.6f} alt={g.alt/1000:.1f} m")
            report[gname.lower()] = dict(fix=fix, sats=g.satellites_visible, hdop=g.eph / 100,
                                         lat=g.lat / 1e7, lon=g.lon / 1e7)

    rc = latest.get("RC_CHANNELS")
    if rc:
        chans = [getattr(rc, f"chan{i}_raw") for i in range(1, 19)][: rc.chancount]
        print(f"\n-- RC_CHANNELS count={rc.chancount} rssi={rc.rssi} chans={chans}")
        print("   (count=0 => no receiver detected on the Pixhawk)")
        report["rc"] = dict(count=rc.chancount, rssi=rc.rssi, chans=chans)

    so = latest.get("SERVO_OUTPUT_RAW")
    if so:
        outs = [getattr(so, f"servo{i}_raw") for i in range(1, 9)]
        print(f"\n-- SERVO_OUTPUT_RAW ch1-8 = {outs}")
        report["servo_out"] = outs

    b = latest.get("BATTERY_STATUS")
    if b:
        volts = [v / 1000 for v in b.voltages if v != 65535]
        print(f"\n-- BATTERY_STATUS id={b.id} type={enum_name('MAV_BATTERY_TYPE', b.type, 'MAV_BATTERY_TYPE_')} "
              f"V={sum(volts):.2f} I={b.current_battery/100 if b.current_battery!=-1 else 'n/a'} A "
              f"remaining={b.battery_remaining}%")
        report["battery"] = dict(id=b.id, volts=sum(volts), current=b.current_battery, remaining=b.battery_remaining)

    e = latest.get("EKF_STATUS_REPORT")
    if e:
        flags = decode_bitmask("EKF_STATUS_FLAGS", e.flags, "EKF_")
        print(f"\n-- EKF flags={flags}")
        print(f"   var: vel={e.velocity_variance:.2f} pos_h={e.pos_horiz_variance:.2f} "
              f"pos_v={e.pos_vert_variance:.2f} compass={e.compass_variance:.2f}")
        report["ekf"] = dict(flags=flags, compass_var=e.compass_variance, pos_h_var=e.pos_horiz_variance)

    v = latest.get("VIBRATION")
    if v:
        print(f"\n-- VIBRATION x={v.vibration_x:.1f} y={v.vibration_y:.1f} z={v.vibration_z:.1f} "
              f"clips={v.clipping_0},{v.clipping_1},{v.clipping_2}")

    imus = [n for n in ("SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3") if n in latest]
    baros = [n for n in ("SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3") if n in latest]
    print(f"\n-- IMUs streaming: {len(imus)} {imus}   baros streaming: {len(baros)} {baros}")
    report["imu_count"] = len(imus)
    report["baro_count"] = len(baros)

    if statustext:
        print("\n-- STATUSTEXT during sample:")
        for t in statustext:
            print(f"   {t}")
    report["statustext"] = statustext

    print("\n-- message census (Hz):")
    for t, n in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f"   {n/elapsed:6.1f}  {t}")
    report["census_hz"] = {t: round(n / elapsed, 2) for t, n in counts.items()}


def section_params(m, out_path, report):
    hr("PARAMETERS")
    m.param_fetch_all()
    params, seen_idx, total, t0, last_rx = {}, set(), None, time.time(), time.time()
    while True:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        now = time.time()
        if msg:
            total = msg.param_count
            params[msg.param_id] = (msg.param_value, msg.param_type)
            seen_idx.add(msg.param_index)
            last_rx = now
            if len(params) % 100 == 0:
                print(f"  {len(params)}/{total}", end="\r", flush=True)
            if total and len(params) >= total:
                break
        elif total and now - last_rx > 5:
            # over a relayed link packets get dropped; ask for the missing indices one by one
            missing = [i for i in range(total) if i not in seen_idx]
            print(f"\n  stalled at {len(params)}/{total}, re-requesting {len(missing)} ...")
            for i in missing[:200]:
                m.mav.param_request_read_send(m.target_system, m.target_component, b"", i)
                time.sleep(0.01)
            last_rx = now
        if now - t0 > 240:
            print(f"\n  gave up after 240 s with {len(params)}/{total}")
            break
    print(f"  got {len(params)} params in {time.time()-t0:.0f} s")

    with open(out_path, "w") as f:
        f.write(f"# ArduPilot params dumped {time.strftime('%Y-%m-%d %H:%M:%S')} by ap_inventory.py\n")
        for k in sorted(params):
            val = params[k][0]
            f.write(f"{k},{val:.6g}\n" if isinstance(val, float) else f"{k},{val}\n")
    print(f"  wrote {out_path}")

    print("\n-- key parameters")
    for pref in KEY_PARAM_PREFIXES:
        for k in sorted(params):
            if k.startswith(pref):
                print(f"  {k:18s} {params[k][0]:g}")
    report["param_count"] = len(params)
    report["param_file"] = out_path
    report["key_params"] = {k: params[k][0] for pref in KEY_PARAM_PREFIXES for k in params if k.startswith(pref)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default=DEFAULT_CONN, help="pymavlink connection string")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=int, default=8, help="live sample window")
    ap.add_argument("--params", metavar="FILE", help="download all params to FILE (.parm)")
    ap.add_argument("--no-banner", action="store_true")
    ap.add_argument("--json", metavar="FILE", help="write structured report")
    a = ap.parse_args()

    report = dict(when=time.strftime("%Y-%m-%dT%H:%M:%S"), conn=a.conn)
    m, hb = connect(a.conn, a.baud)
    section_heartbeat(m, hb, report)
    section_version(m, report)
    if not a.no_banner:
        section_banner(m, report)
    section_sample(m, a.seconds, report)
    if a.params:
        section_params(m, a.params, report)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(report, f, indent=2, default=str)
        print(f"\nreport -> {a.json}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
