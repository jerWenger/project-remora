#!/usr/bin/env python3
"""Draw the project figures as PNG (2x), into this directory.

    python3 docs/figures/make_figures.py

Needs rsvg-convert (macOS: brew install librsvg, Ubuntu: apt install librsvg2-bin); otherwise
standard library only. Every shape is placed by hand (x, y, w, h in px), so to change a figure
edit the numbers and text below and re-run.

Facts shown come from docs/HARDWARE.md, docs/JETSON.md, docs/VCB_BENCH_FIRMWARE.md,
config/mav.conf.new and params/boat.parm. Dashed boxes are planned, not installed.
"""
import math
import shutil
import subprocess
from pathlib import Path
from xml.sax.saxutils import escape

OUT = Path(__file__).resolve().parent
FONT = "Helvetica Neue, Helvetica, Arial, sans-serif"
INK, MUTED, GREY = "#1b1f24", "#57606a", "#6e7781"
NODE_FILL, NODE_EDGE = "#f2f4f7", "#8c959f"
CMD, SAFE = "#0a58a6", "#b3261e"
# kind -> (colour, stroke width, arrowhead length, arrowhead width)
KINDS = {"data": (GREY, 1.4, 9, 7), "cmd": (CMD, 2.4, 11, 8.5), "safe": (SAFE, 1.8, 10, 8)}


def tw(s, size, bold=False):
    """Rough Helvetica text width, used only to warn about text that overflows a box."""
    w = 0.0
    for c in s:
        if c in "iljtfI.,:;|!'()[] ·/":
            w += 0.30
        elif c in "mwMW@%↔→…":
            w += 0.88
        elif c.isupper() or c in "_":
            w += 0.70
        else:
            w += 0.55
    return w * size * (1.06 if bold else 1.0)


class Fig:
    def __init__(self, name, w, h):
        self.name, self.w, self.h, self.out = name, w, h, []

    def add(self, s):
        self.out.append(s)

    def text(self, x, y, s, size=12.5, weight=400, fill=INK, anchor="start", rotate=None, italic=False, spacing=None):
        extra = f' transform="rotate({rotate} {x} {y})"' if rotate else ""
        extra += ' font-style="italic"' if italic else ""
        extra += f' letter-spacing="{spacing}"' if spacing else ""
        self.add(f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}" font-weight="{weight}" '
                 f'fill="{fill}" text-anchor="{anchor}"{extra}>{escape(s)}</text>')

    def lines(self, x, y, rows, size=12.5, lead=16, **kw):
        for i, row in enumerate(rows):
            self.text(x, y + i * lead, row, size=size, **kw)

    def box(self, x, y, w, h, fill="#ffffff", stroke=GREY, sw=1.2, rx=6, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        s = f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ""
        self.add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}"{s}{d}/>')

    def tag(self, x, y, s="planned"):
        """Small italic tag sitting on a box's top edge, right-aligned at x."""
        w = tw(s, 11) + 12
        self.box(x - w, y - 8, w, 16, fill="#ffffff", stroke=GREY, sw=0.9, rx=8, dash="3 2")
        self.text(x - w / 2, y + 3.5, s, size=11, fill=MUTED, anchor="middle", italic=True)

    def container(self, x, y, w, h, title, sub=None):
        """A machine or board. The title runs up the left edge so links can enter from the top."""
        self.box(x, y, w, h, fill=NODE_FILL, stroke=NODE_EDGE, sw=1.4, rx=10)
        self.add(f'<line x1="{x + 44}" y1="{y + 8}" x2="{x + 44}" y2="{y + h - 8}" stroke="#c9ced6" stroke-width="1"/>')
        cy = y + h / 2
        if tw(title, 15, True) > h - 8 or (sub and tw(sub, 12) > h - 8):
            print(f"  warning: spine text too long for {title!r}")
        self.text(x + (19 if sub else 27), cy, title, size=15, weight=700, anchor="middle", rotate=-90)
        if sub:
            self.text(x + 36, cy, sub, size=12, fill=MUTED, anchor="middle", rotate=-90)

    def group(self, x, y, w, h, title, planned=False):
        self.box(x, y, w, h, fill="#fafbfc", stroke=GREY, sw=1.2, rx=8, dash="6 4" if planned else None)
        self.text(x + 12, y + 18, title, size=13, weight=700)
        if planned:
            self.tag(x + w - 10, y)

    def proc(self, x, y, w, h, name, desc=(), planned=False, fill="#ffffff", stroke=GREY, name_size=14.5, tag_at=None):
        """A process (white) or, through device(), a piece of hardware (grey)."""
        self.box(x, y, w, h, fill=fill, stroke=stroke, dash="6 4" if planned else None)
        if tw(name, name_size, True) > w - 4 or any(tw(d, 12.5) > w - 2 for d in desc):
            print(f"  warning: text may overflow box {name!r}")
        top = y + (h - (name_size + 16 * len(desc))) / 2 + name_size - 2
        self.text(x + w / 2, top, name, size=name_size, weight=700, anchor="middle")
        self.lines(x + w / 2, top + 17, desc, fill=MUTED, anchor="middle")
        if planned:
            self.tag(tag_at or x + w - 8, y)

    def device(self, x, y, w, h, name, desc=(), planned=False, tag_at=None):
        self.proc(x, y, w, h, name, desc, planned, fill=NODE_FILL, stroke=NODE_EDGE, tag_at=tag_at)

    def card(self, x, y, w, h, title, sub=(), body=(), top=False):
        """Plain-language box for the overview figures: title, grey hardware lines, then what it does."""
        self.box(x, y, w, h, fill=NODE_FILL, stroke=NODE_EDGE, sw=1.4, rx=9)
        if (tw(title, 15.5, True) > w - 16 or any(tw(t, 12.5) > w - 14 for t in sub)
                or any(tw(t, 13.5) > w - 14 for t in body)):
            print(f"  warning: text may overflow card {title!r}")
        height = 12 + 16 * len(sub) + (5 + 17 * len(body) if body else 0)
        ty = y + 30 if top else y + (h - height) / 2 + 12  # top=True lines titles up across a row
        self.text(x + 12, ty, title, size=15.5, weight=700)
        self.lines(x + 12, ty + 18, sub, size=12.5, lead=16, fill=MUTED)
        self.lines(x + 12, ty + 23 + 16 * len(sub), body, size=13.5, lead=17)

    def header(self, x1, x2, label):
        """Column-group heading with a rule under it, e.g. ON SHORE / ON THE BOAT."""
        self.text(x1, 14, label, size=11.5, weight=700, fill=MUTED, spacing=1.2)
        self.add(f'<line x1="{x1}" y1="22" x2="{x2}" y2="22" stroke="#aab1bb" stroke-width="1.2"/>')

    def edge(self, pts, kind="data", arrows="end", dash=False):
        colour, sw, hl, hw = KINDS[kind]
        pts = [list(p) for p in pts]
        heads = []
        for tip, prev, on in ((-1, -2, arrows in ("end", "both")), (0, 1, arrows in ("start", "both"))):
            if not on:
                continue
            (px, py), (qx, qy) = pts[tip], pts[prev]
            d = math.hypot(px - qx, py - qy)
            ux, uy = (px - qx) / d, (py - qy) / d
            bx, by = px - ux * hl, py - uy * hl
            heads.append(f"{px:.1f},{py:.1f} {bx - uy * hw / 2:.1f},{by + ux * hw / 2:.1f} "
                         f"{bx + uy * hw / 2:.1f},{by - ux * hw / 2:.1f}")
            pts[tip] = [px - ux * (hl - 1), py - uy * (hl - 1)]  # stop the stroke inside the head
        d = f' stroke-dasharray="{"7 5" if kind == "cmd" else "5 4"}"' if dash else ""
        path = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        self.add(f'<polyline points="{path}" fill="none" stroke="{colour}" stroke-width="{sw}" '
                 f'stroke-linejoin="round"{d}/>')
        for h in heads:
            self.add(f'<polygon points="{h}" fill="{colour}"/>')

    def legend(self, x, y, items, col_w=215, per_row=2):
        """items: (kind, dashed, text); kind 'box' draws a dashed planned box."""
        for i, (kind, dashed, label) in enumerate(items):
            lx, ly = x + (i % per_row) * col_w, y + (i // per_row) * 24
            if kind == "box":
                self.box(lx, ly - 8, 38, 16, dash="6 4", rx=4)
            else:
                self.edge([(lx, ly), (lx + 38, ly)], kind, dash=dashed)
            self.text(lx + 48, ly + 4.5, label, size=12.5)

    def save(self):
        """Render to a 2x PNG. The SVG is only an intermediate and is not kept."""
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" '
               f'width="{self.w}" height="{self.h}">\n'
               f'<rect width="{self.w}" height="{self.h}" fill="#ffffff"/>\n' + "\n".join(self.out) + "\n</svg>\n")
        png = OUT / f"{self.name}.png"
        subprocess.run(["rsvg-convert", "-z", "2", "-o", str(png)], input=svg.encode("utf-8"), check=True)
        print(f"wrote {png.name}")


def fig_architecture():
    """Figure 1: what software runs on which hardware, with the command path in blue."""
    f = Fig("fig1_system_architecture", 1000, 1058)

    # ---- shore ----
    f.container(16, 16, 684, 120, "Shore laptop", "operator")
    f.proc(70, 39, 248, 74, "MOOS-IvP shoreside", ["pMarineViewer · uFldShoreBroker", "pShare · uFldNodeComms"], planned=True)
    f.proc(330, 39, 134, 74, "QGroundControl", ["arm · mode · params", "+ tools/ap_*.py"])
    f.proc(476, 39, 210, 74, "Web browser", ["Boat GCS dashboard", "live camera view"])
    f.device(736, 39, 248, 74, "RC transmitter", ["RadioMaster Boxer (EdgeTX)", "ExpressLRS 2.4 GHz"])

    f.box(16, 152, 684, 24, fill="#e4ebf4", stroke=None, rx=12)
    f.text(263, 168.5, "Tailscale tailnet (WireGuard VPN)", size=12.5, fill="#33435a", anchor="middle")

    # ---- Jetson ----
    f.container(16, 206, 684, 292, "Jetson Orin Nano", "back seat · Ubuntu 22.04 / L4T 36.4")
    f.group(70, 222, 300, 160, "MOOS-IvP vehicle community", planned=True)
    f.lines(82, 266, ["+ MOOSDB, pShare,", "uFldNodeBroker,", "pNodeReporter"], size=12, lead=15, fill=MUTED)
    f.proc(206, 250, 152, 42, "pHelmIvP", ["behaviors"])
    f.proc(178, 334, 180, 42, "iArduRoverBridge", ["MOOS ↔ MAVLink"])
    f.edge([(282, 292), (282, 334)], "cmd", dash=True)
    f.lines(274, 310, ["DESIRED_HEADING", "DESIRED_SPEED"], size=11.5, lead=14, fill=CMD, anchor="end")
    f.proc(424, 222, 130, 74, "boat-gcs", ["dashboard :8080", "(gcs.py)"])
    f.proc(568, 222, 118, 74, "mediamtx", ["RTSP → WebRTC", "and HLS"])
    f.proc(70, 414, 110, 68, "vcb-logger", ["→ vcb.json", "+ CSV logs"])
    f.proc(200, 414, 290, 68, "mavlink-router", ["owns the Pixhawk serial port,", "fans out to TCP 5760, UDP 14551, 14552"])
    f.proc(560, 414, 126, 68, "battery-litime", ["→ battery.json"])

    # shore <-> Jetson
    f.edge([(130, 113), (130, 222)], "cmd", arrows="both", dash=True)
    f.text(138, 195, "pShare, UDP", fill=CMD)
    f.edge([(396, 113), (396, 414)], "cmd", arrows="both")
    f.text(404, 195, "TCP 5760", fill=CMD)
    f.edge([(489, 222), (489, 113)], "data")
    f.text(497, 195, "HTTP 8080")
    f.edge([(627, 222), (627, 113)], "data")
    f.text(635, 195, "WebRTC 8889")
    # inside the Jetson
    f.edge([(268, 376), (268, 414)], "cmd", dash=True)
    f.text(276, 400, "UDP 14551", fill=CMD)
    f.edge([(455, 414), (455, 356), (489, 356), (489, 296)], "data")
    f.text(497, 336, "UDP 14552")

    # Jetson peripherals
    f.device(736, 222, 212, 74, "IP camera (PTZ)", ["192.168.1.110, Ethernet", "RTSP video, ONVIF PTZ"])
    f.edge([(736, 259), (686, 259)], "data")
    f.device(736, 414, 212, 68, "LiTime 48 V 100 Ah pack", ["BMS: RS485 → FTDI USB"])
    f.edge([(736, 448), (686, 448)], "data")

    # ---- Pixhawk ----
    f.container(130, 570, 570, 168, "Pixhawk 6X", "front seat · ArduRover 4.7")
    f.proc(184, 586, 176, 74, "Mode + arming logic", ["Manual · Hold · Guided", "arming checks, failsafes"])
    f.proc(376, 586, 150, 74, "Controllers", ["speed + turn rate", "(ATC_* gains)"])
    f.proc(542, 586, 144, 74, "EKF3 navigation", ["GPS · compass · IMU"])
    f.proc(184, 676, 342, 46, "Skid-steer mixer", ["SERVO9 (AUX1) = right · SERVO10 (AUX2) = left"])
    f.edge([(360, 623), (376, 623)], "cmd")
    f.edge([(542, 623), (526, 623)], "data")
    f.edge([(451, 660), (451, 676)], "cmd")
    f.edge([(345, 482), (345, 570)], "cmd", arrows="both")
    f.lines(353, 528, ["MAVLink 2 over USB", "/dev/pixhawk · 115200 baud"], fill=CMD)
    f.device(736, 586, 212, 60, "GPS + compass puck", ["DroneCAN on CAN1"])
    f.edge([(736, 616), (686, 616)], "data")
    f.device(736, 664, 248, 60, "ELRS receiver", ["CRSF on TELEM3: sticks + mode switch"], planned=True, tag_at=810)
    f.edge([(736, 694), (700, 694)], "cmd", dash=True)
    f.edge([(968, 113), (968, 664)], "cmd", dash=True)
    f.lines(960, 340, ["RC link", "ELRS 2.4 GHz"], fill=CMD, anchor="end")

    # ---- VCB ----
    f.container(16, 814, 684, 106, "VCB", "bench firmware")
    f.proc(70, 830, 90, 74, "Status", ["CSV out", "(UART)"])
    f.proc(178, 830, 170, 74, "PWM capture", ["valid after 3 good pulses,", "lost after 100 ms"])
    f.proc(366, 830, 188, 74, "Arming + thruster FSM", ["precharge, contactors,", "latched FAULT"])
    f.proc(572, 830, 114, 74, "Limiter", ["0–20 %,", "forward only"])
    f.edge([(348, 867), (366, 867)], "cmd")
    f.edge([(554, 867), (572, 867)], "cmd")
    f.edge([(263, 722), (263, 814)], "cmd")
    f.lines(271, 762, ["2 × PWM at 50 Hz", "990 µs = stop … 1200 µs = full (20 % cap)", "AUX1 → ch1 (right) · AUX2 → ch2 (left)"], fill=CMD)
    f.edge([(110, 830), (110, 482)], "data")
    f.lines(118, 528, ["VCB telemetry, read-only", "UART → ST-LINK VCP → USB"])
    f.device(736, 822, 212, 44, "E-stop + TSMS", ["24 V loop to the contactors"])
    f.edge([(736, 844), (700, 844)], "safe")
    f.device(736, 876, 212, 44, "MCB × 2 (aux motors)", ["LARS, REEL · not propulsion"])
    f.edge([(700, 898), (736, 898)], "data")

    # ---- thrusters ----
    f.device(500, 982, 200, 60, "Protocol converter", ["ePropulsion serial"])
    f.edge([(629, 904), (629, 982)], "cmd")
    f.text(621, 956, "UART5: speed command per motor", fill=CMD, anchor="end")
    f.device(736, 982, 248, 60, "2 × ePropulsion Navy 6.0", ["left + right outboards · 48 V"])
    f.edge([(700, 1012), (736, 1012)], "cmd")

    f.legend(20, 996, [("cmd", False, "command path"), ("data", False, "telemetry / data"),
                       ("safe", False, "hardware interlock"), ("box", False, "planned, not installed")])
    f.save()


def fig_network():
    """Figure 2: every link, with addresses, ports and protocols."""
    f = Fig("fig2_network", 1000, 848)

    f.container(16, 16, 744, 120, "Shore laptop", "tailnet member")
    f.proc(70, 39, 160, 74, "QGroundControl", ["+ tools/ap_*.py", "MAVLink, TCP 5760"])
    f.proc(242, 39, 160, 74, "Web browser", ["dashboard: HTTP 8080", "video: WebRTC 8889"])
    f.proc(414, 39, 160, 74, "ssh / mosh", ["TCP 22", "UDP 60000–61000"])
    f.proc(586, 39, 160, 74, "MOOS shoreside", ["pShare, UDP"], planned=True)
    f.device(790, 39, 194, 74, "RC transmitter", ["RadioMaster Boxer", "not an IP device"])

    f.proc(60, 170, 350, 70, "Tailscale tailnet (WireGuard)", ["relayed through DERP “nyc” today,", "no direct path yet"])
    f.edge([(235, 136), (235, 170)], "cmd", arrows="both")
    f.legend(500, 190, [("cmd", False, "IP network"), ("data", False, "USB / serial / bus"),
                        ("cmd", True, "RF link"), ("box", False, "planned")], col_w=170)

    # ---- boat ----
    f.box(8, 280, 984, 552, fill="none", stroke="#aab1bb", sw=1.2, rx=14, dash="2 5")
    f.text(926, 302, "ON THE BOAT", size=11.5, weight=700, fill=MUTED, anchor="end", spacing=1.2)
    f.device(60, 296, 350, 56, "Internet uplink", ["Starlink on the water · Wi-Fi at the dock"])
    f.edge([(235, 240), (235, 296)], "cmd", arrows="both")
    f.edge([(235, 352), (235, 388)], "cmd", arrows="both")

    f.container(16, 388, 444, 430, "Jetson Orin Nano", "hostname HighFieldBoat")
    rows = [
        ("tailscale0", "100.83.34.69", "highfieldboat.tailcacf9.ts.net"),
        ("wlP1p1s0 (Wi-Fi)", "10.31.128.214/20", "internet uplink, DHCP (address on 2026-10-04)"),
        ("enP8p1s0 (Ethernet)", "192.168.1.50/24", "camera LAN"),
        ("lo", "127.0.0.1", "UDP 14551 → MOOS bridge · UDP 14552 → dashboard"),
        ("USB", "/dev/pixhawk", "ttyACM0 · MAVLink 2 · 115200 baud"),
        ("USB", "/dev/vcb", "ST-LINK V3 virtual COM port · CSV, read-only"),
        ("USB", "FTDI FT230X", "ttyUSB0 → RS485 · 19200 8N1"),
    ]
    for i, (name, addr, note) in enumerate(rows):
        y = 400 + 58 * i
        f.box(70, y + 3, 376, 52)
        f.text(82, y + 24, name, size=13.5, weight=700)
        f.text(434, y + 24, addr, size=13.5, anchor="end")
        f.text(82, y + 42, note, size=12, fill=MUTED)
    mid = [400 + 58 * i + 29 for i in range(7)]  # row centres

    f.box(520, 403, 400, 90)
    f.text(534, 424, "Reachable over the tailnet", size=13.5, weight=700)
    f.lines(534, 443, ["ssh 22 · mosh UDP 60000–61000", "MAVLink TCP 5760 (mavlink-router)",
                       "dashboard HTTP 8080 · video RTSP 8554, HLS 8888, WebRTC 8889"], size=12, fill=MUTED)
    f.edge([(446, mid[0]), (520, mid[0])], "cmd", arrows="none")

    f.device(530, mid[2] - 24, 180, 48, "IP camera (PTZ)", ["192.168.1.110 · RTSP, ONVIF"])
    f.edge([(446, mid[2]), (530, mid[2])], "cmd", arrows="both")
    f.text(488, mid[2] - 7, "Ethernet", anchor="middle", fill=CMD)
    f.device(530, mid[4] - 24, 180, 48, "Pixhawk 6X", ["ArduRover 4.7"])
    f.device(530, mid[5] - 24, 180, 48, "VCB", ["STM32H723"])
    f.device(530, mid[6] - 24, 180, 48, "LiTime BMS", ["48 V 100 Ah pack"])
    for i in (4, 5, 6):
        f.edge([(446, mid[i]), (530, mid[i])], "data", arrows="both" if i == 4 else "start")
        f.text(488, mid[i] - 7, "USB", anchor="middle")

    f.device(800, 573, 184, 48, "ELRS receiver", ["CRSF on TELEM3"], planned=True, tag_at=870)
    f.edge([(800, 597), (755, 597), (755, 651), (710, 651)], "data", dash=True)
    f.device(800, 631, 184, 48, "GPS + compass", ["DroneCAN on CAN1"])
    f.edge([(800, 667), (710, 667)], "data")
    f.edge([(710, 677), (726, 677), (726, 709), (710, 709)], "data")
    f.text(733, 697, "2 × PWM")
    f.device(800, 703, 184, 48, "ePropulsion converter", ["→ 2 × Navy 6.0 outboards"])
    f.edge([(710, 727), (800, 727)], "data")
    f.text(763, 721, "UART5", anchor="middle")
    f.edge([(950, 113), (950, 573)], "cmd", dash=True)
    f.text(942, 250, "ELRS 2.4 GHz", fill=CMD, anchor="end")
    f.save()


def fig_control_layers():
    """Figure 3: the command chain, and what makes each layer stop."""
    f = Fig("fig3_control_layers", 1000, 838)

    f.text(16, 24, "OPERATOR", size=11.5, weight=700, fill=MUTED, spacing=1.2)
    f.proc(16, 34, 294, 84, "RC handset", ["manual sticks", "mode switch: Manual / Hold / Guided"], planned=True)
    f.proc(330, 34, 340, 84, "pMarineViewer (shore)", ["mission commands"], planned=True)
    f.proc(690, 34, 294, 84, "QGroundControl", ["arm / disarm · mode · parameters"])

    f.text(96, 172, "LAYER", size=11.5, weight=700, fill=MUTED, spacing=1.2)
    f.text(316, 172, "DECIDES", size=11.5, weight=700, fill=MUTED, spacing=1.2)
    f.text(632, 172, "STOPS OR REFUSES WHEN", size=11.5, weight=700, fill=MUTED, spacing=1.2)

    layers = [
        ("Mission autonomy", "pHelmIvP · Jetson", True,
         ["Behaviors (waypoint, station-keep, …) vote;", "the IvP solver picks one heading + speed."],
         ["MOOS_MANUAL_OVERRIDE → helm parks.", "No active behavior → all-stop (speed 0)."]),
        ("MOOS ↔ MAVLink bridge", "iArduRoverBridge · Jetson", True,
         ["Turns heading + speed into a north / east", "velocity setpoint; relays telemetry back", "to MOOS as NAV_*."],
         ["Sends only if DEPLOY, armed and in Guided.", "Setpoints older than 5 s → stops sending.", "ALLSTOP → disarm. Override → Hold."]),
        ("Vehicle control", "ArduRover 4.7 · Pixhawk 6X", False,
         ["Mode logic (Manual / Hold / Guided), EKF3", "position + heading, speed and turn-rate", "loops, skid-steer mixing."],
         ["Disarmed or Hold → both outputs at 990 µs.", "Guided: no setpoint for 3 s → stop.", "RC / GCS / fence failsafes: disabled today."]),
        ("Thruster gateway", "VCB bench firmware · STM32H723", False,
         ["Checks the PWM, arms after 1 s at stop,", "runs precharge + contactors, limits", "thrust to 0–20 % forward."],
         ["PWM missing > 100 ms or out of range,", "contactor or bus-voltage fault → FAULT,", "latched until the VCB is power-cycled."]),
        ("Propulsion", "2 × ePropulsion Navy 6.0 · 48 V", False,
         ["Protocol converter drives both outboards.", "No reverse with the bench firmware."],
         ["E-stop / TSMS: 24 V hardware loop feeding", "the contactor coils (firmware does not", "check it)."]),
    ]
    links = [  # (signal, transport, planned) on the arrow entering each layer
        ("DEPLOY · RETURN · MOOS_MANUAL_OVERRIDE", "pShare over the tailnet (UDP)", True),
        ("DESIRED_HEADING · DESIRED_SPEED", "MOOSDB mail (degrees, m/s)", True),
        ("SET_POSITION_TARGET_GLOBAL_INT (velocity)", "MAVLink: UDP 14551 → mavlink-router → USB", True),
        ("2 × PWM: 990 µs = stop … 1200 µs = full", "50 Hz · AUX1 → right, AUX2 → left", False),
        ("speed command per motor, 0–20 % forward", "UART5 → ePropulsion protocol converter", False),
    ]
    y, h, gap = 182, 86, 46
    prev_bottom = 118
    for (name, where, planned, decides, stops), (signal, transport, link_planned) in zip(layers, links):
        f.edge([(500, prev_bottom), (500, y)], "cmd", dash=link_planned)
        ly = (prev_bottom + y) / 2 + 4.5 - (12 if prev_bottom == 118 else 0)
        f.text(488, ly, signal, size=12.5, weight=700, fill=CMD, anchor="end")
        f.text(512, ly, transport, size=12.5, fill=MUTED)
        f.box(80, y, 840, h, fill=NODE_FILL, stroke=NODE_EDGE, sw=1.4, rx=10, dash="6 4" if planned else None)
        f.text(96, y + 34, name, size=15, weight=700)
        f.text(96, y + 54, where, size=12.5, fill=MUTED)
        if planned:
            f.tag(910, y)
        f.lines(316, y + 28 + (8 if len(decides) < 3 else 0), decides, lead=17)
        f.box(618, y + 8, 294, h - 16, fill="#ffffff", stroke="#d6dae0", sw=1, rx=6)
        f.add(f'<rect x="618" y="{y + 8}" width="4" height="{h - 16}" fill="{SAFE}"/>')
        f.lines(632, y + 28 + (8 if len(stops) < 3 else 0), stops, lead=17)
        prev_bottom, y = y + h, y + h + gap

    # the operator can bypass the autonomy layers and talk to ArduRover directly
    veh_y = 182 + 2 * (h + gap) + h / 2
    f.edge([(45, 118), (45, veh_y), (80, veh_y)], "cmd", dash=True)
    f.text(32, 300, "RC: sticks + mode switch (CRSF)", fill=CMD, anchor="middle", rotate=-90)
    f.edge([(955, 118), (955, veh_y), (920, veh_y)], "cmd", arrows="both")
    f.text(972, 300, "MAVLink: arm · mode · params (TCP 5760)", fill=CMD, anchor="middle", rotate=90)
    f.legend(80, 818, [("cmd", False, "deployed"), ("cmd", True, "planned, not installed")], col_w=160)
    f.save()


def fig_system_overview():
    """Figure 1, simple version: the stack at a glance, sized for the full width of a document page."""
    f = Fig("fig1_system_overview", 1000, 336)
    f.header(8, 172, "ON SHORE")
    f.header(214, 992, "ON THE BOAT")
    cards = [
        (8, 164, "Operator", ["Shore laptop"], ["Starts and stops", "missions; watches", "telemetry and video."]),
        (214, 167, "Boat computer", ["Jetson Orin Nano"],
         ["Relays telemetry and", "video. MOOS-IvP", "(planned) picks a", "heading and speed."]),
        (417, 167, "Autopilot", ["Pixhawk 6X, ArduRover"], ["Holds heading and", "speed by setting left", "and right throttle."]),
        (620, 167, "Thruster gateway", ["VCB (custom board)"], ["Safety checks and", "contactors; limits", "thrust to 20 %."]),
        (823, 169, "Thrusters", ["2 × Navy 6.0 motors"], ["Twin motors; the", "boat steers by", "differential thrust."]),
    ]
    links = [("mission commands", "Tailscale VPN", "telemetry, video"), ("heading + speed", "MAVLink, USB", "position, status"),
             ("left / right throttle", "2 × PWM", None), ("motor speed", "serial", None)]
    for x, w, title, sub, body in cards:
        f.card(x, 72, w, 150, title, sub, body, top=True)
    for (x, w, *_), (nx, *_), (signal, transport, back) in zip(cards, cards[1:], links):
        mid = (x + w + nx) / 2
        f.edge([(x + w, 104), (nx, 104)], "cmd")
        f.text(mid, 42, signal, size=13, weight=700, fill=CMD, anchor="middle")
        f.text(mid, 58, transport, size=12.5, fill=MUTED, anchor="middle")
        if back:
            f.edge([(nx, 194), (x + w, 194)], "data")
            f.text(mid, 242, back, size=12.5, fill=MUTED, anchor="middle")
    f.card(8, 266, 164, 60, "RC handset", ["manual driving"])
    f.edge([(172, 296), (500, 296), (500, 222)], "cmd", dash=True)
    f.text(336, 288, "direct radio link (not installed yet)", fill=CMD, anchor="middle")
    f.save()


def fig_network_overview():
    """Figure 2, simple version: how you reach the boat and what the boat computer is wired to."""
    f = Fig("fig2_network_overview", 1000, 392)
    f.header(8, 188, "ON SHORE")
    f.header(300, 992, "ON THE BOAT")
    f.card(8, 56, 180, 90, "Shore laptop", ["QGroundControl, web", "dashboard, video, ssh"])
    f.edge([(188, 101), (300, 101)], "cmd", arrows="both")
    f.text(244, 90, "Tailscale VPN", size=13, weight=700, fill=CMD, anchor="middle")
    f.text(244, 120, "via the internet", fill=MUTED, anchor="middle")
    f.card(300, 56, 150, 90, "Internet uplink", ["Starlink or Wi-Fi"])
    f.edge([(450, 101), (490, 101)], "cmd", arrows="both")
    f.card(490, 56, 200, 90, "Boat computer", ["Jetson Orin Nano"], ["Routes telemetry, serves", "the dashboard and video."])

    f.card(760, 40, 232, 48, "Camera", ["Ethernet"])
    f.card(760, 112, 232, 48, "Battery", ["BMS over USB–RS485"])
    for pts in ([(690, 101), (725, 101)], [(725, 64), (725, 136)], [(725, 64), (760, 64)], [(725, 136), (760, 136)],
                [(590, 146), (590, 204)], [(415, 204), (655, 204)], [(415, 204), (415, 262)], [(655, 204), (655, 262)]):
        f.edge(pts, "data", arrows="none")
    f.text(423, 240, "USB")
    f.text(663, 240, "USB, monitor only")

    f.card(8, 262, 180, 80, "RC handset", ["not on the network"])
    f.card(330, 262, 170, 80, "Autopilot", ["Pixhawk 6X, ArduRover"])
    f.card(560, 262, 190, 80, "Thruster gateway", ["VCB (custom board)"])
    f.card(810, 262, 182, 80, "Thrusters", ["2 × Navy 6.0 motors"])
    f.edge([(188, 302), (330, 302)], "cmd", dash=True)
    f.text(259, 292, "radio link", fill=CMD, anchor="middle")
    f.text(259, 320, "(not installed yet)", size=12, fill=CMD, anchor="middle")
    f.edge([(500, 302), (560, 302)], "data")
    f.text(530, 294, "PWM", anchor="middle")
    f.edge([(750, 302), (810, 302)], "data")
    f.text(780, 294, "serial", anchor="middle")
    f.legend(330, 374, [("cmd", False, "internet"), ("data", False, "wired link"), ("cmd", True, "radio")], col_w=170, per_row=3)
    f.save()


if __name__ == "__main__":
    if not shutil.which("rsvg-convert"):
        raise SystemExit("rsvg-convert not found (macOS: brew install librsvg, Ubuntu: apt install librsvg2-bin)")
    fig_system_overview()
    fig_architecture()
    fig_network_overview()
    fig_network()
    fig_control_layers()
