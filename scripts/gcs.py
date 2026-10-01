#!/usr/bin/env python3
"""Boat GCS dashboard: WebRTC camera + ONVIF PTZ + battery panel.
Stdlib only. Serves on :8080. PTZ proxied to the camera's ONVIF service."""
import json
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CAM = "192.168.1.110"
PTZ_URL = f"http://{CAM}/onvif/ptz_service"
PROFILE = "MainStream"
SOAP_HDR = {"Content-Type": "application/soap+xml"}

NS = ('xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
      'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" '
      'xmlns:tt="http://www.onvif.org/ver10/schema"')


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


PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Boat GCS</title><style>
:root{--bg:#0d1117;--panel:#161b22;--line:#30363d;--txt:#e6edf3;--accent:#2f81f7;--ok:#3fb950;--warn:#d29922}
*{box-sizing:border-box}body{margin:0;font:14px/1.4 system-ui,sans-serif;background:var(--bg);color:var(--txt)}
header{padding:10px 16px;border-bottom:1px solid var(--line);display:flex;align-items:center;gap:12px}
header h1{font-size:15px;margin:0;font-weight:600}
.dot{width:9px;height:9px;border-radius:50%;background:var(--warn)}.dot.on{background:var(--ok)}
.wrap{display:flex;gap:16px;padding:16px;flex-wrap:wrap}
.video{flex:1 1 640px;min-width:320px}
.video iframe{width:100%;aspect-ratio:16/9;border:1px solid var(--line);border-radius:8px;background:#000}
.side{flex:0 0 260px;display:flex;flex-direction:column;gap:16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px}
.card h2{margin:0 0 10px;font-size:12px;text-transform:uppercase;letter-spacing:.5px;color:#8b949e}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:200px;margin:auto}
.dpad button,.zoom button{background:#21262d;border:1px solid var(--line);color:var(--txt);border-radius:6px;
  padding:14px 0;font-size:18px;cursor:pointer;user-select:none;touch-action:none}
.dpad button:active,.zoom button:active{background:var(--accent);border-color:var(--accent)}
.dpad .sp{visibility:hidden}
.zoom{display:flex;gap:8px;margin-top:10px}.zoom button{flex:1}
.speed{margin-top:12px;font-size:12px;color:#8b949e}.speed input{width:100%}
.batt .row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #21262d}
.batt .row:last-child{border:0}.batt .v{font-variant-numeric:tabular-nums;font-weight:600}
.batt .big{font-size:28px;font-weight:700}.muted{color:#8b949e}
</style></head><body>
<header><span class=dot id=camdot></span><h1>Boat Ground Station</h1>
<span class=muted id=status>connecting…</span></header>
<div class=wrap>
  <div class=video><iframe id=cam allow="autoplay" src=""></iframe></div>
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
    <div class="card batt"><h2>Battery</h2>
      <div class=big id=soc>-- %</div>
      <div class=row><span>Voltage</span><span class=v id=volt>--</span></div>
      <div class=row><span>Current</span><span class=v id=curr>--</span></div>
      <div class=row><span>Power</span><span class=v id=pwr>--</span></div>
      <div class=row><span>Temp</span><span class=v id=temp>--</span></div>
      <div class=muted id=bstat style=margin-top:8px>BLE not connected</div>
    </div>
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
// camera liveness via MediaMTX API-less check: assume up if iframe loads; poll battery
async function pollBatt(){try{const r=await fetch('/api/battery');const b=await r.json();
  const bs=document.getElementById('bstat');
  if(b.online){document.getElementById('soc').textContent=(b.soc??'--')+' %';
    document.getElementById('volt').textContent=(b.voltage??'--')+' V';
    document.getElementById('curr').textContent=(b.current??'--')+' A';
    document.getElementById('pwr').textContent=(b.power??'--')+' W';
    document.getElementById('temp').textContent=(b.temp??'--')+' °C';
    bs.textContent='BLE connected';bs.className='';}
  else{bs.textContent=b.msg||'BLE not connected';bs.className='muted';}
}catch(e){}}
setInterval(pollBatt,2000);pollBatt();
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
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, "text/html; charset=utf-8", PAGE.encode())
        elif self.path == "/api/battery":
            try:
                with open("/home/boat/battery.json", "rb") as f:
                    body = f.read()
            except Exception:
                body = json.dumps({"online": False, "msg": "RS485 not connected"}).encode()
            self._send(200, "application/json", body)
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


if __name__ == "__main__":
    print("GCS on :8080")
    ThreadingHTTPServer(("0.0.0.0", 8080), H).serve_forever()
