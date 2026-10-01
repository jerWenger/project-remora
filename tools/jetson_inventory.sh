#!/usr/bin/env bash
# Jetson-side discovery. Run ON the Jetson (ssh boat@highfieldboat.tailcacf9.ts.net):
#     bash jetson_inventory.sh | tee jetson_$(date +%F).txt
# Everything is read-only. Sections degrade gracefully when a tool is missing.

sec() { printf '\n=== %s %s\n' "$1" "$(printf '=%.0s' $(seq 1 $((60 - ${#1}))))"; }
have() { command -v "$1" >/dev/null 2>&1; }

sec "HOST"
hostname; uptime
uname -a
[ -f /etc/nv_tegra_release ] && cat /etc/nv_tegra_release
[ -f /etc/os-release ] && grep -E '^(PRETTY_NAME|VERSION_ID)' /etc/os-release
have timedatectl && timedatectl | grep -E 'Local time|synchronized|NTP service'
cat /proc/device-tree/model 2>/dev/null && echo

sec "NETWORK"
ip -br addr
if have tailscale; then
  tailscale status --self --peers=false 2>/dev/null
  tailscale status --json 2>/dev/null | python3 -c '
import json,sys; d=json.load(sys.stdin); s=d["Self"]
print("tailscale:", s["HostName"], s["TailscaleIPs"], "relay=" + str(s.get("Relay")), "online=" + str(s["Online"]))' 2>/dev/null
fi
have ss && { echo "-- listening ports"; ss -tulpn 2>/dev/null | grep -v '^Netid' | sort -k5; }

sec "USB"
have lsusb && lsusb
echo "-- /dev/serial/by-id"
ls -l /dev/serial/by-id 2>/dev/null || echo "(none)"
for d in /dev/ttyACM* /dev/ttyUSB* /dev/ttyTHS*; do
  [ -e "$d" ] || continue
  echo "-- $d"
  udevadm info -q property -n "$d" 2>/dev/null | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_VENDOR|ID_MODEL|ID_SERIAL_SHORT|ID_USB_INTERFACE_NUM|DEVPATH)=' | sed 's/^/   /'
  stat -c '   owner=%U group=%G mode=%a' "$d"
done
echo "-- user 'boat' groups:"; id boat 2>/dev/null
echo "-- processes holding serial devices:"
have fuser && fuser -v /dev/ttyACM* /dev/ttyUSB* 2>&1 | grep -v '^$' || true

sec "CAN"
ip -d -br link show type can 2>/dev/null || echo "(no CAN interfaces on the Jetson; GPS CAN goes to the Pixhawk)"

sec "SERVICES"
for u in boat-gcs mediamtx battery-litime mavlink-router mavlink-routerd tailscaled; do
  printf '%-18s %s\n' "$u" "$(systemctl is-enabled $u 2>/dev/null)/$(systemctl is-active $u 2>/dev/null)"
done
echo "-- user-started processes of interest"
pgrep -af 'mavlink-routerd|mediamtx|gcs.py|battery_|MOOSDB|pHelmIvP|pAntler' || echo "(none)"
if have mavlink-routerd; then mavlink-routerd --version 2>&1 | head -1; fi
for f in /etc/mavlink-router/main.conf /home/boat/mavlink-router/main.conf ~/.config/mavlink-router/main.conf; do
  [ -f "$f" ] && { echo "-- $f"; sed 's/^/   /' "$f"; }
done

sec "BATTERY (BMS on Jetson)"
f=/home/boat/battery.json
if [ -f "$f" ]; then
  echo "age: $(( $(date +%s) - $(stat -c %Y "$f") )) s"; cat "$f"; echo
else echo "(no $f)"; fi

sec "RESOURCES"
df -h / /home 2>/dev/null | grep -v '^Filesystem' | sort -u
free -h | grep -E 'Mem|Swap'
for t in /sys/class/thermal/thermal_zone*; do
  [ -f "$t/temp" ] && printf '%-14s %5.1f C\n' "$(cat $t/type)" "$(( $(cat $t/temp) ))e-3"
done 2>/dev/null
have nvpmodel && nvpmodel -q 2>/dev/null | head -2

sec "SOFTWARE"
python3 --version
for m in pymavlink serial mavsdk fastapi uvicorn; do
  printf '%-10s %s\n' "$m" "$(python3 -c "import $m,sys;print(getattr($m,'__version__','present'))" 2>/dev/null || echo MISSING)"
done
for b in MOOSDB pHelmIvP pAntler iArduRoverBridge mavproxy.py; do
  printf '%-16s %s\n' "$b" "$(command -v $b || echo MISSING)"
done
[ -d ~/moos-ivp ] && (cd ~/moos-ivp && echo "moos-ivp: $(git describe --always 2>/dev/null || svn info 2>/dev/null | grep Revision)")

sec "RECENT KERNEL USB/SERIAL EVENTS"
have journalctl && journalctl -k -b --no-pager 2>/dev/null | grep -iE 'ttyACM|ttyUSB|cdc_acm|ftdi|usb .*(new|disconnect)' | tail -15
