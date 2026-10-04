#!/usr/bin/env bash
# Install the boat-side services on the Jetson from this repo checkout. Idempotent.
#   sudo deploy/install.sh [--dry-run] [--restart]
# --dry-run  print what would be done, change nothing (no sudo needed)
# --restart  restart services that are already running (default: only start stopped ones)
# Does not touch mediamtx's cam.yml. Does not remove the old mavlink-router shell loop;
# it finds it and prints the steps (see docs/JETSON.md).
set -euo pipefail

BOAT=boat
HOME_B=/home/boat
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TS=$(date +%Y%m%d_%H%M%S)
UNITS=(boat-gcs mediamtx mavlink-router battery-litime vcb-logger)
SCRIPTS=(gcs.py battery_litime.py vcb_logger.py)
DRY=0 RESTART=0

for a in "$@"; do
    case $a in
        --dry-run) DRY=1 ;;
        --restart) RESTART=1 ;;
        -h|--help) sed -n '2,7p' "$0"; exit 0 ;;
        *) echo "unknown arg: $a" >&2; exit 2 ;;
    esac
done

if [[ $DRY == 0 && $EUID != 0 ]]; then
    echo "run with sudo (or --dry-run)" >&2; exit 1
fi

run() { echo "+ $*"; if [[ $DRY == 0 ]]; then "$@"; fi; }
say() { printf '\n== %s\n' "$*"; }

# Copy src to dst if different; back up a differing dst first. Returns 1 if unchanged.
put() {
    local src=$1 dst=$2 mode=$3 owner=${4:-root}
    if [[ -f $dst ]] && cmp -s "$src" "$dst"; then echo "  ok $dst"; return 1; fi
    if [[ -e $dst ]]; then run cp -p "$dst" "$dst.bak.$TS"; fi
    run install -m "$mode" -o "$owner" -g "$owner" "$src" "$dst"
}

say "repo: $REPO"
[[ -f $REPO/scripts/vcb_logger.py ]] || { echo "not a project-remora checkout?" >&2; exit 1; }

say "udev rules"
changed=0
for r in "$REPO"/systemd/*.rules; do
    put "$r" "/etc/udev/rules.d/$(basename "$r")" 644 && changed=1
done
if [[ $changed == 1 ]]; then
    run udevadm control --reload
    run udevadm trigger --subsystem-match=tty --action=add
fi

say "dirs"
for d in mavlogs vcblogs; do
    run install -d -m 755 -o "$BOAT" -g "$BOAT" "$HOME_B/$d"
done

say "mavlink-router config"
put "$REPO/config/mav.conf.new" "$HOME_B/mav.conf" 644 "$BOAT" || true

say "script symlinks"
for s in "${SCRIPTS[@]}"; do
    tgt=$REPO/scripts/$s lnk=$HOME_B/$s
    if [[ -L $lnk && $(readlink "$lnk") == "$tgt" ]]; then echo "  ok $lnk"; continue; fi
    if [[ -e $lnk || -L $lnk ]]; then run mv "$lnk" "$lnk.bak.$TS"; fi
    run ln -s "$tgt" "$lnk"
    run chown -h "$BOAT:$BOAT" "$lnk"
done

say "old mavlink-router loop (not systemd)"
hits=()
check() { # label, command...
    local label=$1; shift
    local out; out=$("$@" 2>/dev/null | grep -n 'mavlink-routerd' || true)
    if [[ -n $out ]]; then hits+=("$label"); printf '  FOUND in %s:\n%s\n' "$label" "$out"; fi
}
check "crontab -u $BOAT" crontab -l -u "$BOAT"
check "crontab -u root" crontab -l -u root
for f in /etc/rc.local /etc/crontab "$HOME_B/.bashrc" "$HOME_B/.profile" /root/.bashrc; do
    if [[ -f $f ]]; then check "$f" cat "$f"; fi
done
while IFS= read -r f; do
    [[ $f == /etc/systemd/system/mavlink-router.service ]] && continue
    check "$f" cat "$f"
done < <(grep -rls 'mavlink-routerd' /etc/systemd /etc/cron.d /etc/profile.d "$HOME_B/.config/autostart" 2>/dev/null || true)
# [m] keeps the pattern from matching the pgrep/pkill command line itself
loop_pid=$(pgrep -f 'while true; do .*[m]avlink-routerd' | tr '\n' ' ' || true)
if [[ -n $loop_pid ]]; then echo "  RUNNING loop shell: pid(s) $loop_pid"; fi
old_loop=0
if [[ ${#hits[@]} -gt 0 || -n $loop_pid ]]; then
    old_loop=1
    cat <<EOF
  To switch to mavlink-router.service (do this BEFORE the next reboot, or two routers
  will share the Pixhawk port):
    1. Delete the 'mavlink-routerd' line from: ${hits[*]:-<not found in files; check cron/rc.local by hand>}
       (crontab -e -u <user> / sudo nano <file>)
    2. Stop the running loop:  sudo pkill -f 'while true; do .*[m]avlink-routerd'; sudo pkill -x mavlink-routerd
    3. sudo systemctl enable --now mavlink-router && systemctl status mavlink-router
EOF
else
    echo "  none found"
fi

say "systemd units"
for u in "${UNITS[@]}"; do
    put "$REPO/systemd/$u.service" "/etc/systemd/system/$u.service" 644 || true
done
run systemctl daemon-reload
# Enabling mavlink-router while the old loop exists would start a second router on the
# Pixhawk port the next time it re-enumerates (WantedBy=dev-pixhawk.device).
enable=()
for u in "${UNITS[@]}"; do
    if [[ $u == mavlink-router && $old_loop == 1 ]]; then
        echo "  not enabling $u yet: old loop present (see above), enable it after removing the loop"; continue
    fi
    enable+=("$u.service")
done
run systemctl enable "${enable[@]}"

say "logrotate + journald"
put "$REPO/deploy/logrotate-boat" /etc/logrotate.d/boat 644 || true
if [[ $DRY == 0 ]]; then logrotate -d /etc/logrotate.d/boat >/dev/null 2>&1 \
    || echo "  WARNING: logrotate -d /etc/logrotate.d/boat reports errors"; fi
run install -d -m 755 /etc/systemd/journald.conf.d
if put "$REPO/deploy/journald-boat.conf" /etc/systemd/journald.conf.d/boat.conf 644; then
    run install -d -m 2755 -g systemd-journal /var/log/journal
    run systemd-tmpfiles --create --prefix /var/log/journal
    run systemctl restart systemd-journald
fi

say "services"
for u in "${UNITS[@]}"; do
    if [[ $u == mavlink-router && $old_loop == 1 ]]; then
        echo "  skip $u: old loop still running (see above)"; continue
    fi
    if systemctl is-active --quiet "$u"; then
        if [[ $RESTART == 1 ]]; then run systemctl restart "$u" || echo "  WARNING: $u restart failed"; else echo "  running $u (use --restart to pick up changes)"; fi
    else
        run systemctl start "$u" || echo "  WARNING: $u failed to start: journalctl -u $u"
    fi
done

say "python deps"
if sudo -u "$BOAT" python3 -c 'import pymavlink' 2>/dev/null; then echo "  ok pymavlink"
else echo "  pymavlink missing for $BOAT: dashboard vehicle card is off (pip3 install --user pymavlink)"; fi

say "done"
echo "verify: systemctl status ${UNITS[*]}; ls -l /dev/pixhawk /dev/vcb; cat $HOME_B/vcb.json"
