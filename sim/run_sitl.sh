#!/usr/bin/env bash
# run_sitl.sh -- ArduRover motorboat-skid SITL with the HighFieldBoat endpoint layout.
#
#   TCP 5760          SERIAL0  QGC / ap_inventory.py           (boat: mavlink-router TcpServerPort)
#   UDP 14551 -> out  SERIAL1  iArduRoverBridge (udpin 14551)  (boat: [UdpEndpoint moos])
#   UDP 14552 -> out  SERIAL2  dashboard gcs.py (udpin 14552)  (boat: [UdpEndpoint gui])
#
# With -I N every port above shifts by 10*N (5760+10N, 14551+10N, 14552+10N),
# and so do SITL's internal ports (RC-in 5501+10N, ...).
#
# Usage: sim/run_sitl.sh [-I N] [--speedup X] [--wipe] [--home lat,lon,alt,hdg] [--build]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/dev/ardupilot}"
NATIVE_BUILD="$HERE/run/ap-build"

INSTANCE=0
SPEEDUP=1
WIPE=0
BUILD=0
# MIT Sailing Pavilion is 42.358436,-71.087448 (on the dock). This point is
# ~80 m south of it, in open water of the Charles River basin.
HOME_LOC="42.3577,-71.0872,0,0"

usage() {
    sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'
    cat <<EOF

Options:
  -I, --instance N   instance number; shifts all ports by 10*N (default 0)
  -s, --speedup X    simulation speedup (default 1)
  -w, --wipe         start from a fresh eeprom (re-applies boat-sitl.parm fully)
  --home L           lat,lon,alt,heading (default $HOME_LOC)
  --build            (re)build a native ArduRover SITL into sim/run/ap-build first
Environment:
  ARDUPILOT_DIR      ArduPilot checkout (default ~/dev/ardupilot)
  ARDUROVER          explicit path to an ardurover SITL binary
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -I|--instance) INSTANCE="$2"; shift 2 ;;
        -s|--speedup)  SPEEDUP="$2"; shift 2 ;;
        -w|--wipe)     WIPE=1; shift ;;
        --home)        HOME_LOC="$2"; shift 2 ;;
        --build)       BUILD=1; shift ;;
        -h|--help)     usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done
[[ "$INSTANCE" =~ ^[0-9]+$ ]] || { echo "instance must be a non-negative integer" >&2; exit 2; }

# --- build (optional) -------------------------------------------------------
# The prebuilt $ARDUPILOT_DIR/build/sitl/bin/ardurover may be x86_64, which does
# not run on Apple Silicon without Rosetta. --build makes a native one in
# sim/run/ap-build. WAFLOCK keeps waf from overwriting the checkout's own
# .lock-waf_darwin_build (which would redirect plain ./waf to our build dir).
# -no_fixup_chains works around the Xcode 26+ linker rejecting the packed
# AP_FWVersion struct ("pointer not aligned").
if [[ $BUILD == 1 ]]; then
    (
        cd "$ARDUPILOT_DIR"
        export WAFLOCK=.lock-waf_darwin_hfb_sim
        LDFLAGS="-Wl,-no_fixup_chains" ./waf configure --board sitl -o "$NATIVE_BUILD"
        ./waf rover -o "$NATIVE_BUILD"
    )
fi

# --- pick a runnable binary --------------------------------------------------
can_run() { [[ -x "$1" ]] && "$1" --help >/dev/null 2>&1; }
BIN=""
for cand in "${ARDUROVER:-}" "$NATIVE_BUILD/sitl/bin/ardurover" "$ARDUPILOT_DIR/build/sitl/bin/ardurover"; do
    [[ -n "$cand" ]] || continue
    if can_run "$cand"; then BIN="$cand"; break; fi
    [[ -e "$cand" ]] && echo "skipping $cand (not runnable on $(uname -m): $(file -b "$cand" | cut -c1-40))" >&2
done
[[ -n "$BIN" ]] || { echo "no runnable ardurover SITL binary; try: $0 --build" >&2; exit 1; }

# --- param layering ----------------------------------------------------------
# Same files sim_vehicle.py uses for motorboat-skid (Tools/autotest/pysim/vehicleinfo.py),
# then our boat-specific overlay last so it wins.
DP="$ARDUPILOT_DIR/Tools/autotest/default_params"
DEFAULTS="$DP/rover.parm,$DP/motorboat.parm,$DP/rover-skid.parm,$HERE/boat-sitl.parm"
for f in ${DEFAULTS//,/ }; do [[ -f "$f" ]] || { echo "missing defaults file $f" >&2; exit 1; }; done

# --- ports ---------------------------------------------------------------------
TCP_PORT=$((5760 + 10 * INSTANCE))
MOOS_PORT=$((14551 + 10 * INSTANCE))
GUI_PORT=$((14552 + 10 * INSTANCE))
RC_PORT=$((5501 + 10 * INSTANCE))

if lsof -nP -iTCP:"$TCP_PORT" -sTCP:LISTEN >/dev/null 2>&1; then
    echo "TCP $TCP_PORT is already in use -- another SITL on instance $INSTANCE? pick another -I" >&2
    exit 1
fi

# --- run dir -------------------------------------------------------------------
RUN_DIR="$HERE/run/I$INSTANCE"          # eeprom.bin, logs/, terrain/ land here
STATE_DIR="$HERE/run/state"             # fake dashboard state (GCS_STATE_DIR)
mkdir -p "$RUN_DIR" "$STATE_DIR"
WIPE_ARG=()
[[ $WIPE == 1 ]] && WIPE_ARG=(--wipe)

# Fake battery for the dashboard (same shape as scripts/battery_litime.py output).
# Rewritten every 2 s so freshness checks based on mtime stay happy.
write_battery() {
    local tmp="$STATE_DIR/.battery.json.tmp"
    printf '{"online": true, "sim": true, "voltage": 52.8, "current": -3.1, "power": -164, "soc": 87, "soh": 100, "temp": 21, "full_ah": 100.0, "cells": [%s]}\n' \
        "$(printf '3.300,%.0s' {1..15})3.300" > "$tmp" && mv "$tmp" "$STATE_DIR/battery.json"
}

PIDS=()
cleanup() {
    trap - INT TERM EXIT
    for p in ${PIDS[@]+"${PIDS[@]}"}; do
        [[ -n "$p" ]] && kill "$p" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "run_sitl.sh: stopped instance $INSTANCE"
}
trap cleanup INT TERM EXIT

( while true; do write_battery; sleep 2; done ) &
PIDS+=($!)

cat <<EOF
ArduRover SITL (motorboat-skid), instance $INSTANCE, speedup $SPEEDUP
  binary   $BIN
  run dir  $RUN_DIR
  home     $HOME_LOC
  TCP  $TCP_PORT   QGC / ap_inventory.py        (tcp:127.0.0.1:$TCP_PORT)
  UDP  $MOOS_PORT  iArduRoverBridge             (udpin 0.0.0.0:$MOOS_PORT)
  UDP  $GUI_PORT  dashboard gcs.py / smoke test (udpin 0.0.0.0:$GUI_PORT)
  RC   $RC_PORT   SITL RC input (UDP, 8-16 little-endian uint16 PWM values)
  fake dashboard state: $STATE_DIR
Ctrl-C to stop.
EOF

cd "$RUN_DIR"
# serial0: "tcp:0" (no ":wait"), so the vehicle boots and streams even with no
# TCP client, as on the boat where mavlink-router keeps the serial link open.
"$BIN" \
    --model motorboat-skid \
    --instance "$INSTANCE" \
    --speedup "$SPEEDUP" \
    --home "$HOME_LOC" \
    --defaults "$DEFAULTS" \
    --sysid 1 \
    --serial0 tcp:0 \
    --serial1 "udpclient:127.0.0.1:$MOOS_PORT" \
    --serial2 "udpclient:127.0.0.1:$GUI_PORT" \
    ${WIPE_ARG[@]+"${WIPE_ARG[@]}"} &
PIDS+=($!)
SITL_PID=$!
set +e
wait "$SITL_PID"
RC=$?
set -e
echo "ardurover exited with status $RC"
exit "$RC"
