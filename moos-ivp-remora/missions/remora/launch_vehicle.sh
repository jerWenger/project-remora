#!/bin/bash
#------------------------------------------------------------
#   Script: launch_vehicle.sh
#  Mission: remora
#   Author: Jeremy Wenger
#  Purpose: Build targs/targ_<vname>.moos/.bhv with nsplug and launch
#           the vehicle community. Default mode is ARDUROVER
#           (iArduRoverBridge talks to ArduRover over MAVLink); --sim
#           swaps in uSimMarineV22 + pMarinePIDV22.
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

cd "$(dirname "$0")"

# This tree's own apps first: other MOOS trees on the PATH (the bridge was
# copied from one) must not supply an older iArduRoverBridge.
if [ -d ../../bin ]; then PATH="$(cd ../../bin && pwd):$PATH"; export PATH; fi

#------------------------------------------------------------
#  Part 1: Defaults
#------------------------------------------------------------
ME=$(basename "$0")
CMD_ARGS=""
TIME_WARP=1
VERBOSE=""
JUST_MAKE="no"
AUTO_LAUNCHED="no"

IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"
SHORE_IP="localhost"
SHORE_PSHARE="9200"

VNAME="remora"
COLOR="yellow"
XMODE="ARDUROVER"
SPEED=""
MAX_SPD="2.0"

# Survey pattern, in the pavilion frame (x east, y north of the origin)
PATTERN="box"
FIG8_CENTER="0,-100"      # middle of the 2026-11-15 tuning area
FIG8_SIZE="120"           # m along the long axis; half that across
FIG8_ROT="90"             # long axis, deg clockwise from north
LAPS="2"
OP_MARGIN="30"            # fig8: op region = pattern box + this, m

# The bridge posts BATTERY_OK=false below either limit and the helm returns
RETURN_VOLTS="50.0"
RETURN_PCT="20"

# SITL home (sim/run_sitl.sh) is ~ x=20, y=-82 in the pavilion frame.
START_POS="x=20,y=-82,heading=180"
RETURN_POS="20,-82"

# iArduRoverBridge: udpin on this port. 14551 on the boat and for SITL
# instance 0; --sitl_instance=N adds 10*N (see ../../../sim/README.md).
MAV_PORT="14551"
MAV_PROTO="udpin"
MAV_HOST="0.0.0.0"
SITL_INSTANCE=""
BRIDGE_DEBUG="false"

# Where battery.json and vcb.json live (the boat-gcs state dir). Used only
# if the files are there, so SITL and laptop launches run without them.
STATE_DIR="/home/boat"

#------------------------------------------------------------
#  Part 2: Arguments
#------------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [OPTIONS] [time_warp]                                "
        echo "                                                         "
        echo "Options:                                                 "
        echo "  --help, -h             Show this help message          "
        echo "  --just_make, -j        Only create targ files          "
        echo "  --verbose, -v          Verbose, confirm launch         "
        echo "  --auto, -a             Script launch, no uMAC          "
        echo "                                                         "
        echo "  --sim, -s              uSimMarineV22 instead of ArduRover"
        echo "  --sitl_instance=N      ArduRover SITL instance N: listen "
        echo "                         on 14551+10*N (default N=0)     "
        echo "  --mav_port=<14551>     Explicit MAVLink udpin port     "
        echo "  --boat_tcp             Connect out to the boat's router over   "
        echo "                         Tailscale (tcpout :5760). Link check   "
        echo "                         from the laptop, not for driving       "
        echo "  --state_dir=</home/boat>  Dir with battery.json / vcb.json;  "
        echo "                         each is used only if it exists  "
        echo "  --debug                iArduRoverBridge debug log      "
        echo "                                                         "
        echo "  --ip=<localhost>       This host's IP for pShare/pHostInfo"
        echo "                         (on the Jetson: its Tailscale IP)"
        echo "  --mport=<9001>         Vehicle MOOSDB port             "
        echo "  --pshare=<9201>        Vehicle pShare listen port      "
        echo "  --shore=<localhost>    Shoreside IP (laptop)           "
        echo "  --shore_pshare=<9200>  Shoreside pShare port           "
        echo "                                                         "
        echo "  --vname=<remora>       Vehicle name                    "
        echo "  --color=<yellow>       Vehicle color                   "
        echo "  --start_pos=<x=,y=,heading=>  Sim start (--sim only)   "
        echo "  --return_pos=<x,y>     Return point                    "
        echo "  --speed=<m/s>          Survey/return speed (box 1.0,   "
        echo "                         fig8 1.5)                       "
        echo "  --pattern=<box>        box or fig8                     "
        echo "  --fig8_center=<0,-100> fig8 centre x,y (m)             "
        echo "  --fig8_size=<120>      fig8 length (m); width is half  "
        echo "  --fig8_rot=<90>        fig8 long axis, deg cw from N   "
        echo "  --laps=<2>             fig8 laps                       "
        echo "  --op_margin=<30>       fig8 op region margin (m)       "
        echo "  --return_volts=<50.0>  Return below this pack voltage  "
        echo "  --return_pct=<20>      Return below this state of charge"
        echo "  --max_spd=<2.0>        Helm/sim speed domain max       "
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ]; then
        XMODE="SIM"
    elif [ "${ARGI:0:16}" = "--sitl_instance=" ]; then
        SITL_INSTANCE="${ARGI#--sitl_instance=*}"
    elif [ "${ARGI:0:11}" = "--mav_port=" ]; then
        MAV_PORT="${ARGI#--mav_port=*}"
    elif [ "${ARGI}" = "--boat_tcp" ]; then
        MAV_PROTO="tcpout"
        MAV_HOST="highfieldboat.tailcacf9.ts.net"
        MAV_PORT="5760"
    elif [ "${ARGI:0:12}" = "--state_dir=" ]; then
        STATE_DIR="${ARGI#--state_dir=*}"
    elif [ "${ARGI}" = "--debug" ]; then
        BRIDGE_DEBUG="true"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
        MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--color=" ]; then
        COLOR="${ARGI#--color=*}"
    elif [ "${ARGI:0:12}" = "--start_pos=" ]; then
        START_POS="${ARGI#--start_pos=*}"
    elif [ "${ARGI:0:13}" = "--return_pos=" ]; then
        RETURN_POS="${ARGI#--return_pos=*}"
    elif [ "${ARGI:0:10}" = "--pattern=" ]; then
        PATTERN="${ARGI#--pattern=*}"
    elif [ "${ARGI:0:14}" = "--fig8_center=" ]; then
        FIG8_CENTER="${ARGI#--fig8_center=*}"
    elif [ "${ARGI:0:12}" = "--fig8_size=" ]; then
        FIG8_SIZE="${ARGI#--fig8_size=*}"
    elif [ "${ARGI:0:11}" = "--fig8_rot=" ]; then
        FIG8_ROT="${ARGI#--fig8_rot=*}"
    elif [ "${ARGI:0:7}" = "--laps=" ]; then
        LAPS="${ARGI#--laps=*}"
    elif [ "${ARGI:0:12}" = "--op_margin=" ]; then
        OP_MARGIN="${ARGI#--op_margin=*}"
    elif [ "${ARGI:0:15}" = "--return_volts=" ]; then
        RETURN_VOLTS="${ARGI#--return_volts=*}"
    elif [ "${ARGI:0:13}" = "--return_pct=" ]; then
        RETURN_PCT="${ARGI#--return_pct=*}"
    elif [ "${ARGI:0:8}" = "--speed=" ]; then
        SPEED="${ARGI#--speed=*}"
    elif [ "${ARGI:0:10}" = "--max_spd=" ]; then
        MAX_SPD="${ARGI#--max_spd=*}"
    else
        echo "$ME: Bad Arg: [$ARGI]. Exit Code 1."
        exit 1
    fi
done

# Pattern -> waypoints, repeat count and op region
if [ "$PATTERN" = "box" ]; then
    SURVEY_PTS="pts={0,-60:40,-60:40,-100:0,-100}"
    SURVEY_REPEAT=0
    OPREGION="pts={60,10:-75.54,-54.26:-22,-167:112,-107}"
    WPT_RADIUS=5.0; WPT_SLIP=12.0; WPT_LEAD=8
    [ -z "$SPEED" ] && SPEED="1.0"
elif [ "$PATTERN" = "fig8" ]; then
    # Lemniscate of Gerono, 28 points starting at the crossing: along the
    # long axis a*sin(t), across it (a/2)*sin(2t). The tightest radius is
    # about a/3 (20 m at the default size), no corners anywhere.
    FIG8=$(python3 - "$FIG8_CENTER" "$FIG8_SIZE" "$FIG8_ROT" "$OP_MARGIN" <<'PYEOF'
import math, sys
cx, cy = (float(v) for v in sys.argv[1].split(","))
size, rot, margin = float(sys.argv[2]), math.radians(float(sys.argv[3])), float(sys.argv[4])
a, n = size / 2, 28
pts = []
for k in range(n):
    t = 2 * math.pi * k / n
    along, across = a * math.sin(t), a / 2 * math.sin(2 * t)
    pts.append((cx + along * math.sin(rot) + across * math.cos(rot),
                cy + along * math.cos(rot) - across * math.sin(rot)))
print("pts={" + ":".join(f"{x:.1f},{y:.1f}" for x, y in pts) + "}")
xs, ys = [p[0] for p in pts], [p[1] for p in pts]
x0, x1, y0, y1 = min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin
print("pts={" + f"{x0:.0f},{y1:.0f}:{x1:.0f},{y1:.0f}:{x1:.0f},{y0:.0f}:{x0:.0f},{y0:.0f}" + "}")
PYEOF
    ) || { echo "$ME: could not build the fig8 points (python3?)"; exit 1; }
    SURVEY_PTS=$(echo "$FIG8" | sed -n 1p)
    OPREGION=$(echo "$FIG8" | sed -n 2p)
    SURVEY_REPEAT=$((LAPS - 1))
    # Points are ~12 m apart on a curve: capture each one late and look a
    # short way ahead, or the track cuts inside the loops (sim: 95th pct
    # cross-track 3.9 m with the box settings, 2.0 m with these).
    WPT_RADIUS=3.0; WPT_SLIP=8.0; WPT_LEAD=5
    [ -z "$SPEED" ] && SPEED="1.5"
else
    echo "$ME: --pattern must be box or fig8, not [$PATTERN]. Exit Code 1."
    exit 1
fi

BMS_FILE=""
VCB_FILE=""
if [ -f "$STATE_DIR/battery.json" ]; then BMS_FILE="$STATE_DIR/battery.json"; fi
if [ -f "$STATE_DIR/vcb.json" ];     then VCB_FILE="$STATE_DIR/vcb.json";     fi

if [ -n "$SITL_INSTANCE" ]; then
    MAV_PORT=$((14551 + 10 * SITL_INSTANCE))
fi

if [ "$XMODE" = "ARDUROVER" -a "$TIME_WARP" != "1" ]; then
    echo "$ME: ArduRover runs in real time; time warp must be 1 in ARDUROVER mode."
    exit 1
fi

#------------------------------------------------------------
#  Part 3: Verbose summary
#------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then
    echo "============================================"
    echo "  $ME SUMMARY                  $VNAME"
    echo "============================================"
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "JUST_MAKE =     [${JUST_MAKE}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "----------------------------------"
    echo "XMODE =         [${XMODE}]        "
    echo "MAV_PORT =      [${MAV_PORT}]     "
    echo "BRIDGE_DEBUG =  [${BRIDGE_DEBUG}] "
    echo "BMS_FILE =      [${BMS_FILE}]     "
    echo "VCB_FILE =      [${VCB_FILE}]     "
    echo "----------------------------------"
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "SHORE_IP =      [${SHORE_IP}]     "
    echo "SHORE_PSHARE =  [${SHORE_PSHARE}] "
    echo "----------------------------------"
    echo "COLOR =         [${COLOR}]        "
    echo "START_POS =     [${START_POS}]    "
    echo "RETURN_POS =    [${RETURN_POS}]   "
    echo "SPEED =         [${SPEED}]        "
    echo "PATTERN =       [${PATTERN}]      "
    echo "SURVEY_PTS =    [${SURVEY_PTS}]   "
    echo "OPREGION =      [${OPREGION}]     "
    echo "RETURN_VOLTS =  [${RETURN_VOLTS}] "
    echo "RETURN_PCT =    [${RETURN_PCT}]   "
    echo "MAX_SPD =       [${MAX_SPD}]      "
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi

#------------------------------------------------------------
#  Part 4: Create the .moos and .bhv files
#------------------------------------------------------------
mkdir -p targs logs
NSFLAGS="--strict --force"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="--interactive --force"
fi

nsplug meta_vehicle.moos targs/targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       VNAME=$VNAME             COLOR=$COLOR                 \
       IP_ADDR=$IP_ADDR         MOOS_PORT=$MOOS_PORT         \
       PSHARE_PORT=$PSHARE_PORT SHORE_IP=$SHORE_IP           \
       SHORE_PSHARE=$SHORE_PSHARE                            \
       XMODE=$XMODE             MAV_PORT=$MAV_PORT           \
       MAV_PROTO=$MAV_PROTO     MAV_HOST=$MAV_HOST           \
       BMS_FILE=$BMS_FILE       VCB_FILE=$VCB_FILE           \
       RETURN_VOLTS=$RETURN_VOLTS RETURN_PCT=$RETURN_PCT     \
       BRIDGE_DEBUG=$BRIDGE_DEBUG                            \
       START_POS=$START_POS     MAX_SPD=$MAX_SPD

nsplug meta_vehicle.bhv targs/targ_$VNAME.bhv $NSFLAGS \
       VNAME=$VNAME SPEED=$SPEED RETURN_POS=$RETURN_POS \
       SURVEY_PTS="$SURVEY_PTS" SURVEY_REPEAT=$SURVEY_REPEAT \
       WPT_RADIUS=$WPT_RADIUS WPT_SLIP=$WPT_SLIP WPT_LEAD=$WPT_LEAD \
       OPREGION="$OPREGION"

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: Targ files made; exiting without launch."
    exit 0
fi

#------------------------------------------------------------
#  Part 5: Launch
#------------------------------------------------------------
echo "$ME: Launching $VNAME ($XMODE) MOOS community. WARP=$TIME_WARP"
pAntler targs/targ_$VNAME.moos >& /dev/null &
echo "$ME: Done launching $VNAME"

if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

uMAC targs/targ_$VNAME.moos
trap "" SIGINT
kill -- -$$
