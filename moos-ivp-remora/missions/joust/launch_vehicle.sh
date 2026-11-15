#!/bin/bash
#------------------------------------------------------------
#   Script: launch_vehicle.sh
#  Mission: joust
#   Author: Jeremy Wenger
#  Purpose: Build targs/targ_<vname>.moos/.bhv with nsplug and launch
#           one vehicle community that runs a leg back and forth
#           (BHV_LegRun) and avoids contacts by the COLREGS. Default
#           mode is ARDUROVER (iArduRoverBridge to ArduRover); --sim
#           swaps in uSimMarineV22 + pMarinePIDV22, which is also how
#           the simulated skiff is launched (see launch.sh).
#           App config plugs are shared with ../remora (nsplug --path).
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

# The leg, in the pavilion frame (x east, y north of the origin)
LEG="50,-100:-50,-100"    # vx1:vx2, through the 2026-11-15 tuning area
TURN_RAD="18"             # m; the boat's comfortable turn at 1.5 m/s
OP_MARGIN="30"            # op region = leg + turns + this, m
AVOID="true"              # false: hold the leg whatever the contact does

# The bridge posts BATTERY_OK=false below either limit and the helm returns
RETURN_VOLTS="50.0"
RETURN_PCT="20"

# SITL home (sim/run_sitl.sh) is ~ x=20, y=-82 in the pavilion frame.
START_POS="x=60,y=-80,heading=270"
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
        echo "  --speed=<1.5>          Leg and return speed (m/s)      "
        echo "  --leg=<50,-100:-50,-100>  Leg end points x1,y1:x2,y2 (m)"
        echo "  --turn_rad=<18>        Turn radius at each end (m)     "
        echo "  --op_margin=<30>       Op region margin (m)            "
        echo "  --avoid=<true>         false = no collision avoidance  "
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
    elif [ "${ARGI:0:6}" = "--leg=" ]; then
        LEG="${ARGI#--leg=*}"
    elif [ "${ARGI:0:11}" = "--turn_rad=" ]; then
        TURN_RAD="${ARGI#--turn_rad=*}"
    elif [ "${ARGI:0:12}" = "--op_margin=" ]; then
        OP_MARGIN="${ARGI#--op_margin=*}"
    elif [ "${ARGI:0:8}" = "--avoid=" ]; then
        AVOID="${ARGI#--avoid=*}"
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

# Leg -> vertices and an op region that holds the leg and both turns
[ -z "$SPEED" ] && SPEED="1.5"
VX1="${LEG%%:*}"
VX2="${LEG##*:}"
OPREGION=$(python3 - "$VX1" "$VX2" "$TURN_RAD" "$OP_MARGIN" <<'PYEOF'
import sys
(x1, y1), (x2, y2) = ([float(v) for v in a.split(",")] for a in sys.argv[1:3])
pad = 2 * float(sys.argv[3]) + float(sys.argv[4])      # a turn swings out about two radii
x0, x1, y0, y1 = min(x1, x2) - pad, max(x1, x2) + pad, min(y1, y2) - pad, max(y1, y2) + pad
print("pts={" + f"{x0:.0f},{y1:.0f}:{x1:.0f},{y1:.0f}:{x1:.0f},{y0:.0f}:{x0:.0f},{y0:.0f}" + "}")
PYEOF
) || { echo "$ME: bad --leg [$LEG] (want x1,y1:x2,y2). Exit Code 1."; exit 1; }

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
    echo "LEG =           [${VX1} : ${VX2}] "
    echo "TURN_RAD =      [${TURN_RAD}]     "
    echo "AVOID =         [${AVOID}]        "
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
NSFLAGS="--strict --force --path=../remora"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="--interactive --force --path=../remora"
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
       VX1=$VX1 VX2=$VX2 TURN_RAD=$TURN_RAD AVOID=$AVOID \
       COLOR=$COLOR OPREGION="$OPREGION"

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
