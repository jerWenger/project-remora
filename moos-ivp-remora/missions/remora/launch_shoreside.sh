#!/bin/bash
#------------------------------------------------------------
#   Script: launch_shoreside.sh
#  Mission: remora
#   Author: Jeremy Wenger
#  Purpose: Build targs/targ_shoreside.moos with nsplug and launch the
#           shoreside community (pMarineViewer + field brokers).
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

cd "$(dirname "$0")"

#------------------------------------------------------------
#  Part 1: Defaults
#------------------------------------------------------------
ME=$(basename "$0")
CMD_ARGS=""
TIME_WARP=1
JUST_MAKE="no"
VERBOSE=""
AUTO_LAUNCHED="no"
LAUNCH_GUI="yes"

IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9200"

#------------------------------------------------------------
#  Part 2: Arguments
#------------------------------------------------------------
for ARGI; do
    CMD_ARGS+="${ARGI} "
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME: [OPTIONS] [time_warp]                        "
        echo "                                                  "
        echo "Options:                                          "
        echo "  --help, -h         Show this help message       "
        echo "  --just_make, -j    Only create targ files       "
        echo "  --verbose, -v      Verbose, confirm launch      "
        echo "  --auto, -a         Script launch, no uMAC       "
        echo "  --nogui, -n        Headless, no pMarineViewer   "
        echo "                                                  "
        echo "  --ip=<localhost>   This host's IP (Tailscale IP "
        echo "                     when the vehicle is remote)  "
        echo "  --mport=<9000>     Shoreside MOOSDB port        "
        echo "  --pshare=<9200>    Shoreside pShare listen port "
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        LAUNCH_GUI="no"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
        MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    else
        echo "$ME: Bad Arg: [$ARGI]. Exit Code 1."
        exit 1
    fi
done

if [ "${VERBOSE}" = "yes" ]; then
    echo "=================================="
    echo "  $ME SUMMARY                     "
    echo "=================================="
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "JUST_MAKE =     [${JUST_MAKE}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "LAUNCH_GUI =    [${LAUNCH_GUI}]   "
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo -n "Hit any key to continue launch "
    read ANSWER
fi

#------------------------------------------------------------
#  Part 3: Create the shoreside mission file
#------------------------------------------------------------
mkdir -p targs logs
NSFLAGS="--strict --force"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="--interactive --force"
fi

nsplug meta_shoreside.moos targs/targ_shoreside.moos $NSFLAGS WARP=$TIME_WARP \
       IP_ADDR=$IP_ADDR  MOOS_PORT=$MOOS_PORT  PSHARE_PORT=$PSHARE_PORT \
       LAUNCH_GUI=$LAUNCH_GUI

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: Targ files made; exiting without launch."
    exit 0
fi

#------------------------------------------------------------
#  Part 4: Launch
#------------------------------------------------------------
echo "$ME: Launching shoreside MOOS community. WARP=$TIME_WARP"
pAntler targs/targ_shoreside.moos >& /dev/null &
echo "$ME: Done launching shoreside"

if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

uMAC targs/targ_shoreside.moos
trap "" SIGINT
kill -- -$$
