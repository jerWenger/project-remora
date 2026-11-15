#!/bin/bash
#------------------------------------------------------------
#   Script: launch.sh
#  Mission: joust
#   Author: Jeremy Wenger
#  Purpose: Launch everything on ONE machine (the laptop): the boat's
#           vehicle community, a simulated skiff that crosses its leg,
#           and the shoreside. For the boat, run launch_vehicle.sh on
#           the Jetson and launch_shoreside.sh on the laptop instead;
#           the skiff is then the phone app (see README.md).
#
#   ./launch.sh --sim 10          boat and skiff both simulated, warp 10
#   ./launch.sh --sim --no_skiff  simulated boat alone (send phone fixes
#                                 to the shoreside's UDP port instead)
#   ./launch.sh                   boat = ArduRover SITL instance 0, skiff sim
#------------------------------------------------------------
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

cd "$(dirname "$0")"

#------------------------------------------------------------
#  Part 1: Defaults
#------------------------------------------------------------
ME=$(basename "$0")
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
NOGUI=""
VLAUNCH_ARGS=""
SLAUNCH_ARGS=""
SKIFF="yes"

# The skiff's leg crosses the boat's default leg at right angles in the
# middle. It does not avoid: the boat has to do all the work, as it will
# with a person driving the real skiff.
SKIFF_LEG="0,-50:0,-150"
SKIFF_SPEED="1.2"

#------------------------------------------------------------
#  Part 2: Arguments
#------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [OPTIONS] [time_warp]                              "
        echo "                                                       "
        echo "Options:                                               "
        echo "  --help, -h          Show this help message           "
        echo "  --just_make, -j     Only create targ files           "
        echo "  --verbose, -v       Verbose, confirm launch          "
        echo "  --nogui, -n         Headless, no pMarineViewer       "
        echo "                                                       "
        echo "  --sim, -s           Simulate the boat (uSimMarineV22)"
        echo "  --no_skiff          Do not launch the simulated skiff"
        echo "  --skiff_leg=<0,-50:0,-150>  The skiff's leg          "
        echo "  --skiff_speed=<1.2> The skiff's speed (m/s)          "
        echo "  --phone_port=<9300> UDP port for phone fixes         "
        echo "                                                       "
        echo "  --leg=, --turn_rad=, --op_margin=, --avoid=,         "
        echo "  --speed=, --max_spd=, --return_volts=, --return_pct=,"
        echo "  --sitl_instance=N, --mav_port=P, --boat_tcp, --debug "
        echo "                      passed to the boat's launch_vehicle.sh"
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="--just_make"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="--verbose"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        NOGUI="--nogui"
    elif [ "${ARGI}" = "--no_skiff" ]; then
        SKIFF="no"
    elif [ "${ARGI:0:12}" = "--skiff_leg=" ]; then
        SKIFF_LEG="${ARGI#--skiff_leg=*}"
    elif [ "${ARGI:0:14}" = "--skiff_speed=" ]; then
        SKIFF_SPEED="${ARGI#--skiff_speed=*}"
    elif [ "${ARGI:0:13}" = "--phone_port=" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" -o \
           "${ARGI:0:6}" = "--leg=" -o "${ARGI:0:11}" = "--turn_rad=" -o \
           "${ARGI:0:12}" = "--op_margin=" -o "${ARGI:0:8}" = "--avoid=" -o \
           "${ARGI:0:8}" = "--speed=" -o "${ARGI:0:10}" = "--max_spd=" -o \
           "${ARGI:0:9}" = "--return_" -o "${ARGI:0:16}" = "--sitl_instance=" -o \
           "${ARGI:0:11}" = "--mav_port=" -o "${ARGI}" = "--boat_tcp" -o \
           "${ARGI}" = "--debug" ]; then
        VLAUNCH_ARGS+=" ${ARGI}"
    else
        echo "$ME: Bad Arg: [$ARGI]. Exit Code 1."
        exit 1
    fi
done

#------------------------------------------------------------
#  Part 3: Launch the communities
#------------------------------------------------------------
./launch_vehicle.sh --auto $VLAUNCH_ARGS $VERBOSE $JUST_MAKE $TIME_WARP || exit 1

if [ "$SKIFF" = "yes" ]; then
    SKIFF_START="x=${SKIFF_LEG%%:*}"
    SKIFF_START="${SKIFF_START/,/,y=},heading=180"
    ./launch_vehicle.sh --auto --sim --vname=skiff --color=red \
        --mport=9002 --pshare=9202 --leg=$SKIFF_LEG --speed=$SKIFF_SPEED \
        --avoid=false --start_pos=$SKIFF_START --return_pos=${SKIFF_LEG%%:*} \
        $VERBOSE $JUST_MAKE $TIME_WARP || exit 1
fi

./launch_shoreside.sh --auto $SLAUNCH_ARGS $NOGUI $VERBOSE $JUST_MAKE $TIME_WARP || exit 1

if [ -n "$JUST_MAKE" ]; then
    exit 0
fi

uMAC targs/targ_shoreside.moos
trap "" SIGINT
kill -- -$$
