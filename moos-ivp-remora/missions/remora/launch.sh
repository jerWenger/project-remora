#!/bin/bash
#------------------------------------------------------------
#   Script: launch.sh
#  Mission: remora
#   Author: Jeremy Wenger
#  Purpose: Launch the vehicle and shoreside communities on ONE
#           machine (the laptop). For the boat, run
#           launch_vehicle.sh on the Jetson and launch_shoreside.sh
#           on the laptop instead (see README.md).
#
#   ./launch.sh --sim             pure MOOS sim (uSimMarineV22)
#   ./launch.sh --sim 10          same, time warp 10
#   ./launch.sh                   iArduRoverBridge -> ArduRover SITL
#                                 instance 0 (sim/run_sitl.sh)
#   ./launch.sh --sitl_instance=5 ... SITL instance 5 (run_sitl.sh -I 5)
#------------------------------------------------------------
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

cd "$(dirname "$0")"

ME=$(basename "$0")
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
NOGUI=""
VLAUNCH_ARGS=""

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [OPTIONS] [time_warp]                               "
        echo "                                                        "
        echo "Options:                                                "
        echo "  --help, -h          Show this help message            "
        echo "  --just_make, -j     Only create targ files            "
        echo "  --verbose, -v       Verbose, confirm launch           "
        echo "  --nogui, -n         No pMarineViewer                  "
        echo "  --sim, -s           uSimMarineV22 instead of ArduRover"
        echo "  --sitl_instance=N   ArduRover SITL instance N         "
        echo "  --mav_port=<port>   Explicit MAVLink udpin port       "
        echo "  --debug             iArduRoverBridge debug log        "
        echo "  --speed=<m/s>, --max_spd=<m/s>, --start_pos=, --return_pos="
        echo "                      passed to launch_vehicle.sh       "
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="--just_make"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="--verbose"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        NOGUI="--nogui"
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ]; then
        VLAUNCH_ARGS+=" --sim"
    elif [ "${ARGI:0:16}" = "--sitl_instance=" -o "${ARGI:0:11}" = "--mav_port=" -o \
           "${ARGI}" = "--debug" -o "${ARGI}" = "--boat_tcp" -o "${ARGI:0:12}" = "--state_dir=" -o "${ARGI:0:8}" = "--speed=" -o "${ARGI:0:10}" = "--pattern=" -o \
           "${ARGI:0:7}" = "--fig8_" -o "${ARGI:0:7}" = "--laps=" -o "${ARGI:0:12}" = "--op_margin=" -o \
           "${ARGI:0:9}" = "--return_" -o \
           "${ARGI:0:10}" = "--max_spd=" -o "${ARGI:0:12}" = "--start_pos=" -o \
           "${ARGI:0:13}" = "--return_pos=" ]; then
        VLAUNCH_ARGS+=" ${ARGI}"
    else
        echo "$ME: Bad Arg: [$ARGI]. Exit Code 1."
        exit 1
    fi
done

./launch_vehicle.sh --auto $VLAUNCH_ARGS $VERBOSE $JUST_MAKE $TIME_WARP || exit 1
./launch_shoreside.sh --auto $NOGUI $VERBOSE $JUST_MAKE $TIME_WARP || exit 1

if [ -n "$JUST_MAKE" ]; then
    exit 0
fi

uMAC targs/targ_shoreside.moos
trap "" SIGINT
kill -- -$$
