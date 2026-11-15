#!/bin/bash
#------------------------------------------------------------
#   Script: clean.sh
#  Mission: remora
#  Purpose: Remove generated targ files and logs.
#------------------------------------------------------------
cd "$(dirname "$0")"

VERBOSE=""
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$0 [--verbose]"
        exit 0
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="-v"
    else
        echo "Bad Argument: $ARGI"
        exit 1
    fi
done

rm -rf $VERBOSE logs/LOG_* logs/XLOG_* logs/MOOSLog_*
rm -f  $VERBOSE targs/targ_* *.moos++ DBG_*.log *~
rm -f  $VERBOSE .LastOpenedMOOSLogDirectory
