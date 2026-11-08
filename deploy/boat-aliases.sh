# Boat shell helpers, installed to ~/.boat_aliases by deploy/install.sh (sourced from ~/.bashrc).
# VCB console: `vcb help`, `vcb stop`, `vcb clear`, `vcb bench off`
vcb() { stty -F /dev/vcb 115200 raw -echo clocal 2>/dev/null; printf '%s\n' "$*" > /dev/vcb; }
# Reset the VCB without a power cycle (clears latched fault 8 and fault 3)
alias resetvcb='openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "init; reset run; exit"'
# Un-latch the RC8 e-stop (fault 9); refuses while RC8 is still engaged
alias vcbclear='python3 ~/estop_latch.py --clear'
# One line: RC8 position, VCB fault, HV latched or not
alias estop='python3 ~/estop_latch.py --status'
# VCB state changes only (arm, disarm, faults, contactors, bench)
vcbwatch() { tail -F "$(ls -t ~/vcblogs/*.csv | head -1)" | grep --line-buffered -E "ARMED|DISARMED|FAULT|THRUSTER|BENCH|STOP|CLEAR"; }
