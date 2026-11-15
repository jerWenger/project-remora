/************************************************************/
/*    NAME: Jeremy Wenger                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BoatState.h                                     */
/*    DATE: November 2026                                   */
/************************************************************/

// State of this boat that ArduPilot does not carry: the LiTime pack (read by
// scripts/battery_litime.py into battery.json) and the VCB thruster control
// board (scripts/vcb_logger.py into vcb.json). Both files live on the Jetson
// and are rewritten atomically, about once a second and five times a second.
// A file counts as fresh while its modification time is younger than the
// stale limit, so a dead reader service shows up as stale, not as old data.

#ifndef BoatState_HEADER
#define BoatState_HEADER

#include <string>

struct BmsState
{
    bool   fresh   = false;   // file exists and was written within the stale limit
    bool   online  = false;   // the reader has a live answer from the BMS
    double age     = -1;      // s since the file was written, -1 = no file
    double voltage = 0;       // V
    double current = 0;       // A, discharge positive (the BMS reports it negative)
    double power   = 0;       // W, discharge positive
    double soc     = 0;       // %
    double temp    = 0;       // deg C
    double cell_min = 0;      // V
    double cell_max = 0;      // V
};

struct VcbState
{
    bool   fresh  = false;
    bool   online = false;    // the logger has the VCB's USB port open
    double age    = -1;
    int    state  = -1;       // 0 FAULT, 1 IDLE, 2-4 STARTING, 5 RUN
    int    fault  = 0;        // fault_code, 0 = none (firmware README has the table)
    bool   armed  = false;
    bool   tsms   = false;    // 24 V present on the TSMS sense
    bool   hv_on  = false;    // both contactors closed
    double bus_volts = 0;     // V at the contactor output
    double cmd_left  = 0;     // applied thrust, % of full, signed (ch2)
    double cmd_right = 0;     // applied thrust, % of full, signed (ch1)
    bool   pwm_valid = false; // both PWM inputs valid
    std::string last_event;   // last console line that was not telemetry
};

std::string vcbStateName(int state);
std::string vcbFaultName(int fault);

// Return false when the file is missing or unreadable (state.age stays -1).
bool readBmsFile(const std::string& path, double stale_sec, BmsState& state);
bool readVcbFile(const std::string& path, double stale_sec, VcbState& state);

#endif
