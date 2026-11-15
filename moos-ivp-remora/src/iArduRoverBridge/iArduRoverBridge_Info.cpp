/****************************************************************/
/*   NAME: Jeremy Wenger                                        */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: iArduRoverBridge_Info.cpp                            */
/*   DATE: November 2025                                        */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "iArduRoverBridge_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  iArduRoverBridge is a MOOS application that interfaces with  ");
  blk("  ArduRover via MAVLink/MAVSDK. It receives DESIRED_HEADING    ");
  blk("  and DESIRED_SPEED from MOOS-IVP and sends velocity commands  ");
  blk("  to ArduRover in GUIDED mode. It also publishes telemetry     ");
  blk("  data from ArduRover to MOOS.                                 ");
  blk("                                                                ");
  blk("  This application replaces the UDP-based iBBBSBroker +        ");
  blk("  pDiffThrustPID pipeline with direct MAVLink communication.   ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iArduRoverBridge file.moos [OPTIONS]                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                     ");
  blk("      Launch iArduRoverBridge with the given process name      ");
  blk("      rather than iArduRoverBridge.                            ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                               ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.             ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iArduRoverBridge.         ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,      ");
  blk("      then it will be interpreted as a run alias. This is      ");
  blk("      to support pAntler launching conventions.                ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iArduRoverBridge Example MOOS Configuration                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iArduRoverBridge                               ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                               ");
  blk("  CommsTick = 10                                               ");
  blk("                                                                ");
  blk("  // MAVLink connection URL                                    ");
  blk("  mavlink_connection_url = udpout://192.168.2.2:14550          ");
  blk("                                                                ");
  blk("  // Command timing                                            ");
  blk("  command_timeout_sec = 1.0  // Resend commands every 1 sec   ");
  blk("                                                                ");
  blk("  // Boat state the autopilot does not carry, from files the   ");
  blk("  // Jetson services write. Leave a path empty to turn it off. ");
  blk("  bms_file = /home/boat/battery.json   // LiTime pack          ");
  blk("  vcb_file = /home/boat/vcb.json       // thruster board       ");
  blk("  boat_state_stale_sec = 5   // older file = stale             ");
  blk("  battery_low_pct   = 20     // BATTERY_OK false at or below   ");
  blk("  battery_low_volts = 44     // BATTERY_OK false at or below   ");
  blk("                                                                ");
  blk("  // SERVOn output of each motor, for the report and debug log ");
  blk("  left_motor_servo  = 10                                       ");
  blk("  right_motor_servo = 9                                        ");
  blk("                                                                ");
  blk("  // Debug logging                                             ");
  blk("  debug = true                                                 ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iArduRoverBridge INTERFACE                                     ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                 ");
  blk("------------------------------------                            ");
  blk("  DESIRED_HEADING  = <double>  // Desired heading in degrees  ");
  blk("  DESIRED_SPEED    = <double>  // Desired speed in m/s        ");
  blk("  DEPLOY           = <bool>    // Enable autonomous operation ");
  blk("  ALLSTOP          = <bool>    // Request disarm              ");
  blk("  MOOS_MANUAL_OVERRIDE = <bool> // Request HOLD mode          ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  Position & Navigation:                                       ");
  blk("  NAV_LAT          = <double>  // Latitude in degrees         ");
  blk("  NAV_LONG         = <double>  // Longitude in degrees        ");
  blk("  NAV_X            = <double>  // Local X position (m)        ");
  blk("  NAV_Y            = <double>  // Local Y position (m)        ");
  blk("  NAV_HEADING      = <double>  // Heading in degrees          ");
  blk("  NAV_ROLL         = <double>  // Roll angle in degrees       ");
  blk("  NAV_PITCH        = <double>  // Pitch angle in degrees      ");
  blk("  NAV_SPEED        = <double>  // Ground speed in m/s         ");
  blk("  NAV_ALTITUDE     = <double>  // Altitude (m)                ");
  blk("                                                                ");
  blk("  GPS Data:                                                    ");
  blk("  GPS_SAT          = <int>     // Number of GPS satellites    ");
  blk("  GPS_FIX_TYPE     = <int>     // GPS fix type (3=3D fix)     ");
  blk("  GPS_HDOP         = <double>  // Horizontal DOP              ");
  blk("  GPS_VDOP         = <double>  // Vertical DOP (not posted    ");
  blk("                               // if the receiver has none)   ");
  blk("  GPS_HORIZ_ACCURACY = <double> // Horizontal accuracy (m)    ");
  blk("  GPS_VERT_ACCURACY  = <double> // Vertical accuracy (m)      ");
  blk("                                                                ");
  blk("  IMU Data:                                                    ");
  blk("  IMU_ACCEL_X      = <double>  // Accel forward (m/s^2)       ");
  blk("  IMU_ACCEL_Y      = <double>  // Accel right (m/s^2)         ");
  blk("  IMU_ACCEL_Z      = <double>  // Accel down (m/s^2)          ");
  blk("  IMU_GYRO_X       = <double>  // Gyro forward (rad/s)        ");
  blk("  IMU_GYRO_Y       = <double>  // Gyro right (rad/s)          ");
  blk("  IMU_GYRO_Z       = <double>  // Gyro down (rad/s)           ");
  blk("                                                                ");
  blk("  Magnetometer Data:                                           ");
  blk("  MAG_X            = <double>  // Mag field forward (Gauss)   ");
  blk("  MAG_Y            = <double>  // Mag field right (Gauss)     ");
  blk("  MAG_Z            = <double>  // Mag field down (Gauss)      ");
  blk("                                                                ");
  blk("  Battery Data (from the autopilot, or from the pack's BMS    ");
  blk("  when bms_file is set; current is positive on discharge):     ");
  blk("  BATTERY_VOLTAGE  = <double>  // Battery voltage (V)         ");
  blk("  BATTERY_CURRENT  = <double>  // Battery current (A)         ");
  blk("  BATTERY_REMAINING = <double> // Battery remaining (%)       ");
  blk("  Only with bms_file:                                          ");
  blk("  BATTERY_POWER    = <double>  // Pack power (W)              ");
  blk("  BATTERY_TEMP     = <double>  // Pack temperature (C)        ");
  blk("  BATTERY_CELL_MIN = <double>  // Lowest cell (V)             ");
  blk("  BATTERY_CELL_SPREAD_MV = <double> // Max - min cell (mV)    ");
  blk("  BATTERY_ONLINE   = <bool>    // BMS data fresh              ");
  blk("  BATTERY_OK       = <bool>    // Fresh and above both limits ");
  blk("                                                                ");
  blk("  Thruster board (only with vcb_file):                         ");
  blk("  VCB_ONLINE       = <bool>    // VCB data fresh              ");
  blk("  VCB_STATE        = <string>  // FAULT IDLE STARTING RUN     ");
  blk("  VCB_FAULT_CODE   = <int>     // 0 = none                    ");
  blk("  VCB_ARMED        = <bool>    // VCB armed                   ");
  blk("  VCB_HV_ON        = <bool>    // Both contactors closed      ");
  blk("  VCB_TSMS         = <bool>    // TSMS switch on              ");
  blk("  VCB_BUS_VOLTS    = <double>  // HV bus (V)                  ");
  blk("  VCB_CMD_LEFT     = <double>  // Applied thrust, % signed    ");
  blk("  VCB_CMD_RIGHT    = <double>  // Applied thrust, % signed    ");
  blk("  VCB_PWM_VALID    = <bool>    // Both PWM inputs valid       ");
  blk("  VCB_READY        = <bool>    // RUN, HV on, no fault        ");
  blk("                                                                ");
  blk("  System Health:                                               ");
  blk("  HEALTH_GYRO_CAL  = <bool>    // Gyro calibration OK         ");
  blk("  HEALTH_ACCEL_CAL = <bool>    // Accel calibration OK        ");
  blk("  HEALTH_MAG_CAL   = <bool>    // Mag calibration OK          ");
  blk("  HEALTH_LOCAL_POS = <bool>    // Local position estimate OK  ");
  blk("  HEALTH_GLOBAL_POS = <bool>   // Global position estimate OK ");
  blk("  HEALTH_ARMABLE   = <bool>    // System armable              ");
  blk("                                                                ");
  blk("  System Status:                                               ");
  blk("  ARDUROVER_MODE   = <string>  // Current flight mode         ");
  blk("  ARDUROVER_ARMED  = <bool>    // Armed state                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iArduRoverBridge", "gpl");
  exit(0);
}
