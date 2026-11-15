/************************************************************/
/*    NAME: Jeremy Wenger                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: iArduRoverBridge.h                              */
/*    DATE: November 2025                                   */
/************************************************************/

#ifndef iArduRoverBridge_HEADER
#define iArduRoverBridge_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "BoatState.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

class iArduRoverBridge : public AppCastingMOOSApp
{
public:
    iArduRoverBridge();
    ~iArduRoverBridge();

protected: // Standard MOOSApp functions
    bool OnNewMail(MOOSMSG_LIST &NewMail) override;
    bool Iterate() override;
    bool OnConnectToServer() override;
    bool OnStartUp() override;

protected: // AppCastingMOOSApp function
    bool buildReport() override;

protected:
    void registerVariables();
    bool GeodesySetup();
    bool dbg_print(const char *format, ...);

    // MAVLink connection management
    bool connectToArduRover();
    void connectionThread();

    // State machine methods
    bool checkPreArmHealth();
    bool switchToHoldMode();
    bool requestDisarm();
    void sendGuidedCommand(double heading_deg, double speed_mps);

    // Telemetry handling
    void setupTelemetrySubscriptions();
    void publishTelemetryToMOOS();

    // Boat state the autopilot does not carry (BMS, VCB), from files on the Jetson
    void updateBoatState();
    void publishBoatState();

private: // Configuration parameters
    bool m_debug;
    std::string m_mavlink_connection_url;
    std::string m_mavlink_protocol;
    std::string m_mavlink_host;
    int m_mavlink_port;
    double m_command_timeout_sec;

    // Appcast display options
    bool m_show_imu_data;         // Show IMU/gyro/magnetometer data
    bool m_show_gps_details;      // Show detailed GPS stats (HDOP, VDOP, accuracy)
    bool m_show_nav_controller;   // Show navigation controller internals
    bool m_show_health_flags;     // Show system health flags

private: // Boat state files (empty path = that source is off)
    std::string m_bms_file;
    std::string m_vcb_file;
    double m_boat_state_stale_sec;
    double m_battery_low_pct;     // BATTERY_OK goes false at or below this SoC
    double m_battery_low_volts;   // ... or at or below this pack voltage
    BmsState m_bms;
    VcbState m_vcb;
    double m_last_boat_state_time;
    bool m_bms_seen;              // the file was fresh at least once this run
    bool m_vcb_seen;
    int  m_vcb_last_fault;

private: // MAVSDK objects
    std::shared_ptr<mavsdk::Mavsdk> m_mavsdk;
    std::shared_ptr<mavsdk::System> m_system;
    std::shared_ptr<mavsdk::Action> m_action;
    std::shared_ptr<mavsdk::Telemetry> m_telemetry;
    std::shared_ptr<mavsdk::MavlinkPassthrough> m_passthrough;

private: // State variables
    std::atomic<bool> m_connected;
    bool m_deploy;
    bool m_allstop;
    bool m_moos_manual_override;
    bool m_connection_thread_running;
    std::thread m_connection_thread;
    std::mutex m_mavlink_mutex;

private: // Desired setpoints from MOOS
    double m_desired_heading;
    double m_desired_speed;
    double m_last_desired_heading_time;
    double m_last_desired_speed_time;

private: // Telemetry cache (thread-safe access)
    std::mutex m_telemetry_mutex;
    double m_nav_lat;
    double m_nav_lon;
    double m_nav_heading;
    double m_nav_speed;
    double m_nav_x;
    double m_nav_y;
    double m_nav_altitude;
    int m_gps_num_sats;
    int m_gps_fix_type;
    std::string m_current_flight_mode;
    bool m_telemetry_armed;

    // IMU data (acceleration in m/s^2, gyro in rad/s)
    double m_imu_accel_x;
    double m_imu_accel_y;
    double m_imu_accel_z;
    double m_imu_gyro_x;
    double m_imu_gyro_y;
    double m_imu_gyro_z;

    // Magnetometer data (Gauss)
    double m_mag_x;
    double m_mag_y;
    double m_mag_z;

    // Detailed GPS statistics
    double m_gps_hdop;
    double m_gps_vdop;
    double m_gps_horiz_accuracy;
    double m_gps_vert_accuracy;

    // Battery data
    double m_battery_voltage;
    double m_battery_current;
    double m_battery_remaining_pct;

    // Attitude (roll, pitch in degrees)
    double m_nav_roll;
    double m_nav_pitch;

    // System health flags
    bool m_health_gyrometer_cal;
    bool m_health_accelerometer_cal;
    bool m_health_magnetometer_cal;
    bool m_health_local_position;
    bool m_health_global_position;
    bool m_health_armable;

    // FC target feedback (from POSITION_TARGET_LOCAL_NED)
    float m_fc_target_vx;        // FC's target vx (m/s)
    float m_fc_target_vy;        // FC's target vy (m/s)
    float m_fc_target_yaw;       // FC's target yaw (rad)

    // Motor/servo outputs (from SERVO_OUTPUT_RAW)
    uint16_t m_servo1_pwm;       // Servo output 1 (PWM) - typically left motor or steering
    uint16_t m_servo3_pwm;       // Servo output 3 (PWM) - typically right motor or throttle

    // Throttle percentage (from VFR_HUD)
    double m_throttle_pct;

private: // Statistics
    unsigned long m_commands_sent;
    unsigned long m_telemetry_updates_received;
    double m_last_command_time;
    double m_last_telemetry_time;

private: // Geodesy for coordinate conversion
    CMOOSGeodesy m_geodesy;

private: // Debug logging
    std::string m_fname;
    FILE *m_debug_stream;
    std::mutex m_debug_mutex;
};

#endif
