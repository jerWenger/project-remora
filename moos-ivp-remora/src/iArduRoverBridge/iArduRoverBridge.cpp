/************************************************************/
/*    NAME: Jeremy Wenger                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: iArduRoverBridge.cpp                            */
/*    DATE: November 2025                                   */
/************************************************************/

#include <iterator>
#include <cstring>
#include <algorithm>
#include <cstdarg>
#include <chrono>
#include <thread>
#include <future>
#include "MBUtils.h"
#include "ACTable.h"
#include "iArduRoverBridge.h"

using namespace std;
using namespace mavsdk;

//---------------------------------------------------------
// Constructor

iArduRoverBridge::iArduRoverBridge()
    : m_debug(false),
      m_mavlink_connection_url(""),
      m_mavlink_protocol("tcp"),
      m_mavlink_host("127.0.0.1"),
      m_mavlink_port(5760),
      m_command_timeout_sec(1.0),
      m_show_imu_data(false),
      m_show_gps_details(false),
      m_show_nav_controller(false),
      m_show_health_flags(false),
      m_boat_state_stale_sec(5.0),
      m_battery_low_pct(20.0),
      m_battery_low_volts(44.0),
      m_last_boat_state_time(0.0),
      m_bms_seen(false),
      m_vcb_seen(false),
      m_vcb_last_fault(0),
      m_connected(false),
      m_deploy(false),
      m_allstop(false),
      m_moos_manual_override(false),
      m_connection_thread_running(false),
      m_desired_heading(0.0),
      m_desired_speed(0.0),
      m_last_desired_heading_time(0.0),
      m_last_desired_speed_time(0.0),
      m_nav_lat(0.0),
      m_nav_lon(0.0),
      m_nav_heading(0.0),
      m_nav_speed(0.0),
      m_nav_x(0.0),
      m_nav_y(0.0),
      m_nav_altitude(0.0),
      m_gps_num_sats(0),
      m_gps_fix_type(0),
      m_current_flight_mode("UNKNOWN"),
      m_telemetry_armed(false),
      m_imu_accel_x(0.0),
      m_imu_accel_y(0.0),
      m_imu_accel_z(0.0),
      m_imu_gyro_x(0.0),
      m_imu_gyro_y(0.0),
      m_imu_gyro_z(0.0),
      m_mag_x(0.0),
      m_mag_y(0.0),
      m_mag_z(0.0),
      m_gps_hdop(-1.0),
      m_gps_vdop(-1.0),
      m_gps_horiz_accuracy(0.0),
      m_gps_vert_accuracy(0.0),
      m_battery_voltage(0.0),
      m_battery_current(0.0),
      m_battery_remaining_pct(0.0),
      m_nav_roll(0.0),
      m_nav_pitch(0.0),
      m_health_gyrometer_cal(false),
      m_health_accelerometer_cal(false),
      m_health_magnetometer_cal(false),
      m_health_local_position(false),
      m_health_global_position(false),
      m_health_armable(false),
      m_fc_target_vx(0.0f),
      m_fc_target_vy(0.0f),
      m_fc_target_yaw(0.0f),
      m_left_motor_servo(1),
      m_right_motor_servo(3),
      m_left_motor_pwm(1500),
      m_right_motor_pwm(1500),
      m_throttle_pct(0.0),
      m_commands_sent(0),
      m_telemetry_updates_received(0),
      m_last_command_time(0.0),
      m_last_telemetry_time(0.0),
      m_fname(""),
      m_debug_stream(nullptr)
{
}

//---------------------------------------------------------
// Destructor

iArduRoverBridge::~iArduRoverBridge()
{
    m_connection_thread_running = false;
    if (m_connection_thread.joinable())
        m_connection_thread.join();
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool iArduRoverBridge::OnNewMail(MOOSMSG_LIST &NewMail)
{
    AppCastingMOOSApp::OnNewMail(NewMail);

    for (auto &msg : NewMail)
    {
        string key = msg.GetKey();
        double dval = msg.GetDouble();
        double mtime = msg.GetTime();

        if (key == "DESIRED_HEADING")
        {
            m_desired_heading = dval;
            m_last_desired_heading_time = mtime;
        }
        else if (key == "DESIRED_SPEED")
        {
            m_desired_speed = dval;
            m_last_desired_speed_time = mtime;
        }
        else if (key == "DEPLOY")
        {
            m_deploy = (tolower(msg.GetString()) == "true");
            dbg_print("%0.2f: DEPLOY = %s\n", MOOSTime(), m_deploy ? "true" : "false");
        }
        else if (key == "ALLSTOP")
        {
            m_allstop = (tolower(msg.GetString()) == "true");
            dbg_print("%0.2f: ALLSTOP = %s\n", MOOSTime(), m_allstop ? "true" : "false");
        }
        else if (key == "MOOS_MANUAL_OVERRIDE")
        {
            m_moos_manual_override = (tolower(msg.GetString()) == "true");
            dbg_print("%0.2f: MOOS_MANUAL_OVERRIDE = %s\n", MOOSTime(), m_moos_manual_override ? "true" : "false");
        }
    }

    return true;
}

//---------------------------------------------------------
// Procedure: Iterate()

bool iArduRoverBridge::Iterate()
{
    AppCastingMOOSApp::Iterate();

    // Battery and VCB state come from files, not from the autopilot, so they
    // are published whether or not the MAVLink link is up. 2 Hz is enough.
    if (MOOSTime() - m_last_boat_state_time >= 0.5)
    {
        m_last_boat_state_time = MOOSTime();
        updateBoatState();
        publishBoatState();
    }

    // Check connection status
    if (!m_connected || !m_system)
    {
        reportRunWarning("Not connected to ArduRover");
        AppCastingMOOSApp::PostReport();
        return true;
    }
    retractRunWarning("Not connected to ArduRover");

    // Get current vehicle state from telemetry (this is the source of truth)
    bool vehicle_armed = false;
    bool vehicle_in_guided = false;
    {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        vehicle_armed = m_telemetry_armed;
        vehicle_in_guided = (m_current_flight_mode == "GUIDED");
    }

    // Handle safety commands first (these override everything from MOOS side)
    if (m_allstop && vehicle_armed)
    {
        // ALLSTOP requested - try to disarm the vehicle
        dbg_print("%0.2f: ALLSTOP active - requesting disarm\n", MOOSTime());
        requestDisarm();
    }
    else if (m_moos_manual_override && vehicle_in_guided)
    {
        // MOOS_MANUAL_OVERRIDE (or STATION) - request HOLD mode
        dbg_print("%0.2f: MOOS_MANUAL_OVERRIDE active - requesting HOLD\n", MOOSTime());
        switchToHoldMode();
    }
    else if (m_deploy && vehicle_armed && vehicle_in_guided)
    {
        // Normal autonomous operation:
        // RC has armed and set GUIDED mode, DEPLOY is true, no safety overrides
        // Check if setpoints are recent enough
        double now = MOOSTime();
        double heading_age = now - m_last_desired_heading_time;
        double speed_age = now - m_last_desired_speed_time;
        bool setpoints_fresh = (heading_age < 5.0 && speed_age < 5.0);

        if (setpoints_fresh)
        {
            // At zero desired speed the helm's heading output (e.g. the
            // station-keep behavior's bearing-to-center) is unstable and
            // commanding it makes the boat pivot in place searching for
            // it. Substitute the vehicle's current heading so zero-speed
            // commands are heading-neutral: the boat sits passively until
            // the helm commands actual speed (e.g. station-keep breaching
            // its outer radius).
            double cmd_heading = m_desired_heading;
            if (m_desired_speed <= 0.0)
            {
                std::lock_guard<std::mutex> lock(m_telemetry_mutex);
                cmd_heading = m_nav_heading;
            }
            sendGuidedCommand(cmd_heading, m_desired_speed);
        }
        else
        {
            reportRunWarning("Desired heading/speed too old - not sending commands");
        }
    }

    // Publish telemetry to MOOS
    publishTelemetryToMOOS();

    AppCastingMOOSApp::PostReport();
    return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool iArduRoverBridge::OnConnectToServer()
{
    registerVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool iArduRoverBridge::OnStartUp()
{
    AppCastingMOOSApp::OnStartUp();

    STRING_LIST sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    {
        reportConfigWarning("No config block found for " + GetAppName());
    }

    for (string line : sParams)
    {
        string orig = line;
        string param = tolower(biteStringX(line, '='));
        string value = stripBlankEnds(line);
        bool handled = false;

        if (param == "debug")
        {
            m_debug = (tolower(value) == "true");
            handled = true;
        }
        else if (param == "mavlink_connection_url")
        {
            m_mavlink_connection_url = value;
            handled = true;
        }
        else if (param == "mavlink_protocol")
        {
            m_mavlink_protocol = value;
            handled = true;
        }
        else if (param == "mavlink_host")
        {
            m_mavlink_host = value;
            handled = true;
        }
        else if (param == "mavlink_port")
        {
            handled = setIntOnString(m_mavlink_port, value);
        }
        else if (param == "command_timeout_sec")
        {
            handled = setDoubleOnString(m_command_timeout_sec, value);
        }
        else if (param == "bms_file")
        {
            m_bms_file = value;
            handled = true;
        }
        else if (param == "vcb_file")
        {
            m_vcb_file = value;
            handled = true;
        }
        else if (param == "boat_state_stale_sec")
        {
            handled = setPosDoubleOnString(m_boat_state_stale_sec, value);
        }
        else if (param == "battery_low_pct")
        {
            handled = setNonNegDoubleOnString(m_battery_low_pct, value);
        }
        else if (param == "battery_low_volts")
        {
            handled = setNonNegDoubleOnString(m_battery_low_volts, value);
        }
        else if (param == "left_motor_servo" || param == "right_motor_servo")
        {
            int n = 0;
            handled = setIntOnString(n, value) && n >= 1 && n <= 16;
            if (handled)
                (param == "left_motor_servo" ? m_left_motor_servo : m_right_motor_servo) = n;
        }
        else if (param == "show_imu_data")
        {
            m_show_imu_data = (tolower(value) == "true");
            handled = true;
        }
        else if (param == "show_gps_details")
        {
            m_show_gps_details = (tolower(value) == "true");
            handled = true;
        }
        else if (param == "show_nav_controller")
        {
            m_show_nav_controller = (tolower(value) == "true");
            handled = true;
        }
        else if (param == "show_health_flags")
        {
            m_show_health_flags = (tolower(value) == "true");
            handled = true;
        }

        if (!handled)
        {
            reportUnhandledConfigWarning(orig);
        }
    }

    // Build MAVLink connection URL if not directly specified
    // This allows us to use separate parameters to avoid MOOS comment parsing issues with //
    if (m_mavlink_connection_url.empty())
    {
        m_mavlink_connection_url = m_mavlink_protocol + "://" + m_mavlink_host + ":" + std::to_string(m_mavlink_port);
        dbg_print("Built MAVLink connection URL: %s\n", m_mavlink_connection_url.c_str());
    }

    // Setup debug logging
    if (m_debug)
    {
        m_fname = "DBG_" + GetAppName() + "_" + MOOSFormat("%.0f", MOOSTime()) + ".log";
        dbg_print("%0.2f: Debug logging to file: %s\n", MOOSTime(), m_fname.c_str());
    }

    // Setup Geodesy
    if (!GeodesySetup())
    {
        reportConfigWarning("Geodesy setup failed.");
        return false;
    }

    // Start connection thread
    m_connection_thread_running = true;
    m_connection_thread = std::thread(&iArduRoverBridge::connectionThread, this);

    dbg_print("%0.2f: iArduRoverBridge started, connecting to %s\n",
              MOOSTime(), m_mavlink_connection_url.c_str());

    registerVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: registerVariables

void iArduRoverBridge::registerVariables()
{
    AppCastingMOOSApp::RegisterVariables();
    Register("DESIRED_HEADING", 0);
    Register("DESIRED_SPEED", 0);
    Register("DEPLOY", 0);
    Register("ALLSTOP", 0);
    Register("MOOS_MANUAL_OVERRIDE", 0);
}

//---------------------------------------------------------
// Procedure: connectionThread

void iArduRoverBridge::connectionThread()
{
    while (m_connection_thread_running)
    {
        if (!m_connected)
        {
            dbg_print("%0.2f: Attempting to connect to ArduRover...\n", MOOSTime());
            if (connectToArduRover())
            {
                dbg_print("%0.2f: Successfully connected to ArduRover\n", MOOSTime());
                m_connected = true;
            }
            else
            {
                dbg_print("%0.2f: Connection attempt failed, retrying in 5 seconds\n", MOOSTime());
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        }
        else
        {
            // Check if still connected
            if (!m_system || !m_system->is_connected())
            {
                dbg_print("%0.2f: Lost connection to ArduRover\n", MOOSTime());
                m_connected = false;
                // Note: armed/mode state comes from telemetry, which will stop updating
                // The Iterate() loop will naturally stop sending commands when disconnected
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

//---------------------------------------------------------
// Procedure: connectToArduRover

bool iArduRoverBridge::connectToArduRover()
{
    try
    {
        std::lock_guard<std::mutex> lock(m_mavlink_mutex);

        // Create MAVSDK instance
        Mavsdk::Configuration config(ComponentType::GroundStation);
        m_mavsdk = std::make_shared<Mavsdk>(config);

        // Add connection
        ConnectionResult connection_result = m_mavsdk->add_any_connection(m_mavlink_connection_url);
        if (connection_result != ConnectionResult::Success)
        {
            std::string error_msg = "Failed to add connection: " + m_mavlink_connection_url +
                                   " (error code: " + std::to_string(static_cast<int>(connection_result)) + ")";
            reportRunWarning(error_msg);
            dbg_print("MAVSDK connection error details: %s\n", error_msg.c_str());
            return false;
        }

        dbg_print("MAVSDK connection added successfully: %s\n", m_mavlink_connection_url.c_str());

        // Wait for system discovery (timeout 10 seconds)
        auto prom = std::promise<std::shared_ptr<System>>{};
        auto fut = prom.get_future();

        Mavsdk::NewSystemHandle handle = m_mavsdk->subscribe_on_new_system([&]() {
            const auto system = m_mavsdk->systems().at(0);

            if (system->is_connected())
            {
                prom.set_value(system);
            }
        });

        if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout)
        {
            m_mavsdk->unsubscribe_on_new_system(handle);
            reportRunWarning("Timeout waiting for system discovery");
            return false;
        }

        m_system = fut.get();
        m_mavsdk->unsubscribe_on_new_system(handle);

        // Instantiate plugins
        m_action = std::make_shared<Action>(m_system);
        m_telemetry = std::make_shared<Telemetry>(m_system);
        m_passthrough = std::make_shared<MavlinkPassthrough>(m_system);

        // Setup telemetry subscriptions
        setupTelemetrySubscriptions();

        return true;
    }
    catch (const std::exception &e)
    {
        reportRunWarning("Exception during connection: " + string(e.what()));
        return false;
    }
}

//---------------------------------------------------------
// Procedure: setupTelemetrySubscriptions

void iArduRoverBridge::setupTelemetrySubscriptions()
{
    // Request telemetry stream rates from ArduPilot
    // Note: Some rates may be denied by ArduPilot - we log the results
    dbg_print("%0.2f: Requesting telemetry stream rates...\n", MOOSTime());

    auto pos_result = m_telemetry->set_rate_position(10.0);
    dbg_print("%0.2f: set_rate_position(10Hz): %s\n", MOOSTime(),
              pos_result == Telemetry::Result::Success ? "SUCCESS" : "FAILED");

    auto vel_result = m_telemetry->set_rate_velocity_ned(10.0);
    dbg_print("%0.2f: set_rate_velocity_ned(10Hz): %s\n", MOOSTime(),
              vel_result == Telemetry::Result::Success ? "SUCCESS" : "FAILED");

    auto att_result = m_telemetry->set_rate_attitude_euler(10.0);
    dbg_print("%0.2f: set_rate_attitude_euler(10Hz): %s\n", MOOSTime(),
              att_result == Telemetry::Result::Success ? "SUCCESS" : "FAILED");

    auto gps_result = m_telemetry->set_rate_gps_info(2.0);
    dbg_print("%0.2f: set_rate_gps_info(2Hz): %s\n", MOOSTime(),
              gps_result == Telemetry::Result::Success ? "SUCCESS" : "FAILED");

    auto bat_result = m_telemetry->set_rate_battery(1.0);
    dbg_print("%0.2f: set_rate_battery(1Hz): %s\n", MOOSTime(),
              bat_result == Telemetry::Result::Success ? "SUCCESS" : "FAILED");

    auto imu_result = m_telemetry->set_rate_imu(10.0);
    dbg_print("%0.2f: set_rate_imu(10Hz): %s\n", MOOSTime(),
              imu_result == Telemetry::Result::Success ? "SUCCESS" : "FAILED");

    // Note: No set_rate for raw_gps - it may stream based on ArduPilot SRx_ params

    // Subscribe to position updates
    m_telemetry->subscribe_position([this](Telemetry::Position position) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_nav_lat = position.latitude_deg;
        m_nav_lon = position.longitude_deg;
        m_nav_altitude = position.relative_altitude_m;
        m_last_telemetry_time = MOOSTime();
        m_telemetry_updates_received++;
    });

    // Subscribe to heading
    m_telemetry->subscribe_heading([this](Telemetry::Heading heading) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_nav_heading = heading.heading_deg;
    });

    // Subscribe to ground speed via velocity NED
    m_telemetry->subscribe_velocity_ned([this](Telemetry::VelocityNed velocity) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        // Calculate speed magnitude from NED components
        m_nav_speed = sqrt(velocity.north_m_s * velocity.north_m_s +
                          velocity.east_m_s * velocity.east_m_s);
    });

    // Subscribe to GPS info
    m_telemetry->subscribe_gps_info([this](Telemetry::GpsInfo gps_info) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_gps_num_sats = gps_info.num_satellites;
        m_gps_fix_type = static_cast<int>(gps_info.fix_type);
    });

    // Subscribe to HEARTBEAT to decode the flight mode from the raw ArduPilot
    // Rover custom_mode number. MAVSDK's Telemetry::FlightMode enum has no
    // value for several Rover modes (LOITER, STEERING, SMART_RTL, ...) and
    // reports them all as Unknown, so it cannot be used as the mode source.
    m_passthrough->subscribe_message(
        MAVLINK_MSG_ID_HEARTBEAT,
        [this](const mavlink_message_t& msg) {
            if (msg.compid != MAV_COMP_ID_AUTOPILOT1)
                return;
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (!(hb.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))
                return;
            std::lock_guard<std::mutex> lock(m_telemetry_mutex);
            switch(hb.custom_mode)   // ArduPilot Rover mode numbers
            {
                case 0:  m_current_flight_mode = "MANUAL"; break;
                case 1:  m_current_flight_mode = "ACRO"; break;
                case 3:  m_current_flight_mode = "STEERING"; break;
                case 4:  m_current_flight_mode = "HOLD"; break;
                case 5:  m_current_flight_mode = "LOITER"; break;
                case 6:  m_current_flight_mode = "FOLLOW"; break;
                case 7:  m_current_flight_mode = "SIMPLE"; break;
                case 8:  m_current_flight_mode = "DOCK"; break;
                case 9:  m_current_flight_mode = "CIRCLE"; break;
                case 10: m_current_flight_mode = "AUTO"; break;
                case 11: m_current_flight_mode = "RTL"; break;
                case 12: m_current_flight_mode = "SMART_RTL"; break;
                case 15: m_current_flight_mode = "GUIDED"; break;
                case 16: m_current_flight_mode = "INITIALISING"; break;
                default: m_current_flight_mode = "UNKNOWN"; break;
            }
        });

    // Subscribe to armed state
    m_telemetry->subscribe_armed([this](bool armed) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_telemetry_armed = armed;
    });

    // Subscribe to IMU data (accelerometer, gyroscope, magnetometer)
    m_telemetry->subscribe_imu([this](Telemetry::Imu imu) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_imu_accel_x = imu.acceleration_frd.forward_m_s2;
        m_imu_accel_y = imu.acceleration_frd.right_m_s2;
        m_imu_accel_z = imu.acceleration_frd.down_m_s2;
        m_imu_gyro_x = imu.angular_velocity_frd.forward_rad_s;
        m_imu_gyro_y = imu.angular_velocity_frd.right_rad_s;
        m_imu_gyro_z = imu.angular_velocity_frd.down_rad_s;
        m_mag_x = imu.magnetic_field_frd.forward_gauss;
        m_mag_y = imu.magnetic_field_frd.right_gauss;
        m_mag_z = imu.magnetic_field_frd.down_gauss;
    });

    // Subscribe to raw GPS data for detailed statistics (HDOP, VDOP, accuracy)
    m_telemetry->subscribe_raw_gps([this](Telemetry::RawGps raw_gps) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        // MAVLink sends UINT16_MAX (655.35 after scaling) for a DOP the receiver
        // does not report; the UM982's NMEA stream has no VDOP. Keep -1 = unknown.
        m_gps_hdop = (raw_gps.hdop >= 655.0f) ? -1.0 : raw_gps.hdop;
        m_gps_vdop = (raw_gps.vdop >= 655.0f) ? -1.0 : raw_gps.vdop;
        m_gps_horiz_accuracy = raw_gps.horizontal_uncertainty_m;
        m_gps_vert_accuracy = raw_gps.vertical_uncertainty_m;
    });

    // Subscribe to battery data
    m_telemetry->subscribe_battery([this](Telemetry::Battery battery) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_battery_voltage = battery.voltage_v;
        m_battery_current = std::isnan(battery.current_battery_a) ? 0.0 : battery.current_battery_a;
        m_battery_remaining_pct = battery.remaining_percent * 100.0;
    });

    // Subscribe to attitude (roll, pitch, yaw)
    m_telemetry->subscribe_attitude_euler([this](Telemetry::EulerAngle euler) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_nav_roll = euler.roll_deg;
        m_nav_pitch = euler.pitch_deg;
        // yaw_deg is already captured in heading subscription
    });

    // Subscribe to system health
    m_telemetry->subscribe_health([this](Telemetry::Health health) {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_health_gyrometer_cal = health.is_gyrometer_calibration_ok;
        m_health_accelerometer_cal = health.is_accelerometer_calibration_ok;
        m_health_magnetometer_cal = health.is_magnetometer_calibration_ok;
        m_health_local_position = health.is_local_position_ok;
        m_health_global_position = health.is_global_position_ok;
        m_health_armable = health.is_armable;
    });

    // Subscribe to COMMAND_ACK to see command responses
    m_passthrough->subscribe_message(
        MAVLINK_MSG_ID_COMMAND_ACK,
        [this](const mavlink_message_t& msg) {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            const char* result_str = "UNKNOWN";
            switch(ack.result) {
                case MAV_RESULT_ACCEPTED: result_str = "ACCEPTED"; break;
                case MAV_RESULT_TEMPORARILY_REJECTED: result_str = "TEMP_REJECTED"; break;
                case MAV_RESULT_DENIED: result_str = "DENIED"; break;
                case MAV_RESULT_UNSUPPORTED: result_str = "UNSUPPORTED"; break;
                case MAV_RESULT_FAILED: result_str = "FAILED"; break;
                case MAV_RESULT_IN_PROGRESS: result_str = "IN_PROGRESS"; break;
                case MAV_RESULT_CANCELLED: result_str = "CANCELLED"; break;
                default: break;
            }
            dbg_print("%0.2f: COMMAND_ACK: cmd=%d result=%s (%d)\n",
                     MOOSTime(), ack.command, result_str, ack.result);
        });

    // Subscribe to STATUSTEXT - ArduPilot debug/info messages
    m_passthrough->subscribe_message(
        MAVLINK_MSG_ID_STATUSTEXT,
        [this](const mavlink_message_t& msg) {
            mavlink_statustext_t statustext;
            mavlink_msg_statustext_decode(&msg, &statustext);
            // Ensure null termination
            char text[51];
            memcpy(text, statustext.text, 50);
            text[50] = '\0';
            const char* severity_str = "?";
            switch(statustext.severity) {
                case MAV_SEVERITY_EMERGENCY: severity_str = "EMERG"; break;
                case MAV_SEVERITY_ALERT: severity_str = "ALERT"; break;
                case MAV_SEVERITY_CRITICAL: severity_str = "CRIT"; break;
                case MAV_SEVERITY_ERROR: severity_str = "ERROR"; break;
                case MAV_SEVERITY_WARNING: severity_str = "WARN"; break;
                case MAV_SEVERITY_NOTICE: severity_str = "NOTE"; break;
                case MAV_SEVERITY_INFO: severity_str = "INFO"; break;
                case MAV_SEVERITY_DEBUG: severity_str = "DEBUG"; break;
                default: break;
            }
            dbg_print("%0.2f: STATUSTEXT [%s]: %s\n", MOOSTime(), severity_str, text);
        });

    // Subscribe to POSITION_TARGET_LOCAL_NED (msg 85) - FC's current velocity/yaw target
    // This echoes back what the FC thinks it's targeting when in GUIDED mode
    m_passthrough->subscribe_message(
        MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,
        [this](const mavlink_message_t& msg) {
            mavlink_position_target_local_ned_t target;
            mavlink_msg_position_target_local_ned_decode(&msg, &target);
            std::lock_guard<std::mutex> lock(m_telemetry_mutex);
            m_fc_target_vx = target.vx;
            m_fc_target_vy = target.vy;
            m_fc_target_yaw = target.yaw;
        });

    // Subscribe to SERVO_OUTPUT_RAW - shows actual motor/servo commands
    m_passthrough->subscribe_message(
        MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        [this](const mavlink_message_t& msg) {
            mavlink_servo_output_raw_t servo;
            mavlink_msg_servo_output_raw_decode(&msg, &servo);
            std::lock_guard<std::mutex> lock(m_telemetry_mutex);
            const uint16_t raw[16] = {
                servo.servo1_raw, servo.servo2_raw, servo.servo3_raw, servo.servo4_raw,
                servo.servo5_raw, servo.servo6_raw, servo.servo7_raw, servo.servo8_raw,
                servo.servo9_raw, servo.servo10_raw, servo.servo11_raw, servo.servo12_raw,
                servo.servo13_raw, servo.servo14_raw, servo.servo15_raw, servo.servo16_raw};
            m_left_motor_pwm = raw[m_left_motor_servo - 1];
            m_right_motor_pwm = raw[m_right_motor_servo - 1];
        });

    // Subscribe to VFR_HUD - useful pilot info including throttle percentage
    m_passthrough->subscribe_message(
        MAVLINK_MSG_ID_VFR_HUD,
        [this](const mavlink_message_t& msg) {
            mavlink_vfr_hud_t hud;
            mavlink_msg_vfr_hud_decode(&msg, &hud);
            std::lock_guard<std::mutex> lock(m_telemetry_mutex);
            m_throttle_pct = hud.throttle;
        });
}

//---------------------------------------------------------
// Procedure: publishTelemetryToMOOS

void iArduRoverBridge::publishTelemetryToMOOS()
{
    std::lock_guard<std::mutex> lock(m_telemetry_mutex);

    // Publish GPS position
    Notify("NAV_LAT", m_nav_lat);
    Notify("NAV_LONG", m_nav_lon);
    Notify("NAV_ALTITUDE", m_nav_altitude);

    // Publish heading and speed
    Notify("NAV_HEADING", m_nav_heading);
    Notify("NAV_SPEED", m_nav_speed);

    // Convert lat/lon to local X/Y
    if (m_nav_lat != 0.0 && m_nav_lon != 0.0)
    {
        double x, y;
        if (m_geodesy.LatLong2LocalGrid(m_nav_lat, m_nav_lon, y, x))
        {
            m_nav_x = x;
            m_nav_y = y;
            Notify("NAV_X", m_nav_x);
            Notify("NAV_Y", m_nav_y);
        }
    }

    // Publish GPS info
    Notify("GPS_SAT", m_gps_num_sats);
    Notify("GPS_FIX_TYPE", m_gps_fix_type);
    if (m_gps_hdop >= 0)
        Notify("GPS_HDOP", m_gps_hdop);
    if (m_gps_vdop >= 0)
        Notify("GPS_VDOP", m_gps_vdop);
    Notify("GPS_HORIZ_ACCURACY", m_gps_horiz_accuracy);
    Notify("GPS_VERT_ACCURACY", m_gps_vert_accuracy);

    // Publish attitude (roll, pitch)
    Notify("NAV_ROLL", m_nav_roll);
    Notify("NAV_PITCH", m_nav_pitch);

    // Publish IMU data (accelerometer in m/s^2)
    Notify("IMU_ACCEL_X", m_imu_accel_x);
    Notify("IMU_ACCEL_Y", m_imu_accel_y);
    Notify("IMU_ACCEL_Z", m_imu_accel_z);

    // Publish IMU data (gyroscope in rad/s)
    Notify("IMU_GYRO_X", m_imu_gyro_x);
    Notify("IMU_GYRO_Y", m_imu_gyro_y);
    Notify("IMU_GYRO_Z", m_imu_gyro_z);

    // Publish magnetometer data (Gauss)
    Notify("MAG_X", m_mag_x);
    Notify("MAG_Y", m_mag_y);
    Notify("MAG_Z", m_mag_z);

    // Publish battery data. With a BMS file configured the pack is the source
    // (publishBoatState); this boat's autopilot has no battery monitor and
    // would only publish zeros over the real numbers.
    if (m_bms_file.empty())
    {
        Notify("BATTERY_VOLTAGE", m_battery_voltage);
        Notify("BATTERY_CURRENT", m_battery_current);
        Notify("BATTERY_REMAINING", m_battery_remaining_pct);
    }

    // Publish system health flags
    Notify("HEALTH_GYRO_CAL", m_health_gyrometer_cal ? "true" : "false");
    Notify("HEALTH_ACCEL_CAL", m_health_accelerometer_cal ? "true" : "false");
    Notify("HEALTH_MAG_CAL", m_health_magnetometer_cal ? "true" : "false");
    Notify("HEALTH_LOCAL_POS", m_health_local_position ? "true" : "false");
    Notify("HEALTH_GLOBAL_POS", m_health_global_position ? "true" : "false");
    Notify("HEALTH_ARMABLE", m_health_armable ? "true" : "false");

    // Publish mode and armed state
    Notify("ARDUROVER_MODE", m_current_flight_mode);
    Notify("ARDUROVER_ARMED", m_telemetry_armed ? "true" : "false");
}

//---------------------------------------------------------
// Procedure: updateBoatState
//   Purpose: Re-read the BMS and VCB state files and raise or retract the
//            run warnings that go with them.

void iArduRoverBridge::updateBoatState()
{
    if (!m_bms_file.empty())
    {
        readBmsFile(m_bms_file, m_boat_state_stale_sec, m_bms);
        bool good = m_bms.fresh && m_bms.online;
        // Only complain about a source that worked earlier in this run, so a
        // laptop or SITL launch without the files stays quiet.
        if (good)
        {
            m_bms_seen = true;
            retractRunWarning("BMS data stale or offline");
        }
        else if (m_bms_seen)
            reportRunWarning("BMS data stale or offline");
    }

    if (!m_vcb_file.empty())
    {
        readVcbFile(m_vcb_file, m_boat_state_stale_sec, m_vcb);
        bool good = m_vcb.fresh && m_vcb.online;
        if (good)
        {
            m_vcb_seen = true;
            retractRunWarning("VCB data stale or offline");
            if (m_vcb.fault != m_vcb_last_fault)
            {
                if (m_vcb.fault != 0)
                    reportEvent("VCB fault " + intToString(m_vcb.fault) + ": " + vcbFaultName(m_vcb.fault));
                else
                    reportEvent("VCB fault cleared");
                m_vcb_last_fault = m_vcb.fault;
            }
        }
        else if (m_vcb_seen)
            reportRunWarning("VCB data stale or offline");
    }
}

//---------------------------------------------------------
// Procedure: publishBoatState
//   Purpose: BATTERY_* from the pack and VCB_* from the thruster board.
//            BATTERY_OK and VCB_READY are the two flags a mission can put
//            in a behavior condition; both are false when the data is stale.

void iArduRoverBridge::publishBoatState()
{
    if (!m_bms_file.empty())
    {
        bool good = m_bms.fresh && m_bms.online;
        Notify("BATTERY_ONLINE", good ? "true" : "false");
        if (good)
        {
            Notify("BATTERY_VOLTAGE", m_bms.voltage);
            Notify("BATTERY_CURRENT", m_bms.current);
            Notify("BATTERY_REMAINING", m_bms.soc);
            Notify("BATTERY_POWER", m_bms.power);
            Notify("BATTERY_TEMP", m_bms.temp);
            Notify("BATTERY_CELL_MIN", m_bms.cell_min);
            Notify("BATTERY_CELL_SPREAD_MV", (m_bms.cell_max - m_bms.cell_min) * 1000.0);
        }
        bool ok = good && m_bms.soc > m_battery_low_pct && m_bms.voltage > m_battery_low_volts;
        Notify("BATTERY_OK", ok ? "true" : "false");
    }

    if (!m_vcb_file.empty())
    {
        bool good = m_vcb.fresh && m_vcb.online;
        Notify("VCB_ONLINE", good ? "true" : "false");
        if (good)
        {
            Notify("VCB_STATE", vcbStateName(m_vcb.state));
            Notify("VCB_FAULT_CODE", static_cast<double>(m_vcb.fault));
            Notify("VCB_ARMED", m_vcb.armed ? "true" : "false");
            Notify("VCB_HV_ON", m_vcb.hv_on ? "true" : "false");
            Notify("VCB_TSMS", m_vcb.tsms ? "true" : "false");
            Notify("VCB_BUS_VOLTS", m_vcb.bus_volts);
            Notify("VCB_CMD_LEFT", m_vcb.cmd_left);
            Notify("VCB_CMD_RIGHT", m_vcb.cmd_right);
            Notify("VCB_PWM_VALID", m_vcb.pwm_valid ? "true" : "false");
        }
        // Ready to push water: in RUN with the contactors closed and no fault
        bool ready = good && m_vcb.state == 5 && m_vcb.fault == 0 && m_vcb.hv_on;
        Notify("VCB_READY", ready ? "true" : "false");
    }
}

//---------------------------------------------------------
// Procedure: checkPreArmHealth

bool iArduRoverBridge::checkPreArmHealth()
{
    if (!m_telemetry)
        return false;

    // Check overall health and log specific failures
    bool health_ok = m_telemetry->health_all_ok();
    if (!health_ok)
    {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        std::string failures = "";
        if (!m_health_gyrometer_cal) failures += "GYRO_CAL ";
        if (!m_health_accelerometer_cal) failures += "ACCEL_CAL ";
        if (!m_health_magnetometer_cal) failures += "MAG_CAL ";
        if (!m_health_local_position) failures += "LOCAL_POS ";
        if (!m_health_global_position) failures += "GLOBAL_POS ";
        if (!m_health_armable) failures += "NOT_ARMABLE ";

        dbg_print("%0.2f: Pre-arm check failed: %s\n", MOOSTime(),
                  failures.empty() ? "unknown" : failures.c_str());
        return false;
    }

    // Check GPS fix
    {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        if (m_gps_fix_type < 3)
        {
            dbg_print("%0.2f: Pre-arm check failed: GPS fix type %d (need 3D fix)\n",
                     MOOSTime(), m_gps_fix_type);
            return false;
        }
    }

    return true;
}

//---------------------------------------------------------
// Procedure: switchToHoldMode

bool iArduRoverBridge::switchToHoldMode()
{
    if (!m_passthrough)
        return false;

    std::lock_guard<std::mutex> lock(m_mavlink_mutex);

    dbg_print("%0.2f: Switching to HOLD mode...\n", MOOSTime());

    // ArduRover HOLD mode = 4
    uint8_t target_sysid = m_passthrough->get_target_sysid();

    MavlinkPassthrough::Result result = m_passthrough->queue_message(
        [target_sysid](MavlinkAddress mavlink_address, uint8_t channel) {
            mavlink_message_t msg;
            mavlink_msg_set_mode_pack_chan(
                mavlink_address.system_id,
                mavlink_address.component_id,
                channel,
                &msg,
                target_sysid,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4  // HOLD mode for ArduRover
            );
            return msg;
        });

    if (result != MavlinkPassthrough::Result::Success)
    {
        reportRunWarning("Failed to switch to HOLD mode");
        return false;
    }

    dbg_print("%0.2f: Switched to HOLD mode\n", MOOSTime());
    return true;
}

//---------------------------------------------------------
// Procedure: requestDisarm

bool iArduRoverBridge::requestDisarm()
{
    if (!m_action)
        return false;

    std::lock_guard<std::mutex> lock(m_mavlink_mutex);

    dbg_print("%0.2f: Requesting disarm...\n", MOOSTime());

    Action::Result result = m_action->disarm();

    if (result != Action::Result::Success)
    {
        dbg_print("%0.2f: Disarm request failed with result: %d\n", MOOSTime(), static_cast<int>(result));
        reportRunWarning("Disarm request failed");
        return false;
    }

    dbg_print("%0.2f: Disarm request sent successfully\n", MOOSTime());
    return true;
}

//---------------------------------------------------------
// Procedure: sendGuidedCommand

void iArduRoverBridge::sendGuidedCommand(double heading_deg, double speed_mps)
{
    if (!m_passthrough)
        return;
    // Note: Iterate() already verifies we're in GUIDED mode before calling this

    std::lock_guard<std::mutex> lock(m_mavlink_mutex);

    // Use SET_POSITION_TARGET_GLOBAL_INT (message 86) with velocity control
    // Per ArduPilot docs: https://ardupilot.org/dev/docs/mavlink-rover-commands.html
    //
    // MAV_FRAME_GLOBAL_INT (5): Global coordinate frame with WGS84 coords
    //   vx = velocity North (m/s), positive = North
    //   vy = velocity East (m/s), positive = East
    //   vz = velocity Down (m/s), set to 0 for rovers
    //
    // type_mask 0x0DE7 (3559) = velocity control
    // type_mask bits (1 = ignore this field):
    //   bit 0:  ignore lat_int       = 1
    //   bit 1:  ignore lon_int       = 1
    //   bit 2:  ignore alt           = 1
    //   bit 3:  ignore vx            = 0  (USE vx - North)
    //   bit 4:  ignore vy            = 0  (USE vy - East)
    //   bit 5:  ignore vz            = 0  (USE vz - Down)
    //   bit 6:  ignore afx           = 1
    //   bit 7:  ignore afy           = 1
    //   bit 8:  ignore afz           = 1
    //   bit 9:  ignore force         = 1
    //   bit 10: ignore yaw           = 1
    //   bit 11: ignore yaw_rate      = 1
    //
    // type_mask = 0b110111100111 = 0x0DE7 = 3559

    // Convert heading/speed to North/East velocity components set a minimum speed of 0.0001m/s to prevent spinning
    // heading_deg: 0 = North, 90 = East, 180 = South, 270 = West
    double heading_rad = heading_deg * M_PI / 180.0;
    double cmd_speed = std::max(speed_mps, 0.0001);

    float vn = static_cast<float>(cmd_speed * cos(heading_rad));   // North component
    float ve = static_cast<float>(cmd_speed * sin(heading_rad));   // East component
    float vd = 0.0f;                                               // Down component (0 for rovers)

    uint8_t target_sysid = m_passthrough->get_target_sysid();
    uint8_t target_compid = m_passthrough->get_target_compid();

    // type_mask = 3559 = velocity control (per ArduPilot rover docs)
    uint16_t type_mask = 0x0DE7;

    MavlinkPassthrough::Result result = m_passthrough->queue_message(
        [target_sysid, target_compid, vn, ve, vd, type_mask](
            MavlinkAddress mavlink_address, uint8_t channel) {
            mavlink_message_t msg;
            mavlink_msg_set_position_target_global_int_pack_chan(
                mavlink_address.system_id,
                mavlink_address.component_id,
                channel,
                &msg,
                0,                          // time_boot_ms (not used)
                target_sysid,
                target_compid,
                MAV_FRAME_GLOBAL_INT,       // Frame 5: global coords
                type_mask,                  // 0x0DE7 = velocity control
                0,                          // lat_int (ignored)
                0,                          // lon_int (ignored)
                0.0f,                       // alt (ignored)
                vn,                         // vx: North velocity (m/s)
                ve,                         // vy: East velocity (m/s)
                vd,                         // vz: Down velocity (m/s)
                0.0f, 0.0f, 0.0f,          // afx, afy, afz acceleration (ignored)
                0.0f,                       // yaw (ignored)
                0.0f                        // yaw_rate (ignored)
            );
            return msg;
        });

    if (result == MavlinkPassthrough::Result::Success)
    {
        m_commands_sent++;
        m_last_command_time = MOOSTime();

        // Log current state, desired setpoints, commanded values, and FC internals
        double cur_hdg, cur_spd, cur_x, cur_y;
        float fc_tgt_vx, fc_tgt_vy, fc_tgt_yaw;
        double fc_throttle;
        uint16_t fc_left_pwm, fc_right_pwm;
        {
            std::lock_guard<std::mutex> lock(m_telemetry_mutex);
            cur_hdg = m_nav_heading;
            cur_spd = m_nav_speed;
            cur_x = m_nav_x;
            cur_y = m_nav_y;
            fc_tgt_vx = m_fc_target_vx;
            fc_tgt_vy = m_fc_target_vy;
            fc_tgt_yaw = m_fc_target_yaw;
            fc_throttle = m_throttle_pct;
            fc_left_pwm = m_left_motor_pwm;
            fc_right_pwm = m_right_motor_pwm;
        }
        dbg_print("%0.2f: CMD: cur_pos=(%.1f,%.1f) cur_hdg=%.1f cur_spd=%.2f | "
                 "des_hdg=%.1f des_spd=%.2f | "
                 "cmd_vn=%.2f cmd_ve=%.2f m/s | "
                 "FC: tgt_vx=%.2f tgt_vy=%.2f tgt_yaw=%.1f thr=%.0f%% L=%d R=%d\n",
                 MOOSTime(),
                 cur_x, cur_y, cur_hdg, cur_spd,
                 m_desired_heading, m_desired_speed,
                 vn, ve,
                 fc_tgt_vx, fc_tgt_vy, fc_tgt_yaw * 180.0 / M_PI, fc_throttle, fc_right_pwm, fc_left_pwm);
    }
    else
    {
        reportRunWarning("Failed to send guided command");
    }
}

//---------------------------------------------------------
// Procedure: GeodesySetup()

bool iArduRoverBridge::GeodesySetup()
{
    double LatOrigin = 0.0;
    double LonOrigin = 0.0;

    bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
    if (!latOK)
    {
        reportConfigWarning("Latitude origin missing in MOOS file.");
        return false;
    }

    bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
    if (!lonOK)
    {
        reportConfigWarning("Longitude origin missing in MOOS file.");
        return false;
    }

    bool geoOK = m_geodesy.Initialise(LatOrigin, LonOrigin);
    if (!geoOK)
    {
        reportConfigWarning("CMOOSGeodesy::Initialise() failed. Invalid origin.");
        return false;
    }

    return true;
}

//---------------------------------------------------------
// Procedure: dbg_print

bool iArduRoverBridge::dbg_print(const char *format, ...)
{
    if (m_debug)
    {
        va_list args;
        va_start(args, format);
        std::lock_guard<std::mutex> lock(m_debug_mutex);
        m_debug_stream = fopen(m_fname.c_str(), "a");
        if (m_debug_stream)
        {
            vfprintf(m_debug_stream, format, args);
            fclose(m_debug_stream);
        }
        va_end(args);
    }
    return true;
}

//---------------------------------------------------------
// Procedure: buildReport()

bool iArduRoverBridge::buildReport()
{
    m_msgs << "============================================" << endl;
    m_msgs << "iArduRoverBridge Report" << endl;
    m_msgs << "============================================" << endl;

    m_msgs << "Configuration:" << endl;
    m_msgs << "  MAVLink URL: " << m_mavlink_connection_url << endl;
    m_msgs << "  Command timeout: " << m_command_timeout_sec << " sec" << endl;
    m_msgs << endl;

    m_msgs << "Vehicle State (from telemetry):" << endl;
    {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_msgs << "  Connected: " << (m_connected ? "YES" : "NO") << endl;
        m_msgs << "  Armed: " << (m_telemetry_armed ? "YES" : "NO") << endl;
        m_msgs << "  Flight Mode: " << m_current_flight_mode << endl;
    }
    m_msgs << endl;

    m_msgs << "MOOS Commands:" << endl;
    m_msgs << "  DEPLOY: " << (m_deploy ? "true" : "false") << endl;
    m_msgs << "  ALLSTOP: " << (m_allstop ? "true" : "false") << endl;
    m_msgs << "  MOOS_MANUAL_OVERRIDE: " << (m_moos_manual_override ? "true" : "false") << endl;
    m_msgs << endl;

    // Determine if autonomy is active
    bool vehicle_armed = false;
    bool vehicle_in_guided = false;
    {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        vehicle_armed = m_telemetry_armed;
        vehicle_in_guided = (m_current_flight_mode == "GUIDED");
    }
    bool autonomy_active = m_deploy && vehicle_armed && vehicle_in_guided && !m_allstop && !m_moos_manual_override;
    m_msgs << "Autonomy Status: " << (autonomy_active ? "ACTIVE (sending commands)" : "INACTIVE") << endl;
    if (!autonomy_active && m_deploy)
    {
        m_msgs << "  Reason: ";
        if (m_allstop) m_msgs << "ALLSTOP ";
        if (m_moos_manual_override) m_msgs << "MOOS_MANUAL_OVERRIDE ";
        if (!vehicle_armed) m_msgs << "NOT_ARMED ";
        if (!vehicle_in_guided) m_msgs << "NOT_IN_GUIDED ";
        m_msgs << endl;
    }
    m_msgs << endl;

    m_msgs << "Current Setpoints:" << endl;
    if (m_last_desired_heading_time > 0)
    {
        double hdg_age = MOOSTime() - m_last_desired_heading_time;
        double spd_age = MOOSTime() - m_last_desired_speed_time;
        bool fresh = (hdg_age < 5.0 && spd_age < 5.0);
        m_msgs << "  Desired Heading: " << doubleToStringX(m_desired_heading, 1)
               << " deg (age " << doubleToStringX(hdg_age, 1) << " s)" << endl;
        m_msgs << "  Desired Speed: " << doubleToStringX(m_desired_speed, 2)
               << " m/s (age " << doubleToStringX(spd_age, 1) << " s)" << endl;
        m_msgs << "  Fresh: " << (fresh ? "YES" : "NO (stale > 5 s)") << endl;
        if (m_desired_speed <= 0.0)
            m_msgs << "  Zero speed: commanding current heading "
                   << "(heading-neutral, no searching)" << endl;
    }
    else
        m_msgs << "  (none received from helm yet)" << endl;
    m_msgs << endl;

    // Core telemetry (always shown)
    m_msgs << "Telemetry:" << endl;
    {
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_msgs << "  Position: " << doubleToStringX(m_nav_lat, 6) << ", "
               << doubleToStringX(m_nav_lon, 6) << endl;
        m_msgs << "  Local X,Y: " << doubleToStringX(m_nav_x, 2) << ", "
               << doubleToStringX(m_nav_y, 2) << endl;
        m_msgs << "  Heading: " << doubleToStringX(m_nav_heading, 1) << " deg" << endl;
        m_msgs << "  Speed: " << doubleToStringX(m_nav_speed, 2) << " m/s" << endl;
        m_msgs << "  GPS Sats: " << m_gps_num_sats << " (fix type: " << m_gps_fix_type << ")" << endl;
        if (m_bms_file.empty())
            m_msgs << "  Battery: " << doubleToStringX(m_battery_voltage, 2) << "V, "
                   << doubleToStringX(m_battery_remaining_pct, 1) << "%" << endl;
    }
    m_msgs << endl;

    // Boat state from the Jetson's files (only the sources that are configured)
    if (!m_bms_file.empty() || !m_vcb_file.empty())
    {
        m_msgs << "Boat State (files, stale after "
               << doubleToStringX(m_boat_state_stale_sec, 1) << " s):" << endl;
        if (!m_bms_file.empty())
        {
            if (m_bms.age < 0)
                m_msgs << "  Battery: no file at " << m_bms_file << endl;
            else if (!m_bms.fresh || !m_bms.online)
                m_msgs << "  Battery: " << (m_bms.fresh ? "BMS OFFLINE" : "STALE")
                       << " (file age " << doubleToStringX(m_bms.age, 0) << " s)" << endl;
            else
            {
                bool ok = m_bms.soc > m_battery_low_pct && m_bms.voltage > m_battery_low_volts;
                m_msgs << "  Battery: " << doubleToStringX(m_bms.soc, 0) << " %, "
                       << doubleToStringX(m_bms.voltage, 2) << " V, "
                       << doubleToStringX(m_bms.current, 1) << " A, "
                       << doubleToStringX(m_bms.power, 0) << " W, "
                       << doubleToStringX(m_bms.temp, 0) << " C, cells "
                       << doubleToStringX(m_bms.cell_min, 3) << "-"
                       << doubleToStringX(m_bms.cell_max, 3) << " V" << endl;
                m_msgs << "  BATTERY_OK: " << (ok ? "true" : "FALSE") << " (limits "
                       << doubleToStringX(m_battery_low_pct, 0) << " %, "
                       << doubleToStringX(m_battery_low_volts, 1) << " V)" << endl;
            }
        }
        if (!m_vcb_file.empty())
        {
            if (m_vcb.age < 0)
                m_msgs << "  VCB: no file at " << m_vcb_file << endl;
            else if (!m_vcb.fresh || !m_vcb.online)
                m_msgs << "  VCB: " << (m_vcb.fresh ? "OFFLINE" : "STALE")
                       << " (file age " << doubleToStringX(m_vcb.age, 0) << " s)" << endl;
            else
            {
                m_msgs << "  VCB: " << vcbStateName(m_vcb.state);
                if (m_vcb.fault != 0)
                    m_msgs << " " << m_vcb.fault << " (" << vcbFaultName(m_vcb.fault) << ")";
                m_msgs << ", HV " << (m_vcb.hv_on ? "on" : "off") << ", bus "
                       << doubleToStringX(m_vcb.bus_volts, 1) << " V, TSMS "
                       << (m_vcb.tsms ? "on" : "off") << ", thrust L/R "
                       << doubleToStringX(m_vcb.cmd_left, 0) << "/"
                       << doubleToStringX(m_vcb.cmd_right, 0) << " %" << endl;
                bool ready = m_vcb.state == 5 && m_vcb.fault == 0 && m_vcb.hv_on;
                m_msgs << "  VCB_READY: " << (ready ? "true" : "FALSE") << endl;
            }
        }
        m_msgs << endl;
    }

    // Detailed GPS data (optional)
    if (m_show_gps_details)
    {
        m_msgs << "GPS Details: (show_gps_details=true)" << endl;
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_msgs << "  HDOP: " << (m_gps_hdop >= 0 ? doubleToStringX(m_gps_hdop, 2) : "not reported")
               << "  VDOP: " << (m_gps_vdop >= 0 ? doubleToStringX(m_gps_vdop, 2) : "not reported") << endl;
        m_msgs << "  Horiz Accuracy: " << doubleToStringX(m_gps_horiz_accuracy, 2) << " m" << endl;
        m_msgs << "  Vert Accuracy: " << doubleToStringX(m_gps_vert_accuracy, 2) << " m" << endl;
        m_msgs << endl;
    }

    // IMU/Attitude data (optional)
    if (m_show_imu_data)
    {
        m_msgs << "IMU Data: (show_imu_data=true)" << endl;
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_msgs << "  Roll/Pitch: " << doubleToStringX(m_nav_roll, 1) << " / "
               << doubleToStringX(m_nav_pitch, 1) << " deg" << endl;
        m_msgs << "  Accel: [" << doubleToStringX(m_imu_accel_x, 2) << ", "
               << doubleToStringX(m_imu_accel_y, 2) << ", "
               << doubleToStringX(m_imu_accel_z, 2) << "] m/s^2" << endl;
        m_msgs << "  Gyro: [" << doubleToStringX(m_imu_gyro_x, 2) << ", "
               << doubleToStringX(m_imu_gyro_y, 2) << ", "
               << doubleToStringX(m_imu_gyro_z, 2) << "] rad/s" << endl;
        m_msgs << "  Mag: [" << doubleToStringX(m_mag_x, 2) << ", "
               << doubleToStringX(m_mag_y, 2) << ", "
               << doubleToStringX(m_mag_z, 2) << "] Gauss" << endl;
        m_msgs << endl;
    }

    // System health flags (optional)
    if (m_show_health_flags)
    {
        m_msgs << "System Health: (show_health_flags=true)" << endl;
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_msgs << "  Gyro Cal: " << (m_health_gyrometer_cal ? "OK" : "FAIL") << endl;
        m_msgs << "  Accel Cal: " << (m_health_accelerometer_cal ? "OK" : "FAIL") << endl;
        m_msgs << "  Mag Cal: " << (m_health_magnetometer_cal ? "OK" : "FAIL") << endl;
        m_msgs << "  Local Pos: " << (m_health_local_position ? "OK" : "FAIL") << endl;
        m_msgs << "  Global Pos: " << (m_health_global_position ? "OK" : "FAIL") << endl;
        m_msgs << "  Armable: " << (m_health_armable ? "YES" : "NO") << endl;
        m_msgs << endl;
    }

    // FC target feedback and motor outputs (optional)
    if (m_show_nav_controller)
    {
        m_msgs << "FC Target Feedback: (show_nav_controller=true)" << endl;
        std::lock_guard<std::mutex> lock(m_telemetry_mutex);
        m_msgs << "  FC Target Vx: " << doubleToStringX(m_fc_target_vx, 2) << " m/s" << endl;
        m_msgs << "  FC Target Vy: " << doubleToStringX(m_fc_target_vy, 2) << " m/s" << endl;
        m_msgs << "  FC Target Yaw: " << doubleToStringX(m_fc_target_yaw * 180.0 / M_PI, 1) << " deg" << endl;
        m_msgs << "  Throttle: " << doubleToStringX(m_throttle_pct, 0) << "%" << endl;
        m_msgs << "  Left Motor PWM: " << m_left_motor_pwm << " (SERVO" << m_left_motor_servo << ")" << endl;
        m_msgs << "  Right Motor PWM: " << m_right_motor_pwm << " (SERVO" << m_right_motor_servo << ")" << endl;
        m_msgs << endl;
    }

    m_msgs << "Statistics:" << endl;
    m_msgs << "  Commands sent: " << m_commands_sent << endl;
    m_msgs << "  Telemetry updates: " << m_telemetry_updates_received << endl;

    if (m_last_command_time > 0)
    {
        double age = MOOSTime() - m_last_command_time;
        m_msgs << "  Last command: " << doubleToStringX(age, 2) << " sec ago" << endl;
    }

    if (m_last_telemetry_time > 0)
    {
        double age = MOOSTime() - m_last_telemetry_time;
        m_msgs << "  Last telemetry: " << doubleToStringX(age, 2) << " sec ago" << endl;
    }

    return true;
}
