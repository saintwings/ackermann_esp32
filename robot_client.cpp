/**
 * robot_client.cpp
 * 
 * Integrated robot client combining:
 * - Server connectivity and command handling from imu_gps_client
 * - Real robot task execution from task_navigator
 * - Excludes obstacle avoidance as requested
 * 
 * Features:
 * - WebSocket connection to multi-robot control server
 * - Mission execution (GOTO_WAYPOINT, SPIRAL, WAIT, LOOP)
 * - Real-time GPS/IMU data fusion
 * - Pause/resume mission capability
 * - Cancel mission with immediate stop
 * - Telemetry reporting back to server
 */

#include "robot_client.hpp"
#include "waypoint_utils.hpp"
#include "sensor_fusion.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <signal.h>
#include <yaml-cpp/yaml.h>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using WsClient = websocketpp::client<websocketpp::config::asio_client>;

// ============================================================================
// Configuration & Globals
// ============================================================================

constexpr int HEARTBEAT_INTERVAL_SECONDS = 5;
double g_waypoint_acceptance_radius = 1.0;  // meters
constexpr double SPIRAL_SPEED = 0.20;  // m/s
constexpr double WHEELBASE = 0.36;     // meters
constexpr double TRACK_WIDTH = 0.30;   // meters (wheel separation for differential drive)

// Navigation parameters
constexpr double NAV_LOOK_AHEAD_DIST = 2.0;  // meters
constexpr double NAV_MAX_SPEED = 0.30;       // m/s
constexpr double NAV_MIN_SPEED = 0.10;       // m/s
constexpr double NAV_KP_HEADING_DEFAULT = 1.5;  // Default proportional gain for heading
constexpr double NAV_MAX_ANGLE = 30.0;       // degrees max steering
constexpr double NAV_HEADING_DEADBAND_DEG = 2.0;   // Ignore tiny heading jitter
constexpr double NAV_HEADING_ERR_ALPHA = 0.25;     // Low-pass filter for heading error
constexpr double NAV_MAX_ANGULAR = 0.7;            // rad/s hard clamp for stability
constexpr double NAV_MAX_ANGULAR_ACCEL = 1.2;      // rad/s^2 slew-rate limit
constexpr int CONTROL_RECONNECT_TIMEOUT = 10;  // Max 10 reconnect attempts per interval
double g_nav_kp_heading = NAV_KP_HEADING_DEFAULT;

// Robot client global
RobotClient* g_robot_client = nullptr;

// Control connection with auto-reconnect
WsClient control_client;
websocketpp::connection_hdl control_hdl;
std::atomic<bool> control_connected{false};
std::string g_control_url;
std::chrono::steady_clock::time_point last_control_reconnect;
constexpr int RECONNECT_INTERVAL_SECONDS = 5;  // Try to reconnect every 5 seconds if disconnected

// Robot state
struct RobotState {
    double lat = 0.0;
    double lon = 0.0;
    double yaw = 0.0;
    double timestamp = 0.0;
    bool valid = false;
    bool gps_valid = false;
    bool imu_valid = false;
};

// Mission state
struct MissionStatus {
    bool active = false;
    std::string mission_name;
    int total_tasks = 0;
    int current_task = 0;
    std::string current_task_type;
    std::string status_message;
    double progress_percent = 0.0;
    
    json to_json() const {
        json j;
        j["active"] = active;
        j["mission_name"] = mission_name;
        j["total_tasks"] = total_tasks;
        j["current_task"] = current_task;
        j["current_task_type"] = current_task_type;
        j["status_message"] = status_message;
        j["progress_percent"] = progress_percent;
        return j;
    }
};

// Task definitions
enum class TaskType {
    GOTO_WAYPOINT = 0,
    SPIRAL = 1,
    WAIT = 2,
    LOOP_START = 3,
    LOOP_END = 4
};

struct Task {
    TaskType type;
    std::string description;
    int waypoint_index = -1;
    int loop_count = 1;
    double speed_mps = -1.0;  // <= 0 means use default navigation profile
    double wait_duration = 0.0;
    double spiral_radius = 5.0;
    double spiral_spacing = 1.0;
    int spiral_loops = 3;
    std::string spiral_direction = "ccw";
};

// Global state variables
RobotState current_state;
IMUData latest_imu;
MissionStatus mission_status;
std::mutex state_mutex;
std::mutex mission_status_mutex;

std::vector<Waypoint> mission_waypoints;
std::vector<Task> mission_tasks;

// Mission control flags (pause/cancel)
std::atomic<bool> mission_active{false};
std::atomic<bool> mission_pause_requested{false};
std::atomic<bool> mission_cancel_requested{false};
std::atomic<bool> mission_paused{false};
std::atomic<int> current_task_idx{0};

// Task execution state
std::atomic<bool> waypoint_navigation_active{false};
std::atomic<bool> spiral_active{false};
std::atomic<bool> move_active{false};  // Flag for COMMAND_MOVE thread
std::atomic<uint64_t> waypoint_nav_generation{0};  // Generation token for waypoint threads
Waypoint current_navigation_target;  // Current target for navigation
std::mutex navigation_target_mutex;
Waypoint spiral_center;
double spiral_radius_start = 0.0;
double spiral_loops_target = 0.0;
std::string spiral_direction = "ccw";
std::chrono::steady_clock::time_point spiral_start_time;

// Sensor fusion
rainav::YawSensorFusion yaw_fusion;

double g_yaw_offset_deg = 0.0;

double normalizeYawDeg(double angle_deg) {
    while (angle_deg > 180.0) angle_deg -= 360.0;
    while (angle_deg < -180.0) angle_deg += 360.0;
    return angle_deg;
}

double getTaskSpeedOrDefault(double requested_speed_mps, double fallback_speed_mps) {
    if (requested_speed_mps > 1e-6) {
        return requested_speed_mps;
    }
    return fallback_speed_mps;
}

// ============================================================================
// Signal Handler
// ============================================================================

void signal_handler(int sig) {
    std::cout << "\n[CLIENT] Received signal " << sig << ", shutting down..." << std::endl;
    if (g_robot_client) {
        g_robot_client->stop();
    }
    exit(0);
}

// ============================================================================
// Sensor Data Handlers
// ============================================================================

void updateRobotStateFromSensors() {
    std::lock_guard<std::mutex> state_lock(state_mutex);
    
    // GPS and IMU data are accessed through robot_client which wraps sensor_client
    // These will be updated in main loop
}

// ============================================================================
// Mission Status Reporting
// ============================================================================

json getMissionStatusJson() {
    std::lock_guard<std::mutex> lock(mission_status_mutex);
    return mission_status.to_json();
}

void updateMissionStatusMessage(const std::string& message) {
    std::lock_guard<std::mutex> lock(mission_status_mutex);
    mission_status.status_message = message;
}

// ============================================================================
// Control WebSocket Functions
// ============================================================================

// Generalized velocity command: always send linear (m/s) + angular (rad/s)
// Motor controllers handle their own protocol conversion
void sendVelCmd(double linear_mps, double angular_rads) {
    if (!control_connected) return;
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{\"type\":\"vel_cmd\",\"linear\":" << linear_mps 
        << ",\"angular\":" << angular_rads << "}";
    std::string msg = oss.str();
    
    try {
        control_client.send(control_hdl, msg, websocketpp::frame::opcode::text);
    } catch (const std::exception& e) {
        std::cerr << "[CONTROL] Send error: " << e.what() << std::endl;
    }
}

void stopRobot() {
    sendVelCmd(0.0, 0.0);
}

// Auto-reconnect for control connection
void reconnectControlIfNeeded() {
    if (control_connected) {
        last_control_reconnect = std::chrono::steady_clock::now();
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_control_reconnect).count();
    
    if (elapsed < RECONNECT_INTERVAL_SECONDS) {
        return;  // Not time yet
    }
    
    std::cout << "[CONTROL] Attempting to reconnect to motor controller..." << std::endl;
    last_control_reconnect = now;
    
    try {
        websocketpp::lib::error_code ec;
        WsClient::connection_ptr con = control_client.get_connection(g_control_url, ec);
        if (ec) {
            std::cerr << "[CONTROL] Connection error: " << ec.message() << std::endl;
        } else {
            control_client.connect(con);
        }
    } catch (const std::exception& e) {
        std::cerr << "[CONTROL] Reconnect error: " << e.what() << std::endl;
    }
}

// Auto-reconnect for sensor connections
void reconnectSensorsIfNeeded() {
    if (!g_robot_client) return;
    
    // Check if sensors need reconnection
    bool imu_ok = g_robot_client->is_imu_connected();
    bool gps_ok = g_robot_client->is_gps_connected();
    
    static auto last_sensor_reconnect = std::chrono::steady_clock::now();
    
    if (imu_ok && gps_ok) {
        last_sensor_reconnect = std::chrono::steady_clock::now();
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_sensor_reconnect).count();
    
    if (elapsed < RECONNECT_INTERVAL_SECONDS) {
        return;  // Not time yet
    }
    
    if (!imu_ok) {
        std::cout << "[SENSOR] Attempting to reconnect to IMU..." << std::endl;
    }
    if (!gps_ok) {
        std::cout << "[SENSOR] Attempting to reconnect to GPS..." << std::endl;
    }
    
    last_sensor_reconnect = now;
    
    // Attempt reconnection by restarting threads
    std::thread reconnect_thread([imu_ok, gps_ok]() {
        try {
            if (g_robot_client) {
                // Stop and restart sensor connections
                g_robot_client->reconnect_sensors();
            }
        } catch (const std::exception& e) {
            std::cerr << "[SENSOR] Reconnect error: " << e.what() << std::endl;
        }
    });
    reconnect_thread.detach();
}

void computePurePursuit(double& linear_out, double& angular_out) {
    static std::mutex pp_mutex;
    static double filtered_heading_error_rad = 0.0;
    static double prev_angular_cmd = 0.0;
    static auto prev_time = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> pp_lock(pp_mutex);

    std::lock_guard<std::mutex> lock(state_mutex);
    
    if (!current_state.valid || !waypoint_navigation_active) {
        linear_out = 0.0;
        angular_out = 0.0;
        return;
    }
    
    // Get target waypoint (either from mission or from direct command)
    Waypoint target;
    {
        std::lock_guard<std::mutex> nav_lock(navigation_target_mutex);
        target = current_navigation_target;
    }
    
    // Calculate distance and bearing to target
    double distance = haversineDistance(current_state.lat, current_state.lon,
                                        target.lat, target.lon);
    double target_bearing = bearing(current_state.lat, current_state.lon,
                                    target.lat, target.lon);
    
    // Heading error in degrees then radians
    double heading_error_deg = normalizeAngle(target_bearing - current_state.yaw);
    if (std::abs(heading_error_deg) < NAV_HEADING_DEADBAND_DEG) {
        heading_error_deg = 0.0;
    }
    const double heading_error_rad = heading_error_deg * M_PI / 180.0;

    // Low-pass filter heading error to suppress left-right jitter from yaw noise.
    filtered_heading_error_rad =
        (1.0 - NAV_HEADING_ERR_ALPHA) * filtered_heading_error_rad +
        NAV_HEADING_ERR_ALPHA * heading_error_rad;

    // Slightly reduce gain near target to avoid oscillation on final approach.
    double kp = g_nav_kp_heading;
    if (distance < NAV_LOOK_AHEAD_DIST) {
        kp *= 0.75;
    }

    double angular_cmd = kp * filtered_heading_error_rad;
    angular_cmd = std::clamp(angular_cmd, -NAV_MAX_ANGULAR, NAV_MAX_ANGULAR);

    // Slew-rate limit angular command to avoid rapid sign flips.
    auto now_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now_time - prev_time).count();
    prev_time = now_time;
    if (dt <= 0.0) {
        dt = 0.1;
    }
    const double max_delta = NAV_MAX_ANGULAR_ACCEL * dt;
    const double delta = std::clamp(angular_cmd - prev_angular_cmd, -max_delta, max_delta);
    angular_out = prev_angular_cmd + delta;
    prev_angular_cmd = angular_out;
    
    // Linear speed control based on distance (m/s)
    if (distance > NAV_LOOK_AHEAD_DIST) {
        linear_out = NAV_MAX_SPEED;
    } else {
        // Slow down as approaching waypoint
        linear_out = NAV_MIN_SPEED + (NAV_MAX_SPEED - NAV_MIN_SPEED) * 
                    (distance / NAV_LOOK_AHEAD_DIST);
    }
    
    // Reduce speed on sharp turns
    double turn_factor = 1.0 - std::abs(angular_out) * 0.5;
    linear_out *= turn_factor;
}

bool loadRobotClientConfig(const std::string& config_path,
                          std::string& robot_id,
                          std::string& robot_name,
                          std::string& robot_type,
                          std::string& imu_url,
                          std::string& gps_url,
                          std::string& server_url,
                          std::string& control_url,
                          double& waypoint_acceptance_radius,
                          double& nav_kp_heading,
                          double& yaw_offset_deg) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        if (config["robot"]) {
            const auto& robot = config["robot"];
            if (robot["id"]) robot_id = robot["id"].as<std::string>();
            if (robot["name"]) robot_name = robot["name"].as<std::string>();
            if (robot["type"]) robot_type = robot["type"].as<std::string>();
        }

        if (config["network"]) {
            const auto& network = config["network"];
            if (network["imu_url"]) imu_url = network["imu_url"].as<std::string>();
            if (network["gps_url"]) gps_url = network["gps_url"].as<std::string>();
            if (network["server_url"]) server_url = network["server_url"].as<std::string>();
            if (network["control_url"]) control_url = network["control_url"].as<std::string>();
        }

        if (config["navigation"]) {
            const auto& navigation = config["navigation"];
            if (navigation["waypoint_acceptance_radius"]) {
                waypoint_acceptance_radius = navigation["waypoint_acceptance_radius"].as<double>();
            }
            if (navigation["steering_kp"]) {
                nav_kp_heading = navigation["steering_kp"].as<double>();
            } else if (navigation["heading_kp"]) {
                nav_kp_heading = navigation["heading_kp"].as<double>();
            }
            if (navigation["yaw_offset_deg"]) {
                yaw_offset_deg = navigation["yaw_offset_deg"].as<double>();
            }
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load config file '" << config_path << "': " << e.what() << std::endl;
        return false;
    }
}

// ============================================================================
// Task Execution Functions
// ============================================================================

std::string taskTypeToString(TaskType type) {
    switch(type) {
        case TaskType::GOTO_WAYPOINT: return "GOTO_WAYPOINT";
        case TaskType::SPIRAL: return "SPIRAL";
        case TaskType::WAIT: return "WAIT";
        case TaskType::LOOP_START: return "LOOP_START";
        case TaskType::LOOP_END: return "LOOP_END";
        default: return "UNKNOWN";
    }
}

void executeGotoWaypoint(const Waypoint& waypoint, double task_speed_mps = -1.0) {
    std::cout << "[TASK] Executing GOTO_WAYPOINT #" << waypoint.index << std::endl;
    std::cout << "  Target: (" << std::fixed << std::setprecision(6)
              << waypoint.lat << ", " << waypoint.lon << ")" << std::endl;

    const double target_speed_mps = getTaskSpeedOrDefault(task_speed_mps, NAV_MAX_SPEED);
    const double min_speed_mps = std::min(NAV_MIN_SPEED, target_speed_mps);
    std::cout << "  Task speed: " << std::fixed << std::setprecision(3)
              << target_speed_mps << " m/s" << std::endl;

    {
        std::lock_guard<std::mutex> lock(navigation_target_mutex);
        current_navigation_target = waypoint;
    }

    waypoint_navigation_active = true;
    spiral_active = false;
    const uint64_t my_generation = waypoint_nav_generation.load();
    std::cout << "[NAV] Start generation=" << my_generation << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_cmd_time = start_time;

    while (waypoint_navigation_active && !mission_cancel_requested &&
           waypoint_nav_generation.load() == my_generation) {
        if (mission_pause_requested && !mission_paused) {
            mission_paused = true;
            stopRobot();
            updateMissionStatusMessage("Navigation paused");
            std::cout << "[NAV] Paused at waypoint approach" << std::endl;
        }

        while (mission_paused && !mission_cancel_requested &&
               waypoint_nav_generation.load() == my_generation) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (mission_cancel_requested || waypoint_nav_generation.load() != my_generation) {
            if (waypoint_nav_generation.load() != my_generation) {
                std::cout << "[NAV] Preempted generation=" << my_generation
                          << " by generation=" << waypoint_nav_generation.load() << std::endl;
            }
            if (waypoint_nav_generation.load() == my_generation) {
                stopRobot();
            }
            break;
        }

        mission_paused = false;
        mission_pause_requested = false;

        double distance = 0.0;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            if (current_state.valid) {
                distance = haversineDistance(
                    current_state.lat, current_state.lon,
                    waypoint.lat, waypoint.lon
                );

                if (distance < g_waypoint_acceptance_radius) {
                    std::cout << "[NAV] Waypoint reached! Distance: " << std::fixed
                              << std::setprecision(2) << distance << " m" << std::endl;
                    if (waypoint_nav_generation.load() == my_generation) {
                        stopRobot();
                    }
                    waypoint_navigation_active = false;
                    break;
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed_cmd = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_cmd_time).count();

        if (elapsed_cmd >= 100) {  // 10Hz control rate
            double linear_vel = 0.0, angular_vel = 0.0;
            computePurePursuit(linear_vel, angular_vel);
            linear_vel = std::clamp(linear_vel, min_speed_mps, target_speed_mps);

            if (control_connected) {
                sendVelCmd(linear_vel, angular_vel);

                static int print_counter = 0;
                if (++print_counter % 10 == 0) {
                    std::cout << "[NAV] Dist: " << std::fixed << std::setprecision(2)
                              << distance << "m | Linear: " << std::setprecision(3) << linear_vel
                              << " m/s | Angular: " << angular_vel << " rad/s" << std::endl;
                }
            }

            last_cmd_time = now;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - start_time).count();
        if (elapsed > 300) {
            std::cout << "[NAV] Waypoint timeout" << std::endl;
            if (waypoint_nav_generation.load() == my_generation) {
                stopRobot();
            }
            waypoint_navigation_active = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (waypoint_nav_generation.load() == my_generation) {
        stopRobot();
    }
    std::cout << "[NAV] Exit generation=" << my_generation
              << " active_generation=" << waypoint_nav_generation.load() << std::endl;
}

void executeSpiral(const Waypoint& center, double radius, double spacing, int loops, const std::string& direction, double task_speed_mps = -1.0) {
    const double target_speed_mps = getTaskSpeedOrDefault(task_speed_mps, SPIRAL_SPEED);
    const double spacing_m = std::max(0.0, spacing);
    const double start_radius_m = std::max(0.2, radius);

    std::cout << "[TASK] Executing SPIRAL from (" << std::fixed << std::setprecision(6)
              << center.lat << ", " << center.lon << ")" << std::endl;
    std::cout << "  Radius: " << start_radius_m << " m, Spacing: " << spacing_m << " m/turn, Loops: " << loops
              << ", Direction: " << direction << ", Speed: "
              << std::setprecision(3) << target_speed_mps << " m/s" << std::endl;

    spiral_active = true;
    waypoint_navigation_active = false;
    spiral_center = center;
    spiral_radius_start = start_radius_m;
    spiral_loops_target = loops;
    spiral_direction = direction;
    spiral_start_time = std::chrono::steady_clock::now();

    // Calculate total distance: sum of circumferences for each turn
    // Start at start_radius_m, each loop adds spacing_m to radius
    double total_distance = 0.0;
    for (int loop_idx = 0; loop_idx < loops; ++loop_idx) {
        double loop_radius = start_radius_m + spacing_m * loop_idx;
        total_distance += 2.0 * M_PI * std::max(0.2, loop_radius);
    }
    double estimated_time = total_distance / std::max(1e-6, target_speed_mps);

    auto last_cmd_time = spiral_start_time;

    while (spiral_active && !mission_cancel_requested) {
        if (mission_pause_requested && !mission_paused) {
            mission_paused = true;
            stopRobot();
            updateMissionStatusMessage("Spiral paused");
            std::cout << "[SPIRAL] Paused" << std::endl;
        }

        while (mission_paused && !mission_cancel_requested) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (mission_cancel_requested) {
            stopRobot();
            break;
        }

        mission_paused = false;
        mission_pause_requested = false;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - spiral_start_time).count();

        auto elapsed_cmd = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_cmd_time).count();

        if (elapsed_cmd >= 100) {  // 10Hz
            double linear_vel = target_speed_mps;
            // Compute current position in the spiral (0 to loops)
            const double progress = std::clamp(
                static_cast<double>(elapsed) / std::max(1.0, estimated_time),
                0.0, 1.0);
            const double turns_done = progress * loops;  // Range: 0 to loops
            const double cur_radius_m = std::max(0.2, start_radius_m + spacing_m * turns_done);
            const double omega_mag = linear_vel / cur_radius_m;
            const double signed_dir = (direction == "cw") ? -1.0 : 1.0;
            double angular_vel = signed_dir * std::clamp(omega_mag, 0.05, 1.2);

            if (control_connected) {
                sendVelCmd(linear_vel, angular_vel);

                static int print_counter = 0;
                if (++print_counter % 10 == 0) {
                    std::cout << "[SPIRAL] Time: " << elapsed << "/"
                              << (int)estimated_time << "s | Radius: " << std::setprecision(2) << cur_radius_m
                              << " m | Linear: " << std::setprecision(3) << linear_vel
                              << " m/s | Angular: " << angular_vel << " rad/s" << std::endl;
                }
            }

            last_cmd_time = now;
        }

        if (elapsed > estimated_time) {
            std::cout << "[SPIRAL] Complete" << std::endl;
            stopRobot();
            spiral_active = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    stopRobot();
}

void executeWait(double duration) {
    std::cout << "[TASK] Executing WAIT for " << std::fixed << std::setprecision(1) 
              << duration << " seconds" << std::endl;
    
    waypoint_navigation_active = false;
    spiral_active = false;
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (!mission_cancel_requested) {
        if (mission_pause_requested && !mission_paused) {
            mission_paused = true;
            updateMissionStatusMessage("Wait paused");
            std::cout << "[WAIT] Paused" << std::endl;
        }
        
        while (mission_paused && !mission_cancel_requested) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        if (mission_cancel_requested) break;
        
        mission_paused = false;
        mission_pause_requested = false;
        
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        
        if (elapsed > duration * 1000.0) {
            std::cout << "[WAIT] Complete" << std::endl;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ============================================================================
// Mission Execution
// ============================================================================

void executeMission(const json& mission_payload) {
    std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              MISSION EXECUTION STARTED (REAL ROBOT)         ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝" << std::endl;
    
    try {
        // Parse mission payload
        const json& data = mission_payload.contains("payload") ? mission_payload["payload"] : mission_payload;
        
        if (!data.contains("tasks") || !data.contains("waypoints")) {
            std::cerr << "[ERROR] Missing tasks or waypoints in mission" << std::endl;
            return;
        }
        
        // Load waypoints
        mission_waypoints.clear();
        int idx = 0;
        for (const auto& wp : data["waypoints"]) {
            Waypoint w;
            w.lat = wp["lat"].get<double>();
            w.lon = wp["lon"].get<double>();
            w.yaw = wp.value("yaw", 0.0);
            w.index = idx++;
            mission_waypoints.push_back(w);
        }
        
        // Load tasks
        mission_tasks.clear();
        for (const auto& task_json : data["tasks"]) {
            Task task;
            std::string type_str = task_json["type"].get<std::string>();
            
            if (type_str == "goto_waypoint") {
                task.type = TaskType::GOTO_WAYPOINT;
                // Accept both "waypoint" (from mission_planner) and "waypoint_index" (legacy)
                if (task_json.contains("waypoint_index")) {
                    task.waypoint_index = task_json["waypoint_index"].get<int>();
                } else if (task_json.contains("waypoint")) {
                    task.waypoint_index = task_json["waypoint"].get<int>();
                } else {
                    std::cerr << "[ERROR] goto_waypoint task missing waypoint index" << std::endl;
                    continue;  // Skip task with missing waypoint
                }
                task.speed_mps = task_json.value("speed", -1.0);
            } else if (type_str == "spiral") {
                task.type = TaskType::SPIRAL;
                if (task_json.contains("waypoint_index")) {
                    task.waypoint_index = task_json["waypoint_index"].get<int>();
                } else if (task_json.contains("waypoint")) {
                    task.waypoint_index = task_json["waypoint"].get<int>();
                }
                task.spiral_radius = task_json.value("radius", 5.0);
                task.spiral_spacing = task_json.value("spacing", 0.0);
                task.spiral_loops = task_json.value("loops", 3);
                task.spiral_direction = task_json.value("direction", "ccw");
                task.speed_mps = task_json.value("speed", -1.0);
            } else if (type_str == "wait") {
                task.type = TaskType::WAIT;
                task.wait_duration = task_json.value("duration", 1.0);
            } else if (type_str == "loop_start") {
                task.type = TaskType::LOOP_START;
            } else if (type_str == "loop_end") {
                task.type = TaskType::LOOP_END;
                task.loop_count = std::max(1, task_json.value("count", 1));
            } else {
                continue;  // Skip unknown tasks
            }
            
            task.description = task_json.value("description", "");
            mission_tasks.push_back(task);
        }
        
        // Update mission status
        {
            std::lock_guard<std::mutex> lock(mission_status_mutex);
            mission_status.active = true;
            mission_status.total_tasks = mission_tasks.size();
            mission_status.mission_name = data.value("name", "Unnamed Mission");
        }
        
        // Reset control flags
        mission_pause_requested = false;
        mission_cancel_requested = false;
        mission_paused = false;
        
        std::cout << "Waypoints: " << mission_waypoints.size() << std::endl;
        std::cout << "Tasks: " << mission_tasks.size() << std::endl;
        
        // Precompute loop pairs: each LOOP_END maps to the nearest unmatched LOOP_START.
        std::unordered_map<size_t, size_t> loop_end_to_start;
        std::vector<size_t> loop_start_stack;
        for (size_t idx = 0; idx < mission_tasks.size(); ++idx) {
            if (mission_tasks[idx].type == TaskType::LOOP_START) {
                loop_start_stack.push_back(idx);
            } else if (mission_tasks[idx].type == TaskType::LOOP_END) {
                if (loop_start_stack.empty()) {
                    std::cerr << "[MISSION] LOOP_END without matching LOOP_START at task " << idx + 1 << std::endl;
                    continue;
                }
                loop_end_to_start[idx] = loop_start_stack.back();
                loop_start_stack.pop_back();
            }
        }
        if (!loop_start_stack.empty()) {
            for (size_t start_idx : loop_start_stack) {
                std::cerr << "[MISSION] LOOP_START without matching LOOP_END at task " << start_idx + 1 << std::endl;
            }
        }

        // Execute tasks with loop support.
        std::unordered_map<size_t, int> loop_remaining;
        for (size_t i = 0; i < mission_tasks.size(); ) {
            if (mission_cancel_requested) {
                std::cout << "[MISSION] Cancelled by user" << std::endl;
                updateMissionStatusMessage("Mission cancelled by user");
                break;
            }
            
            const Task& task = mission_tasks[i];
            current_task_idx = i;
            
            {
                std::lock_guard<std::mutex> lock(mission_status_mutex);
                mission_status.current_task = i + 1;
                mission_status.current_task_type = taskTypeToString(task.type);
                mission_status.progress_percent = (100.0 * (i + 1)) / mission_tasks.size();
            }
            
            // Execute appropriate task type
            if (task.type == TaskType::GOTO_WAYPOINT) {
                if (task.waypoint_index >= 0 && task.waypoint_index < (int)mission_waypoints.size()) {
                    executeGotoWaypoint(mission_waypoints[task.waypoint_index], task.speed_mps);
                }
            } else if (task.type == TaskType::SPIRAL) {
                Waypoint center;
                bool have_center = false;

                if (task.waypoint_index >= 0 && task.waypoint_index < (int)mission_waypoints.size()) {
                    center = mission_waypoints[task.waypoint_index];
                    have_center = true;

                    // Ensure we actually reach the intended spiral center waypoint first.
                    double dist_to_center = 0.0;
                    bool have_state = false;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        if (current_state.valid) {
                            have_state = true;
                            dist_to_center = haversineDistance(
                                current_state.lat, current_state.lon,
                                center.lat, center.lon
                            );
                        }
                    }
                    if (have_state && dist_to_center > g_waypoint_acceptance_radius * 1.5) {
                        std::cout << "[SPIRAL] Moving to center waypoint first ("
                                  << std::fixed << std::setprecision(2) << dist_to_center << " m away)" << std::endl;
                        executeGotoWaypoint(center, task.speed_mps);
                    }
                } else {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    if (current_state.valid) {
                        center.lat = current_state.lat;
                        center.lon = current_state.lon;
                        center.yaw = current_state.yaw;
                        center.index = 0;
                        have_center = true;
                    }
                }

                if (have_center) {
                    executeSpiral(center, task.spiral_radius, task.spiral_spacing,
                                  task.spiral_loops, task.spiral_direction, task.speed_mps);
                } else {
                    std::cerr << "[SPIRAL] Skipped: no valid center/state available" << std::endl;
                }
            } else if (task.type == TaskType::WAIT) {
                executeWait(task.wait_duration);
            } else if (task.type == TaskType::LOOP_START) {
                std::cout << "[MISSION] LOOP_START at task " << (i + 1) << std::endl;
            } else if (task.type == TaskType::LOOP_END) {
                auto pair_it = loop_end_to_start.find(i);
                if (pair_it == loop_end_to_start.end()) {
                    std::cerr << "[MISSION] LOOP_END ignored (no matching LOOP_START) at task " << (i + 1) << std::endl;
                } else {
                    const size_t start_idx = pair_it->second;
                    auto rem_it = loop_remaining.find(i);
                    if (rem_it == loop_remaining.end()) {
                        // Run block once initially, then repeat (count - 1) times.
                        loop_remaining[i] = std::max(0, task.loop_count - 1);
                        rem_it = loop_remaining.find(i);
                    }

                    if (rem_it->second > 0) {
                        rem_it->second -= 1;
                        std::cout << "[MISSION] LOOP repeat -> jumping from task " << (i + 1)
                                  << " to task " << (start_idx + 2)
                                  << " (remaining=" << rem_it->second << ")" << std::endl;
                        i = start_idx + 1;
                        continue;
                    }

                    loop_remaining.erase(i);
                    std::cout << "[MISSION] LOOP complete at task " << (i + 1) << std::endl;
                }
            }

            ++i;
        }
        
        std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║                   MISSION COMPLETE                         ║" << std::endl;
        std::cout << "╚════════════════════════════════════════════════════════════╝\n" << std::endl;
        
        {
            std::lock_guard<std::mutex> lock(mission_status_mutex);
            mission_status.active = false;
            mission_status.progress_percent = 100.0;
            mission_status.status_message = "Mission completed";
        }
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Mission execution failed: " << e.what() << std::endl;
        updateMissionStatusMessage("Mission failed: " + std::string(e.what()));
    }
}

// ============================================================================
// Command Handler
// ============================================================================

void handleRobotCommand(const RobotCommandMessage& cmd) {
    std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              COMMAND RECEIVED FROM SERVER                  ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝" << std::endl;
    
    if (cmd.type == MessageType::COMMAND_EXECUTE_MISSION) {
        std::cout << "📋 Command: EXECUTE MISSION" << std::endl;
        
        try {
            // Execute mission in separate thread to avoid blocking
            std::thread mission_thread([](const json& payload) {
                executeMission(payload);
            }, cmd.payload);
            mission_thread.detach();
            
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to process mission: " << e.what() << std::endl;
        }
        
    } else if (cmd.type == MessageType::COMMAND_PAUSE_MISSION) {
        std::cout << "⏸️  Command: PAUSE/RESUME MISSION" << std::endl;
        
        if (mission_pause_requested || mission_paused) {
            mission_pause_requested = false;
            mission_paused = false;
            std::cout << "  └─ Mission resume" << std::endl;
            updateMissionStatusMessage("Mission resumed");
        } else {
            mission_pause_requested = true;
            stopRobot();  // Immediately stop when pausing
            std::cout << "  └─ Mission pause" << std::endl;
            updateMissionStatusMessage("Mission paused");
        }
        
    } else if (cmd.type == MessageType::COMMAND_CANCEL_MISSION) {
        std::cout << "⚠️  Command: CANCEL MISSION" << std::endl;
        mission_cancel_requested = true;
        waypoint_nav_generation.fetch_add(1);
        std::cout << "[NAV] Generation bumped to " << waypoint_nav_generation.load()
                  << " (cancel mission)" << std::endl;
        waypoint_navigation_active = false;
        move_active = false;  // Stop any MOVE command
        stopRobot();  // Immediately stop motors
        updateMissionStatusMessage("Mission cancelled");
        
    } else if (cmd.type == MessageType::COMMAND_EMERGENCY_STOP) {
        std::cout << "🚨 Command: EMERGENCY STOP" << std::endl;
        mission_cancel_requested = true;
        waypoint_nav_generation.fetch_add(1);
        std::cout << "[NAV] Generation bumped to " << waypoint_nav_generation.load()
                  << " (emergency stop)" << std::endl;
        waypoint_navigation_active = false;
        spiral_active = false;
        move_active = false;  // Stop any MOVE command
        stopRobot();  // Immediately stop motors
        updateMissionStatusMessage("Emergency stop");
        
    } else if (cmd.type == MessageType::COMMAND_MOVE) {
        std::cout << "🚗 Command: MOVE" << std::endl;
        
        try {
            // Extract movement parameters from payload
            if (!cmd.payload.contains("linear_velocity") || !cmd.payload.contains("angular_velocity")) {
                std::cerr << "[ERROR] COMMAND_MOVE missing velocity parameters" << std::endl;
                return;
            }
            
            double linear_vel = cmd.payload["linear_velocity"].get<double>();  // m/s
            double angular_vel = cmd.payload["angular_velocity"].get<double>(); // rad/s
            double duration = cmd.payload.value("duration", 5.0);               // seconds (default 5s)
            
            std::cout << "  Linear: " << linear_vel << " m/s | Angular: " << angular_vel 
                      << " rad/s | Duration: " << duration << "s" << std::endl;
            std::cout << "  Unified vel_cmd format: {type: vel_cmd, linear: m/s, angular: rad/s}" << std::endl;

            mission_cancel_requested = false;
            mission_pause_requested = false;
            mission_paused = false;
            waypoint_nav_generation.fetch_add(1);
            std::cout << "[NAV] Generation bumped to " << waypoint_nav_generation.load()
                      << " (new move command)" << std::endl;
            waypoint_navigation_active = false;
            spiral_active = false;
            move_active = false;
            
            // Execute movement in separate thread
            move_active = true;
            std::thread move_thread([linear_vel, angular_vel, duration]() {
                auto start_time = std::chrono::steady_clock::now();
                
                while (move_active) {
                    auto elapsed = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - start_time).count();
                    
                    if (elapsed >= duration) {
                        move_active = false;
                        stopRobot();
                        std::cout << "[MOVE] Movement complete" << std::endl;
                        break;
                    }
                    
                    // Check if cancelled
                    if (!move_active) {
                        stopRobot();
                        std::cout << "[MOVE] Movement cancelled" << std::endl;
                        break;
                    }
                    
                    // Send unified control format: m/s and rad/s
                    double linear_mps = linear_vel;       // m/s (keep as is)
                    double angular_rads = angular_vel;    // rad/s (keep as is)
                    
                    static int move_print_counter = 0;
                    if (++move_print_counter % 10 == 0) {
                        std::cout << "[MOVE] vel_cmd => linear=" << std::fixed << std::setprecision(3)
                                  << linear_mps << " m/s, angular=" << std::fixed << std::setprecision(3)
                                  << angular_rads << " rad/s" << std::endl;
                    }
                    sendVelCmd(linear_mps, angular_rads);
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz control
                }
            });
            move_thread.detach();
            
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to process MOVE command: " << e.what() << std::endl;
        }
        
    } else if (cmd.type == MessageType::COMMAND_WAYPOINT) {
        std::cout << "🎯 Command: NAVIGATE TO WAYPOINT" << std::endl;
        
        try {
            if (!cmd.payload.contains("latitude") || !cmd.payload.contains("longitude")) {
                std::cerr << "[ERROR] COMMAND_WAYPOINT missing coordinates" << std::endl;
                return;
            }
            
            double lat = cmd.payload["latitude"].get<double>();
            double lon = cmd.payload["longitude"].get<double>();
            
            std::cout << "  Target: (" << std::fixed << std::setprecision(6) 
                      << lat << ", " << lon << ")" << std::endl;

            mission_cancel_requested = false;
            mission_pause_requested = false;
            mission_paused = false;
            waypoint_nav_generation.fetch_add(1);
            std::cout << "[NAV] Generation bumped to " << waypoint_nav_generation.load()
                      << " (new waypoint command)" << std::endl;
            waypoint_navigation_active = false;
            spiral_active = false;
            move_active = false;
            
            // Execute waypoint navigation in separate thread
            std::thread waypoint_thread([lat, lon]() {
                Waypoint wp;
                wp.lat = lat;
                wp.lon = lon;
                wp.index = 0;
                executeGotoWaypoint(wp);
            });
            waypoint_thread.detach();
            
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to process WAYPOINT command: " << e.what() << std::endl;
        }
        
    } else if (cmd.type == MessageType::COMMAND_CANCEL_TASK) {
        std::cout << "⏹️  Command: CANCEL TASK" << std::endl;
        waypoint_nav_generation.fetch_add(1);
        std::cout << "[NAV] Generation bumped to " << waypoint_nav_generation.load()
                  << " (cancel task)" << std::endl;
        waypoint_navigation_active = false;
        spiral_active = false;
        move_active = false;  // Stop any MOVE command
        stopRobot();
        updateMissionStatusMessage("Task cancelled");
        
    } else {
        std::cout << "? Unknown command type: " << static_cast<int>(cmd.type) << std::endl;
    }
    
    std::cout << std::endl;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║             RaiNav Robot Client                            ║" << std::endl;
    std::cout << "║   Server-Controlled Navigation with Real Task Execution    ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;
    
    // Configuration
    std::string robot_id = "robot_001";
    std::string robot_name = "Scout_01";
    std::string robot_type = "ackermann";
    std::string imu_url = "ws://127.0.0.1:9201";
    std::string gps_url = "ws://127.0.0.1:9200";
    std::string server_url = "ws://127.0.0.1:8765";
    std::string control_url = "ws://127.0.0.1:9100";
    std::string config_path;
    
    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [OPTIONS]\n"
                      << "Options:\n"
                      << "  --robot-id ID          Robot identifier (default: robot_001)\n"
                      << "  --robot-name NAME      Robot name (default: Scout_01)\n"
                      << "  --robot-type TYPE      Robot type: ackermann|differential (default: ackermann)\n"
                      << "  --config FILE          YAML config file for robot parameters\n"
                      << "  --imu-url URL          IMU WebSocket URL (default: ws://127.0.0.1:9201)\n"
                      << "  --gps-url URL          GPS WebSocket URL (default: ws://127.0.0.1:9200)\n"
                      << "  --server-url URL       Server URL (default: ws://127.0.0.1:8765)\n"
                      << "  --control-url URL      Control URL (default: ws://127.0.0.1:9100)\n"
                      << "  -h, --help             Show this help message\n";
            return 0;
        } else if (arg == "--robot-id" && i + 1 < argc) {
            robot_id = argv[++i];
        } else if (arg == "--robot-name" && i + 1 < argc) {
            robot_name = argv[++i];
        } else if (arg == "--robot-type" && i + 1 < argc) {
            robot_type = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
            if (!loadRobotClientConfig(config_path,
                                       robot_id,
                                       robot_name,
                                       robot_type,
                                       imu_url,
                                       gps_url,
                                       server_url,
                                       control_url,
                                       g_waypoint_acceptance_radius,
                                       g_nav_kp_heading,
                                       g_yaw_offset_deg)) {
                return 1;
            }
        } else if (arg == "--imu-url" && i + 1 < argc) {
            imu_url = argv[++i];
        } else if (arg == "--gps-url" && i + 1 < argc) {
            gps_url = argv[++i];
        } else if (arg == "--server-url" && i + 1 < argc) {
            server_url = argv[++i];
        } else if (arg == "--control-url" && i + 1 < argc) {
            control_url = argv[++i];
        }
    }
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Store control URL globally
    g_control_url = control_url;
    
    // Initialize control WebSocket client
    try {
        control_client.clear_access_channels(websocketpp::log::alevel::all);
        control_client.clear_error_channels(websocketpp::log::elevel::all);
        control_client.init_asio();
        
        control_client.set_open_handler([](websocketpp::connection_hdl hdl) {
            control_hdl = hdl;
            control_connected = true;
            std::cout << "[CONTROL] Connected to motor controller" << std::endl;
        });
        
        control_client.set_close_handler([](websocketpp::connection_hdl) {
            control_connected = false;
            std::cout << "[CONTROL] Disconnected from motor controller" << std::endl;
        });
        
        control_client.set_fail_handler([](websocketpp::connection_hdl) {
            control_connected = false;
            std::cerr << "[CONTROL] Connection failed to motor controller" << std::endl;
        });
        
        // Connect to control server
        websocketpp::lib::error_code ec;
        WsClient::connection_ptr con = control_client.get_connection(control_url, ec);
        if (ec) {
            std::cerr << "[CONTROL] Connection error: " << ec.message() << std::endl;
        } else {
            control_client.connect(con);
            std::thread control_thread([&]() {
                control_client.run();
            });
            control_thread.detach();
            
            // Wait for connection
            for (int i = 0; i < 30; ++i) {
                if (control_connected) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[CONTROL] WebSocket error: " << e.what() << std::endl;
    }
    
    // Create robot client
    RobotClient robot(robot_id, robot_name, robot_type, imu_url, gps_url, server_url);
    g_robot_client = &robot;
    
    // Register callbacks
    robot.set_command_callback(handleRobotCommand);
    robot.set_mission_status_callback(getMissionStatusJson);
    
    // Print configuration
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Robot ID:   " << robot_id << std::endl;
    std::cout << "  Robot Name: " << robot_name << std::endl;
    std::cout << "  Robot Type: " << robot_type << std::endl;
    if (!config_path.empty()) {
        std::cout << "  Config File: " << config_path << std::endl;
    }
    std::cout << "  IMU Server: " << imu_url << std::endl;
    std::cout << "  GPS Server: " << gps_url << std::endl;
    std::cout << "  Control Server: " << server_url << std::endl;
    std::cout << "  Motor Control: " << control_url << " [" 
              << (control_connected ? "Connected" : "Disconnected") << "]" << std::endl;
    std::cout << "  Velocity Protocol: Unified (linear: m/s, angular: rad/s)" << std::endl;
    std::cout << "  Waypoint Radius: " << std::fixed << std::setprecision(1)
              << g_waypoint_acceptance_radius << " m" << std::endl << std::endl;
        std::cout << "  Steering Kp: " << std::fixed << std::setprecision(3)
                  << g_nav_kp_heading << std::endl << std::endl;
        std::cout << "  Yaw Offset: " << std::fixed << std::setprecision(2)
                  << g_yaw_offset_deg << " deg" << std::endl << std::endl;
    
    // Connect to servers
    if (!robot.connect()) {
        std::cerr << "[ERROR] Failed to connect to server" << std::endl;
        return 1;
    }
    
    std::cout << "[CLIENT] Connected! Running (press Ctrl+C to exit)...\n" << std::endl;
    
    // Initialize reconnection tracking
    last_control_reconnect = std::chrono::steady_clock::now();
    
    try {
        int frame_count = 0;
        auto last_heartbeat = std::chrono::steady_clock::now();
        
        while (true) {
            // Update sensor data
            IMUData imu = robot.get_imu_data();
            GPSData gps = robot.get_gps_data();
            
            // Update robot state
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                if (gps.valid) {
                    current_state.lat = gps.latitude;
                    current_state.lon = gps.longitude;
                    current_state.timestamp = gps.timestamp;
                    current_state.gps_valid = true;
                }
                
                if (imu.valid) {
                    // Calculate yaw from magnetometer
                    double mag_yaw = std::atan2(imu.my, imu.mx) * 180.0 / M_PI;
                    mag_yaw = normalizeYawDeg(mag_yaw + g_yaw_offset_deg);
                    
                    yaw_fusion.updateMagnetometer(mag_yaw);
                    current_state.yaw = yaw_fusion.getFusedYaw();
                    
                    if (std::isnan(current_state.yaw)) {
                        current_state.yaw = mag_yaw;
                    }
                    current_state.imu_valid = true;
                }
                
                current_state.valid = current_state.gps_valid && current_state.imu_valid;
            }
            
            frame_count++;
            
            // Check and reconnect if needed
            reconnectControlIfNeeded();
            reconnectSensorsIfNeeded();
            
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat).count() >= HEARTBEAT_INTERVAL_SECONDS) {
                auto info = robot.get_info();
                std::cout << "[STATUS] Frame " << std::setw(6) << frame_count
                          << " | Server: " << (info.server_ok ? "/" : "/x")
                          << " | Sensors: " << (info.sensors_ok ? "/" : "x")
                          << " | Control: " << (control_connected ? "/" : "x");
                
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    if (current_state.valid) {
                        std::cout << " | Pos: (" << std::fixed << std::setprecision(6)
                                  << current_state.lat << "," << current_state.lon << ")"
                                  << " | Yaw: " << std::fixed << std::setprecision(1)
                                  << current_state.yaw << "degree";
                    }
                }
                
                {
                    std::lock_guard<std::mutex> lock(mission_status_mutex);
                    if (mission_status.active) {
                        std::cout << " | Mission: " << mission_status.current_task 
                                  << "/" << mission_status.total_tasks
                                  << " (" << std::fixed << std::setprecision(0) 
                                  << mission_status.progress_percent << "%)";
                    }
                }
                
                std::cout << std::endl;
                last_heartbeat = now;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
