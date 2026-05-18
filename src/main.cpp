#include <Arduino.h>
#include "driver/twai.h"
#include <PS2X_lib.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include "DoubleAckermann.hpp"
#include "ControlActuationService.hpp"
#include "Config.h"
#include "ControlInputService.hpp"
#include "MotorInterfaces.hpp"
#include "RobotClient.hpp"
#include "SensorCommsService.hpp"
#include "SystemState.hpp"

HardwareSerial gps_uart(1);
WiFiClient ntrip_client;
// Use default SPI bus for BNO085 (compatible with S3)
Adafruit_BNO08x bno08x(BNO085_SPI_RST_PIN);
sh2_SensorValue_t bno_sensor_value;

GpsFix gps_fix;
ImuState imu_state;

String gps_line_buffer;
unsigned long last_ntrip_reconnect_ms = 0;
unsigned long last_ntrip_gga_send_ms = 0;
unsigned long last_sensor_log_ms = 0;
unsigned long ntrip_reconnect_backoff_ms = 3000;
uint8_t ntrip_fail_count = 0;
bool ntrip_connected = false;
bool wifi_connected = false;
bool imu_ready = false;
RobotClient robot_client;
Adafruit_NeoPixel status_leds(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

ControlTargets control_targets;
MissionContext mission_ctx;
LoopStats loop_stats;

// 1 Hz mission diagnostics state used by logSensorsAt1Hz().
bool mission_debug_valid = false;
bool mission_debug_coverage = false;
int mission_debug_task_index = -1;
int mission_debug_target_waypoint = -1;
int mission_debug_coverage_current = -1;
int mission_debug_coverage_end = -1;
float mission_debug_dist_m = 0.0f;
float mission_debug_heading_error_deg = 0.0f;
float mission_debug_speed_cmd_mps = 0.0f;

SensorCommsContext sensor_comms{
  gps_uart,
  ntrip_client,
  bno08x,
  bno_sensor_value,
  gps_fix,
  imu_state,
  gps_line_buffer,
  last_ntrip_reconnect_ms,
  last_ntrip_gga_send_ms,
  ntrip_reconnect_backoff_ms,
  ntrip_fail_count,
  ntrip_connected,
  wifi_connected,
  imu_ready,
};

static void controlTask(void* pvParameters);
static void commsTask(void* pvParameters);
static float estimateHeadingDeg();
static void handleUsbSerialControl();

static String usb_serial_line_buffer;
static bool usb_serial_line_overflow = false;
static constexpr size_t USB_SERIAL_MAX_LINE_LENGTH = 96;
static constexpr unsigned long USB_CMD_VEL_DEFAULT_TIMEOUT_MS = 250;
static constexpr unsigned long USB_CMD_VEL_MIN_TIMEOUT_MS = 50;
static constexpr unsigned long USB_CMD_VEL_MAX_TIMEOUT_MS = 5000;

static void logSensorsAt1Hz() {
#if !DEBUG_LOG_SENSOR_1HZ
  return;
#endif
  unsigned long now = millis();
  if (now - last_sensor_log_ms < 1000) return;
  last_sensor_log_ms = now;

  Serial.printf("[WIFI] %s", wifi_connected ? WiFi.localIP().toString().c_str() : "disconnected");
  Serial.print(" | [GPS] ");
  if (gps_fix.valid) {
    Serial.print("fix=");
    Serial.print(gps_fix.fix_quality);
    Serial.print(" sats=");
    Serial.print(gps_fix.satellites);
    Serial.print(" lat=");
    Serial.print(gps_fix.latitude, 7);
    Serial.print(" lon=");
    Serial.print(gps_fix.longitude, 7);
    Serial.print(" hdop=");
    Serial.print(gps_fix.hdop, 2);
  } else if (gps_fix.has_gga) {
    Serial.print("uart-ok no-fix q=");
    Serial.print(gps_fix.fix_quality);
    Serial.print(" sats=");
    Serial.print(gps_fix.satellites);
    Serial.print(" hdop=");
    Serial.print(gps_fix.hdop, 2);
  } else if ((now - gps_fix.last_rx_ms) < 2000 && gps_fix.last_rx_ms != 0) {
    Serial.print("uart-rx no-gga-yet");
  } else {
    Serial.print("no-uart-data");
  }

  Serial.print(" | [NTRIP] ");
#if NTRIP_ENABLE
  Serial.print(ntrip_connected ? "connected" : "disconnected");
#else
  Serial.print("disabled");
#endif

  Serial.print(" | [IMU] ");
  if (imu_state.valid) {
    Serial.print("ax=");
    Serial.print(imu_state.ax, 3);
    Serial.print(" ay=");
    Serial.print(imu_state.ay, 3);
    Serial.print(" az=");
    Serial.print(imu_state.az, 3);
    Serial.print(" gx=");
    Serial.print(imu_state.gx, 3);
    Serial.print(" gy=");
    Serial.print(imu_state.gy, 3);
    Serial.print(" gz=");
    Serial.print(imu_state.gz, 3);
  } else {
    Serial.print("no-data");
  }

  Serial.printf(" | [MISSION] %s", missionStateToString(mission_ctx.state));
  if (mission_ctx.name.length() > 0) {
    Serial.print("(");
    Serial.print(mission_ctx.name);
    Serial.print(")");
  }
  if (mission_debug_valid) {
    Serial.print(" wp=");
    Serial.print(mission_debug_target_waypoint);
    Serial.print(" dist=");
    Serial.print(mission_debug_dist_m, 2);
    Serial.print(" herr=");
    Serial.print(mission_debug_heading_error_deg, 1);
    Serial.print(" spd=");
    Serial.print(mission_debug_speed_cmd_mps, 2);
    if (mission_debug_coverage) {
      Serial.print(" cov=");
      Serial.print(mission_debug_coverage_current);
      Serial.print("->");
      Serial.print(mission_debug_coverage_end);
    }
  }

  Serial.print(" | [LOOP] max_dt_ms=");
  Serial.print(loop_stats.max_control_dt_ms);
  Serial.print(" overruns=");
  Serial.print(loop_stats.overrun_count);
  Serial.print(" cycles=");
  Serial.print(loop_stats.control_cycle_count);
  Serial.print(" | [COMMS] max_dt_ms=");
  Serial.print(loop_stats.max_comms_dt_ms);
  Serial.print(" overruns=");
  Serial.print(loop_stats.comms_overrun_count);
  Serial.print(" cycles=");
  Serial.print(loop_stats.comms_cycle_count);
  Serial.print(" | [CAN] ok=");
  Serial.print(loop_stats.can_runtime_tx_ok_count);
  Serial.print(" drop=");
  Serial.print(loop_stats.can_runtime_tx_drop_count);
  Serial.print(" | [SAFE] imu=");
  Serial.print(loop_stats.safety_stale_imu_count);
  Serial.print(" gps=");
  Serial.print(loop_stats.safety_stale_gps_count);
  Serial.print(" rc_to=");
  Serial.print(loop_stats.safety_remote_timeout_count);
  Serial.print(" mission_abort=");
  Serial.print(loop_stats.safety_mission_abort_count);

  loop_stats.max_control_dt_ms = 0;
  loop_stats.overrun_count = 0;
  loop_stats.control_cycle_count = 0;
  loop_stats.max_comms_dt_ms = 0;
  loop_stats.comms_overrun_count = 0;
  loop_stats.comms_cycle_count = 0;
  loop_stats.can_runtime_tx_ok_count = 0;
  loop_stats.can_runtime_tx_drop_count = 0;
  loop_stats.safety_stale_imu_count = 0;
  loop_stats.safety_stale_gps_count = 0;
  loop_stats.safety_remote_timeout_count = 0;
  loop_stats.safety_mission_abort_count = 0;
  Serial.println();
}

// ==========================================
// HARDWARE PINS
// ==========================================




// ==========================================
// 3. GLOBAL INSTANCES
// ==========================================
#if DRIVE_MOTOR_TYPE == DRIVE_MOTOR_TYPE_ZLAC
ZlacDriveMotorPair drive_motors_impl(DRIVE_FRONT_MOTOR_ID, DRIVE_REAR_MOTOR_ID, &loop_stats);
#else
#error "Unsupported DRIVE_MOTOR_TYPE"
#endif

#if STEER_MOTOR_TYPE == STEER_MOTOR_TYPE_ODRIVE
OdriveSteeringMotorPair steering_motors_impl(STEER_LEFT_MOTOR_ID, STEER_RIGHT_MOTOR_ID, &loop_stats);
#elif STEER_MOTOR_TYPE == STEER_MOTOR_TYPE_GIM8108
Gim8108SteeringMotorPair steering_motors_impl(
  STEER_LEFT_MOTOR_ID,
  STEER_RIGHT_MOTOR_ID,
  &loop_stats,
  STEER_GIM_LEFT_SIGN,
  STEER_GIM_RIGHT_SIGN);
#else
#error "Unsupported STEER_MOTOR_TYPE"
#endif

IDriveMotorPair& drive_motors = drive_motors_impl;
ISteeringMotorPair& steering_motors = steering_motors_impl;

PS2X ps2x;
int ps2_error = 0;

const int MAX_RPM = 150;
bool e_stop_active = false;

enum class RobotMode : uint8_t {
  Normal = 0,
  ZeroTurn = 1,
};

static void applyModeInit(RobotMode mode);

static constexpr float WHEELBASE_M = 0.36f;
static constexpr float TRACK_WIDTH_M = 0.36f;
static constexpr float MAX_LINEAR_MPS = 1.50f;
static constexpr float MAX_STEER_DEG = 30.0f;
static constexpr float WHEEL_DIAMETER_M = 0.20f;
static constexpr unsigned long IMU_STALE_TIMEOUT_MS = 1500;
static constexpr unsigned long GPS_STALE_TIMEOUT_MS = 3000;
static constexpr unsigned long MISSION_SENSOR_GRACE_MS = 2000;
static constexpr unsigned long COMMS_OVERRUN_MS = 20;
static constexpr float MISSION_WAYPOINT_REACHED_M = 1.5f;
static constexpr float MISSION_COVERAGE_WAYPOINT_REACHED_M = 0.5f;
static constexpr float MISSION_SLOWDOWN_RADIUS_M = 1.0f;
static constexpr float MISSION_HEADING_SLOWDOWN_CRAWL_MPS = 0.12f;
static constexpr size_t MISSION_MAX_WAYPOINTS = 200;
static constexpr size_t MISSION_MAX_TASKS = 512;

enum class MissionTaskType : uint8_t {
  GotoWaypoint = 0,
  Wait,
  LoopStart,
  LoopEnd,
  CoveragePath,
  Spiral,
};

struct MissionWaypointPlan {
  double lat{0.0};
  double lon{0.0};
  double alt{0.0};
};

struct MissionTaskPlan {
  MissionTaskType type{MissionTaskType::GotoWaypoint};
  int waypoint_index{-1};
  int waypoint_index_end{-1};
  float speed_mps{0.2f};
  unsigned long wait_ms{0};
  int loop_count{0};
  int loop_start_index{-1};
  // Spiral task params (only used when type == Spiral)
  float spiral_radius_start{1.0f};
  float spiral_radius_end{1.3f};
  float spiral_loops{1.0f};
  int8_t spiral_direction{1};  // +1 = ccw, -1 = cw
};

MissionWaypointPlan mission_waypoints[MISSION_MAX_WAYPOINTS];
MissionTaskPlan mission_tasks[MISSION_MAX_TASKS];
size_t mission_waypoint_count = 0;
size_t mission_task_count = 0;
size_t mission_task_cursor = 0;
unsigned long mission_task_started_ms = 0;
size_t mission_active_task_cursor = static_cast<size_t>(-1);
int mission_loop_remaining[MISSION_MAX_TASKS];
int mission_coverage_current_index = -1;

static constexpr int8_t MOTOR_DI_LEFT = 1;
static constexpr int8_t MOTOR_DI_RIGHT = -1;

RobotMode robot_mode = RobotMode::Normal;
DoubleAckermannSteering robot(WHEELBASE_M, TRACK_WIDTH_M);

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float normalizeAngleDeg(float angle_deg) {
  while (angle_deg > 180.0f) angle_deg -= 360.0f;
  while (angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}

static float distanceMeters(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
  static constexpr double kEarthRadiusM = 6371000.0;
  static constexpr double kDeg2Rad = M_PI / 180.0;
  double lat1 = lat1_deg * kDeg2Rad;
  double lat2 = lat2_deg * kDeg2Rad;
  double dlat = lat2 - lat1;
  double dlon = (lon2_deg - lon1_deg) * kDeg2Rad;

  double a = sin(dlat * 0.5) * sin(dlat * 0.5) +
             cos(lat1) * cos(lat2) * sin(dlon * 0.5) * sin(dlon * 0.5);
  double c = 2.0 * atan2(sqrt(a), sqrt(max(0.0, 1.0 - a)));
  return static_cast<float>(kEarthRadiusM * c);
}

static float bearingDeg(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
  static constexpr double kDeg2Rad = M_PI / 180.0;
  static constexpr double kRad2Deg = 180.0 / M_PI;
  double lat1 = lat1_deg * kDeg2Rad;
  double lat2 = lat2_deg * kDeg2Rad;
  double dlon = (lon2_deg - lon1_deg) * kDeg2Rad;

  double y = sin(dlon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
  double brng = atan2(y, x) * kRad2Deg;
  if (brng < 0.0) brng += 360.0;
  return static_cast<float>(brng);
}

static void clearMissionPlan() {
  mission_waypoint_count = 0;
  mission_task_count = 0;
  mission_task_cursor = 0;
  mission_task_started_ms = 0;
  mission_active_task_cursor = static_cast<size_t>(-1);
  mission_coverage_current_index = -1;
  for (size_t i = 0; i < MISSION_MAX_TASKS; ++i) {
    mission_loop_remaining[i] = -1;
  }
}

static bool parseMissionPlan(const String& mission_json) {
  clearMissionPlan();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, mission_json);
  if (err) {
#if DEBUG_LOG_MISSION
    Serial.printf("[MISSION] JSON parse failed: %s\n", err.c_str());
#endif
    return false;
  }

  JsonArrayConst waypoints = doc["waypoints"].as<JsonArrayConst>();
  for (JsonVariantConst wp_var : waypoints) {
    if (mission_waypoint_count >= MISSION_MAX_WAYPOINTS) break;
    JsonObjectConst wp = wp_var.as<JsonObjectConst>();
    double lat = wp["lat"].isNull() ? (wp["latitude"] | 0.0) : (wp["lat"] | 0.0);
    double lon = wp["lon"].isNull() ? (wp["longitude"] | 0.0) : (wp["lon"] | 0.0);
    double alt = wp["alt"].isNull() ? (wp["altitude"] | 0.0) : (wp["alt"] | 0.0);
    mission_waypoints[mission_waypoint_count++] = {lat, lon, alt};
  }

  JsonArrayConst tasks = doc["tasks"].as<JsonArrayConst>();
  JsonArrayConst coverage_saved_paths = doc["coverage"]["saved_paths"].as<JsonArrayConst>();
  int loop_start_stack[MISSION_MAX_TASKS];
  size_t loop_start_stack_size = 0;

  auto appendCoverageWaypoints = [&](JsonArrayConst coverage_points, int& wp_start, int& wp_end) -> bool {
    if (coverage_points.isNull() || coverage_points.size() == 0) {
      return false;
    }

    if (mission_waypoint_count >= MISSION_MAX_WAYPOINTS) {
#if DEBUG_LOG_MISSION
      Serial.println("[MISSION] Cannot append coverage waypoints: waypoint buffer full");
#endif
      return false;
    }

    wp_start = static_cast<int>(mission_waypoint_count);
    for (JsonVariantConst cov_var : coverage_points) {
      if (mission_waypoint_count >= MISSION_MAX_WAYPOINTS) {
#if DEBUG_LOG_MISSION
        Serial.println("[MISSION] Coverage waypoints exceeded waypoint limit");
#endif
        return false;
      }

      JsonObjectConst cov_wp = cov_var.as<JsonObjectConst>();
      if (cov_wp.isNull()) continue;

      double lat = cov_wp["lat"].isNull() ? (cov_wp["latitude"] | 0.0) : (cov_wp["lat"] | 0.0);
      double lon = cov_wp["lon"].isNull() ? (cov_wp["longitude"] | 0.0) : (cov_wp["lon"] | 0.0);
      double alt = cov_wp["alt"].isNull() ? (cov_wp["altitude"] | 0.0) : (cov_wp["alt"] | 0.0);
      mission_waypoints[mission_waypoint_count++] = {lat, lon, alt};
    }

    wp_end = static_cast<int>(mission_waypoint_count) - 1;
    return wp_end >= wp_start;
  };

  for (JsonVariantConst task_var : tasks) {
    if (mission_task_count >= MISSION_MAX_TASKS) break;
    JsonObjectConst task = task_var.as<JsonObjectConst>();
    String type = task["type"] | "";

    if (type == "goto_waypoint") {
      int wp_idx = task["waypoint"] | -1;
      float speed = task["speed"] | 0.2f;
      mission_tasks[mission_task_count++] = {MissionTaskType::GotoWaypoint, wp_idx, -1, clampf(speed, 0.05f, MAX_LINEAR_MPS), 0, 0, -1};
    } else if (type == "spiral") {
      float radius   = task["radius"]  | 1.0f;
      float spacing  = task["spacing"] | 0.3f;
      float loops_f  = task["loops"]   | 1.0f;
      float speed    = task["speed"]   | 0.2f;
      String dir_str = task["direction"] | "ccw";
      MissionTaskPlan spiral_task{};
      spiral_task.type               = MissionTaskType::Spiral;
      spiral_task.speed_mps          = clampf(speed, 0.05f, MAX_LINEAR_MPS);
      spiral_task.spiral_radius_start = max(0.1f, radius);
      spiral_task.spiral_radius_end   = max(0.1f, radius + spacing * loops_f);
      spiral_task.spiral_loops        = max(0.1f, loops_f);
      spiral_task.spiral_direction    = (dir_str == "cw") ? -1 : 1;
      mission_tasks[mission_task_count++] = spiral_task;
    } else if (type == "wait") {
      float duration_s = task["duration"] | 1.0f;
      unsigned long wait_ms = static_cast<unsigned long>(max(0.1f, duration_s) * 1000.0f);
      mission_tasks[mission_task_count++] = {MissionTaskType::Wait, -1, -1, 0.0f, wait_ms, 0, -1};
    } else if (type == "coverage_path") {
      int wp_start = task["start_waypoint"].isNull() ? (task["waypoint_start"] | -1) : (task["start_waypoint"] | -1);
      int wp_end = task["end_waypoint"].isNull() ? (task["waypoint_end"] | -1) : (task["end_waypoint"] | -1);

      if (wp_start < 0 || wp_end < 0) {
        JsonArrayConst coverage_points = task["coverage_waypoints"].as<JsonArrayConst>();

        if ((coverage_points.isNull() || coverage_points.size() == 0) && !coverage_saved_paths.isNull()) {
          int path_index = task["coverage_path_index"] | -1;
          if (path_index >= 0 && static_cast<size_t>(path_index) < coverage_saved_paths.size()) {
            JsonObjectConst saved_path_obj = coverage_saved_paths[path_index].as<JsonObjectConst>();
            if (!saved_path_obj.isNull()) {
              coverage_points = saved_path_obj["coverage_waypoints"].as<JsonArrayConst>();
            }
          }
        }

        int resolved_start = -1;
        int resolved_end = -1;
        if (appendCoverageWaypoints(coverage_points, resolved_start, resolved_end)) {
          wp_start = resolved_start;
          wp_end = resolved_end;
        }
      }

      float speed = task["speed"] | 0.2f;
      mission_tasks[mission_task_count++] = {MissionTaskType::CoveragePath, wp_start, wp_end, clampf(speed, 0.05f, MAX_LINEAR_MPS), 0, 0, -1};
    } else if (type == "loop_start") {
      mission_tasks[mission_task_count++] = {MissionTaskType::LoopStart, -1, -1, 0.0f, 0, 0, -1};
      if (loop_start_stack_size < MISSION_MAX_TASKS) {
        loop_start_stack[loop_start_stack_size++] = static_cast<int>(mission_task_count - 1);
      }
    } else if (type == "loop_end") {
      int count = task["count"] | (task["loop_count"] | 1);
      int start_idx = -1;
      if (loop_start_stack_size > 0) {
        start_idx = loop_start_stack[--loop_start_stack_size];
      }
      mission_tasks[mission_task_count++] = {MissionTaskType::LoopEnd, -1, -1, 0.0f, 0, max(1, count), start_idx};
    }
  }

  if (mission_task_count == 0 && mission_waypoint_count > 0) {
    for (size_t i = 0; i < mission_waypoint_count && mission_task_count < MISSION_MAX_TASKS; ++i) {
      mission_tasks[mission_task_count++] = {MissionTaskType::GotoWaypoint, static_cast<int>(i), -1, 0.2f, 0, 0, -1};
    }
  }

  if (mission_waypoint_count == 0 || mission_task_count == 0) {
#if DEBUG_LOG_MISSION
    Serial.println("[MISSION] No valid waypoints/tasks in mission payload");
#endif
    return false;
  }

  mission_task_cursor = 0;
  mission_task_started_ms = millis();
#if DEBUG_LOG_MISSION
  Serial.printf("[MISSION] Loaded plan: waypoints=%u tasks=%u\n",
                static_cast<unsigned>(mission_waypoint_count),
                static_cast<unsigned>(mission_task_count));
#endif
  return true;
}

static void loadSingleWaypointMission(float lat, float lon, float alt) {
  clearMissionPlan();
  mission_waypoint_count = 1;
  mission_waypoints[0] = {lat, lon, alt};
  mission_task_count = 1;
  mission_tasks[0] = {MissionTaskType::GotoWaypoint, 0, -1, 0.2f, 0, 0, -1};
  mission_task_cursor = 0;
  mission_task_started_ms = millis();
}

static void setMissionState(MissionState new_state) {
  if (mission_ctx.state == new_state) return;
  mission_ctx.state = new_state;
  mission_ctx.state_since_ms = millis();
#if DEBUG_LOG_MISSION
  Serial.printf("[MISSION] state=%s\n", missionStateToString(mission_ctx.state));
#endif
}

static void resetMissionCommandTargets() {
  control_targets.linear_mps = 0.0f;
  control_targets.angle_deg = 0.0f;
  control_targets.remote_move_expire_ms = 0;
}

static void updateMissionExecutor() {
  static unsigned long invalid_sensor_since_ms = 0;
  if (!mission_ctx.active) {
    mission_debug_valid = false;
    invalid_sensor_since_ms = 0;
    if (mission_ctx.state != MissionState::Idle && mission_ctx.state != MissionState::Cancelled && mission_ctx.state != MissionState::Completed && mission_ctx.state != MissionState::Failed && mission_ctx.state != MissionState::EStop) {
      setMissionState(MissionState::Idle);
    }
    return;
  }

  unsigned long now = millis();
  if (mission_ctx.state == MissionState::Queued) {
    mission_ctx.started_ms = now;
    mission_ctx.phase = 0;
    mission_ctx.waypoint_index = (mission_task_count > 0) ? mission_tasks[0].waypoint_index : -1;
    mission_task_cursor = 0;
    mission_task_started_ms = now;
    setMissionState(MissionState::Running);
  }

  if (mission_ctx.state == MissionState::Paused) {
    mission_debug_valid = false;
    resetMissionCommandTargets();
    return;
  }

  if (mission_ctx.state != MissionState::Running) {
    mission_debug_valid = false;
    return;
  }

  // Spiral tasks are purely time-based and do not require GPS or IMU
  const bool task_needs_gps = !(mission_task_cursor < mission_task_count &&
                                mission_tasks[mission_task_cursor].type == MissionTaskType::Spiral);

  if (task_needs_gps && (!gps_fix.valid || !imu_state.valid)) {
    mission_debug_valid = false;
    if (invalid_sensor_since_ms == 0) {
      invalid_sensor_since_ms = now;
    } else if ((now - invalid_sensor_since_ms) > MISSION_SENSOR_GRACE_MS) {
      ++loop_stats.safety_mission_abort_count;
      mission_ctx.active = false;
      mission_ctx.phase = 0;
      mission_ctx.waypoint_index = -1;
      resetMissionCommandTargets();
      setMissionState(MissionState::Failed);
#if DEBUG_LOG_SAFETY
      Serial.println("[SAFETY] Mission aborted due to invalid GPS/IMU");
#endif
    }
    return;
  }
  invalid_sensor_since_ms = 0;

  if (mission_task_cursor >= mission_task_count) {
    mission_debug_valid = false;
    mission_ctx.active = false;
    mission_ctx.phase = 0;
    mission_ctx.waypoint_index = -1;
    resetMissionCommandTargets();
    setMissionState(MissionState::Completed);
#if DEBUG_LOG_MISSION
    Serial.println("[MISSION] Mission plan completed");
#endif
    return;
  }

  MissionTaskPlan& task = mission_tasks[mission_task_cursor];
  if (mission_active_task_cursor != mission_task_cursor) {
    mission_active_task_cursor = mission_task_cursor;
    mission_coverage_current_index = -1;
  }
  mission_ctx.phase = static_cast<uint8_t>(mission_task_cursor);
  mission_ctx.waypoint_index = task.waypoint_index;

  if (task.type == MissionTaskType::LoopStart) {
    mission_debug_valid = false;
    ++mission_task_cursor;
    mission_task_started_ms = now;
    mission_active_task_cursor = static_cast<size_t>(-1);
    return;
  }

  if (task.type == MissionTaskType::LoopEnd) {
    mission_debug_valid = false;
    int& loop_remaining = mission_loop_remaining[mission_task_cursor];
    if (loop_remaining < 0) {
      loop_remaining = max(1, task.loop_count);
    }

    if (task.loop_start_index >= 0 && static_cast<size_t>(task.loop_start_index) < mission_task_count && loop_remaining > 1) {
      --loop_remaining;
      mission_task_cursor = static_cast<size_t>(task.loop_start_index + 1);
      mission_task_started_ms = now;
      mission_active_task_cursor = static_cast<size_t>(-1);
      return;
    }

    loop_remaining = -1;
    ++mission_task_cursor;
    mission_task_started_ms = now;
    mission_active_task_cursor = static_cast<size_t>(-1);
    return;
  }

  if (task.type == MissionTaskType::Wait) {
    mission_debug_valid = false;
    control_targets.linear_mps = 0.0f;
    control_targets.angle_deg = 0.0f;
    control_targets.remote_move_expire_ms = now + 150;
    if ((now - mission_task_started_ms) >= task.wait_ms) {
      ++mission_task_cursor;
      mission_task_started_ms = now;
    }
    return;
  }

  if (task.type == MissionTaskType::Spiral) {
    mission_debug_valid = false;
    float elapsed_s  = static_cast<float>(now - mission_task_started_ms) * 0.001f;
    float avg_radius  = (task.spiral_radius_start + task.spiral_radius_end) * 0.5f;
    float total_angle = task.spiral_loops * 2.0f * static_cast<float>(M_PI);
    float duration_s  = (total_angle * avg_radius) / task.speed_mps;

    if (elapsed_s >= duration_s) {
      control_targets.linear_mps = 0.0f;
      control_targets.angle_deg  = 0.0f;
      control_targets.remote_move_expire_ms = now + 150;
#if DEBUG_LOG_MISSION
      Serial.printf("[MISSION] Spiral done (task=%u, dur=%.1fs)\n",
                    static_cast<unsigned>(mission_task_cursor), duration_s);
#endif
      ++mission_task_cursor;
      mission_task_started_ms = now;
      mission_active_task_cursor = static_cast<size_t>(-1);
      return;
    }

    float t = clampf(elapsed_s / duration_s, 0.0f, 1.0f);
    float current_radius = task.spiral_radius_start +
                           (task.spiral_radius_end - task.spiral_radius_start) * t;
    current_radius = max(0.05f, current_radius);

    float steering_rad = atanf((WHEELBASE_M * 0.5f) / current_radius);
    float steering_deg = steering_rad * (180.0f / static_cast<float>(M_PI))
                         * static_cast<float>(task.spiral_direction);

    control_targets.linear_mps = task.speed_mps;
    control_targets.angle_deg  = clampf(steering_deg, -MAX_STEER_DEG, MAX_STEER_DEG);
    control_targets.remote_move_expire_ms = now + 150;
    return;
  }

  int target_waypoint_index = task.waypoint_index;
  if (task.type == MissionTaskType::CoveragePath) {
    if (task.waypoint_index < 0 || task.waypoint_index_end < 0) {
      ++mission_task_cursor;
      mission_task_started_ms = now;
      mission_active_task_cursor = static_cast<size_t>(-1);
      return;
    }

    if (mission_coverage_current_index < 0) {
      mission_coverage_current_index = task.waypoint_index;
    }
    target_waypoint_index = mission_coverage_current_index;
    mission_ctx.waypoint_index = target_waypoint_index;
  }

  if (target_waypoint_index < 0 || static_cast<size_t>(target_waypoint_index) >= mission_waypoint_count) {
#if DEBUG_LOG_MISSION
    Serial.printf("[MISSION] Invalid waypoint index in task=%u (idx=%d), skipping\n",
                  static_cast<unsigned>(mission_task_cursor),
                  target_waypoint_index);
#endif
    ++mission_task_cursor;
    mission_task_started_ms = now;
    mission_active_task_cursor = static_cast<size_t>(-1);
    return;
  }

  MissionTaskPlan* active_waypoint_task = &task;
  MissionWaypointPlan* wp = &mission_waypoints[target_waypoint_index];
  float dist_m = 0.0f;

  while (true) {
    const float waypoint_reached_threshold_m =
        (active_waypoint_task->type == MissionTaskType::CoveragePath)
            ? MISSION_COVERAGE_WAYPOINT_REACHED_M
            : MISSION_WAYPOINT_REACHED_M;

    wp = &mission_waypoints[target_waypoint_index];
    dist_m = distanceMeters(gps_fix.latitude, gps_fix.longitude, wp->lat, wp->lon);
    if (dist_m > waypoint_reached_threshold_m) {
      break;
    }

#if DEBUG_LOG_MISSION
    Serial.printf("[MISSION] Reached waypoint %d (task=%u, dist=%.2fm)\n",
                  target_waypoint_index,
                  static_cast<unsigned>(mission_task_cursor),
                  dist_m);
#endif

    if (active_waypoint_task->type == MissionTaskType::CoveragePath) {
      if (mission_coverage_current_index == active_waypoint_task->waypoint_index_end) {
        ++mission_task_cursor;
        mission_coverage_current_index = -1;
        mission_active_task_cursor = static_cast<size_t>(-1);
        mission_task_started_ms = now;
        return;
      }

      int step = (active_waypoint_task->waypoint_index_end >= active_waypoint_task->waypoint_index) ? 1 : -1;
      mission_coverage_current_index += step;
      target_waypoint_index = mission_coverage_current_index;
      mission_ctx.waypoint_index = target_waypoint_index;
      continue;
    }

    ++mission_task_cursor;
    mission_active_task_cursor = static_cast<size_t>(-1);
    mission_task_started_ms = now;

    if (mission_task_cursor >= mission_task_count) {
      return;
    }

    MissionTaskPlan* next_task = &mission_tasks[mission_task_cursor];
    if (next_task->type == MissionTaskType::LoopStart ||
        next_task->type == MissionTaskType::LoopEnd ||
        next_task->type == MissionTaskType::Wait ||
        next_task->type == MissionTaskType::Spiral) {
      return;
    }

    active_waypoint_task = next_task;
    target_waypoint_index = active_waypoint_task->waypoint_index;
    if (active_waypoint_task->type == MissionTaskType::CoveragePath) {
      if (active_waypoint_task->waypoint_index < 0 || active_waypoint_task->waypoint_index_end < 0) {
        return;
      }
      if (mission_coverage_current_index < 0) {
        mission_coverage_current_index = active_waypoint_task->waypoint_index;
      }
      target_waypoint_index = mission_coverage_current_index;
    }

    if (target_waypoint_index < 0 || static_cast<size_t>(target_waypoint_index) >= mission_waypoint_count) {
      return;
    }

    mission_ctx.phase = static_cast<uint8_t>(mission_task_cursor);
    mission_ctx.waypoint_index = target_waypoint_index;
  }

  float desired_heading_deg = bearingDeg(gps_fix.latitude, gps_fix.longitude, wp->lat, wp->lon);
  float current_heading_deg = estimateHeadingDeg();
  float heading_error_deg = -1 * normalizeAngleDeg(desired_heading_deg - current_heading_deg);

  float speed_cmd = clampf(active_waypoint_task->speed_mps, 0.05f, MAX_LINEAR_MPS);
  if (fabsf(heading_error_deg) > 70.0f) {

    // will implement zero turn next //
    speed_cmd *= 0.5f;
    const float crawl_speed_floor = min(MISSION_HEADING_SLOWDOWN_CRAWL_MPS, clampf(active_waypoint_task->speed_mps, 0.05f, MAX_LINEAR_MPS));
    speed_cmd = max(speed_cmd, crawl_speed_floor);
  }

  mission_debug_valid = true;
  mission_debug_coverage = (active_waypoint_task->type == MissionTaskType::CoveragePath);
  mission_debug_task_index = static_cast<int>(mission_task_cursor);
  mission_debug_target_waypoint = target_waypoint_index;
  mission_debug_coverage_current = mission_coverage_current_index;
  mission_debug_coverage_end = active_waypoint_task->waypoint_index_end;
  mission_debug_dist_m = dist_m;
  mission_debug_heading_error_deg = heading_error_deg;
  mission_debug_speed_cmd_mps = speed_cmd;

  control_targets.linear_mps = speed_cmd;
  control_targets.angle_deg = clampf(heading_error_deg * 0.7f, -MAX_STEER_DEG, MAX_STEER_DEG);
  control_targets.remote_move_expire_ms = now + 150;
}

static void updateSafetyGuards() {
  unsigned long now = millis();

  if (imu_ready && imu_state.valid && imu_state.last_update_ms != 0 && (now - imu_state.last_update_ms) > IMU_STALE_TIMEOUT_MS) {
    imu_state.valid = false;
    ++loop_stats.safety_stale_imu_count;
#if DEBUG_LOG_SAFETY
    Serial.println("[SAFETY] IMU stale, marking invalid");
#endif
  }

  if (gps_fix.has_gga && gps_fix.last_update_ms != 0 && (now - gps_fix.last_update_ms) > GPS_STALE_TIMEOUT_MS) {
    gps_fix.valid = false;
    gps_fix.has_gga = false;
    ++loop_stats.safety_stale_gps_count;
#if DEBUG_LOG_SAFETY
    Serial.println("[SAFETY] GPS stale, marking invalid");
#endif
  }

  if (control_targets.remote_move_expire_ms != 0 && now > control_targets.remote_move_expire_ms) {
    if (control_targets.linear_mps != 0.0f || control_targets.angle_deg != 0.0f) {
      ++loop_stats.safety_remote_timeout_count;
      control_targets.linear_mps = 0.0f;
      control_targets.angle_deg = 0.0f;
    }
  }
}

static void updateStatusLeds() {
  static unsigned long last_led_update_ms = 0;
  unsigned long now = millis();
  if (now - last_led_update_ms < 50) return;
  last_led_update_ms = now;

  static constexpr uint32_t LED_OFF = 0x000000;
  static constexpr uint32_t LED_GREEN = 0x00FF00;
  static constexpr uint32_t LED_RED = 0xFF0000;
  static constexpr uint32_t LED_YELLOW = 0xFFD000;
  static constexpr uint32_t LED_BLUE = 0x0060FF;
  static constexpr uint32_t LED_CYAN = 0x00FFFF;
  static constexpr uint32_t LED_MAGENTA = 0xFF00FF;
  static constexpr uint32_t LED_WHITE = 0xFFFFFF;
  static constexpr uint32_t LED_ORANGE = 0xFF8000;

  bool gps_ok = gps_fix.valid;
  bool imu_ok = imu_ready && imu_state.valid;
  bool can_ok = (loop_stats.can_runtime_tx_drop_count < 5);
  bool system_ok = !e_stop_active && gps_ok && imu_ok && can_ok;

  // LED0: overall health (requested policy: green=ok, red=not ok)
  uint32_t overall_color = system_ok ? LED_GREEN : LED_RED;
  if (e_stop_active) {
    overall_color = ((now / 200) % 2 == 0) ? LED_RED : LED_OFF;
  }
  status_leds.setPixelColor(0, overall_color);

  // LED1: WiFi connectivity
  status_leds.setPixelColor(1, wifi_connected ? LED_GREEN : LED_RED);

  // LED2: GPS fix quality mapping
  if (gps_fix.fix_quality == 4) {
    status_leds.setPixelColor(2, LED_GREEN);
  } else if (gps_fix.fix_quality == 5) {
    status_leds.setPixelColor(2, LED_BLUE);
  } else {
    status_leds.setPixelColor(2, LED_RED);
  }

  // LED3: IMU health
  status_leds.setPixelColor(3, imu_ok ? LED_GREEN : LED_RED);

  // LED4: correction stream / NTRIP
#if NTRIP_ENABLE
  status_leds.setPixelColor(4, ntrip_connected ? LED_GREEN : LED_RED);
#else
  status_leds.setPixelColor(4, LED_YELLOW);
#endif

  // LED5: control source/status
  if (e_stop_active) {
    status_leds.setPixelColor(5, LED_RED);
  } else if (control_targets.remote_move_expire_ms > now) {
    status_leds.setPixelColor(5, LED_CYAN);      // remote command active
  } else if (ps2_error == 0) {
    status_leds.setPixelColor(5, LED_MAGENTA);   // manual (PS2) available
  } else {
    status_leds.setPixelColor(5, LED_YELLOW);    // no active command source
  }

  // LED6: mission state
  uint32_t mission_color = LED_OFF;
  switch (mission_ctx.state) {
    case MissionState::Idle: mission_color = LED_BLUE; break;
    case MissionState::Queued: mission_color = LED_YELLOW; break;
    case MissionState::Running: mission_color = LED_GREEN; break;
    case MissionState::Paused: mission_color = LED_ORANGE; break;
    case MissionState::Completed: mission_color = LED_CYAN; break;
    case MissionState::Failed: mission_color = LED_RED; break;
    case MissionState::Cancelled: mission_color = LED_WHITE; break;
    case MissionState::EStop: mission_color = ((now / 200) % 2 == 0) ? LED_RED : LED_OFF; break;
  }
  status_leds.setPixelColor(6, mission_color);

  // LED7: control loop timing health
  status_leds.setPixelColor(7, loop_stats.overrun_count < 3 ? LED_GREEN : LED_RED);
  status_leds.show();
}

static float convertAngularToSteeringDeg(float linear_mps, float angular_rads, float wheelbase_m, float max_angle_deg) {
  if (fabsf(linear_mps) < 0.001f) return 0.0f;
  float tan_angle = (-angular_rads * wheelbase_m) / linear_mps;
  float angle_deg = DoubleAckermannSteering::rad2deg(atanf(tan_angle));
  return clampf(angle_deg, -max_angle_deg, max_angle_deg);
}

static bool missionBlocksManualVelocityControl() {
  return mission_ctx.active &&
         (mission_ctx.state == MissionState::Queued ||
          mission_ctx.state == MissionState::Running ||
          mission_ctx.state == MissionState::Paused);
}

static void applyUsbVelocityCommand(float linear_mps, float angular_rads, unsigned long timeout_ms) {
  const float linear_limited = clampf(linear_mps, -MAX_LINEAR_MPS, MAX_LINEAR_MPS);
  
  // In zero-turn mode, angular_rads controls rotation intensity (scaled to steering angle range)
  // In normal mode, convert angular_rads to steering angle using Ackermann geometry
  float angle_deg;
  if (robot_mode == RobotMode::ZeroTurn) {
    // Scale angular velocity to [-MAX_STEER_DEG, MAX_STEER_DEG] range
    // Assuming typical angular_rads in [-π, π], scale to steering deg range
    float angular_deg = (angular_rads * 180.0f / M_PI);
    angle_deg = clampf(angular_deg * 0.5f, -MAX_STEER_DEG, MAX_STEER_DEG);
  } else {
    // Normal Ackermann mode
    angle_deg = convertAngularToSteeringDeg(linear_limited, angular_rads, WHEELBASE_M, MAX_STEER_DEG);
  }
  
  // timeout_ms == 0 means infinite timeout (command persists until new command)
  unsigned long remote_expire = 0;
  if (timeout_ms > 0) {
    const unsigned long timeout_limited = max(USB_CMD_VEL_MIN_TIMEOUT_MS, min(timeout_ms, USB_CMD_VEL_MAX_TIMEOUT_MS));
    remote_expire = millis() + timeout_limited;
  }

  control_targets.linear_mps = linear_limited;
  control_targets.angle_deg = angle_deg;
  control_targets.remote_move_expire_ms = remote_expire;

  const char* mode_str = (robot_mode == RobotMode::ZeroTurn) ? "ZERO_TURN" : "NORMAL";
  if (timeout_ms == 0) {
    Serial.printf("[USB_SERIAL] CMD_VEL accepted (mode=%s) linear=%.3f m/s angular=%.3f rad/s angle=%.2f deg timeout=INFINITE\n",
                  mode_str,
                  linear_limited,
                  angular_rads,
                  angle_deg);
  } else {
    const unsigned long timeout_limited = max(USB_CMD_VEL_MIN_TIMEOUT_MS, min(timeout_ms, USB_CMD_VEL_MAX_TIMEOUT_MS));
    Serial.printf("[USB_SERIAL] CMD_VEL accepted (mode=%s) linear=%.3f m/s angular=%.3f rad/s angle=%.2f deg timeout=%lu ms\n",
                  mode_str,
                  linear_limited,
                  angular_rads,
                  angle_deg,
                  timeout_limited);
  }
}

static void handleUsbSerialLine(const String& raw_line) {
  String line = raw_line;
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

  if (upper == "HELP" || upper == "?") {
    Serial.println("[USB_SERIAL] Commands: CMD_VEL <linear_mps> <angular_rad_s> [timeout_ms], STOP, MODE <NORMAL|ZERO_TURN>, STATUS?");
    return;
  }

  if (upper == "STOP") {
    control_targets.linear_mps = 0.0f;
    control_targets.angle_deg = 0.0f;
    control_targets.remote_move_expire_ms = millis() + 150;
    Serial.println("[USB_SERIAL] STOP accepted");
    return;
  }

  if (upper.startsWith("MODE")) {
    String args = line.substring(4);
    args.trim();
    args.toUpperCase();

    if (args == "NORMAL" || args == "0") {
      if (robot_mode != RobotMode::Normal) {
        robot_mode = RobotMode::Normal;
        applyModeInit(robot_mode);
        Serial.println("[USB_SERIAL] MODE set to NORMAL");
      } else {
        Serial.println("[USB_SERIAL] MODE already NORMAL");
      }
      return;
    }

    if (args == "ZERO_TURN" || args == "ZERO" || args == "1") {
      if (robot_mode != RobotMode::ZeroTurn) {
        robot_mode = RobotMode::ZeroTurn;
        applyModeInit(robot_mode);
        Serial.println("[USB_SERIAL] MODE set to ZERO_TURN");
      } else {
        Serial.println("[USB_SERIAL] MODE already ZERO_TURN");
      }
      return;
    }

    Serial.println("[USB_SERIAL] Invalid MODE. Usage: MODE <NORMAL|ZERO_TURN>");
    return;
  }

  if (upper == "STATUS?") {
    const unsigned long now = millis();
    const bool remote_active = (control_targets.remote_move_expire_ms == 0) || (control_targets.remote_move_expire_ms > now);
    const unsigned long remaining_ms = (control_targets.remote_move_expire_ms == 0) ? 0 : remote_active ? (control_targets.remote_move_expire_ms - now) : 0;
    const char* mode_str = (robot_mode == RobotMode::Normal) ? "NORMAL" : "ZERO_TURN";
    Serial.printf("[USB_SERIAL] status mode=%s e_stop=%d mission=%s remote_active=%d linear=%.3f angle=%.2f timeout_left_ms=%lu\n",
                  mode_str,
                  e_stop_active ? 1 : 0,
                  missionStateToString(mission_ctx.state),
                  remote_active ? 1 : 0,
                  control_targets.linear_mps,
                  control_targets.angle_deg,
                  remaining_ms);
    return;
  }

  if (upper.startsWith("CMD_VEL")) {
    String args = line.substring(7);
    args.trim();

    float linear_mps = 0.0f;
    float angular_rads = 0.0f;
    unsigned long timeout_ms = USB_CMD_VEL_DEFAULT_TIMEOUT_MS;
    int parsed = sscanf(args.c_str(), "%f %f %lu", &linear_mps, &angular_rads, &timeout_ms);
    if (parsed < 2) {
      Serial.println("[USB_SERIAL] Invalid CMD_VEL. Usage: CMD_VEL <linear_mps> <angular_rad_s> [timeout_ms]");
      return;
    }

    if (e_stop_active) {
      Serial.println("[USB_SERIAL] CMD_VEL rejected: e-stop active");
      return;
    }

    if (missionBlocksManualVelocityControl()) {
      Serial.println("[USB_SERIAL] CMD_VEL rejected: mission control active");
      return;
    }

    // change direction form this //
    applyUsbVelocityCommand(linear_mps, -angular_rads, timeout_ms);
    return;
  }

  Serial.printf("[USB_SERIAL] Unknown command: %s\n", line.c_str());
}

static void handleUsbSerialControl() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') continue;

    if (c == '\n') {
      if (usb_serial_line_overflow) {
        Serial.println("[USB_SERIAL] Ignored line: too long");
      } else if (usb_serial_line_buffer.length() > 0) {
        handleUsbSerialLine(usb_serial_line_buffer);
      }
      usb_serial_line_buffer = "";
      usb_serial_line_overflow = false;
      continue;
    }

    if (usb_serial_line_overflow) continue;

    if (usb_serial_line_buffer.length() >= USB_SERIAL_MAX_LINE_LENGTH) {
      usb_serial_line_overflow = true;
      continue;
    }

    usb_serial_line_buffer += c;
  }
}

static float estimateHeadingDeg() {
  if (!imu_state.valid) return 0.0f;
  float qw = imu_state.qw;
  float qx = imu_state.qx;
  float qy = imu_state.qy;
  float qz = imu_state.qz;
  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  //float yaw_deg = DoubleAckermannSteering::rad2deg(atan2f(siny_cosp, cosy_cosp));
  float yaw_deg = DoubleAckermannSteering::rad2deg(atan2f(cosy_cosp, siny_cosp));
  if (yaw_deg < 0.0f) yaw_deg += 360.0f;
  return yaw_deg;
}

static void handleRobotClientCommand() {
#if !CONTROL_SERVER_ENABLE
  return;
#endif
  if (!robot_client.hasPendingCommand()) return;

  RobotCommand command = robot_client.consumeCommand();
  switch (command.type) {
    case RobotCommandType::Move:
      control_targets.linear_mps = command.linear_velocity;
      control_targets.angle_deg = convertAngularToSteeringDeg(command.linear_velocity, command.angular_velocity, WHEELBASE_M, MAX_STEER_DEG);
      control_targets.remote_move_expire_ms = millis() + static_cast<unsigned long>(command.duration * 1000.0f);
      robot_client.sendCommandAck(command, "accepted");
#if DEBUG_LOG_ROBOT_CLIENT
      Serial.println("[ROBOT_CLIENT] Move command accepted");
#endif
      break;
    case RobotCommandType::EmergencyStop:
      e_stop_active = true;
      drive_motors.emergencyStop();
      steering_motors.emergencyStop();
      mission_ctx.active = false;
      setMissionState(MissionState::EStop);
      robot_client.sendCommandAck(command, "accepted");
    #if DEBUG_LOG_ROBOT_CLIENT
      Serial.println("[ROBOT_CLIENT] Emergency stop accepted");
    #endif
      break;
    case RobotCommandType::ExecuteMission:
      if (parseMissionPlan(command.mission_json)) {
        mission_ctx.active = true;
        mission_ctx.name = command.mission_name;
        mission_ctx.phase = 0;
        mission_ctx.waypoint_index = -1;
        mission_ctx.started_ms = millis();
        setMissionState(MissionState::Queued);
        robot_client.sendCommandAck(command, "accepted", "mission plan loaded");
#if DEBUG_LOG_ROBOT_CLIENT
        Serial.println("[ROBOT_CLIENT] Mission plan received and queued");
#endif
      } else {
        mission_ctx.active = false;
        mission_ctx.phase = 0;
        mission_ctx.waypoint_index = -1;
        resetMissionCommandTargets();
        setMissionState(MissionState::Failed);
        robot_client.sendCommandAck(command, "rejected", "invalid mission payload");
#if DEBUG_LOG_ROBOT_CLIENT
        Serial.println("[ROBOT_CLIENT] Mission rejected: invalid payload");
#endif
      }
      break;
    case RobotCommandType::PauseMission:
      if (mission_ctx.active) {
        if (mission_ctx.state == MissionState::Paused) {
          setMissionState(MissionState::Running);
        } else {
          setMissionState(MissionState::Paused);
          resetMissionCommandTargets();
        }
        robot_client.sendCommandAck(command, "accepted");
      } else {
        robot_client.sendCommandAck(command, "rejected", "no active mission");
      }
      break;
    case RobotCommandType::CancelMission:
      mission_ctx.active = false;
      mission_ctx.name = "";
      mission_ctx.phase = 0;
      mission_ctx.waypoint_index = -1;
      clearMissionPlan();
      resetMissionCommandTargets();
      setMissionState(MissionState::Cancelled);
      robot_client.sendCommandAck(command, "accepted");
      break;
    case RobotCommandType::Waypoint:
      loadSingleWaypointMission(command.latitude, command.longitude, command.altitude);
      mission_ctx.active = true;
      mission_ctx.name = "single_waypoint";
      mission_ctx.phase = 0;
      mission_ctx.waypoint_index = 0;
      mission_ctx.started_ms = millis();
      setMissionState(MissionState::Queued);
      robot_client.sendCommandAck(command, "accepted", "single waypoint mission queued");
      break;
    case RobotCommandType::CancelTask:
      mission_ctx.active = false;
      mission_ctx.name = "";
      mission_ctx.phase = 0;
      mission_ctx.waypoint_index = -1;
      clearMissionPlan();
      resetMissionCommandTargets();
      setMissionState(MissionState::Cancelled);
      robot_client.sendCommandAck(command, "accepted");
      break;
    case RobotCommandType::None:
      break;
  }
}

static void publishRobotTelemetry() {
#if !CONTROL_SERVER_ENABLE
  return;
#endif

  static unsigned long last_telemetry_publish_ms = 0;
  const unsigned long now = millis();
  const unsigned long telemetry_interval_ms = max(20UL, static_cast<unsigned long>(ROBOT_TELEMETRY_INTERVAL_MS));
  if (last_telemetry_publish_ms != 0 && (now - last_telemetry_publish_ms) < telemetry_interval_ms) {
    return;
  }
  last_telemetry_publish_ms = now;

  RobotTelemetry telemetry;
  telemetry.battery_level = 100.0f;
  telemetry.speed = control_targets.linear_mps;
  telemetry.heading = estimateHeadingDeg();

  telemetry.gps_valid = gps_fix.valid;
  telemetry.gps_latitude = gps_fix.latitude;
  telemetry.gps_longitude = gps_fix.longitude;
  telemetry.gps_altitude = gps_fix.altitude;
  telemetry.gps_hdop = gps_fix.hdop;
  telemetry.gps_fix_quality = gps_fix.fix_quality;
  telemetry.gps_satellites = gps_fix.satellites;

  telemetry.imu_valid = imu_state.valid;
  telemetry.imu_ax = imu_state.ax;
  telemetry.imu_ay = imu_state.ay;
  telemetry.imu_az = imu_state.az;
  telemetry.imu_gx = imu_state.gx;
  telemetry.imu_gy = imu_state.gy;
  telemetry.imu_gz = imu_state.gz;
  telemetry.imu_qx = imu_state.qx;
  telemetry.imu_qy = imu_state.qy;
  telemetry.imu_qz = imu_state.qz;
  telemetry.imu_qw = imu_state.qw;

  telemetry.mission_active = mission_ctx.active && (mission_ctx.state == MissionState::Queued || mission_ctx.state == MissionState::Running || mission_ctx.state == MissionState::Paused);
  telemetry.mission_name = mission_ctx.name;
  telemetry.mission_state = missionStateToString(mission_ctx.state);
  telemetry.mission_waypoint_index = mission_ctx.waypoint_index;
  telemetry.mission_waypoint_total = static_cast<int>(mission_waypoint_count);
  telemetry.mission_coverage_active = false;
  telemetry.mission_coverage_progress_index = -1;
  telemetry.mission_coverage_total = 0;

  if (mission_ctx.active && mission_task_cursor < mission_task_count) {
    const MissionTaskPlan& active_task = mission_tasks[mission_task_cursor];
    if (active_task.type == MissionTaskType::CoveragePath &&
        active_task.waypoint_index >= 0 &&
        active_task.waypoint_index_end >= 0) {
      const int start_idx = active_task.waypoint_index;
      const int end_idx = active_task.waypoint_index_end;
      const int step = (end_idx >= start_idx) ? 1 : -1;
      const int current_idx = (mission_coverage_current_index >= 0) ? mission_coverage_current_index : start_idx;
      const int progress_idx = (step > 0) ? (current_idx - start_idx) : (start_idx - current_idx);
      const int total = (step > 0) ? (end_idx - start_idx + 1) : (start_idx - end_idx + 1);

      telemetry.mission_coverage_active = true;
      telemetry.mission_coverage_progress_index = max(0, min(progress_idx, max(0, total - 1)));
      telemetry.mission_coverage_total = max(0, total);
    }
  }

  robot_client.sendTelemetry(telemetry);
}

static void applyModeInit(RobotMode mode) {
  if (mode == RobotMode::Normal) {
    Serial.println("Enter Mode 0 : normal mode");
    steering_motors.setSteeringTurns(0.0f, 0.0f);
  } else {
    Serial.println("Enter Mode 1 : zero turn mode");
    steering_motors.setSteeringTurns(0.5f, -0.5f);
  }
  delay(200);
}

// ==========================================
// 4. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  unsigned long serial_wait_start = millis();
  while (!Serial && (millis() - serial_wait_start) < 2000) {
    delay(10);
  }
  delay(1000);
  Serial.println("[USB_SERIAL] Ready. Commands: CMD_VEL <linear_mps> <angular_rad_s> [timeout_ms], STOP, MODE <NORMAL|ZERO_TURN>, STATUS?");
  Serial.println("[USB_SERIAL]   timeout_ms=0 for infinite timeout (command persists until new command)");

  // connectWifiIfNeeded();
  gps_uart.begin(GPS_UART_BAUD, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
  Serial.print("[GPS] UART started on RX=");
  Serial.print(GPS_UART_RX_PIN);
  Serial.print(" TX=");
  Serial.println(GPS_UART_TX_PIN);
  SensorComms::initBno085(sensor_comms);
#if CONTROL_SERVER_ENABLE
  robot_client.begin();
#else
  Serial.println("[ROBOT_CLIENT] Disabled by config");
#endif

  // --- Initialize WS2812B Status LEDs ---
  status_leds.begin();
  status_leds.setBrightness(NEOPIXEL_BRIGHTNESS);
  status_leds.fill(0x001000);  // Dim everything initially
  status_leds.show();
  Serial.printf("[LEDS] WS2812B initialized: %d LEDs on GPIO %d, brightness=%d\n", NEOPIXEL_COUNT, NEOPIXEL_PIN, NEOPIXEL_BRIGHTNESS);

  // --- Initialize PS2 Controller ---
  Serial.println("Connecting to PS2 Controller...");
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
  if(ps2_error == 0) {
    Serial.println("PS2 Controller found and configured!");
  } else {
    Serial.println("Error configuring PS2 Controller. Check wiring!");
  }

  // --- Initialize TWAI (CAN) ---
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  // --- Initialize Motors ---
  Serial.println("Configuring drive motor backend...");
  drive_motors.begin();
  drive_motors.enable();

  Serial.println("Configuring steering motor backend...");
  steering_motors.begin();

  applyModeInit(robot_mode);

  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    8192,
    nullptr,
    3,
    nullptr,
    1);

  xTaskCreatePinnedToCore(
    commsTask,
    "CommsTask",
    8192,
    nullptr,
    2,
    nullptr,
    0);

  Serial.println("[RTOS] ControlTask pinned to Core 1");
  Serial.println("[RTOS] CommsTask pinned to Core 0");
  
  Serial.println("Setup Complete! Ready to Drive.");
}

// ==========================================
// 5. MAIN LOOP
// ==========================================
static void controlTask(void* /*pvParameters*/) {
  unsigned long last_control_time = millis();
  for (;;) {
    if (millis() - last_control_time >= 20) {
      unsigned long now = millis();
      unsigned long dt_ms = now - last_control_time;
      last_control_time = now;
      ++loop_stats.control_cycle_count;
      if (dt_ms > loop_stats.max_control_dt_ms) loop_stats.max_control_dt_ms = dt_ms;
      if (dt_ms > 25) ++loop_stats.overrun_count;

      if (ps2_error != 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }

      ps2x.read_gamepad();

      if (ps2x.ButtonPressed(PSB_CIRCLE) && !ps2x.Button(PSB_START)) {
        e_stop_active = !e_stop_active;

        if (e_stop_active) {
          Serial.println("!!! E-STOP ACTIVATED !!!");
          drive_motors.emergencyStop();
          steering_motors.emergencyStop();
        } else {
          Serial.println(">>> E-STOP RELEASED <<<");
          drive_motors.enable();
          steering_motors.enable();
        }
      }

      if (!e_stop_active) {
        bool control_enable = ps2x.Button(PSB_R1);
        bool mode_combo_enable = ps2x.Button(PSB_R1) && ps2x.Button(PSB_R2);
        if (mode_combo_enable && ps2x.Button(PSB_TRIANGLE) && robot_mode != RobotMode::ZeroTurn) {
          robot_mode = RobotMode::ZeroTurn;
          applyModeInit(robot_mode);
        }
        if (mode_combo_enable && ps2x.Button(PSB_CROSS) && robot_mode != RobotMode::Normal) {
          robot_mode = RobotMode::Normal;
          applyModeInit(robot_mode);
        }

        // Remote command active if timeout is 0 (infinite) or expire time hasn't passed
        bool remote_active = (control_targets.remote_move_expire_ms == 0) ? true : (control_targets.remote_move_expire_ms > millis());
        int ly = ps2x.Analog(PSS_RY);
        int lx = ps2x.Analog(PSS_LX);
        DriveCommand cmd = ControlInput::resolveDriveCommand(
          control_targets,
          control_enable,
          remote_active,
          ly,
          lx,
          MAX_LINEAR_MPS,
          MAX_STEER_DEG);

        ControlActuation::applyDriveOutputs(
          robot_mode == RobotMode::ZeroTurn,
          cmd.linear_mps,
          cmd.angle_deg,
          WHEELBASE_M,
          WHEEL_DIAMETER_M,
          MAX_RPM,
          MOTOR_DI_LEFT,
          MOTOR_DI_RIGHT,
          robot,
          drive_motors,
          steering_motors);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void commsTask(void* /*pvParameters*/) {
  unsigned long last_comms_time = millis();
  for (;;) {
    unsigned long now = millis();
    unsigned long dt_ms = now - last_comms_time;
    last_comms_time = now;
    ++loop_stats.comms_cycle_count;
    if (dt_ms > loop_stats.max_comms_dt_ms) loop_stats.max_comms_dt_ms = dt_ms;
    if (dt_ms > COMMS_OVERRUN_MS) ++loop_stats.comms_overrun_count;

    SensorComms::connectWifiIfNeeded(sensor_comms);
#if CONTROL_SERVER_ENABLE
    robot_client.update();
#endif
    handleUsbSerialControl();
  #if NTRIP_ENABLE
    SensorComms::connectNtripIfNeeded(sensor_comms);
  #endif
    SensorComms::updateGpsAndNtrip(sensor_comms);
    SensorComms::updateImu(sensor_comms);
    updateSafetyGuards();
    handleRobotClientCommand();
    updateMissionExecutor();
    publishRobotTelemetry();
    logSensorsAt1Hz();
    updateStatusLeds();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void loop() {
  // Runtime is handled by pinned FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}