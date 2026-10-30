#include <Arduino.h>
#include "driver/twai.h"
#include <PS2X_lib.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include "DoubleAckermann.hpp"
#include "Ds3502.hpp"
#include "LinearActuator.hpp"
#include "Pcf8574.hpp"
#include "ControlActuationService.hpp"
#include "RobotConfig.h"
#include "ControlInputService.hpp"
#include "MotorInterfaces.hpp"
#include "NetManager.hpp"
#include "RobotClient.hpp"
#include "SensorCommsService.hpp"
#include "SystemState.hpp"

// Add-on I/O modules default to off when the config doesn't mention them.
#ifndef DS3502_ENABLE
#define DS3502_ENABLE 0
#endif
#ifndef PCF8574_ENABLE
#define PCF8574_ENABLE 0
#endif
#ifndef ACTUATOR_ENABLE
#define ACTUATOR_ENABLE 0
#endif
#ifndef LIMIT_SWITCH_ESTOP_ENABLE
#define LIMIT_SWITCH_ESTOP_ENABLE 0
#endif
#ifndef MOTOR_STATUS_POLL_MS
#define MOTOR_STATUS_POLL_MS 1000UL
#endif

HardwareSerial gps_uart(1);
NetManager net_manager;
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
#if DS3502_ENABLE
static constexpr uint8_t kDs3502Addresses[DS3502_COUNT] = DS3502_ADDRESSES;
Ds3502 pots[DS3502_COUNT];
#endif
#if PCF8574_ENABLE
Pcf8574 input_expander;
// Bit n = digital input n+1 active (after DIGITAL_INPUT_ACTIVE_LOW is applied).
static uint8_t digital_inputs = 0;
static uint8_t digital_inputs_raw = 0xFF;
static bool digital_inputs_ok = false;
#endif
#if ACTUATOR_ENABLE
LinearActuator actuator({
  ACT_INA_PIN, ACT_INB_PIN, ACT_PWM_PIN, ACT_POS_PIN, ACT_LEDC_CHANNEL, ACT_PWM_FREQ_HZ,
  ACT_ADC_RETRACTED, ACT_ADC_EXTENDED, ACT_ADC_FAULT_MARGIN, ACT_DIRECTION_SIGN,
  ACT_DEADBAND_PCT, ACT_SLOW_ZONE_PCT, ACT_MIN_DUTY_PCT, ACT_MAX_DUTY_PCT,
  ACT_STALL_TIMEOUT_MS, ACT_STALL_MIN_CHANGE_PCT,
});
#endif

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
  nullptr,       // Client* ntrip_client — set each tick by net_manager in commsTask
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
static void serialCmdTask(void* pvParameters);
static void navTask(void* pvParameters);
static void motorStatusTask(void* pvParameters);

// Guards GPS fix, IMU state and mission state shared between NavTask (owner),
// CommsTask (server commands, telemetry, logging) and SerialCmdTask.
// Never hold it across a modem/network call — that is what NavTask exists to avoid.
static SemaphoreHandle_t nav_mutex = nullptr;
struct NavLock {
  NavLock() { if (nav_mutex) xSemaphoreTake(nav_mutex, portMAX_DELAY); }
  ~NavLock() { if (nav_mutex) xSemaphoreGive(nav_mutex); }
  NavLock(const NavLock&) = delete;
  NavLock& operator=(const NavLock&) = delete;
};

// Heading source (serial "HEADING MAG|GPS"; default HEADING_SOURCE_DEFAULT):
//   MAG: BNO08x rotation vector, magnetometer north (+ HEADING_OFFSET_DEG).
//   GPS: game rotation vector (gyro+accel, no magnetometer) + an offset learned from
//        the GPS course over ground while driving straight on RTK fixed.
// The GPS offset is learned in both modes, so switching is instant and the log can
// compare both sources against the GPS course. All fields guarded by nav_mutex.
enum class HeadingSource : uint8_t { Mag = 0, Gps = 1 };
static HeadingSource heading_source = HEADING_SOURCE_DEFAULT ? HeadingSource::Gps : HeadingSource::Mag;
struct HeadingFusion {
  bool offset_valid = false;   // seeded from the magnetometer, then GPS-corrected
  float offset_deg = 0.0f;     // compass heading = game yaw + offset
  uint32_t corrections = 0;    // GPS course samples applied since boot
  float last_gps_err_deg = 0.0f;  // GPS course - fused heading at the last sample
  float last_mag_err_deg = 0.0f;  // GPS course - magnetometer heading at the last sample
  bool anchor_valid = false;
  double anchor_lat = 0.0, anchor_lon = 0.0;
  float anchor_game_deg = 0.0f;
  unsigned long last_gps_update_ms = 0;
};
static HeadingFusion heading_fusion;
static float estimateHeadingDeg();
static float magHeadingDeg();
static void handleUsbSerialControl();

static String usb_serial_line_buffer;
static bool usb_serial_line_overflow = false;
// NAV line on USB serial for a companion computer; toggled by "NAV ON" / "NAV OFF".
static bool nav_serial_output = false;
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

  // Snapshot shared state under the nav lock; print without holding it.
  GpsFix g;
  HeadingFusion hdg;
  bool hdg_src_gps = false;
  unsigned long nav_max_dt_ms = 0;
  unsigned long nav_cycles = 0;
  {
    NavLock lock;
    g = gps_fix;
    gps_fix.gga_ok_count = 0;
    gps_fix.gga_bad_count = 0;
    nav_max_dt_ms = loop_stats.max_nav_dt_ms;
    nav_cycles = loop_stats.nav_cycle_count;
    loop_stats.max_nav_dt_ms = 0;
    loop_stats.nav_cycle_count = 0;
    hdg_src_gps = heading_source == HeadingSource::Gps;
    hdg = heading_fusion;
  }

  Serial.printf("[NET] %s", net_manager.sourceName());
  Serial.print(" | [GPS] ");
  if (g.valid) {
    Serial.print("fix=");
    Serial.print(g.fix_quality);
    Serial.print(" sats=");
    Serial.print(g.satellites);
    Serial.print(" lat=");
    Serial.print(g.latitude, 7);
    Serial.print(" lon=");
    Serial.print(g.longitude, 7);
    Serial.print(" hdop=");
    Serial.print(g.hdop, 2);
    if (g.has_gst) Serial.printf(" herr=%.3fm", g.h_err_m);
  } else if (g.has_gga) {
    Serial.print("uart-ok no-fix q=");
    Serial.print(g.fix_quality);
    Serial.print(" sats=");
    Serial.print(g.satellites);
    Serial.print(" hdop=");
    Serial.print(g.hdop, 2);
  } else if ((now - g.last_rx_ms) < 2000 && g.last_rx_ms != 0) {
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

  // Comms loop health + GPS parse health since the previous line.
  Serial.printf(" | [COMMS] max_dt=%lums overruns=%lu cycles=%lu | [NAV] max_dt=%lums cycles=%lu | [GGA] ok=%lu bad=%lu",
                (unsigned long)loop_stats.max_comms_dt_ms,
                (unsigned long)loop_stats.comms_overrun_count,
                (unsigned long)loop_stats.comms_cycle_count,
                nav_max_dt_ms,
                nav_cycles,
                (unsigned long)g.gga_ok_count,
                (unsigned long)g.gga_bad_count);
  // Heading: errors are GPS course minus each source at the last straight segment.
  Serial.printf(" | [HDG] src=%s n=%lu fused_err=%.1f mag_err=%.1f",
                hdg_src_gps ? "GPS" : "MAG", (unsigned long)hdg.corrections,
                hdg.last_gps_err_deg, hdg.last_mag_err_deg);
  loop_stats.max_comms_dt_ms = 0;
  loop_stats.comms_overrun_count = 0;
  loop_stats.comms_cycle_count = 0;

  // Serial.print(" | [IMU] ");
  // if (imu_state.valid) {
  //   Serial.print("ax=");
  //   Serial.print(imu_state.ax, 3);
  //   Serial.print(" ay=");
  //   Serial.print(imu_state.ay, 3);
  //   Serial.print(" az=");
  //   Serial.print(imu_state.az, 3);
  //   Serial.print(" gx=");
  //   Serial.print(imu_state.gx, 3);
  //   Serial.print(" gy=");
  //   Serial.print(imu_state.gy, 3);
  //   Serial.print(" gz=");
  //   Serial.print(imu_state.gz, 3);
  // } else {
  //   Serial.print("no-data");
  // }

  // Serial.printf(" | [MISSION] %s", missionStateToString(mission_ctx.state));
  // if (mission_ctx.name.length() > 0) {
  //   Serial.print("(");
  //   Serial.print(mission_ctx.name);
  //   Serial.print(")");
  // }
  // if (mission_debug_valid) {
  //   Serial.print(" wp=");
  //   Serial.print(mission_debug_target_waypoint);
  //   Serial.print(" dist=");
  //   Serial.print(mission_debug_dist_m, 2);
  //   Serial.print(" herr=");
  //   Serial.print(mission_debug_heading_error_deg, 1);
  //   Serial.print(" spd=");
  //   Serial.print(mission_debug_speed_cmd_mps, 2);
  //   if (mission_debug_coverage) {
  //     Serial.print(" cov=");
  //     Serial.print(mission_debug_coverage_current);
  //     Serial.print("->");
  //     Serial.print(mission_debug_coverage_end);
  //   }
  // }

  // Serial.print(" | [LOOP] max_dt_ms=");
  // Serial.print(loop_stats.max_control_dt_ms);
  // Serial.print(" overruns=");
  // Serial.print(loop_stats.overrun_count);
  // Serial.print(" cycles=");
  // Serial.print(loop_stats.control_cycle_count);
  // Serial.print(" | [COMMS] max_dt_ms=");
  // Serial.print(loop_stats.max_comms_dt_ms);
  // Serial.print(" overruns=");
  // Serial.print(loop_stats.comms_overrun_count);
  // Serial.print(" cycles=");
  // Serial.print(loop_stats.comms_cycle_count);
  // Serial.print(" | [CAN] ok=");
  // Serial.print(loop_stats.can_runtime_tx_ok_count);
  // Serial.print(" drop=");
  // Serial.print(loop_stats.can_runtime_tx_drop_count);
  // Serial.print(" | [SAFE] imu=");
  // Serial.print(loop_stats.safety_stale_imu_count);
  // Serial.print(" gps=");
  // Serial.print(loop_stats.safety_stale_gps_count);
  // Serial.print(" rc_to=");
  // Serial.print(loop_stats.safety_remote_timeout_count);
  // Serial.print(" mission_abort=");
  // Serial.print(loop_stats.safety_mission_abort_count);

  // loop_stats.max_control_dt_ms = 0;
  // loop_stats.overrun_count = 0;
  // loop_stats.control_cycle_count = 0;
  // loop_stats.max_comms_dt_ms = 0;
  // loop_stats.comms_overrun_count = 0;
  // loop_stats.comms_cycle_count = 0;
  // loop_stats.can_runtime_tx_ok_count = 0;
  // loop_stats.can_runtime_tx_drop_count = 0;
  // loop_stats.safety_stale_imu_count = 0;
  // loop_stats.safety_stale_gps_count = 0;
  // loop_stats.safety_remote_timeout_count = 0;
  // loop_stats.safety_mission_abort_count = 0;
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

// Motor driver health, refreshed by MotorStatusTask (nav lock held while copying).
// 0/1 = drive front/rear, 2/3 = steer left/right.
static constexpr int kMotorCount = 4;
static const char* const kMotorNames[kMotorCount] = {"Drive F", "Drive R", "Steer L", "Steer R"};
static MotorStatus motor_status[kMotorCount];
// Fault reset requested by serial/server, run by ControlTask (the task that drives the
// motors). Bits: kMotorResetDrive, kMotorResetSteer.
static constexpr uint8_t kMotorResetDrive = 0x01;
static constexpr uint8_t kMotorResetSteer = 0x02;
static volatile uint8_t motor_reset_request = 0;
// Motors unpowered to save energy (MOTOR RELEASE / server motor_release): no drive or
// steering output and no mission start until MOTOR ACTIVATE. Set by ControlTask when it
// executes motor_power_request.
static volatile bool motors_released = false;
enum class MotorPowerRequest : uint8_t { None, Release, Activate };
static volatile MotorPowerRequest motor_power_request = MotorPowerRequest::None;

PS2X ps2x;
int ps2_error = 0;

const int MAX_RPM = 150;
bool e_stop_active = false;
// Why e-stop was triggered ("ps2", "server", "limit: Front L", ...); "" when released.
const char* e_stop_reason = "";

enum class RobotMode : uint8_t {
  Normal = 0,
  ZeroTurn = 1,
};

static void applyModeInit(RobotMode mode);

static constexpr float WHEELBASE_M = ROBOT_GEOMETRY_H_M;
static constexpr float TRACK_WIDTH_M = ROBOT_GEOMETRY_W_M;
static constexpr float MAX_LINEAR_MPS = MAX_LINEAR_SPEED_MS;
static constexpr float MAX_STEER_DEG = MAX_STEERING_ANGLE_DEG;
static constexpr float WHEEL_DIAMETER_M = DRIVE_WHEEL_DIAMETER_M;
static constexpr unsigned long IMU_STALE_TIMEOUT_MS = 1500;
static constexpr unsigned long GPS_STALE_TIMEOUT_MS = 3000;
static constexpr unsigned long MISSION_SENSOR_GRACE_MS = 2000;
static constexpr unsigned long COMMS_OVERRUN_MS = 20;
static constexpr unsigned long NAV_PERIOD_MS = 10;  // NavTask: GPS/IMU/safety/mission
static constexpr unsigned long NAV_SERIAL_PERIOD_MS = 100;  // NAV line for companion computer
// Waypoint arrival radii (m); override in Config*.h.
#ifndef MISSION_WAYPOINT_REACHED_M
#define MISSION_WAYPOINT_REACHED_M 1.5f
#endif
#ifndef MISSION_COVERAGE_WAYPOINT_REACHED_M
#define MISSION_COVERAGE_WAYPOINT_REACHED_M 0.5f
#endif
#ifndef MISSION_LINK_LOSS_HOLD_MS
#define MISSION_LINK_LOSS_HOLD_MS 30000UL
#endif
#ifndef HEADING_SOURCE_DEFAULT
#define HEADING_SOURCE_DEFAULT 0
#endif
#ifndef HEADING_GPS_MIN_BASELINE_M
#define HEADING_GPS_MIN_BASELINE_M 0.5f
#endif
#ifndef HEADING_GPS_MAX_YAW_CHANGE_DEG
#define HEADING_GPS_MAX_YAW_CHANGE_DEG 3.0f
#endif
#ifndef HEADING_GPS_GAIN
#define HEADING_GPS_GAIN 0.3f
#endif
#ifndef HEADING_GPS_MAX_HERR_M
#define HEADING_GPS_MAX_HERR_M 0.05f
#endif
static constexpr float MISSION_SLOWDOWN_RADIUS_M = 1.0f;
static constexpr float MISSION_HEADING_SLOWDOWN_CRAWL_MPS = 0.12f;
static constexpr size_t MISSION_MAX_WAYPOINTS = 1000;
static constexpr size_t MISSION_MAX_TASKS = 512;

enum class MissionTaskType : uint8_t {
  GotoWaypoint = 0,
  Wait,
  LoopStart,
  LoopEnd,
  CoveragePath,
  Spiral,
  SetOutput,  // turn output channel A/B/C/D on or off
  SetSpeed,       // set a DS3502 speed output (Speed1..N) in percent
  MoveActuator,   // move the linear actuator, optionally waiting until it arrives
  WaitInput,      // hold still until a PCF8574 input reaches a state
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
  // SetOutput params (only used when type == SetOutput)
  uint8_t output_channel{0};   // 0=A 1=B 2=C 3=D
  bool    output_state{false}; // true=ON false=OFF
  // I/O task params. SetSpeed: io_index = speed channel (1-based), io_value = %.
  // MoveActuator: io_value = position %, io_wait, wait_ms = timeout.
  // WaitInput: io_index = input (1-based), output_state = wanted state (true = active),
  // wait_ms = timeout (0 = none).
  uint8_t io_index{0};
  float   io_value{0.0f};
  bool    io_wait{false};
};

MissionWaypointPlan mission_waypoints[MISSION_MAX_WAYPOINTS];
MissionTaskPlan mission_tasks[MISSION_MAX_TASKS];
size_t mission_waypoint_count = 0;
size_t mission_task_count = 0;

struct MissionUploadCtx {
  bool   active{false};
  String session_id{};
  size_t expected_total{0};
  size_t expected_batches{0};
  size_t received_count{0};
  size_t received_batches{0};
};
static MissionUploadCtx upload_ctx;

static constexpr uint8_t kOutputPins[OUTPUT_COUNT] = {
  OUTPUT_A_PIN, OUTPUT_B_PIN, OUTPUT_C_PIN, OUTPUT_D_PIN
};

// ── USB serial FUNC_<n> actuators ───────────────────────────────────────────
enum class FuncActuatorType : uint8_t {
  GPIO    = FUNC_TYPE_GPIO,
  ODRIVE  = FUNC_TYPE_ODRIVE,
  GIM8108 = FUNC_TYPE_GIM8108,
};

struct FuncActuatorConfig {
  FuncActuatorType type;
  uint32_t id;
};

static const FuncActuatorConfig kFuncConfigs[FUNC_COUNT] = {
  { static_cast<FuncActuatorType>(FUNC_0_TYPE), FUNC_0_ID },
  { static_cast<FuncActuatorType>(FUNC_1_TYPE), FUNC_1_ID },
};

struct FuncActuator {
  FuncActuatorType type{FuncActuatorType::GPIO};
  uint32_t id{0};
  ODriveCAN* odrive{nullptr};
  GIM8108CAN* gim{nullptr};
  float current_deg{0.0f};  // last known/commanded resting position, degrees (CAN actuators only)
  // False until current_deg reflects a real reading/command (boot read, or any accepted
  // move). REL_DEG's arrival check is skipped while false, so a failed boot-time position
  // read can't permanently deadlock every future REL_DEG against a fictitious 0 baseline.
  bool current_deg_valid{false};
  // 0 = no active reject streak; otherwise millis() when the current run of "still moving"
  // rejections started, used to auto-resync if it runs longer than a real move ever would.
  unsigned long rel_deg_reject_since_ms{0};
  // The boot-time position read can succeed but still be stale (caught mid-leftover-motion
  // from before an ESP reset — the ODrive doesn't reset with it). So the arrival check isn't
  // enforced until after the first REL_DEG actually commits — that command always trusts the
  // live reading outright rather than waiting out FUNC_REL_DEG_STUCK_TIMEOUT_MS to resync.
  bool rel_deg_committed_once{false};
};
static FuncActuator func_actuators[FUNC_COUNT];

enum class FuncPhase : uint8_t { Idle, Active };
struct FuncRuntimeState {
  FuncPhase phase{FuncPhase::Idle};
  unsigned long off_at_ms{0};
  // true (DEG): updateFuncExecutor() actuates (off/return-to-0) when off_at_ms elapses.
  // false (REL_DEG busy-lock on GIM8108): off_at_ms elapsing just releases the lock —
  // no actuation, the move was already sent.
  bool auto_return{false};
};
static FuncRuntimeState func_state[FUNC_COUNT];
size_t mission_task_cursor = 0;
unsigned long mission_task_started_ms = 0;
size_t mission_active_task_cursor = static_cast<size_t>(-1);
int mission_loop_remaining[MISSION_MAX_TASKS];
int mission_coverage_current_index = -1;

static constexpr int8_t MOTOR_DI_LEFT = DRIVE_LEFT_SIGN;
static constexpr int8_t MOTOR_DI_RIGHT = DRIVE_RIGHT_SIGN;

RobotMode robot_mode = RobotMode::Normal;
DoubleAckermannSteering robot(WHEELBASE_M, TRACK_WIDTH_M, STEER_MAX_WHEEL_ANGLE_DEG);

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
    } else if (type == "set_output") {
      String ch_str = task["channel"] | "A";
      ch_str.toUpperCase();
      uint8_t ch = (ch_str == "B") ? 1 : (ch_str == "C") ? 2 : (ch_str == "D") ? 3 : 0;
      bool state = task["state"] | false;
      MissionTaskPlan t{};
      t.type           = MissionTaskType::SetOutput;
      t.output_channel = ch;
      t.output_state   = state;
      mission_tasks[mission_task_count++] = t;
    } else if (type == "set_speed") {
      MissionTaskPlan t{};
      t.type     = MissionTaskType::SetSpeed;
      t.io_index = static_cast<uint8_t>(constrain(task["channel"] | 1, 0, 255));
      t.io_value = task["percent"] | 0.0f;
      mission_tasks[mission_task_count++] = t;
    } else if (type == "move_actuator") {
      MissionTaskPlan t{};
      t.type     = MissionTaskType::MoveActuator;
      t.io_value = task["position"] | 0.0f;
      t.io_wait  = task["wait"] | true;
      t.wait_ms  = static_cast<unsigned long>(max(1.0f, task["timeout"] | 30.0f) * 1000.0f);
      mission_tasks[mission_task_count++] = t;
    } else if (type == "wait_input") {
      String state_str = task["state"] | "active";
      MissionTaskPlan t{};
      t.type         = MissionTaskType::WaitInput;
      t.io_index     = static_cast<uint8_t>(constrain(task["input"] | 1, 0, 255));
      t.output_state = (state_str != "inactive");
      t.wait_ms      = static_cast<unsigned long>(max(0.0f, task["timeout"] | 0.0f) * 1000.0f);
      mission_tasks[mission_task_count++] = t;
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

// Defined with the USB POT/ACT commands further down.
static const char* setSpeedOutput(int ch, int raw);
static const char* moveActuator(float pct);
static void zeroSpeedOutputs(const char* why);

static void setMissionState(MissionState new_state) {
  if (mission_ctx.state == new_state) return;
  mission_ctx.state = new_state;
  mission_ctx.state_since_ms = millis();
  if (new_state == MissionState::Failed) zeroSpeedOutputs("mission failed");
#if DEBUG_LOG_MISSION
  Serial.printf("[MISSION] state=%s\n", missionStateToString(mission_ctx.state));
#endif
}

static void resetMissionCommandTargets() {
  control_targets.linear_mps = 0.0f;
  control_targets.angle_deg = 0.0f;
  control_targets.remote_move_expire_ms = 0;
}

static_assert(MISSION_ZERO_TURN_EXIT_DEG < MISSION_ZERO_TURN_ENTER_DEG,
              "MISSION_ZERO_TURN_EXIT_DEG must be below MISSION_ZERO_TURN_ENTER_DEG");

// True while the mission executor itself put the robot in zero-turn mode, so it only
// ever reverts a mode it set (a manual MODE ZERO_TURN is left alone).
static bool mission_auto_zero_turn = false;
// Set once a mode switch is decided: the drive ramps down to 0 in the current mode
// first, then the switch happens. Latched so the decision can't flip back mid-ramp.
static bool mission_mode_switch_pending = false;
static unsigned long mission_mode_switch_ms = 0;

// Slew-limited mission drive speed (m/s), ramped by slewMissionSpeed().
static float mission_speed_cmd_mps = 0.0f;
static unsigned long mission_speed_cmd_ms = 0;
// Last normal-mode steering angle sent by the mission (held during a ramp-down into zero-turn).
static float mission_last_angle_cmd_deg = 0.0f;

static void resetMissionSpeedSlew() {
  mission_speed_cmd_mps = 0.0f;
  mission_speed_cmd_ms = 0;
  mission_last_angle_cmd_deg = 0.0f;
}

static float slewMissionSpeed(float target_mps, unsigned long now) {
  // Cap dt so a stalled comms loop can't turn into a single big speed jump.
  const float dt_s = (mission_speed_cmd_ms == 0) ? 0.0f : min(0.1f, (now - mission_speed_cmd_ms) * 0.001f);
  mission_speed_cmd_ms = now;
  const float max_step = MISSION_SPEED_SLEW_MPS2 * dt_s;
  mission_speed_cmd_mps += clampf(target_mps - mission_speed_cmd_mps, -max_step, max_step);
  return mission_speed_cmd_mps;
}

static bool missionModeSettling(unsigned long now) {
  return (now - mission_mode_switch_ms) < MISSION_MODE_SWITCH_SETTLE_MS;
}

// Switches immediately. The smooth path (ramp down -> switch -> settle -> ramp up) is
// driven from updateMissionExecutor(); direct calls are for forced reverts when the
// mission stops driving anyway (pause/end/Wait/Spiral).
// Mode is changed directly rather than via applyModeInit() (which blocks for 200ms);
// the control task re-applies steering for robot_mode every cycle, and the drive is
// held at 0 for MISSION_MODE_SWITCH_SETTLE_MS while the steering moves.
static void setMissionAutoZeroTurn(bool on) {
  mission_mode_switch_pending = false;
  if (on == mission_auto_zero_turn) return;
  if (!on && robot_mode != RobotMode::ZeroTurn) {
    mission_auto_zero_turn = false;
    return;
  }
  mission_auto_zero_turn = on;
  mission_mode_switch_ms = millis();
  resetMissionSpeedSlew();
  control_targets.linear_mps = 0.0f;
  control_targets.angle_deg = 0.0f;
  robot_mode = on ? RobotMode::ZeroTurn : RobotMode::Normal;
#if DEBUG_LOG_MISSION
  Serial.printf("[MISSION] Auto mode -> %s\n", on ? "ZERO_TURN" : "NORMAL");
#endif
}

// GPS good enough to drive: GST horizontal error within MISSION_GPS_MAX_HERR_M.
// While holding, require 80% of the limit to resume so an error hovering at the
// limit can't flap between hold and drive. A receiver that never sent GST (UBX
// config NAK) can't be judged and is allowed.
static bool gpsAccurateForMission(unsigned long now, bool holding) {
  if (!gps_fix.has_gst) return true;
  if (now - gps_fix.last_gst_ms > GPS_STALE_TIMEOUT_MS) return false;
  const float limit_m = holding ? 0.8f * MISSION_GPS_MAX_HERR_M : MISSION_GPS_MAX_HERR_M;
  return gps_fix.h_err_m <= limit_m;
}

// millis() when the accuracy gate started holding the mission; 0 = not holding.
// File scope so telemetry can report the hold (mission.holding).
static unsigned long mission_gps_hold_since_ms = 0;

// millis() of the last CommsTask tick with the control server connected. Written
// by CommsTask, read by NavTask (aligned 32-bit, so no lock needed).
static volatile unsigned long server_link_ok_ms = 0;
// millis() when the link-loss hold started; 0 = not holding.
static unsigned long mission_link_hold_since_ms = 0;

// Serial CMD_VEL / STOP override (companion computer, e.g. lidar obstacle avoidance):
// until this time the mission keeps its state and target but does not drive.
// 0 = no override. Written by SerialCmdTask, read by NavTask (both under nav lock).
static unsigned long serial_override_until_ms = 0;

static bool serialOverrideActive(unsigned long now) {
  return serial_override_until_ms != 0 && static_cast<long>(serial_override_until_ms - now) > 0;
}

// True when the control server has been unreachable for MISSION_LINK_LOSS_HOLD_MS.
// Never true without a control server (offline / serial-only builds).
static bool serverLinkLost(unsigned long now) {
#if NET_MODE != 0 && CONTROL_SERVER_ENABLE
  // Signed: CommsTask may stamp server_link_ok_ms just after `now` was read.
  return static_cast<long>(now - server_link_ok_ms) > static_cast<long>(MISSION_LINK_LOSS_HOLD_MS);
#else
  (void)now;
  return false;
#endif
}

// Tasks that only touch I/O run with the robot standing still, so they don't need GPS.
static bool missionTaskNeedsGps(MissionTaskType type) {
  switch (type) {
    case MissionTaskType::Spiral:
    case MissionTaskType::SetOutput:
    case MissionTaskType::SetSpeed:
    case MissionTaskType::MoveActuator:
    case MissionTaskType::WaitInput:
      return false;
    default:
      return true;
  }
}

static void holdMissionStill(unsigned long now) {
  setMissionAutoZeroTurn(false);
  resetMissionSpeedSlew();
  control_targets.linear_mps = 0.0f;
  control_targets.angle_deg = 0.0f;
  control_targets.remote_move_expire_ms = now + 150;
}

static void failMission(const char* why) {
  mission_debug_valid = false;
  mission_ctx.active = false;
  mission_ctx.phase = 0;
  mission_ctx.waypoint_index = -1;
  resetMissionCommandTargets();
  setMissionState(MissionState::Failed);
  Serial.printf("[MISSION] Failed: task %u: %s\n", static_cast<unsigned>(mission_task_cursor), why);
}

static void advanceMissionTask(unsigned long now) {
  ++mission_task_cursor;
  mission_task_started_ms = now;
  mission_active_task_cursor = static_cast<size_t>(-1);
}

// SetOutput / SetSpeed / MoveActuator / WaitInput. Returns false if the task is not one
// of these. Instant tasks advance at once; waiting ones hold the robot still.
static bool runMissionIoTask(MissionTaskPlan& task, unsigned long now) {
  switch (task.type) {
    case MissionTaskType::SetOutput: {
      const uint8_t ch = task.output_channel;
      if (ch < OUTPUT_COUNT) {
        digitalWrite(kOutputPins[ch], task.output_state ? HIGH : LOW);
        Serial.printf("[MISSION] Output %c %s (pin %u)\n", 'A' + ch, task.output_state ? "ON" : "OFF", kOutputPins[ch]);
      }
      advanceMissionTask(now);
      return true;
    }
    case MissionTaskType::SetSpeed: {
#if DS3502_ENABLE
      const float pct = clampf(task.io_value, 0.0f, 100.0f);
      const char* err = setSpeedOutput(task.io_index, lroundf(pct * Ds3502::kMaxWiper / 100.0f));
#else
      const float pct = 0.0f;
      const char* err = setSpeedOutput(task.io_index, 0);
#endif
      if (err) {
        failMission(err);
        return true;
      }
      Serial.printf("[MISSION] Speed%u -> %.0f%%\n", task.io_index, pct);
      advanceMissionTask(now);
      return true;
    }
    case MissionTaskType::MoveActuator: {
      holdMissionStill(now);
#if ACTUATOR_ENABLE
      // Start the move on the first tick of this task (a loop re-runs it with a new start time).
      static size_t started_cursor = static_cast<size_t>(-1);
      static unsigned long started_ms = 0;
      if (started_cursor != mission_task_cursor || started_ms != mission_task_started_ms) {
        started_cursor = mission_task_cursor;
        started_ms = mission_task_started_ms;
        if (const char* err = moveActuator(task.io_value)) {
          failMission(err);
          return true;
        }
        Serial.printf("[MISSION] Actuator -> %.1f%%%s\n", task.io_value, task.io_wait ? " (waiting)" : "");
        if (!task.io_wait) {
          advanceMissionTask(now);
          return true;
        }
      }
      if (actuator.state() == LinearActuator::State::Fault) {
        failMission(LinearActuator::faultName(actuator.fault()));
      } else if (actuator.state() == LinearActuator::State::Idle) {
        Serial.printf("[MISSION] Actuator arrived at %.1f%%\n", actuator.positionPct());
        advanceMissionTask(now);
      } else if (now - mission_task_started_ms >= task.wait_ms) {
        actuator.stop();
        failMission("actuator move timed out");
      }
#else
      failMission("ACTUATOR_ENABLE is 0");
#endif
      return true;
    }
    case MissionTaskType::WaitInput: {
      holdMissionStill(now);
#if PCF8574_ENABLE
      if (task.io_index < 1 || task.io_index > DIGITAL_INPUT_COUNT) {
        failMission("invalid input number");
      } else if (digital_inputs_ok &&
                 ((digital_inputs >> (task.io_index - 1)) & 1u) == (task.output_state ? 1u : 0u)) {
        Serial.printf("[MISSION] Input %u is %s\n", task.io_index, task.output_state ? "active" : "inactive");
        advanceMissionTask(now);
      } else if (task.wait_ms > 0 && now - mission_task_started_ms >= task.wait_ms) {
        failMission("wait for input timed out");
      }
#else
      failMission("PCF8574_ENABLE is 0");
#endif
      return true;
    }
    default:
      return false;
  }
}

static void updateMissionExecutor() {
  static unsigned long invalid_sensor_since_ms = 0;
  unsigned long& gps_hold_since_ms = mission_gps_hold_since_ms;
  if (!mission_ctx.active) {
    mission_debug_valid = false;
    invalid_sensor_since_ms = 0;
    gps_hold_since_ms = 0;
    mission_link_hold_since_ms = 0;
    setMissionAutoZeroTurn(false);
    resetMissionSpeedSlew();
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
    setMissionAutoZeroTurn(false);
    resetMissionSpeedSlew();
    resetMissionCommandTargets();
    return;
  }

  if (mission_ctx.state != MissionState::Running) {
    mission_debug_valid = false;
    gps_hold_since_ms = 0;
    mission_link_hold_since_ms = 0;
    return;
  }

  // Serial CMD_VEL/STOP override: drive targets belong to the companion computer until
  // its commands expire; then the mission resumes toward its current target from
  // wherever the robot is. Checked before the holds, which would zero the targets.
  static bool yielding_to_serial = false;
  if (serialOverrideActive(now)) {
    mission_debug_valid = false;
    resetMissionSpeedSlew();
    if (!yielding_to_serial) {
      yielding_to_serial = true;
      Serial.println("[MISSION] Serial override: mission yielding");
    }
    return;
  }
  if (yielding_to_serial) {
    yielding_to_serial = false;
    Serial.println("[MISSION] Serial override ended: mission resuming");
  }

  // Server unreachable: hold in place so the robot never drives on with no way to
  // stop it remotely; resume automatically when the link returns.
  if (serverLinkLost(now)) {
    mission_debug_valid = false;
    resetMissionCommandTargets();
    resetMissionSpeedSlew();
    if (mission_link_hold_since_ms == 0) {
      mission_link_hold_since_ms = now;
      Serial.printf("[MISSION] Holding: no server link for %lu s\n",
                    MISSION_LINK_LOSS_HOLD_MS / 1000UL);
    }
    return;
  }
  if (mission_link_hold_since_ms != 0) {
    Serial.printf("[MISSION] Resumed: server link back (held %lu ms)\n",
                  now - mission_link_hold_since_ms);
    mission_link_hold_since_ms = 0;
  }

  // Spiral tasks are purely time-based and do not require GPS or IMU
  const bool task_needs_gps = !(mission_task_cursor < mission_task_count &&
                                !missionTaskNeedsGps(mission_tasks[mission_task_cursor].type));

  if (task_needs_gps && (!gps_fix.valid || !imu_state.valid)) {
    mission_debug_valid = false;
    // Stop at once; don't keep driving the last command through the grace period.
    resetMissionCommandTargets();
    resetMissionSpeedSlew();
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

  // GPS valid but not accurate enough: hold in place, resume when it recovers,
  // fail if it stays poor for MISSION_GPS_HOLD_TIMEOUT_MS.
  if (task_needs_gps && !gpsAccurateForMission(now, gps_hold_since_ms != 0)) {
    mission_debug_valid = false;
    resetMissionCommandTargets();
    resetMissionSpeedSlew();
    if (gps_hold_since_ms == 0) {
      gps_hold_since_ms = now;
      Serial.printf("[MISSION] Holding: GPS error %.3f m > %.2f m\n",
                    gps_fix.h_err_m, MISSION_GPS_MAX_HERR_M);
    } else if (now - gps_hold_since_ms > MISSION_GPS_HOLD_TIMEOUT_MS) {
      ++loop_stats.safety_mission_abort_count;
      mission_ctx.active = false;
      mission_ctx.phase = 0;
      mission_ctx.waypoint_index = -1;
      resetMissionCommandTargets();
      setMissionState(MissionState::Failed);
      Serial.printf("[MISSION] Failed: GPS error above %.2f m for %lu s\n",
                    MISSION_GPS_MAX_HERR_M, MISSION_GPS_HOLD_TIMEOUT_MS / 1000UL);
      gps_hold_since_ms = 0;
    }
    return;
  }
  if (gps_hold_since_ms != 0) {
    Serial.printf("[MISSION] Resumed: GPS error %.3f m (held %lu ms)\n",
                  gps_fix.h_err_m, now - gps_hold_since_ms);
    gps_hold_since_ms = 0;
  }

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

  if (runMissionIoTask(task, now)) {
    mission_debug_valid = false;
    return;
  }

  if (task.type == MissionTaskType::Wait) {
    mission_debug_valid = false;
    holdMissionStill(now);
    if ((now - mission_task_started_ms) >= task.wait_ms) {
      ++mission_task_cursor;
      mission_task_started_ms = now;
    }
    return;
  }

  if (task.type == MissionTaskType::Spiral) {
    mission_debug_valid = false;
    // Spiral steers with angle_deg, which zero-turn mode ignores.
    setMissionAutoZeroTurn(false);
    resetMissionSpeedSlew();
    if (missionModeSettling(now)) {
      // Steering is still returning from zero-turn; hold still and start the
      // (time-based) spiral only once it has settled.
      control_targets.linear_mps = 0.0f;
      control_targets.angle_deg  = 0.0f;
      control_targets.remote_move_expire_ms = now + 150;
      mission_task_started_ms = now;
      return;
    }
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

    // Non-waypoint tasks (I/O, wait, loop, spiral) run from the top on the next tick.
    if (next_task->type == MissionTaskType::SetOutput ||
        next_task->type == MissionTaskType::SetSpeed ||
        next_task->type == MissionTaskType::MoveActuator ||
        next_task->type == MissionTaskType::WaitInput ||
        next_task->type == MissionTaskType::LoopStart ||
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

  const bool mode_settling = missionModeSettling(now);
#if MISSION_AUTO_ZERO_TURN_ENABLE
  // Decide every tick (also while settling, so a new waypoint at a sharp angle is caught
  // right away instead of being steered toward first), then latch it until the
  // ramp-down completes. The switch itself still only happens once stopped.
  if (!mission_mode_switch_pending) {
    const float abs_heading_error_deg = fabsf(heading_error_deg);
    if (!mission_auto_zero_turn && robot_mode == RobotMode::Normal &&
        abs_heading_error_deg > MISSION_ZERO_TURN_ENTER_DEG) {
      mission_mode_switch_pending = true;
    } else if (mission_auto_zero_turn && abs_heading_error_deg < MISSION_ZERO_TURN_EXIT_DEG) {
      mission_mode_switch_pending = true;
    }
  }
#endif

  float target_speed_mps = 0.0f;
  float angle_cmd_deg = 0.0f;
  if (mission_auto_zero_turn) {
    // Zero-turn drive: +linear_mps spins clockwise (heading increasing). A positive
    // heading_error_deg means the target is counter-clockwise, so rotate the other way.
    const float rotate_dir = (heading_error_deg > 0.0f ? -1.0f : 1.0f) * MISSION_ZERO_TURN_DIRECTION_SIGN;
    target_speed_mps = rotate_dir * clampf(MISSION_ZERO_TURN_SPEED_MPS, 0.0f, MAX_LINEAR_MPS);
  } else {
    target_speed_mps = clampf(active_waypoint_task->speed_mps, 0.05f, MAX_LINEAR_MPS);
    if (fabsf(heading_error_deg) > 70.0f) {
      target_speed_mps *= 0.5f;
      const float crawl_speed_floor = min(MISSION_HEADING_SLOWDOWN_CRAWL_MPS, clampf(active_waypoint_task->speed_mps, 0.05f, MAX_LINEAR_MPS));
      target_speed_mps = max(target_speed_mps, crawl_speed_floor);
    }
    if (mission_mode_switch_pending) {
      // About to enter zero-turn: don't steer toward the large new error while ramping
      // down, just hold the angle the robot was already driving with.
      angle_cmd_deg = mission_last_angle_cmd_deg;
    } else {
      angle_cmd_deg = clampf(heading_error_deg * 0.7f, -MAX_STEER_DEG, MAX_STEER_DEG);
    }
  }

  // Hold still while the steering mechanism moves; ramp down before a pending switch.
  if (mode_settling || mission_mode_switch_pending) {
    target_speed_mps = 0.0f;
  }
  float speed_cmd = slewMissionSpeed(target_speed_mps, now);

  if (mission_mode_switch_pending && fabsf(speed_cmd) < 0.005f) {
    // Stopped: switch mode now; the settle hold + ramp-up take over from here.
    setMissionAutoZeroTurn(!mission_auto_zero_turn);
    speed_cmd = 0.0f;
    angle_cmd_deg = 0.0f;
  }
  mission_last_angle_cmd_deg = angle_cmd_deg;

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
  control_targets.angle_deg = angle_cmd_deg;
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

  // LED1: Internet connection type
  //   Green          = WiFi active
  //   Blue           = SIM / cellular active
  //   Blinking Yellow= No network, actively trying (WiFi lost or SIM connecting)
  //   Red            = No network and not connecting
  {
    uint32_t net_color;
    switch (net_manager.source()) {
      case NetSource::WIFI:
        net_color = LED_GREEN;
        break;
      case NetSource::SIM:
        net_color = LED_BLUE;
        break;
      default:
        // Blink yellow while at least one bearer is attempting to connect
        net_color = ((now / 400) % 2 == 0) ? LED_YELLOW : LED_OFF;
        break;
    }
    status_leds.setPixelColor(1, net_color);
  }

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
    // Assuming typical angular_rads in [-pi, pi], scale to steering deg range
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

static void initFuncActuators() {
  for (size_t i = 0; i < FUNC_COUNT; ++i) {
    func_actuators[i].type = kFuncConfigs[i].type;
    func_actuators[i].id = kFuncConfigs[i].id;

    switch (func_actuators[i].type) {
      case FuncActuatorType::ODRIVE: {
        ODriveCAN* odrive = new ODriveCAN(kFuncConfigs[i].id, &loop_stats);
        func_actuators[i].odrive = odrive;
        odrive->set_controller_mode(3, 1);
        odrive->set_axis_state(8);  // closed loop control — ODrive syncs input_pos to the
                                     // current pos_estimate on entry, so this holds wherever
                                     // the axis physically already is; no move is commanded.
        float raw_turns = 0.0f;
        if (odrive->get_position_turns(&raw_turns, 250)) {
          func_actuators[i].current_deg = (raw_turns / odrive->gear_ratio) * 360.0f;
          func_actuators[i].current_deg_valid = true;
          Serial.printf("[FUNC_%d] ODrive id=%u boot position read: %.1f deg\n",
                        static_cast<int>(i), kFuncConfigs[i].id, func_actuators[i].current_deg);
        } else {
          func_actuators[i].current_deg = 0.0f;
          func_actuators[i].current_deg_valid = false;
          Serial.printf("[FUNC_%d] WARNING: could not read ODrive id=%u position at boot "
                        "(REL_DEG's arrival check will be skipped until its first successful move)\n",
                        static_cast<int>(i), kFuncConfigs[i].id);
        }
        break;
      }
      case FuncActuatorType::GIM8108:
        func_actuators[i].gim = new GIM8108CAN(kFuncConfigs[i].id, &loop_stats);
        func_actuators[i].gim->set_position_max_speed_rpm(100.0f);
        func_actuators[i].gim->set_max_current_amp(5.0f);
        func_actuators[i].gim->set_acceleration_rpm_per_sec(200.0f);
        func_actuators[i].gim->clear_fault();
        // No boot-time move — clear_fault() alone holds the current physical position.
        break;
      case FuncActuatorType::GPIO:
        // kOutputPins already configured as OUTPUT/LOW earlier in setup().
        break;
    }
  }
}

// FUNC_<idx> ON|OFF for a GPIO/relay actuator — direct, immediate, no timer.
static bool setFuncGpio(int idx, bool on) {
  if (idx < 0 || idx >= static_cast<int>(FUNC_COUNT)) {
    Serial.printf("[USB_SERIAL] Invalid FUNC index: %d\n", idx);
    return false;
  }
  FuncActuator& act = func_actuators[idx];
  if (act.type != FuncActuatorType::GPIO) {
    Serial.printf("[USB_SERIAL] FUNC_%d is not a GPIO actuator — ON/OFF only applies to GPIO\n", idx);
    return false;
  }
  if (on && e_stop_active) {
    Serial.println("[USB_SERIAL] FUNC rejected: e-stop active");
    return false;
  }
  digitalWrite(kOutputPins[act.id], on ? HIGH : LOW);
  Serial.printf("[USB_SERIAL] FUNC_%d %s (GPIO pin %d)\n", idx, on ? "ON" : "OFF", kOutputPins[act.id]);
  return true;
}

// Starts FUNC_<idx>:
//   CAN actuator: <degrees> DEG     — move +<degrees> from home, hold, then auto-return to 0
//   CAN actuator: <degrees> REL_DEG — move +<degrees> from its current resting position and
//                                     just hold there (no auto-off/return)
// Non-blocking — DEG completion is handled by updateFuncExecutor(); REL_DEG (ODrive) completes
// immediately, and REL_DEG (GIM8108) is busy-locked for a fixed window instead.
static bool triggerFunc(int idx, float value, const String& unit) {
  if (idx < 0 || idx >= static_cast<int>(FUNC_COUNT)) {
    Serial.printf("[USB_SERIAL] Invalid FUNC index: %d\n", idx);
    return false;
  }
  if (e_stop_active) {
    Serial.println("[USB_SERIAL] FUNC rejected: e-stop active");
    return false;
  }
  if (func_state[idx].phase == FuncPhase::Active) {
    Serial.printf("[USB_SERIAL] FUNC_%d rejected: still active from a previous command\n", idx);
    return false;
  }

  FuncActuator& act = func_actuators[idx];

  if (act.type == FuncActuatorType::GPIO) {
    Serial.printf("[USB_SERIAL] FUNC_%d is a GPIO actuator — usage: FUNC_%d ON|OFF\n", idx, idx);
    return false;
  }

  if (unit == "REL_DEG") {
    if (value == 0.0f) {
      Serial.printf("[USB_SERIAL] FUNC_%d rejected: degrees must be non-zero\n", idx);
      return false;
    }

    if (act.type == FuncActuatorType::GIM8108) {
      // Native relative move on the drive itself — no position readback needed. Busy-locked
      // for a fixed window since there's no feedback here to know when it actually arrives.
      act.gim->move_relative_position_turns(value / 360.0f);
      func_state[idx].phase = FuncPhase::Active;
      func_state[idx].auto_return = false;
      func_state[idx].off_at_ms = millis() + FUNC_REL_DEG_GIM_BUSY_MS;
      Serial.printf("[USB_SERIAL] FUNC_%d moved %+.1f deg relative\n", idx, value);
      return true;
    }

    // ODrive has no native relative-move command, so read its actual live position first
    // and compute the new absolute target from that — never from a software-cached value,
    // which would drift from reality across restarts or external motion.
    float raw_turns = 0.0f;
    if (!act.odrive->get_position_turns(&raw_turns, 200)) {
      Serial.printf("[USB_SERIAL] FUNC_%d rejected: could not read current ODrive position (no CAN response)\n", idx);
      return false;
    }
    const float actual_deg = (raw_turns / act.odrive->gear_ratio) * 360.0f;

    // Reject if the last commanded target hasn't actually been reached yet — otherwise a
    // rapid re-press (or a bounced/duplicate button edge) would chain a new +value on top
    // of a move still in flight and compound into runaway motion. Skipped when current_deg
    // isn't valid yet (e.g. the boot-time position read failed), and skipped for the very
    // first REL_DEG since boot even if it was "valid" — that boot read can still be stale
    // (leftover motion from before an ESP reset), so the first command always trusts the
    // live reading immediately instead of waiting out the stuck-timeout to resync.
    if (act.current_deg_valid && act.rel_deg_committed_once &&
        fabsf(actual_deg - act.current_deg) > FUNC_REL_DEG_ARRIVAL_TOLERANCE_DEG) {
      const unsigned long now_ms = millis();
      if (act.rel_deg_reject_since_ms == 0) {
        act.rel_deg_reject_since_ms = now_ms;
      }
      // A single 45deg-ish move settles in well under a second, so if the SAME mismatch is
      // still being rejected this long later, it isn't "our own move still in flight" — the
      // tracked baseline is desynced from reality (stale boot read, leftover motion from
      // before an ESP reset, etc.) and would otherwise block this actuator forever. Resync
      // to the live reading instead of continuing to reject.
      if (now_ms - act.rel_deg_reject_since_ms < FUNC_REL_DEG_STUCK_TIMEOUT_MS) {
        Serial.printf("[USB_SERIAL] FUNC_%d rejected: still moving (actual=%.1f deg, previous target=%.1f deg)\n",
                      idx, actual_deg, act.current_deg);
        return false;
      }
      Serial.printf("[USB_SERIAL] FUNC_%d resync: stuck for over %lu ms (actual=%.1f deg, stale target=%.1f deg) — using live position as new baseline\n",
                    idx, static_cast<unsigned long>(FUNC_REL_DEG_STUCK_TIMEOUT_MS), actual_deg, act.current_deg);
    }

    act.rel_deg_reject_since_ms = 0;
    act.rel_deg_committed_once = true;
    act.current_deg = actual_deg + value;
    act.current_deg_valid = true;
    act.odrive->set_position(act.current_deg / 360.0f, 0.0f, 0.0f, /*force=*/true);
    Serial.printf("[USB_SERIAL] FUNC_%d moved %+.1f deg (was %.1f deg, now targeting %.1f deg)\n",
                  idx, value, actual_deg, act.current_deg);
    return true;
  }

  if (unit != "DEG") {
    Serial.printf("[USB_SERIAL] FUNC_%d is a CAN actuator — usage: FUNC_%d <degrees> DEG|REL_DEG\n", idx, idx);
    return false;
  }
  if (value <= 0.0f) {
    Serial.printf("[USB_SERIAL] FUNC_%d rejected: degrees must be > 0\n", idx);
    return false;
  }
  const float turns = value / 360.0f;
  if (act.type == FuncActuatorType::ODRIVE) {
    act.odrive->set_position(turns, 0.0f, 0.0f, /*force=*/true);
  } else {
    act.gim->set_absolute_position_turns(turns);
  }
  func_state[idx].phase = FuncPhase::Active;
  func_state[idx].auto_return = true;
  func_state[idx].off_at_ms = millis() + FUNC_DEG_HOLD_MS;
  Serial.printf("[USB_SERIAL] FUNC_%d moving +%.1f deg, will hold %lu ms then return to 0\n",
                idx, value, static_cast<unsigned long>(FUNC_DEG_HOLD_MS));
  return true;
}

static void updateFuncExecutor() {
  const unsigned long now = millis();
  for (size_t i = 0; i < FUNC_COUNT; ++i) {
    if (func_state[i].phase != FuncPhase::Active) continue;
    if (now < func_state[i].off_at_ms) continue;

    FuncActuator& act = func_actuators[i];
    if (func_state[i].auto_return) {
      // Only DEG (ODRIVE/GIM8108) sets auto_return — GPIO is now ON/OFF-only (setFuncGpio())
      // and never goes through this timer.
      if (!e_stop_active) {
        switch (act.type) {
          case FuncActuatorType::ODRIVE:
            act.odrive->set_position(0.0f, 0.0f, 0.0f, /*force=*/true);
            act.current_deg = 0.0f;
            act.current_deg_valid = true;
            break;
          case FuncActuatorType::GIM8108:
            act.gim->set_absolute_position_turns(0.0f);
            act.current_deg = 0.0f;
            act.current_deg_valid = true;
            break;
          case FuncActuatorType::GPIO:
            break;
        }
      }
      Serial.printf("[USB_SERIAL] FUNC_%d OFF\n", static_cast<int>(i));
    }
    // else: this was a REL_DEG busy-lock (GIM8108) — just release it, no actuation.
    func_state[i].phase = FuncPhase::Idle;
  }
}

// Listens on the CAN bus for any frame from <node_id> (ODrive-style identifier:
// node_id = identifier >> 5) for up to timeout_ms. Blocking, but bounded — this is a
// manual diagnostic command, not part of the normal control path.
static void handleCanPing(const String& args) {
  unsigned long node_id = 0;
  unsigned long timeout_ms = 500;
  int parsed = sscanf(args.c_str(), "%lu %lu", &node_id, &timeout_ms);
  if (parsed < 1) {
    Serial.println("[USB_SERIAL] Invalid CAN_PING. Usage: CAN_PING <node_id> [timeout_ms]");
    return;
  }
  timeout_ms = max(50UL, min(timeout_ms, 2000UL));

  Serial.printf("[USB_SERIAL] CAN_PING node_id=%lu listening for %lu ms...\n", node_id, timeout_ms);
  CanRxLock can_lock;

  const unsigned long start = millis();
  unsigned long other_frame_count = 0;
  bool matched = false;
  twai_message_t msg;

  while (millis() - start < timeout_ms) {
    if (twai_receive(&msg, pdMS_TO_TICKS(10)) != ESP_OK) continue;

    const unsigned long frame_node_id = msg.identifier >> 5;
    const unsigned long frame_cmd_id = msg.identifier & 0x1F;

    if (frame_node_id == node_id) {
      String hex;
      for (int i = 0; i < msg.data_length_code; ++i) {
        char byte_buf[4];
        snprintf(byte_buf, sizeof(byte_buf), "%02X ", msg.data[i]);
        hex += byte_buf;
      }
      Serial.printf("[USB_SERIAL] CAN_PING match: id=0x%03lX (node=%lu cmd=0x%02lX) dlc=%d data=%s(after %lu ms)\n",
                    static_cast<unsigned long>(msg.identifier), frame_node_id, frame_cmd_id,
                    msg.data_length_code, hex.c_str(), millis() - start);
      matched = true;
      break;
    }
    ++other_frame_count;
  }

  if (matched) return;

  Serial.printf("[USB_SERIAL] CAN_PING node_id=%lu: no response in %lu ms (saw %lu frames from other node ids)\n",
                node_id, timeout_ms, other_frame_count);
  if (other_frame_count == 0) {
    Serial.println("[USB_SERIAL]   No CAN traffic at all — check bus wiring/termination/power, or a baud rate mismatch");
  } else {
    Serial.println("[USB_SERIAL]   Bus has traffic but none from this node id — check the device's configured CAN node id");
  }
}

// GIM_PROBE <id> [cmd_hex] [window_ms] — sends a one-byte GIM8108 command (default 0xAE,
// read status) to <id> and prints every frame from ids 0x000-0x0FF for window_ms
// (default 150, max 1000). Diagnostic, blocking like CAN_PING.
static void handleGimProbe(const String& args) {
  unsigned int id = 0, cmd = 0xAE;
  unsigned long window_ms = 150;
  if (sscanf(args.c_str(), "%u %x %lu", &id, &cmd, &window_ms) < 1 || id > 0x7FF || cmd > 0xFF) {
    Serial.println("[USB_SERIAL] Invalid GIM_PROBE. Usage: GIM_PROBE <id> [cmd_hex] [window_ms]");
    return;
  }
  window_ms = max(10UL, min(window_ms, 1000UL));

  CanRxLock can_lock;
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
  }
  twai_message_t req = {};
  req.identifier = id;
  req.data_length_code = 1;
  req.data[0] = static_cast<uint8_t>(cmd);
  const esp_err_t tx = twai_transmit(&req, pdMS_TO_TICKS(20));
  Serial.printf("[USB_SERIAL] GIM_PROBE id=0x%03X cmd=0x%02X tx=%s, frames from 0x000-0x0FF for %lu ms:\n", id, cmd,
                tx == ESP_OK ? "ok" : esp_err_to_name(tx), window_ms);

  const unsigned long start = millis();
  int shown = 0, other = 0;
  while (millis() - start < window_ms) {
    if (twai_receive(&msg, pdMS_TO_TICKS(5)) != ESP_OK) continue;
    if (msg.identifier > 0xFF) {
      ++other;
      continue;
    }
    if (shown >= 30) continue;
    ++shown;
    char hex[3 * 8 + 1] = "";
    for (int i = 0; i < msg.data_length_code && i < 8; ++i) snprintf(hex + 3 * i, 4, "%02X ", msg.data[i]);
    Serial.printf("[USB_SERIAL]   +%3lums id=0x%03lX dlc=%d %s%s\n", millis() - start,
                  static_cast<unsigned long>(msg.identifier), msg.data_length_code, hex,
                  msg.data[0] == cmd ? "<- reply" : "");
  }
  Serial.printf("[USB_SERIAL] GIM_PROBE done: %d frame(s) shown, %d from other ids\n", shown, other);
}

// Speed output / actuator actions shared by the USB POT/ACT commands and the server
// "io" command (caller holds the nav lock). Each returns nullptr on success, else the
// reason it was rejected (a string literal).
static const char* setSpeedOutput(int ch, int raw) {
#if DS3502_ENABLE
  if (ch < 1 || ch > static_cast<int>(DS3502_COUNT)) return "invalid speed channel";
  if (raw < 0 || raw > Ds3502::kMaxWiper) return "value out of range";
  if (e_stop_active && raw > 0) return "e-stop active (only 0 allowed)";
  Ds3502& pot = pots[ch - 1];
  if (!pot.present()) return "DS3502 not detected on I2C";
  if (!pot.setWiper(static_cast<uint8_t>(raw))) return "I2C write error";
  return nullptr;
#else
  (void)ch;
  (void)raw;
  return "DS3502_ENABLE is 0";
#endif
}

// Sets every present speed output to 0 % (e-stop, mission cancel, server link loss).
static void zeroSpeedOutputs(const char* why) {
#if DS3502_ENABLE
  for (size_t i = 0; i < DS3502_COUNT; ++i) {
    if (pots[i].present()) pots[i].setWiper(0);
  }
  Serial.printf("[DS3502] %s: all speed outputs set to 0%%\n", why);
#else
  (void)why;
#endif
}

static const char* moveActuator(float pct) {
#if ACTUATOR_ENABLE
  if (!(pct >= 0.0f && pct <= 100.0f)) return "position out of range";
  if (e_stop_active) return "e-stop active";
  actuator.setTarget(pct);
  return nullptr;
#else
  (void)pct;
  return "ACTUATOR_ENABLE is 0";
#endif
}

static const char* stopActuator() {
#if ACTUATOR_ENABLE
  actuator.stop();
  return nullptr;
#else
  return "ACTUATOR_ENABLE is 0";
#endif
}

// Queues a motor fault reset for ControlTask. mask: kMotorResetDrive / kMotorResetSteer.
// Rejected during e-stop, since the reset re-enables the motors.
static const char* requestMotorReset(uint8_t mask) {
  if (e_stop_active) return "e-stop active: release it first (release also re-enables the motors)";
  if (motors_released) return "motors released: activate them instead";
  motor_reset_request |= mask;
  return nullptr;
}

// Queues motor release (release = true) or re-activation for ControlTask (caller holds
// the nav lock). Clears any held drive command so the robot doesn't move off it on
// activation.
static const char* requestMotorPower(bool release) {
  if (release) {
    if (mission_ctx.active) return "mission active: cancel it first";
  } else if (e_stop_active) {
    return "e-stop active: release it first";
  }
  control_targets.linear_mps = 0.0f;
  control_targets.angle_deg = 0.0f;
  control_targets.remote_move_expire_ms = 1;
  motor_power_request = release ? MotorPowerRequest::Release : MotorPowerRequest::Activate;
  return nullptr;
}

// Server "io" command: action set_speed (channel 1..N, value 0-100 %),
// actuator_move (value 0-100 %), actuator_stop, motor_reset (channel 0 = all,
// 1 = drive, 2 = steering), motor_release or motor_activate.
static const char* applyIoCommand(const String& action, int channel, float value) {
  if (action == "set_speed") {
    if (!(value >= 0.0f && value <= 100.0f)) return "percent out of range";
#if DS3502_ENABLE
    return setSpeedOutput(channel, lroundf(value * Ds3502::kMaxWiper / 100.0f));
#else
    return setSpeedOutput(channel, 0);
#endif
  }
  if (action == "actuator_move") return moveActuator(value);
  if (action == "actuator_stop") return stopActuator();
  if (action == "motor_reset") {
    if (channel == 0) return requestMotorReset(kMotorResetDrive | kMotorResetSteer);
    if (channel == 1) return requestMotorReset(kMotorResetDrive);
    if (channel == 2) return requestMotorReset(kMotorResetSteer);
    return "motor_reset channel must be 0 (all), 1 (drive) or 2 (steering)";
  }
  if (action == "motor_release") return requestMotorPower(true);
  if (action == "motor_activate") return requestMotorPower(false);
  return "unknown io action";
}

// POT <ch> <0-100>     — set DS3502 <ch> (1..DS3502_COUNT) wiper in percent
// POT <ch> RAW <0-127> — set the wiper in raw steps
// POT?                 — read every wiper back from the chips
static void handlePotCommand(String args) {
#if DS3502_ENABLE
  args.trim();
  if (args == "?") {
    for (size_t i = 0; i < DS3502_COUNT; ++i) {
      const int w = pots[i].present() ? pots[i].readWiper() : -1;
      if (w < 0) {
        Serial.printf("[USB_SERIAL] POT %u (0x%02X): %s\n", static_cast<unsigned>(i + 1), kDs3502Addresses[i],
                      pots[i].present() ? "I2C read error" : "not detected");
      } else {
        Serial.printf("[USB_SERIAL] POT %u (0x%02X): wiper=%d/%u (%.1f%%)\n", static_cast<unsigned>(i + 1),
                      kDs3502Addresses[i], w, Ds3502::kMaxWiper, w * 100.0f / Ds3502::kMaxWiper);
      }
    }
    return;
  }

  int ch = 0;
  char mode_buf[8] = {0};
  char value_buf[16] = {0};
  int raw = -1;
  const int parsed = sscanf(args.c_str(), "%d %7s %15s", &ch, mode_buf, value_buf);
  char* end = nullptr;
  if (parsed == 2) {
    const float pct = strtof(mode_buf, &end);
    if (*end == '\0' && pct >= 0.0f && pct <= 100.0f) raw = lroundf(pct * Ds3502::kMaxWiper / 100.0f);
  } else if (parsed == 3 && strcmp(mode_buf, "RAW") == 0) {
    const long n = strtol(value_buf, &end, 10);
    if (*end == '\0' && n >= 0 && n <= Ds3502::kMaxWiper) raw = static_cast<int>(n);
  }
  if (ch < 1 || ch > static_cast<int>(DS3502_COUNT) || raw < 0) {
    Serial.printf("[USB_SERIAL] Invalid POT. Usage: POT <1-%d> <0-100>, POT <1-%d> RAW <0-127>, POT?\n",
                  DS3502_COUNT, DS3502_COUNT);
    return;
  }
  if (const char* err = setSpeedOutput(ch, raw)) {
    Serial.printf("[USB_SERIAL] POT %d rejected: %s\n", ch, err);
    return;
  }
  Serial.printf("[USB_SERIAL] POT %d set wiper=%d/%u (%.1f%%)\n", ch, raw, Ds3502::kMaxWiper, raw * 100.0f / Ds3502::kMaxWiper);
#else
  (void)args;
  Serial.println("[USB_SERIAL] POT rejected: DS3502_ENABLE is 0");
#endif
}

#if PCF8574_ENABLE
static const char* const kInputNames[DIGITAL_INPUT_COUNT] = DIGITAL_INPUT_NAMES;
#endif

// Latched e-stop from NavTask (nav lock held). ControlTask stops the motors on its next
// cycle; release is PS2 L1 + L2 + Circle, as for the other e-stop sources. reason must outlive the
// e-stop (string literal or static buffer).
static void triggerEStop(const char* reason) {
  if (e_stop_active) return;
  e_stop_active = true;
  e_stop_reason = reason;
  if (mission_ctx.active) {
    mission_ctx.active = false;
    setMissionState(MissionState::EStop);
  }
  resetMissionCommandTargets();
  Serial.printf("!!! E-STOP ACTIVATED: %s !!!\n", reason);
}

static void handleInputQuery() {
#if PCF8574_ENABLE
  if (!digital_inputs_ok) {
    Serial.printf("[USB_SERIAL] IN: PCF8574 0x%02X %s\n", PCF8574_I2C_ADDR,
                  input_expander.present() ? "read error" : "not detected");
    return;
  }
  Serial.print("[USB_SERIAL] IN");
  for (int i = 0; i < DIGITAL_INPUT_COUNT; ++i) {
    Serial.printf(" %d=%d", i + 1, (digital_inputs >> i) & 1);
  }
  Serial.printf(" raw=0x%02X", digital_inputs_raw);
  for (int i = 0; i < DIGITAL_INPUT_COUNT; ++i) {
    if ((digital_inputs >> i) & 1) Serial.printf(" [%s active]", kInputNames[i]);
  }
  Serial.println();
#else
  Serial.println("[USB_SERIAL] IN rejected: PCF8574_ENABLE is 0");
#endif
}

// ACT <0-100>                — move the linear actuator to percent of stroke
// ACT STOP                   — stop it
// ACT?                       — position, raw ADC (for calibration), state
// ACT JOG <-100..100> <ms>   — open-loop drive (+ = extend) for calibration
static void handleActuatorCommand(String args) {
#if ACTUATOR_ENABLE
  args.trim();
  if (args == "?") {
    Serial.printf("[USB_SERIAL] ACT pos=%.1f%% raw=%d target=%.1f%% state=%s fault=%s\n",
                  actuator.positionPct(), actuator.rawAdc(), actuator.targetPct(),
                  LinearActuator::stateName(actuator.state()), LinearActuator::faultName(actuator.fault()));
    return;
  }
  if (args == "STOP") {
    stopActuator();
    Serial.println("[USB_SERIAL] ACT stopped");
    return;
  }
  if (e_stop_active) {
    Serial.println("[USB_SERIAL] ACT rejected: e-stop active");
    return;
  }
  char* end = nullptr;
  if (args.startsWith("JOG")) {
    float duty = 0.0f;
    unsigned long ms = 0;
    String v = args.substring(3);
    if (sscanf(v.c_str(), "%f %lu", &duty, &ms) != 2 || duty < -100.0f || duty > 100.0f || ms == 0 || ms > 5000) {
      Serial.println("[USB_SERIAL] Invalid ACT JOG. Usage: ACT JOG <-100..100> <1-5000 ms>");
      return;
    }
    actuator.jog(duty, ms);
    Serial.printf("[USB_SERIAL] ACT jog %+.0f%% for %lu ms\n", duty, ms);
    return;
  }
  const float pct = strtof(args.c_str(), &end);
  if (args.length() == 0 || *end != '\0' || pct < 0.0f || pct > 100.0f) {
    Serial.println("[USB_SERIAL] Invalid ACT. Usage: ACT <0-100>, ACT STOP, ACT?, ACT JOG <-100..100> <ms>");
    return;
  }
  moveActuator(pct);
  Serial.printf("[USB_SERIAL] ACT moving to %.1f%% (now %.1f%%)\n", pct, actuator.positionPct());
#else
  (void)args;
  Serial.println("[USB_SERIAL] ACT rejected: ACTUATOR_ENABLE is 0");
#endif
}

#if PCF8574_ENABLE && LIMIT_SWITCH_ESTOP_ENABLE
// Edge-triggered: a switch e-stops once when it becomes active (debounced). It must
// clear before it can trigger again, so after a PS2 release the robot can be driven
// off a switch that is still pressed. A missing or silent PCF8574 also e-stops.
static void updateLimitSwitches(bool polled) {
  static uint8_t active_polls[DIGITAL_INPUT_COUNT] = {};
  static uint8_t reported_mask = 0;
  static uint8_t read_fail_polls = 0;
  static bool expander_fault_reported = false;
  static char reason[40];

  if (!input_expander.present()) {
    if (!expander_fault_reported) {
      expander_fault_reported = true;
      triggerEStop("limit: PCF8574 not detected");
    }
    return;
  }
  if (!polled) return;

  if (!digital_inputs_ok) {
    if (read_fail_polls < 255) ++read_fail_polls;
    if (read_fail_polls >= LIMIT_SWITCH_READ_FAIL_POLLS && !expander_fault_reported) {
      expander_fault_reported = true;
      triggerEStop("limit: PCF8574 read error");
    }
    return;
  }
  read_fail_polls = 0;
  expander_fault_reported = false;

  for (int i = 0; i < DIGITAL_INPUT_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if (!(digital_inputs & bit)) {
      active_polls[i] = 0;
      reported_mask &= static_cast<uint8_t>(~bit);
      continue;
    }
    if (active_polls[i] < 255) ++active_polls[i];
    if (active_polls[i] >= LIMIT_SWITCH_DEBOUNCE_POLLS && !(reported_mask & bit)) {
      reported_mask |= bit;
      snprintf(reason, sizeof(reason), "limit: %s", kInputNames[i]);
      triggerEStop(reason);
    }
  }
}
#endif

// Polls the PCF8574 inputs, checks the limit switches, applies e-stop to the pots and
// runs the actuator position loop (NavTask, nav lock held).
static void updateIoDevices(unsigned long now) {
#if PCF8574_ENABLE
  static unsigned long last_input_poll_ms = 0;
  bool polled = false;
  if (input_expander.present() && now - last_input_poll_ms >= DIGITAL_INPUT_POLL_MS) {
    last_input_poll_ms = now;
    polled = true;
    uint8_t levels = 0;
    digital_inputs_ok = input_expander.read(levels);
    if (digital_inputs_ok) {
      digital_inputs_raw = levels;
      const uint8_t mask = static_cast<uint8_t>((1u << DIGITAL_INPUT_COUNT) - 1);
      digital_inputs = (DIGITAL_INPUT_ACTIVE_LOW ? ~levels : levels) & mask;
    }
  }
#if LIMIT_SWITCH_ESTOP_ENABLE
  updateLimitSwitches(polled);
#else
  (void)polled;
#endif
#endif

  // On e-stop every speed output (Speed1-3) goes to 0 %; the actuator just stops.
  static bool prev_e_stop = false;
  if (e_stop_active && !prev_e_stop) zeroSpeedOutputs("e-stop");
  prev_e_stop = e_stop_active;

  // Same when the server link drops (after MISSION_LINK_LOSS_HOLD_MS). Starts as "lost"
  // so running without a server (USB serial only) never zeroes the outputs.
  static bool prev_link_lost = true;
  const bool link_lost = serverLinkLost(now);
  if (link_lost && !prev_link_lost) zeroSpeedOutputs("server link lost");
  prev_link_lost = link_lost;

#if ACTUATOR_ENABLE
  if (e_stop_active && (actuator.state() == LinearActuator::State::Moving ||
                        actuator.state() == LinearActuator::State::Jogging)) {
    actuator.stop();
    Serial.println("[ACT] stopped by e-stop");
  }
  actuator.update(now);
#endif
}

// MOTOR?                     — last polled status of every motor driver
// MOTOR RESET [ALL|DRIVE|STEER] — clear latched faults and re-enable (default ALL)
// MOTOR RELEASE / MOTOR ACTIVATE — unpower all motors to save energy / power them again
static void handleMotorCommand(String args) {
  args.trim();
  if (args == "RELEASE" || args == "ACTIVATE") {
    if (const char* err = requestMotorPower(args == "RELEASE")) {
      Serial.printf("[USB_SERIAL] MOTOR %s rejected: %s\n", args.c_str(), err);
    } else {
      Serial.printf("[USB_SERIAL] MOTOR %s queued\n", args.c_str());
    }
    return;
  }
  if (args == "?") {
    const unsigned long now = millis();
    Serial.printf("[USB_SERIAL] MOTOR power: %s\n", motors_released ? "RELEASED" : "active");
    for (int i = 0; i < kMotorCount; ++i) {
      const MotorStatus& m = motor_status[i];
      if (!m.online) {
        Serial.printf("[USB_SERIAL] MOTOR %s: offline (no CAN reply)\n", kMotorNames[i]);
        continue;
      }
      Serial.printf("[USB_SERIAL] MOTOR %s: %s %s code=0x%08lX V=%.1f T=%.1fC age=%lums%s%s\n", kMotorNames[i],
                    m.enabled ? "enabled" : "disabled", m.fault ? "FAULT" : "ok",
                    static_cast<unsigned long>(m.fault_code), m.voltage_v, m.temp_c, now - m.updated_ms,
                    m.fault_text[0] ? " " : "", m.fault_text);
    }
    return;
  }

  uint8_t mask = 0;
  if (args == "RESET" || args == "RESET ALL") mask = kMotorResetDrive | kMotorResetSteer;
  else if (args == "RESET DRIVE") mask = kMotorResetDrive;
  else if (args == "RESET STEER") mask = kMotorResetSteer;
  if (!mask) {
    Serial.println("[USB_SERIAL] Invalid MOTOR. Usage: MOTOR?, MOTOR RESET [ALL|DRIVE|STEER], MOTOR RELEASE, MOTOR ACTIVATE");
    return;
  }
  if (const char* err = requestMotorReset(mask)) {
    Serial.printf("[USB_SERIAL] MOTOR RESET rejected: %s\n", err);
    return;
  }
  Serial.println("[USB_SERIAL] MOTOR RESET queued");
}

static void handleUsbSerialLine(const String& raw_line) {
  String line = raw_line;
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

  if (upper == "HELP" || upper == "?") {
    Serial.println("[USB_SERIAL] Commands: CMD_VEL <linear_mps> <angular_rad_s> [timeout_ms], STOP, MODE <NORMAL|ZERO_TURN>, STATUS?, MISSION_PAUSE, MISSION_RESUME, NAV ON|OFF, HEADING MAG|GPS, HEADING?, FUNC_<n> ON|OFF|<value> <DEG|REL_DEG>, POT <ch> <0-100>|<ch> RAW <0-127>|?, IN?, ACT <0-100>|STOP|?|JOG <duty> <ms>, MOTOR?|RESET [ALL|DRIVE|STEER]|RELEASE|ACTIVATE, CAN_PING <node_id> [timeout_ms], GIM_PROBE <id> [cmd_hex] [window_ms]");
    return;
  }

  if (upper.startsWith("CAN_PING")) {
    String args = line.substring(8);
    args.trim();
    handleCanPing(args);
    return;
  }

  if (upper.startsWith("GIM_PROBE")) {
    handleGimProbe(line.substring(9));
    return;
  }

  if (upper.startsWith("FUNC_")) {
    int sep = line.indexOf(' ');
    String func_tok = (sep > 0) ? upper.substring(0, sep) : upper;
    String args = (sep > 0) ? line.substring(sep + 1) : "";
    args.trim();

    int idx = func_tok.substring(5).toInt();

    String args_upper = args;
    args_upper.toUpperCase();
    if (args_upper == "ON" || args_upper == "OFF") {
      setFuncGpio(idx, args_upper == "ON");
      return;
    }

    float value = 0.0f;
    char unit_buf[8] = {0};
    int parsed = sscanf(args.c_str(), "%f %7s", &value, unit_buf);
    if (parsed < 2) {
      Serial.println("[USB_SERIAL] Invalid FUNC command. Usage: FUNC_<n> ON|OFF, or FUNC_<n> <value> <DEG|REL_DEG>");
      return;
    }
    String unit = String(unit_buf);
    unit.toUpperCase();

    triggerFunc(idx, value, unit);
    return;
  }

  if (upper == "POT" || upper.startsWith("POT ") || upper == "POT?") {
    handlePotCommand(upper.substring(3));
    return;
  }

  if (upper == "IN?") {
    handleInputQuery();
    return;
  }

  if (upper == "ACT" || upper.startsWith("ACT ") || upper == "ACT?") {
    handleActuatorCommand(upper.substring(3));
    return;
  }

  if (upper == "MOTOR?" || upper.startsWith("MOTOR ")) {
    handleMotorCommand(upper.substring(5));
    return;
  }

  if (upper == "STOP") {
    control_targets.linear_mps = 0.0f;
    control_targets.angle_deg = 0.0f;
    control_targets.remote_move_expire_ms = millis() + 150;
    // During a mission STOP must also hold the mission off, or it would drive again
    // on the next NavTask pass. Repeat STOP (or CMD_VEL 0 0) to keep it stopped.
    serial_override_until_ms = millis() + USB_CMD_VEL_DEFAULT_TIMEOUT_MS;
    Serial.println("[USB_SERIAL] STOP accepted");
    return;
  }

  if (upper == "MISSION_PAUSE") {
    if (mission_ctx.active && mission_ctx.state == MissionState::Running) {
      setMissionState(MissionState::Paused);
      resetMissionCommandTargets();
      Serial.println("[USB_SERIAL] MISSION_PAUSE accepted");
    } else {
      Serial.println("[USB_SERIAL] MISSION_PAUSE rejected: no running mission");
    }
    return;
  }

  if (upper == "MISSION_RESUME") {
    if (motors_released) {
      Serial.println("[USB_SERIAL] MISSION_RESUME rejected: motors released (MOTOR ACTIVATE first)");
    } else if (mission_ctx.active && mission_ctx.state == MissionState::Paused) {
      setMissionState(MissionState::Running);
      Serial.println("[USB_SERIAL] MISSION_RESUME accepted");
    } else {
      Serial.println("[USB_SERIAL] MISSION_RESUME rejected: no paused mission");
    }
    return;
  }

  if (upper == "HEADING MAG" || upper == "HEADING GPS") {
    heading_source = (upper == "HEADING GPS") ? HeadingSource::Gps : HeadingSource::Mag;
    Serial.printf("[USB_SERIAL] HEADING source %s\n", heading_source == HeadingSource::Gps ? "GPS" : "MAG");
    return;
  }

  if (upper == "HEADING?") {
    Serial.printf("[USB_SERIAL] heading src=%s heading=%.1f mag=%.1f offset=%.1f corrections=%lu fused_err=%.1f mag_err=%.1f\n",
                  heading_source == HeadingSource::Gps ? "GPS" : "MAG", estimateHeadingDeg(),
                  imu_state.valid ? magHeadingDeg() : 0.0f, heading_fusion.offset_deg,
                  (unsigned long)heading_fusion.corrections, heading_fusion.last_gps_err_deg,
                  heading_fusion.last_mag_err_deg);
    return;
  }

  if (upper == "NAV ON" || upper == "NAV OFF") {
    nav_serial_output = (upper == "NAV ON");
    Serial.printf("[USB_SERIAL] NAV output %s\n", nav_serial_output ? "ON" : "OFF");
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
    Serial.printf("[USB_SERIAL] status mode=%s e_stop=%d (%s) motors_released=%d mission=%s remote_active=%d linear=%.3f angle=%.2f timeout_left_ms=%lu can_tx_ok=%lu can_tx_drop=%lu\n",
                  mode_str,
                  e_stop_active ? 1 : 0,
                  e_stop_active ? e_stop_reason : "-",
                  motors_released ? 1 : 0,
                  missionStateToString(mission_ctx.state),
                  remote_active ? 1 : 0,
                  control_targets.linear_mps,
                  control_targets.angle_deg,
                  remaining_ms,
                  loop_stats.can_runtime_tx_ok_count,
                  loop_stats.can_runtime_tx_drop_count);
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
      // Override the mission instead of rejecting. Never indefinitely: an infinite
      // command (timeout 0) would stop the mission forever if the sender died.
      if (timeout_ms == 0) timeout_ms = USB_CMD_VEL_DEFAULT_TIMEOUT_MS;
      const unsigned long t = max(USB_CMD_VEL_MIN_TIMEOUT_MS, min(timeout_ms, USB_CMD_VEL_MAX_TIMEOUT_MS));
      serial_override_until_ms = millis() + t;
    }

    // change direction form this //
    applyUsbVelocityCommand(linear_mps, -angular_rads, timeout_ms);
    return;
  }

  Serial.printf("[USB_SERIAL] Unknown command: %s\n", line.c_str());
}

// Serial commands change mode, mission and actuator state, so they run under the
// nav lock — except CAN_PING / GIM_PROBE, bounded blocking diagnostics that touch none of it.
static void handleUsbSerialLineLocked(const String& line) {
  String upper = line;
  upper.trim();
  upper.toUpperCase();
  if (upper.startsWith("CAN_PING") || upper.startsWith("GIM_PROBE")) {
    handleUsbSerialLine(line);
    return;
  }
  NavLock lock;
  handleUsbSerialLine(line);
}

static void handleUsbSerialControl() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') continue;

    if (c == '\n') {
      if (usb_serial_line_overflow) {
        Serial.println("[USB_SERIAL] Ignored line: too long");
      } else if (usb_serial_line_buffer.length() > 0) {
        handleUsbSerialLineLocked(usb_serial_line_buffer);
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

// Compass-convention yaw (deg, unwrapped) from a BNO08x quaternion.
static float quatCompassYawDeg(float qw, float qx, float qy, float qz) {
  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  //float yaw_deg = DoubleAckermannSteering::rad2deg(atan2f(siny_cosp, cosy_cosp));
  return DoubleAckermannSteering::rad2deg(atan2f(cosy_cosp, siny_cosp));
}

static float wrap360(float deg) {
  deg = fmodf(deg, 360.0f);
  return deg < 0.0f ? deg + 360.0f : deg;
}

static float magHeadingDeg() {
  return wrap360(quatCompassYawDeg(imu_state.qw, imu_state.qx, imu_state.qy, imu_state.qz) + HEADING_OFFSET_DEG);
}

static float gameYawDeg() {
  return quatCompassYawDeg(imu_state.gqw, imu_state.gqx, imu_state.gqy, imu_state.gqz);
}

static float estimateHeadingDeg() {
  if (!imu_state.valid) return 0.0f;
  if (heading_source == HeadingSource::Gps && imu_state.game_valid && heading_fusion.offset_valid) {
    return wrap360(gameYawDeg() + heading_fusion.offset_deg);
  }
  return magHeadingDeg();
}

// NavTask: learn the game-yaw offset from GPS course over ground. On each new GPS
// epoch while driving forward in normal mode on RTK fixed (GST error within
// HEADING_GPS_MAX_HERR_M): once HEADING_GPS_MIN_BASELINE_M has been covered with less
// than HEADING_GPS_MAX_YAW_CHANGE_DEG of turning, the bearing of that segment is the
// true heading; nudge the offset toward it by HEADING_GPS_GAIN.
static void updateHeadingFusion() {
  HeadingFusion& hf = heading_fusion;
  if (!imu_state.valid || !imu_state.rv_valid || !imu_state.game_valid) return;
  const float game_deg = gameYawDeg();
  if (!hf.offset_valid) {
    hf.offset_deg = normalizeAngleDeg(magHeadingDeg() - game_deg);
    hf.offset_valid = true;
  }

  if (gps_fix.last_update_ms == hf.last_gps_update_ms) return;  // once per GPS epoch
  hf.last_gps_update_ms = gps_fix.last_update_ms;

  const bool usable = gps_fix.valid && gps_fix.fix_quality == 4 && gps_fix.has_gst &&
                      gps_fix.h_err_m <= HEADING_GPS_MAX_HERR_M &&
                      robot_mode == RobotMode::Normal && control_targets.linear_mps > 0.05f;
  if (!usable) {
    hf.anchor_valid = false;
    return;
  }
  const bool turned = hf.anchor_valid &&
                      fabsf(normalizeAngleDeg(game_deg - hf.anchor_game_deg)) > HEADING_GPS_MAX_YAW_CHANGE_DEG;
  if (!hf.anchor_valid || turned) {
    hf.anchor_valid = true;
    hf.anchor_lat = gps_fix.latitude;
    hf.anchor_lon = gps_fix.longitude;
    hf.anchor_game_deg = game_deg;
    return;
  }
  if (distanceMeters(hf.anchor_lat, hf.anchor_lon, gps_fix.latitude, gps_fix.longitude) < HEADING_GPS_MIN_BASELINE_M) {
    return;
  }

  const float course_deg = bearingDeg(hf.anchor_lat, hf.anchor_lon, gps_fix.latitude, gps_fix.longitude);
  const float mid_game_deg = hf.anchor_game_deg + 0.5f * normalizeAngleDeg(game_deg - hf.anchor_game_deg);
  const float err_deg = normalizeAngleDeg(course_deg - (mid_game_deg + hf.offset_deg));
  hf.offset_deg = normalizeAngleDeg(hf.offset_deg + HEADING_GPS_GAIN * err_deg);
  hf.last_gps_err_deg = err_deg;
  hf.last_mag_err_deg = normalizeAngleDeg(course_deg - magHeadingDeg());
  ++hf.corrections;

  hf.anchor_lat = gps_fix.latitude;
  hf.anchor_lon = gps_fix.longitude;
  hf.anchor_game_deg = game_deg;
}

static void handleRobotClientCommand() {
#if !CONTROL_SERVER_ENABLE
  return;
#endif
  if (!robot_client.hasPendingCommand()) return;

  RobotCommand command = robot_client.consumeCommand();
  // Apply under the nav lock, ack after releasing it: on SIM an ack is a blocking
  // modem round-trip and must not stall NavTask.
  const char* ack_status = nullptr;
  const char* ack_detail = nullptr;
  {
    NavLock lock;
    // Released motors can't drive: refuse anything that would start or resume a mission.
    const bool starts_mission = command.type == RobotCommandType::ExecuteMission ||
                                command.type == RobotCommandType::Waypoint ||
                                command.type == RobotCommandType::MissionUploadExecute ||
                                (command.type == RobotCommandType::PauseMission && mission_ctx.active &&
                                 mission_ctx.state == MissionState::Paused);
    if (motors_released && starts_mission) {
      ack_status = "rejected";
      ack_detail = "motors released: activate them first";
    } else switch (command.type) {
      case RobotCommandType::Move:
        control_targets.linear_mps = command.linear_velocity;
        control_targets.angle_deg = convertAngularToSteeringDeg(command.linear_velocity, command.angular_velocity, WHEELBASE_M, MAX_STEER_DEG);
        control_targets.remote_move_expire_ms = millis() + static_cast<unsigned long>(command.duration * 1000.0f);
        ack_status = "accepted";
#if DEBUG_LOG_ROBOT_CLIENT
        Serial.println("[ROBOT_CLIENT] Move command accepted");
#endif
        break;
      case RobotCommandType::EmergencyStop:
        e_stop_active = true;
        e_stop_reason = "server";
        drive_motors.emergencyStop();
        steering_motors.emergencyStop();
        mission_ctx.active = false;
        setMissionState(MissionState::EStop);
        ack_status = "accepted";
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
          ack_status = "accepted";
          ack_detail = "mission plan loaded";
#if DEBUG_LOG_ROBOT_CLIENT
          Serial.println("[ROBOT_CLIENT] Mission plan received and queued");
#endif
        } else {
          mission_ctx.active = false;
          mission_ctx.phase = 0;
          mission_ctx.waypoint_index = -1;
          resetMissionCommandTargets();
          setMissionState(MissionState::Failed);
          ack_status = "rejected";
          ack_detail = "invalid mission payload";
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
          ack_status = "accepted";
        } else {
          ack_status = "rejected";
          ack_detail = "no active mission";
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
        zeroSpeedOutputs("mission cancelled");
        ack_status = "accepted";
        break;
      case RobotCommandType::Io:
        ack_detail = applyIoCommand(command.io_action, command.io_channel, command.io_value);
        ack_status = ack_detail ? "rejected" : "accepted";
        break;
      case RobotCommandType::Waypoint:
        loadSingleWaypointMission(command.latitude, command.longitude, command.altitude);
        mission_ctx.active = true;
        mission_ctx.name = "single_waypoint";
        mission_ctx.phase = 0;
        mission_ctx.waypoint_index = 0;
        mission_ctx.started_ms = millis();
        setMissionState(MissionState::Queued);
        ack_status = "accepted";
        ack_detail = "single waypoint mission queued";
        break;
      case RobotCommandType::CancelTask:
        mission_ctx.active = false;
        mission_ctx.name = "";
        mission_ctx.phase = 0;
        mission_ctx.waypoint_index = -1;
        clearMissionPlan();
        resetMissionCommandTargets();
        setMissionState(MissionState::Cancelled);
        zeroSpeedOutputs("mission cancelled");
        ack_status = "accepted";
        break;
      case RobotCommandType::MissionUploadStart: {
        // Parse tasks now; waypoints arrive in subsequent BATCH messages
        String wrapped = "{\"waypoints\":[],\"tasks\":";
        wrapped += command.mission_json;
        wrapped += "}";
        parseMissionPlan(wrapped);  // clears arrays, fills mission_tasks[]
        upload_ctx.active           = true;
        upload_ctx.session_id       = command.upload_session_id;
        upload_ctx.expected_total   = (size_t)command.upload_total_waypoints;
        upload_ctx.expected_batches = (size_t)command.upload_total_batches;
        upload_ctx.received_count   = 0;
        upload_ctx.received_batches = 0;
        Serial.printf("[UPLOAD] Start session=%s total=%u batches=%u tasks=%u\n",
          upload_ctx.session_id.c_str(),
          (unsigned)upload_ctx.expected_total,
          (unsigned)upload_ctx.expected_batches,
          (unsigned)mission_task_count);
        ack_status = "ok";
        break;
      }
      case RobotCommandType::MissionUploadBatch: {
        if (!upload_ctx.active || command.upload_session_id != upload_ctx.session_id) {
          ack_status = "error";
          ack_detail = "no_active_session";
          break;
        }
        JsonDocument doc;
        deserializeJson(doc, command.mission_json);
        JsonArrayConst wps = doc.as<JsonArrayConst>();
        size_t offset = (size_t)command.upload_offset;
        for (JsonVariantConst wp_var : wps) {
          if (offset >= MISSION_MAX_WAYPOINTS) break;
          JsonObjectConst wp = wp_var.as<JsonObjectConst>();
          mission_waypoints[offset++] = {wp["lat"] | 0.0, wp["lon"] | 0.0, wp["alt"] | 0.0};
        }
        upload_ctx.received_count = offset;
        upload_ctx.received_batches++;
        Serial.printf("[UPLOAD] Batch %d: %u/%u wps\n",
          command.upload_batch_index,
          (unsigned)upload_ctx.received_count,
          (unsigned)upload_ctx.expected_total);
        ack_status = "ok";
        break;
      }
      case RobotCommandType::MissionUploadExecute: {
        if (!upload_ctx.active || command.upload_session_id != upload_ctx.session_id) {
          ack_status = "error";
          ack_detail = "session_mismatch";
          break;
        }
        size_t expected = (size_t)command.upload_total_waypoints;
        if (upload_ctx.received_count != expected) {
          Serial.printf("[UPLOAD] Execute failed: got=%u expected=%u\n",
            (unsigned)upload_ctx.received_count, (unsigned)expected);
          ack_status = "error";
          ack_detail = "waypoint_count_mismatch";
          upload_ctx.active = false;
          break;
        }
        mission_waypoint_count = upload_ctx.received_count;
        upload_ctx.active = false;
        mission_ctx.active = true;
        mission_ctx.name = "";
        mission_ctx.phase = 0;
        mission_ctx.waypoint_index = -1;
        mission_ctx.started_ms = millis();
        setMissionState(MissionState::Queued);
        Serial.printf("[UPLOAD] Execute: %u wps %u tasks — QUEUED\n",
          (unsigned)mission_waypoint_count, (unsigned)mission_task_count);
        ack_status = "accepted";
        break;
      }
      case RobotCommandType::None:
        break;
    }
  }
  if (ack_status) robot_client.sendCommandAck(command, ack_status, ack_detail);
}

static void publishRobotTelemetry() {
#if !CONTROL_SERVER_ENABLE
  return;
#endif

  static unsigned long last_telemetry_publish_ms = 0;
  const unsigned long now = millis();
  // SIM cellular link: each AT+CIPSEND round-trip takes 200-500ms, so 500ms
  // WiFi interval floods the link and leaves no time to receive server messages.
  const bool on_sim = (net_manager.source() == NetSource::SIM);
  const unsigned long telemetry_interval_ms = on_sim
      ? max(20UL, static_cast<unsigned long>(SIM_TELEMETRY_INTERVAL_MS))
      : max(20UL, static_cast<unsigned long>(ROBOT_TELEMETRY_INTERVAL_MS));
  if (last_telemetry_publish_ms != 0 && (now - last_telemetry_publish_ms) < telemetry_interval_ms) {
    return;
  }
  last_telemetry_publish_ms = now;

  RobotTelemetry telemetry;
  {
    // Copy shared state under the nav lock; the send below can block on SIM.
    NavLock lock;
    telemetry.battery_level = 100.0f;
    telemetry.e_stop = e_stop_active;
    telemetry.e_stop_reason = e_stop_reason;
    telemetry.motors_released = motors_released;
    telemetry.speed = control_targets.linear_mps;
    telemetry.heading = estimateHeadingDeg();

    telemetry.gps_valid = gps_fix.valid;
    telemetry.gps_latitude = gps_fix.latitude;
    telemetry.gps_longitude = gps_fix.longitude;
    telemetry.gps_altitude = gps_fix.altitude;
    telemetry.gps_hdop = gps_fix.hdop;
    telemetry.gps_fix_quality = gps_fix.fix_quality;
    telemetry.gps_satellites = gps_fix.satellites;
    // GST arrives every epoch; treat it as unknown if it stopped (e.g. config NAK), and
    // without a fix, where the receiver reports a meaningless huge error.
    telemetry.gps_h_err_m = (gps_fix.valid && gps_fix.fix_quality > 0 && gps_fix.has_gst &&
                             now - gps_fix.last_gst_ms < GPS_STALE_TIMEOUT_MS)
                                ? gps_fix.h_err_m : -1.0f;

    telemetry.mission_active = mission_ctx.active && (mission_ctx.state == MissionState::Queued || mission_ctx.state == MissionState::Running || mission_ctx.state == MissionState::Paused);
    telemetry.mission_name = mission_ctx.name;
    telemetry.mission_state = missionStateToString(mission_ctx.state);
    telemetry.mission_waypoint_index = mission_ctx.waypoint_index;
    telemetry.mission_waypoint_total = static_cast<int>(mission_waypoint_count);
    telemetry.mission_holding = mission_ctx.active && mission_ctx.state == MissionState::Running &&
                                (mission_gps_hold_since_ms != 0 || mission_link_hold_since_ms != 0);
#if DS3502_ENABLE
    telemetry.pot_count = min(static_cast<int>(DS3502_COUNT), RobotTelemetry::kMaxPots);
    for (int i = 0; i < telemetry.pot_count; ++i) {
      telemetry.pot_present[i] = pots[i].present();
      telemetry.pot_wiper[i] = pots[i].wiper();
    }
#endif
#if PCF8574_ENABLE
    telemetry.input_count = DIGITAL_INPUT_COUNT;
    telemetry.inputs_ok = digital_inputs_ok;
    telemetry.inputs = digital_inputs;
#endif
#if ACTUATOR_ENABLE
    telemetry.actuator_enabled = true;
    telemetry.actuator_position_pct = actuator.positionPct();
    telemetry.actuator_target_pct = actuator.targetPct();
    telemetry.actuator_state = LinearActuator::stateName(actuator.state());
    telemetry.actuator_fault = LinearActuator::faultName(actuator.fault());
#endif
    telemetry.motor_count = kMotorCount;
    for (int i = 0; i < kMotorCount; ++i) {
      telemetry.motor_names[i] = kMotorNames[i];
      telemetry.motors[i] = motor_status[i];
    }
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
  }

#if GPS_SERIAL_OUTPUT_ENABLE
  {
    JsonDocument gps_doc;
    gps_doc["type"] = "gps";
    gps_doc["timestamp"] = now;
    JsonObject gps_out = gps_doc["gps"].to<JsonObject>();
    gps_out["valid"] = telemetry.gps_valid;
    gps_out["latitude"] = telemetry.gps_latitude;
    gps_out["longitude"] = telemetry.gps_longitude;
    gps_out["altitude"] = telemetry.gps_altitude;
    gps_out["hdop"] = telemetry.gps_hdop;
    gps_out["fix_quality"] = telemetry.gps_fix_quality;
    gps_out["satellites"] = telemetry.gps_satellites;
    String gps_json;
    serializeJson(gps_doc, gps_json);
    Serial.println(gps_json);
  }
#endif

  robot_client.sendTelemetry(telemetry);
}

static void applyModeInit(RobotMode mode) {
  if (motors_released) {
    // The drive loop sets the mode's steering once the motors are activated.
    Serial.println("Mode changed while motors released: steering not moved");
    return;
  }
  if (mode == RobotMode::Normal) {
    Serial.println("Enter Mode 0 : normal mode");
    steering_motors.setSteeringTurns(0.0f, 0.0f);
  } else {
    const float zero_turn_angle_deg = ControlActuation::computeZeroTurnSteeringAngleDeg(WHEELBASE_M, TRACK_WIDTH_M);
    Serial.print("WHEELBASE_M, TRACK_WIDTH_M");
    Serial.println(WHEELBASE_M, TRACK_WIDTH_M);
    Serial.print("zero turn angle deg: ");
    Serial.println(zero_turn_angle_deg);

    const float zero_turn_turns = ControlActuation::steeringAngleDegToTurns(zero_turn_angle_deg);
    Serial.println("Enter Mode 1 : zero turn mode");
    Serial.print("zero turn angle deg: ");
    Serial.println(zero_turn_angle_deg);
    steering_motors.setSteeringTurns(zero_turn_turns, -zero_turn_turns);
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
  Serial.println("[USB_SERIAL] Ready. Commands: CMD_VEL <linear_mps> <angular_rad_s> [timeout_ms], STOP, MODE <NORMAL|ZERO_TURN>, STATUS?, MISSION_PAUSE, MISSION_RESUME, NAV ON|OFF, HEADING MAG|GPS, HEADING?, FUNC_<n> ON|OFF|<value> <DEG|REL_DEG>, POT <ch> <0-100>|<ch> RAW <0-127>|?, IN?, ACT <0-100>|STOP|?|JOG <duty> <ms>, MOTOR?|RESET [ALL|DRIVE|STEER]|RELEASE|ACTIVATE, CAN_PING <node_id> [timeout_ms], GIM_PROBE <id> [cmd_hex] [window_ms]");
  Serial.println("[USB_SERIAL]   timeout_ms=0 for infinite timeout (command persists until new command)");
  Serial.println("[USB_SERIAL]   FUNC_<n> ON|OFF drives a GPIO actuator directly; FUNC_<n> <degrees> DEG moves a CAN actuator then returns it to 0");
  Serial.println("[USB_SERIAL]   FUNC_<n> <degrees> REL_DEG moves a CAN actuator relative to its current position and just holds there");
  Serial.println("[USB_SERIAL]   CAN_PING <node_id> [timeout_ms] listens for any CAN frame from that node id (diagnostic)");

  for (uint8_t pin : kOutputPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

#if ACTUATOR_ENABLE
  // Early, so the H-bridge inputs are driven LOW (motor off) before anything slow runs.
  actuator.begin();
  Serial.printf("[ACT] VNH2SP30 INA=%d INB=%d PWM=%d, feedback GPIO%d raw=%d (%.1f%%)\n",
                ACT_INA_PIN, ACT_INB_PIN, ACT_PWM_PIN, ACT_POS_PIN, actuator.rawAdc(), actuator.positionPct());
#endif

#if DS3502_ENABLE || PCF8574_ENABLE
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
#endif
#if DS3502_ENABLE
  {
    const uint8_t boot_wiper = static_cast<uint8_t>(lroundf(clampf(DS3502_BOOT_PERCENT, 0.0f, 100.0f) * Ds3502::kMaxWiper / 100.0f));
    for (size_t i = 0; i < DS3502_COUNT; ++i) {
      if (pots[i].begin(kDs3502Addresses[i])) {
        pots[i].setWiper(boot_wiper);
        Serial.printf("[DS3502] POT %u found at 0x%02X, wiper=%u\n", static_cast<unsigned>(i + 1), kDs3502Addresses[i], pots[i].wiper());
      } else {
        Serial.printf("[DS3502] POT %u not found at 0x%02X — check wiring / A0 A1 straps\n", static_cast<unsigned>(i + 1), kDs3502Addresses[i]);
      }
    }
  }
#endif
#if PCF8574_ENABLE
  if (input_expander.begin(PCF8574_I2C_ADDR)) {
    Serial.printf("[PCF8574] found at 0x%02X, %d inputs\n", PCF8574_I2C_ADDR, DIGITAL_INPUT_COUNT);
  } else {
    Serial.printf("[PCF8574] not found at 0x%02X — check wiring / address (PCF8574A = 0x38)\n", PCF8574_I2C_ADDR);
  }
#endif

  net_manager.begin();   // starts WiFi; activates SIM fallback automatically if WiFi absent
  // Default 256 B RX buffer fills in ~67 ms at 38400 baud; the GPS is polled from the
  // same loop as WebSocket/NTRIP/SIM work, which can stall longer than that and drop bytes.
  gps_uart.setRxBufferSize(2048);  // must be called before begin()
  // RTCM is written from CommsTask while NavTask reads; without a TX buffer a 256 B
  // write blocks ~67 ms at 38400 holding the UART lock, stalling NavTask's reads.
  gps_uart.setTxBufferSize(1024);
  gps_uart.begin(GPS_UART_BAUD, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
  Serial.print("[GPS] UART started on RX=");
  Serial.print(GPS_UART_RX_PIN);
  Serial.print(" TX=");
  Serial.println(GPS_UART_TX_PIN);
  SensorComms::configureGnssOutput(sensor_comms);
  SensorComms::initBno085(sensor_comms);
#if CONTROL_SERVER_ENABLE
  robot_client.begin(&net_manager);
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
  canRxLockInit();

  // --- Initialize Motors ---
  Serial.println("Configuring drive motor backend...");
  drive_motors.begin();
  drive_motors.enable();

  Serial.println("Configuring steering motor backend...");
  steering_motors.begin();

  Serial.println("Configuring FUNC actuators...");
  initFuncActuators();

  applyModeInit(robot_mode);

  nav_mutex = xSemaphoreCreateMutex();

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
    16384,   // enlarged: TinyGSM AT buffers need extra stack
    nullptr,
    2,
    nullptr,
    0);

  // Above CommsTask on the same core: preempts its blocking modem waits.
  xTaskCreatePinnedToCore(
    navTask,
    "NavTask",
    8192,
    nullptr,
    3,
    nullptr,
    0);

  // USB serial commands get their own task on the control core so they are never
  // stuck behind blocking LTE/AT-command calls in CommsTask. Priority is below
  // ControlTask so a slow command (e.g. CAN_PING) can't stall the drive loop.
  xTaskCreatePinnedToCore(
    serialCmdTask,
    "SerialCmdTask",
    6144,
    nullptr,
    2,
    nullptr,
    1);

  // Lowest priority: its CAN status reads block for tens of ms per motor.
  xTaskCreatePinnedToCore(
    motorStatusTask,
    "MotorStatusTask",
    4096,
    nullptr,
    1,
    nullptr,
    1);

  Serial.println("[RTOS] ControlTask pinned to Core 1");
  Serial.println("[RTOS] SerialCmdTask pinned to Core 1");
  Serial.println("[RTOS] MotorStatusTask pinned to Core 1");
  Serial.println("[RTOS] CommsTask pinned to Core 0");
  Serial.println("[RTOS] NavTask pinned to Core 0");
  
  Serial.println("Setup Complete! Ready to Drive.");
}

// ==========================================
// 5. MAIN LOOP
// ==========================================
static void controlTask(void* /*pvParameters*/) {
  unsigned long last_control_time = millis();
  bool func0_on_combo_prev = false;
  unsigned long func0_on_combo_last_trigger_ms = 0;
  bool func0_off_combo_prev = false;
  unsigned long func0_off_combo_last_trigger_ms = 0;
  bool e_stop_applied = false;
  bool estop_on_combo_prev = false;
  bool estop_off_combo_prev = false;
  for (;;) {
    if (millis() - last_control_time >= 20) {
      unsigned long now = millis();
      unsigned long dt_ms = now - last_control_time;
      last_control_time = now;
      ++loop_stats.control_cycle_count;
      if (dt_ms > loop_stats.max_control_dt_ms) loop_stats.max_control_dt_ms = dt_ms;
      if (dt_ms > 25) ++loop_stats.overrun_count;

      // Stop the motors from this task, which is the one that drives them, whatever set
      // e-stop (limit switch, server): a drive command sent here in the same cycle could
      // otherwise overwrite a stop issued from another task.
      if (e_stop_active && !e_stop_applied) {
        drive_motors.emergencyStop();
        steering_motors.emergencyStop();
      }
      e_stop_applied = e_stop_active;

      // Motor release / activation (MOTOR RELEASE|ACTIVATE / server motor_release|activate).
      const MotorPowerRequest power_req = motor_power_request;
      if (power_req != MotorPowerRequest::None) {
        motor_power_request = MotorPowerRequest::None;
        if (power_req == MotorPowerRequest::Release) {
          drive_motors.setWheelRpm(0, 0);
          drive_motors.release();
          steering_motors.release();
          motors_released = true;
          Serial.println("[MOTOR] released (unpowered) — MOTOR ACTIVATE to drive again");
        } else if (e_stop_active) {
          Serial.println("[MOTOR] activate dropped: e-stop active");
        } else {
          drive_motors.enable();
          steering_motors.enable();
          motors_released = false;
          Serial.println("[MOTOR] activated");
        }
      }

      // Motor fault reset (MOTOR RESET / server motor_reset). Re-checked here because
      // e-stop may have latched after the request was queued.
      const uint8_t reset_mask = motor_reset_request;
      if (reset_mask) {
        motor_reset_request = 0;
        if (e_stop_active) {
          Serial.println("[MOTOR] reset dropped: e-stop active");
        } else {
          if (reset_mask & kMotorResetDrive) drive_motors.clearFaults();
          if (reset_mask & kMotorResetSteer) steering_motors.clearFaults();
          Serial.printf("[MOTOR] fault reset sent:%s%s\n", (reset_mask & kMotorResetDrive) ? " drive" : "",
                        (reset_mask & kMotorResetSteer) ? " steer" : "");
        }
      }

      if (ps2_error != 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }

      ps2x.read_gamepad();

      // E-stop: L1 + L2 + Cross latches it, L1 + L2 + Circle releases it. Edge-detected on
      // the whole combo so holding the buttons doesn't re-fire every cycle.
      const bool estop_shift = ps2x.Button(PSB_L1) && ps2x.Button(PSB_L2);
      const bool estop_on_combo = estop_shift && ps2x.Button(PSB_CROSS);
      const bool estop_off_combo = estop_shift && ps2x.Button(PSB_CIRCLE);
      if (estop_on_combo && !estop_on_combo_prev && !e_stop_active) {
        e_stop_active = true;
        e_stop_reason = "ps2";
        Serial.println("!!! E-STOP ACTIVATED !!!");
        drive_motors.emergencyStop();
        steering_motors.emergencyStop();
      } else if (estop_off_combo && !estop_off_combo_prev && e_stop_active) {
        e_stop_active = false;
        e_stop_reason = "";
        Serial.println(">>> E-STOP RELEASED <<<");
        if (!motors_released) {
          drive_motors.enable();
          steering_motors.enable();
        }
      }
      estop_on_combo_prev = estop_on_combo;
      estop_off_combo_prev = estop_off_combo;

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

        // R1 + L1 / R1 + L2 drive whichever FUNC is selected by JOYSTICK_COMBO_FUNC_TARGET
        // in Config.h (0 = FUNC_0 GPIO ON/OFF, 1 = FUNC_1 CAN +/-45 REL_DEG jog). Both combos
        // are edge-detected so holding one doesn't repeatedly re-trigger it every control
        // cycle, plus a minimum gap between triggers so PS2 button-read bounce/noise across
        // a couple of 20ms polls can't be mistaken for a release-and-re-press and double-fire
        // the move.

        bool func0_on_combo_enable = ps2x.Button(PSB_R1) && ps2x.Button(PSB_L1);
        if (func0_on_combo_enable && !func0_on_combo_prev && (now - func0_on_combo_last_trigger_ms) > 500) {
#if JOYSTICK_COMBO_FUNC_TARGET == 0
          setFuncGpio(0, true);
#else
          triggerFunc(1, 45.0f, "REL_DEG");
#endif
          func0_on_combo_last_trigger_ms = now;
        }
        func0_on_combo_prev = func0_on_combo_enable;

        bool func0_off_combo_enable = ps2x.Button(PSB_R1) && ps2x.Button(PSB_L2);
        if (func0_off_combo_enable && !func0_off_combo_prev && (now - func0_off_combo_last_trigger_ms) > 500) {
#if JOYSTICK_COMBO_FUNC_TARGET == 0
          setFuncGpio(0, false);
#else
          triggerFunc(1, -45.0f, "REL_DEG");
#endif
          func0_off_combo_last_trigger_ms = now;
        }
        func0_off_combo_prev = func0_off_combo_enable;

        

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

        // Released motors get no commands: a GIM8108 position command could re-power it.
        if (!motors_released) ControlActuation::applyDriveOutputs(
          robot_mode == RobotMode::ZeroTurn,
          cmd.linear_mps,
          cmd.angle_deg,
          WHEELBASE_M,
          TRACK_WIDTH_M,
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
  Client* prev_ntrip_client = nullptr;  // detect transport switches for NTRIP reconnect

  for (;;) {
    unsigned long now = millis();
    unsigned long dt_ms = now - last_comms_time;
    last_comms_time = now;
    ++loop_stats.comms_cycle_count;
    if (dt_ms > loop_stats.max_comms_dt_ms) loop_stats.max_comms_dt_ms = dt_ms;
    if (dt_ms > COMMS_OVERRUN_MS) ++loop_stats.comms_overrun_count;

#if NET_MODE != 0
    // ── Network management: WiFi primary, SIM fallback ────────────────────────
    net_manager.update();
    const NetSource net_src = net_manager.source();
    Client* cur_client = net_manager.activeClient();
    if (cur_client != prev_ntrip_client) {
      // Bearer changed (WiFi↔SIM or gained/lost) — force NTRIP reconnect
      if (prev_ntrip_client) prev_ntrip_client->stop();
      ntrip_connected = false;
      ntrip_reconnect_backoff_ms = 3000;
      prev_ntrip_client = cur_client;
      Serial.printf("[NET] Active bearer → %s\n", net_manager.sourceName());
    }
    sensor_comms.ntrip_client = cur_client;
    wifi_connected = (net_src == NetSource::WIFI);
    // ─────────────────────────────────────────────────────────────────────────

    // ── Control server update ─────────────────────────────────────────────────
    // WiFi uses WebSocketsClient (Links2004); SIM uses SimWsClient (AT+CCH).
    // Both paths are handled inside robot_client.update().
#if CONTROL_SERVER_ENABLE
    if (net_manager.isConnected()) {
      robot_client.update();
    }
    if (net_manager.isConnected() && robot_client.isConnected()) {
      server_link_ok_ms = millis();
    }
#endif
#endif // NET_MODE != 0

#if NET_MODE != 0 && NTRIP_ENABLE
    SensorComms::connectNtripIfNeeded(sensor_comms);
#endif
    // GPS/IMU reading, safety guards and the mission executor run in NavTask.
    {
      // Latest GGA for the VRS caster: snapshot under the lock, send without it.
      String latest_gga;
      {
        NavLock lock;
        if (gps_fix.valid) latest_gga = gps_fix.last_gga;
      }
      SensorComms::forwardNtrip(sensor_comms, latest_gga);
    }
#if NET_MODE != 0
    handleRobotClientCommand();
#endif
#if NET_MODE != 0
    publishRobotTelemetry();
#endif
    logSensorsAt1Hz();
    updateStatusLeds();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// Sensors, safety guards and the mission executor run here rather than in CommsTask:
// SIM modem AT round-trips block CommsTask for 100 ms to 2 s, which overflowed the GPS
// UART and delayed the mission and stale-sensor checks. Higher priority than CommsTask
// on the same core, so it preempts those waits.
static void navTask(void* /*pvParameters*/) {
  unsigned long last_nav_ms = millis();
  for (;;) {
    const unsigned long now = millis();
    const unsigned long dt_ms = now - last_nav_ms;
    last_nav_ms = now;
    {
      NavLock lock;
      ++loop_stats.nav_cycle_count;
      if (dt_ms > loop_stats.max_nav_dt_ms) loop_stats.max_nav_dt_ms = dt_ms;
      SensorComms::readGps(sensor_comms);
      SensorComms::updateImu(sensor_comms);
      updateHeadingFusion();
      updateSafetyGuards();
      updateMissionExecutor();
      updateFuncExecutor();
      updateIoDevices(now);
    }
    vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
  }
}

// Machine-readable navigation state for a companion computer (NAV_SERIAL_PERIOD_MS):
//   NAV {"t":ms,"lat":..,"lon":..,"fix":4,"herr":0.014,"heading":deg,"mission":"running",
//        "holding":false,"override":false,"wp":2,"target_lat":..,"target_lon":..,
//        "dist_m":..,"bearing":deg,"speed_cmd":..}
// herr = -1 when unknown; wp = -1 and target_* / dist / bearing omitted with no target.
static void publishNavSerial() {
  static unsigned long last_ms = 0;
  const unsigned long now = millis();
  if (!nav_serial_output || now - last_ms < NAV_SERIAL_PERIOD_MS) return;
  last_ms = now;

  double lat, lon, tlat = 0.0, tlon = 0.0;
  int fix, wp = -1;
  float herr, heading, dist = 0.0f, speed_cmd = 0.0f;
  bool holding, override_active, hdg_gps;
  uint32_t hdg_n;
  const char* mission_state;
  {
    NavLock lock;
    lat = gps_fix.latitude;
    lon = gps_fix.longitude;
    fix = gps_fix.valid ? gps_fix.fix_quality : 0;
    herr = (fix > 0 && gps_fix.has_gst && now - gps_fix.last_gst_ms < GPS_STALE_TIMEOUT_MS) ? gps_fix.h_err_m : -1.0f;
    heading = estimateHeadingDeg();
    mission_state = missionStateToString(mission_ctx.state);
    holding = mission_ctx.active && (mission_gps_hold_since_ms != 0 || mission_link_hold_since_ms != 0);
    override_active = serialOverrideActive(now);
    hdg_gps = heading_source == HeadingSource::Gps;
    hdg_n = heading_fusion.corrections;
    if (mission_ctx.active && mission_debug_target_waypoint >= 0 &&
        static_cast<size_t>(mission_debug_target_waypoint) < mission_waypoint_count) {
      wp = mission_debug_target_waypoint;
      tlat = mission_waypoints[wp].lat;
      tlon = mission_waypoints[wp].lon;
      dist = mission_debug_dist_m;
      speed_cmd = mission_debug_speed_cmd_mps;
    }
  }

  char buf[320];
  int n = snprintf(buf, sizeof(buf),
                   "NAV {\"t\":%lu,\"lat\":%.7f,\"lon\":%.7f,\"fix\":%d,\"herr\":%.3f,\"heading\":%.1f,"
                   "\"hdg_src\":\"%s\",\"hdg_n\":%lu,"
                   "\"mission\":\"%s\",\"holding\":%s,\"override\":%s,\"wp\":%d",
                   now, lat, lon, fix, herr, heading, hdg_gps ? "GPS" : "MAG", (unsigned long)hdg_n,
                   mission_state, holding ? "true" : "false", override_active ? "true" : "false", wp);
  if (wp >= 0 && n > 0 && n < static_cast<int>(sizeof(buf))) {
    n += snprintf(buf + n, sizeof(buf) - n,
                  ",\"target_lat\":%.7f,\"target_lon\":%.7f,\"dist_m\":%.2f,\"bearing\":%.1f,\"speed_cmd\":%.2f",
                  tlat, tlon, dist, bearingDeg(lat, lon, tlat, tlon), speed_cmd);
  }
  Serial.printf("%s}\n", buf);
}

static void motorStatusTask(void* /*pvParameters*/) {
  for (;;) {
    MotorStatus fresh[kMotorCount];
    drive_motors.readStatus(&fresh[0]);
    steering_motors.readStatus(&fresh[2]);
    {
      NavLock lock;
      for (int i = 0; i < kMotorCount; ++i) {
        // Log transitions so a fault shows up in the serial log without polling MOTOR?.
        const MotorStatus& prev = motor_status[i];
        if (fresh[i].online != prev.online || fresh[i].fault != prev.fault || fresh[i].fault_code != prev.fault_code) {
          Serial.printf("[MOTOR] %s: %s%s\n", kMotorNames[i],
                        !fresh[i].online ? "offline" : fresh[i].fault ? "FAULT " : "ok",
                        fresh[i].online && fresh[i].fault ? fresh[i].fault_text : "");
        }
        motor_status[i] = fresh[i];
      }
    }
    vTaskDelay(pdMS_TO_TICKS(MOTOR_STATUS_POLL_MS));
  }
}

static void serialCmdTask(void* /*pvParameters*/) {
  for (;;) {
    handleUsbSerialControl();
    publishNavSerial();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void loop() {
  // Runtime is handled by pinned FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}