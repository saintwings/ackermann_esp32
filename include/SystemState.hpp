#pragma once

#include <Arduino.h>

struct GpsFix {
  bool valid{false};
  bool has_gga{false};
  double latitude{0.0};
  double longitude{0.0};
  double altitude{0.0};
  float hdop{0.0f};
  int fix_quality{0};
  int satellites{0};
  String last_gga{};
  unsigned long last_update_ms{0};
  unsigned long last_rx_ms{0};
  // From NMEA GST: receiver's 1-sigma position error estimate (metres).
  bool has_gst{false};
  float std_lat_m{0.0f};
  float std_lon_m{0.0f};
  float std_alt_m{0.0f};
  float h_err_m{0.0f};  // sqrt(std_lat^2 + std_lon^2)
  unsigned long last_gst_ms{0};
  // Diagnostics (reset by the 1 Hz logger): GGA sentences accepted / rejected
  // by checksum. Rejects mean bytes were dropped (GPS UART RX overflow).
  uint32_t gga_ok_count{0};
  uint32_t gga_bad_count{0};
};

struct ImuState {
  bool valid{false};
  float ax{0.0f}, ay{0.0f}, az{0.0f};
  float gx{0.0f}, gy{0.0f}, gz{0.0f};
  float qx{0.0f}, qy{0.0f}, qz{0.0f}, qw{1.0f};      // rotation vector (magnetometer north)
  float gqx{0.0f}, gqy{0.0f}, gqz{0.0f}, gqw{1.0f};  // game rotation vector (gyro+accel, no mag)
  bool rv_valid{false};    // a rotation vector report has arrived
  bool game_valid{false};  // a game rotation vector report has arrived
  unsigned long last_update_ms{0};
};

enum class MissionState : uint8_t {
  Idle = 0,
  Queued,
  Running,
  Paused,
  Completed,
  Failed,
  Cancelled,
  EStop,
};

inline const char* missionStateToString(MissionState state) {
  switch (state) {
    case MissionState::Idle: return "idle";
    case MissionState::Queued: return "queued";
    case MissionState::Running: return "running";
    case MissionState::Paused: return "paused";
    case MissionState::Completed: return "completed";
    case MissionState::Failed: return "failed";
    case MissionState::Cancelled: return "cancelled";
    case MissionState::EStop: return "e_stop";
    default: return "unknown";
  }
}

// One motor driver's health, read over CAN by MotorStatusTask. For a ZLAC8015D one entry
// covers the driver (both hub wheels on that axle).
struct MotorStatus {
  bool online{false};      // answered the last status request
  bool enabled{false};     // powered / closed loop
  bool fault{false};
  uint32_t fault_code{0};  // driver-specific raw code (ZLAC 0x603F, GIM fault byte, ODrive axis_error)
  char fault_text[64]{};   // decoded fault names, "" when none
  float voltage_v{NAN};    // bus voltage, NAN = not available
  float temp_c{NAN};       // hottest motor/driver temperature, NAN = not available
  unsigned long updated_ms{0};
};

struct ControlTargets {
  float linear_mps{0.0f};
  float angle_deg{0.0f};
  // remote_move_expire_ms: 0 = infinite timeout (command persists), 1+ = specific expire time, initial value 1 = no command active
  unsigned long remote_move_expire_ms{1};
};

struct MissionContext {
  bool active{false};
  String name{};
  MissionState state{MissionState::Idle};
  unsigned long state_since_ms{0};
  unsigned long started_ms{0};
  uint8_t phase{0};
  int waypoint_index{-1};
};

struct LoopStats {
  unsigned long overrun_count{0};
  unsigned long control_cycle_count{0};
  unsigned long max_control_dt_ms{0};
  unsigned long comms_cycle_count{0};
  unsigned long max_comms_dt_ms{0};
  unsigned long comms_overrun_count{0};
  unsigned long nav_cycle_count{0};
  unsigned long max_nav_dt_ms{0};
  unsigned long can_runtime_tx_ok_count{0};
  unsigned long can_runtime_tx_drop_count{0};
  unsigned long safety_stale_imu_count{0};
  unsigned long safety_stale_gps_count{0};
  unsigned long safety_remote_timeout_count{0};
  unsigned long safety_mission_abort_count{0};
};