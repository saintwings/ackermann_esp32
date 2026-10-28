#pragma once

#include <Arduino.h>
#include "NetManager.hpp"
#include "SimWsClient.hpp"

enum class RobotCommandType : uint8_t {
  None = 0,
  Move,
  Waypoint,
  EmergencyStop,
  CancelTask,
  ExecuteMission,
  PauseMission,
  CancelMission,
  MissionUploadStart,    // type 40 — begin batched upload, carries tasks JSON
  MissionUploadBatch,    // type 41 — one chunk of waypoints
  MissionUploadExecute,  // type 42 — all batches done, start mission
  Io,                    // type 17 — speed output / actuator (io_action)
};

struct RobotCommand {
  RobotCommandType type{RobotCommandType::None};
  String client_id{};
  String command_name{};
  float linear_velocity{0.0f};
  float angular_velocity{0.0f};
  float duration{0.0f};
  float latitude{0.0f};
  float longitude{0.0f};
  float altitude{0.0f};
  String mission_name{};
  String mission_json{};        // reused: tasks JSON for START, waypoints JSON for BATCH
  unsigned long received_ms{0};
  // Batch upload fields
  String upload_session_id{};
  int    upload_batch_index{-1};
  int    upload_offset{0};
  int    upload_total_waypoints{0};
  int    upload_total_batches{0};
  // Io fields
  String io_action{};  // set_speed, actuator_move, actuator_stop
  int    io_channel{0};
  float  io_value{0.0f};
};

struct RobotTelemetry {
  float battery_level{100.0f};
  bool e_stop{false};
  const char* e_stop_reason{""};  // "ps2", "server", "limit: Front L", ...
  float speed{0.0f};
  float heading{0.0f};

  bool gps_valid{false};
  double gps_latitude{0.0};
  double gps_longitude{0.0};
  double gps_altitude{0.0};
  float gps_hdop{0.0f};
  int gps_fix_quality{0};
  int gps_satellites{0};
  float gps_h_err_m{-1.0f};  // GST horizontal 1-sigma error; -1 = unknown

  bool imu_valid{false};
  float imu_ax{0.0f}, imu_ay{0.0f}, imu_az{0.0f};
  float imu_gx{0.0f}, imu_gy{0.0f}, imu_gz{0.0f};
  float imu_qx{0.0f}, imu_qy{0.0f}, imu_qz{0.0f}, imu_qw{1.0f};
  float imu_mx{0.0f}, imu_my{0.0f}, imu_mz{0.0f};

  bool mission_active{false};
  String mission_state{"idle"};
  String mission_name{};
  int mission_waypoint_index{-1};
  int mission_waypoint_total{0};
  bool mission_holding{false};  // running, but stopped by the GPS accuracy gate
  bool mission_coverage_active{false};
  int mission_coverage_progress_index{-1};
  int mission_coverage_total{0};

  // DS3502 digital pots
  static constexpr int kMaxPots = 4;
  int pot_count{0};
  bool pot_present[kMaxPots]{};
  int pot_wiper[kMaxPots]{};  // 0..127

  // PCF8574 digital inputs; bit n = input n+1 active
  int input_count{0};
  bool inputs_ok{false};
  uint8_t inputs{0};

  // Linear actuator
  bool actuator_enabled{false};
  float actuator_position_pct{0.0f};
  float actuator_target_pct{0.0f};
  const char* actuator_state{"idle"};
  const char* actuator_fault{"none"};
};

class RobotClient {
 public:
  RobotClient();

  // Pass the NetManager so RobotClient can route through WiFi or SIM transport.
  void begin(NetManager* net);
  void update();
  bool isConnected() const;

  void sendTelemetry(const RobotTelemetry& telemetry);
  bool hasPendingCommand() const;
  RobotCommand consumeCommand();
  void sendCommandAck(const RobotCommand& command, const char* status, const char* detail = nullptr);

 private:
  void sendRegister();
  void handleTextMessage(const String& text);
  void queueCommand(const RobotCommand& command);

  // Reconnect helpers
  void reconnectWifi();
  void reconnectSim();
  void beginSimWs();

  struct Impl;
  Impl* impl_;

  NetManager*   _net{nullptr};
  SimWsClient*  _simWs{nullptr};
  NetSource     _lastSource{NetSource::NONE};
  unsigned long _lastReconnectMs{0};
  unsigned long _lastPingMs{0};
  bool          _simConnected{false};
  uint8_t       _simFailCount{0};
};