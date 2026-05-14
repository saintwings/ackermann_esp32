#pragma once

#include <Arduino.h>

enum class RobotCommandType : uint8_t {
  None = 0,
  Move,
  Waypoint,
  EmergencyStop,
  CancelTask,
  ExecuteMission,
  PauseMission,
  CancelMission,
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
  String mission_json{};
  unsigned long received_ms{0};
};

struct RobotTelemetry {
  float battery_level{100.0f};
  float speed{0.0f};
  float heading{0.0f};

  bool gps_valid{false};
  double gps_latitude{0.0};
  double gps_longitude{0.0};
  double gps_altitude{0.0};
  float gps_hdop{0.0f};
  int gps_fix_quality{0};
  int gps_satellites{0};

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
  bool mission_coverage_active{false};
  int mission_coverage_progress_index{-1};
  int mission_coverage_total{0};
};

class RobotClient {
 public:
  RobotClient();

  void begin();
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

  struct Impl;
  Impl* impl_;
};