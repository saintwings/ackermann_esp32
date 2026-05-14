#include "RobotClient.hpp"

#include <ArduinoJson.h>
#include <WebSocketsClient.h>

#include "Config.h"

namespace {

constexpr uint8_t MSG_ROBOT_REGISTER = 0;
constexpr uint8_t MSG_SENSOR_TELEMETRY = 1;
constexpr uint8_t MSG_COMMAND_ACK = 2;
constexpr uint8_t MSG_COMMAND_MOVE = 10;
constexpr uint8_t MSG_COMMAND_WAYPOINT = 11;
constexpr uint8_t MSG_COMMAND_EMERGENCY_STOP = 12;
constexpr uint8_t MSG_COMMAND_CANCEL_TASK = 13;
constexpr uint8_t MSG_COMMAND_EXECUTE_MISSION = 14;
constexpr uint8_t MSG_COMMAND_PAUSE_MISSION = 15;
constexpr uint8_t MSG_COMMAND_CANCEL_MISSION = 16;
constexpr uint8_t MSG_HEARTBEAT = 99;

template <typename T>
static void assignIfPresent(T& out, JsonVariantConst value) {
  if (!value.isNull()) {
    out = value.as<T>();
  }
}

}  // namespace

struct RobotClient::Impl {
  WebSocketsClient websocket;
  bool connected{false};
  bool has_pending_command{false};
  unsigned long last_telemetry_ms{0};
  RobotCommand pending_command{};
};

RobotClient::RobotClient() : impl_(new Impl()) {}

void RobotClient::begin() {
  impl_->websocket.begin(CONTROL_SERVER_HOST, CONTROL_SERVER_PORT, CONTROL_SERVER_PATH);
  impl_->websocket.setReconnectInterval(10000);
  impl_->websocket.enableHeartbeat(15000, 3000, 2);
  impl_->websocket.onEvent([this](WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
      case WStype_CONNECTED:
        impl_->connected = true;
#if DEBUG_LOG_ROBOT_CLIENT
        Serial.println("[ROBOT_CLIENT] WebSocket connected");
#endif
        sendRegister();
        break;
      case WStype_DISCONNECTED:
        impl_->connected = false;
#if DEBUG_LOG_ROBOT_CLIENT
        Serial.println("[ROBOT_CLIENT] WebSocket disconnected");
#endif
        break;
      case WStype_TEXT:
        handleTextMessage(String(reinterpret_cast<const char*>(payload), length));
        break;
      default:
        break;
    }
  });
}

void RobotClient::update() {
  impl_->websocket.loop();
}

bool RobotClient::isConnected() const {
  return impl_->connected;
}

void RobotClient::sendRegister() {
  JsonDocument doc;
  doc["type"] = MSG_ROBOT_REGISTER;
  doc["robot_id"] = ROBOT_ID;
  doc["timestamp"] = millis();

  JsonObject payload = doc["payload"].to<JsonObject>();
  payload["robot_name"] = ROBOT_NAME;
  payload["robot_type"] = ROBOT_TYPE;
  payload["max_speed"] = 0.6f;

  String json;
  serializeJson(doc, json);
  impl_->websocket.sendTXT(json);
}

void RobotClient::sendTelemetry(const RobotTelemetry& telemetry) {
  if (!impl_->connected) return;
  unsigned long now = millis();
  if (now - impl_->last_telemetry_ms < 200) return;
  impl_->last_telemetry_ms = now;

  JsonDocument doc;
  doc["type"] = MSG_SENSOR_TELEMETRY;
  doc["robot_id"] = ROBOT_ID;
  doc["timestamp"] = now;

  JsonObject payload = doc["payload"].to<JsonObject>();
  JsonObject status = payload["status"].to<JsonObject>();
  status["battery_level"] = telemetry.battery_level;
  status["speed"] = telemetry.speed;
  status["heading"] = telemetry.heading;

  JsonObject gps = payload["gps"].to<JsonObject>();
  gps["valid"] = telemetry.gps_valid;
  gps["latitude"] = telemetry.gps_latitude;
  gps["longitude"] = telemetry.gps_longitude;
  gps["altitude"] = telemetry.gps_altitude;
  gps["hdop"] = telemetry.gps_hdop;
  gps["fix_quality"] = telemetry.gps_fix_quality;
  gps["satellites"] = telemetry.gps_satellites;

  JsonObject imu = payload["imu"].to<JsonObject>();
  imu["valid"] = telemetry.imu_valid;
  imu["ax"] = telemetry.imu_ax;
  imu["ay"] = telemetry.imu_ay;
  imu["az"] = telemetry.imu_az;
  imu["gx"] = telemetry.imu_gx;
  imu["gy"] = telemetry.imu_gy;
  imu["gz"] = telemetry.imu_gz;
  imu["qx"] = telemetry.imu_qx;
  imu["qy"] = telemetry.imu_qy;
  imu["qz"] = telemetry.imu_qz;
  imu["qw"] = telemetry.imu_qw;
  imu["mx"] = telemetry.imu_mx;
  imu["my"] = telemetry.imu_my;
  imu["mz"] = telemetry.imu_mz;

  JsonObject odometry = payload["odometry"].to<JsonObject>();
  odometry["speed"] = telemetry.speed;

  JsonObject mission = payload["mission"].to<JsonObject>();
  mission["active"] = telemetry.mission_active;
  mission["state"] = telemetry.mission_state;
  mission["name"] = telemetry.mission_name;
  mission["waypoint_index"] = telemetry.mission_waypoint_index;
  mission["waypoint_total"] = telemetry.mission_waypoint_total;
  mission["coverage_active"] = telemetry.mission_coverage_active;
  mission["coverage_progress_index"] = telemetry.mission_coverage_progress_index;
  mission["coverage_total"] = telemetry.mission_coverage_total;

  String json;
  serializeJson(doc, json);
  impl_->websocket.sendTXT(json);
}

bool RobotClient::hasPendingCommand() const {
  return impl_->has_pending_command;
}

RobotCommand RobotClient::consumeCommand() {
  RobotCommand cmd = impl_->pending_command;
  impl_->pending_command = RobotCommand{};
  impl_->has_pending_command = false;
  return cmd;
}

void RobotClient::queueCommand(const RobotCommand& command) {
  impl_->pending_command = command;
  impl_->has_pending_command = true;
}

void RobotClient::sendCommandAck(const RobotCommand& command, const char* status, const char* detail) {
  if (!impl_->connected) return;

  JsonDocument doc;
  doc["type"] = MSG_COMMAND_ACK;
  doc["robot_id"] = ROBOT_ID;
  doc["client_id"] = command.client_id;
  doc["timestamp"] = millis();

  JsonObject payload = doc["payload"].to<JsonObject>();
  payload["status"] = status;
  payload["command"] = command.command_name;
  if (detail != nullptr) {
    payload["detail"] = detail;
  }

  String json;
  serializeJson(doc, json);
  impl_->websocket.sendTXT(json);
}

void RobotClient::handleTextMessage(const String& text) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, text);
  if (err) {
    return;
  }

  const uint8_t type = doc["type"] | 255;
  if (type == MSG_HEARTBEAT || type == MSG_COMMAND_ACK) {
    return;
  }

  JsonObjectConst payload = doc["payload"].as<JsonObjectConst>();
  RobotCommand command;
  command.client_id = doc["client_id"] | "";
  command.received_ms = millis();

  switch (type) {
    case MSG_COMMAND_MOVE:
      command.type = RobotCommandType::Move;
      command.command_name = "move";
      assignIfPresent(command.linear_velocity, payload["linear_velocity"]);
      assignIfPresent(command.angular_velocity, payload["angular_velocity"]);
      assignIfPresent(command.duration, payload["duration"]);
      break;
    case MSG_COMMAND_WAYPOINT:
      command.type = RobotCommandType::Waypoint;
      command.command_name = "waypoint";
      assignIfPresent(command.latitude, payload["latitude"]);
      assignIfPresent(command.longitude, payload["longitude"]);
      assignIfPresent(command.altitude, payload["altitude"]);
      break;
    case MSG_COMMAND_EMERGENCY_STOP:
      command.type = RobotCommandType::EmergencyStop;
      command.command_name = "emergency_stop";
      break;
    case MSG_COMMAND_CANCEL_TASK:
      command.type = RobotCommandType::CancelTask;
      command.command_name = "cancel_task";
      break;
    case MSG_COMMAND_EXECUTE_MISSION:
      command.type = RobotCommandType::ExecuteMission;
      command.command_name = "execute_mission";
      command.mission_name = payload["name"] | "Unnamed Mission";
      serializeJson(payload, command.mission_json);
      break;
    case MSG_COMMAND_PAUSE_MISSION:
      command.type = RobotCommandType::PauseMission;
      command.command_name = "pause_mission";
      break;
    case MSG_COMMAND_CANCEL_MISSION:
      command.type = RobotCommandType::CancelMission;
      command.command_name = "cancel_mission";
      break;
    default:
      return;
  }

  queueCommand(command);
}