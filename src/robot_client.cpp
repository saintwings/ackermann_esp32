#include "RobotClient.hpp"

#include <ArduinoJson.h>
#include <WebSocketsClient.h>

#include "RobotConfig.h"

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

void RobotClient::begin(NetManager* net) {
  _net = net;

  // WiFi WebSocket setup (Links2004 library) — used when source == WIFI
#if CONTROL_SERVER_USE_SSL
  impl_->websocket.beginSSL(CONTROL_SERVER_HOST, CONTROL_SERVER_PORT, CONTROL_SERVER_PATH);
#else
  impl_->websocket.begin(CONTROL_SERVER_HOST, CONTROL_SERVER_PORT, CONTROL_SERVER_PATH);
#endif
  impl_->websocket.setReconnectInterval(10000);
  impl_->websocket.enableHeartbeat(15000, 3000, 2);
  impl_->websocket.onEvent([this](WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
      case WStype_CONNECTED:
        impl_->connected = true;
#if DEBUG_LOG_ROBOT_CLIENT
        Serial.println("[ROBOT_CLIENT] WiFi WebSocket connected");
#endif
        sendRegister();
        break;
      case WStype_DISCONNECTED:
        impl_->connected = false;
#if DEBUG_LOG_ROBOT_CLIENT
        Serial.println("[ROBOT_CLIENT] WiFi WebSocket disconnected");
#endif
        break;
      case WStype_TEXT:
        handleTextMessage(String(reinterpret_cast<const char*>(payload), length));
        break;
      default:
        break;
    }
  });

  // SIM WebSocket (SimWsClient) — allocated lazily when SIM is active
  // _simWs is created on first SIM connect in beginSimWs()
}

void RobotClient::beginSimWs() {
  delete _simWs;
  _simWs = new SimWsClient(_net->simStream());
  _net->setSkipGprsCheck(true);  // own the serial during WS operations
  if (_simWs->connect(CONTROL_SERVER_HOST, SIM_CONTROL_SERVER_PORT, CONTROL_SERVER_PATH)) {
    _simConnected  = true;
    _lastPingMs    = millis();
    sendRegister();
  } else {
    _simConnected = false;
    Serial.println("[RC] SIM WebSocket connect failed — will retry");
  }
}

void RobotClient::update() {
  if (!_net) return;

  const NetSource src = _net->source();

  // ── Transport switch: tear down whichever side was active ─────────────────
  if (src != _lastSource) {
    if (_lastSource == NetSource::SIM && _simWs) {
      _simWs->disconnect();
      _simConnected = false;
      _net->setSkipGprsCheck(false);
    }
    if (_lastSource == NetSource::WIFI) {
      impl_->connected = false;
    }
    _lastSource = src;
  }

  // ── WiFi path (unchanged behaviour via WebSocketsClient) ─────────────────
  if (src == NetSource::WIFI) {
    impl_->websocket.loop();
    return;
  }

  // ── SIM path via SimWsClient ──────────────────────────────────────────────
  if (src != NetSource::SIM) return;

  if (!_simConnected || !_simWs || !_simWs->connected()) {
    _simConnected = false;
    unsigned long now = millis();
    if (now - _lastReconnectMs >= 12000) {
      _lastReconnectMs = now;
      beginSimWs();
    }
    return;
  }

  // Periodic ping to keep connection alive through Cloudflare idle timeout
  unsigned long now = millis();
  if (now - _lastPingMs >= 15000) {
    _lastPingMs = now;
    _simWs->sendPing();
  }

  // Poll for incoming frames
  String rx;
  if (_simWs->poll(rx)) {
    handleTextMessage(rx);
  }

  // Detect connection drop reported by poll()
  if (!_simWs->connected()) {
    _simConnected = false;
    Serial.println("[RC] SIM WebSocket dropped");
  }
}

bool RobotClient::isConnected() const {
  if (_net && _net->source() == NetSource::SIM) return _simConnected;
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
  payload["max_speed"] = MAX_LINEAR_SPEED_MS;

  String json;
  serializeJson(doc, json);
  NetSource src = _net ? _net->source() : NetSource::WIFI;
  if (src == NetSource::SIM && _simWs) {
    bool ok = _simWs->sendText(json);
    Serial.printf("[RC] Register send %s (%u bytes)\n", ok ? "OK" : "FAILED", json.length());
  } else {
    impl_->websocket.sendTXT(json.c_str());
  }
}

void RobotClient::sendTelemetry(const RobotTelemetry& telemetry) {
  if (!isConnected()) return;
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
  NetSource src = _net ? _net->source() : NetSource::WIFI;
  if (src == NetSource::SIM && _simWs) { _simWs->sendText(json); }
  else { impl_->websocket.sendTXT(json.c_str()); }
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
  if (!isConnected()) return;

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
  NetSource src = _net ? _net->source() : NetSource::WIFI;
  if (src == NetSource::SIM && _simWs) { _simWs->sendText(json); }
  else { impl_->websocket.sendTXT(json.c_str()); }
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