#include "CanDrivers.hpp"

#include <string.h>

#include "driver/twai.h"
#include "freertos/semphr.h"

namespace {
constexpr uint32_t kODriveAxisStateHoming = 11;

SemaphoreHandle_t can_rx_mutex = nullptr;

// Discards everything already in the RX queue, so the next matching frame is a fresh
// reply rather than backlog (the queue is kept full by drive acks and broadcasts).
void drainRxQueue() {
  twai_message_t stale;
  while (twai_receive(&stale, 0) == ESP_OK) {
    // discard
  }
}

// Appends ", <name>" (or "<prefix><name>") to a MotorStatus fault_text.
void appendFault(MotorStatus* out, const char* prefix, const char* name) {
  const size_t len = strlen(out->fault_text);
  snprintf(out->fault_text + len, sizeof(out->fault_text) - len, "%s%s%s", len ? ", " : "", prefix, name);
}
}  // namespace

void canRxLockInit() {
  if (!can_rx_mutex) can_rx_mutex = xSemaphoreCreateRecursiveMutex();
}

CanRxLock::CanRxLock() {
  if (can_rx_mutex) xSemaphoreTakeRecursive(can_rx_mutex, portMAX_DELAY);
}

CanRxLock::~CanRxLock() {
  if (can_rx_mutex) xSemaphoreGiveRecursive(can_rx_mutex);
}

ODriveCAN::ODriveCAN(uint32_t id, LoopStats* stats) : node_id(id), loop_stats_(stats) {}

void ODriveCAN::set_axis_state(uint32_t state_id) {
  twai_message_t msg = {};
  msg.identifier = (node_id << 5) | 0x07;
  msg.extd = 0;
  msg.data_length_code = 8;
  memcpy(&msg.data[0], &state_id, 4);
  memset(&msg.data[4], 0, 4);
  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void ODriveCAN::set_controller_mode(int32_t control_mode, int32_t input_mode) {
  twai_message_t msg = {};
  msg.identifier = (node_id << 5) | 0x0B;
  msg.extd = 0;
  msg.data_length_code = 8;
  memcpy(&msg.data[0], &control_mode, 4);
  memcpy(&msg.data[4], &input_mode, 4);
  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void ODriveCAN::set_position(float pos_turns, float vel_ff, float torque_ff, bool force) {
  pos_turns = pos_turns * gear_ratio;
  current_target_pos = pos_turns;

  unsigned long now = millis();
  if (!force && fabsf(pos_turns - last_sent_pos) < 0.01f && (now - last_sent_pos_ms) < 100) {
    return;
  }

  twai_message_t msg = {};
  msg.identifier = (node_id << 5) | 0x0C;
  msg.extd = 0;
  msg.data_length_code = 8;

  int16_t v_ff_int = static_cast<int16_t>(vel_ff * 1000);
  int16_t t_ff_int = static_cast<int16_t>(torque_ff * 1000);

  memcpy(&msg.data[0], &pos_turns, 4);
  memcpy(&msg.data[4], &v_ff_int, 2);
  memcpy(&msg.data[6], &t_ff_int, 2);
  if (twai_transmit(&msg, 0) == ESP_OK) {
    last_sent_pos = pos_turns;
    last_sent_pos_ms = now;
    if (loop_stats_) ++loop_stats_->can_runtime_tx_ok_count;
  } else {
    if (loop_stats_) ++loop_stats_->can_runtime_tx_drop_count;
  }
}

void ODriveCAN::go_home() {
  //send_cmd(0xC4);
}

bool ODriveCAN::get_position_turns(float* out_turns, unsigned long timeout_ms) {
  const uint32_t estimate_id = (node_id << 5) | 0x09;
  CanRxLock lock;

  // The ODrive broadcasts Get_Encoder_Estimates on its own, independent of our request.
  // Between calls (seconds apart, since this is only hit on a FUNC command) the RX queue
  // backs up with stale broadcasts, and twai_receive() below would hand us the oldest one
  // first. Drain whatever's already queued so the frame we accept after the request below
  // is guaranteed to be a fresh reply, not leftover backlog.
  drainRxQueue();

  twai_message_t req = {};
  req.identifier = estimate_id;
  req.extd = 0;
  req.rtr = 1;
  req.data_length_code = 8;
  if (twai_transmit(&req, pdMS_TO_TICKS(10)) != ESP_OK) {
    if (loop_stats_) ++loop_stats_->can_runtime_tx_drop_count;
    return false;
  }
  if (loop_stats_) ++loop_stats_->can_runtime_tx_ok_count;

  const unsigned long start = millis();
  twai_message_t resp;
  while (millis() - start < timeout_ms) {
    if (twai_receive(&resp, pdMS_TO_TICKS(10)) != ESP_OK) continue;
    if (resp.rtr) continue;
    if (resp.identifier != estimate_id) continue;
    if (resp.data_length_code < 4) continue;

    float pos_turns = 0.0f;
    memcpy(&pos_turns, &resp.data[0], 4);
    *out_turns = pos_turns;
    return true;
  }
  return false;
}

void ODriveCAN::clear_errors() {
  twai_message_t msg = {};
  msg.identifier = (node_id << 5) | 0x18;
  msg.extd = 0;
  msg.data_length_code = 8;
  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

bool ODriveCAN::read_status(MotorStatus* out, unsigned long timeout_ms) {
  const uint32_t heartbeat_id = (node_id << 5) | 0x01;
  CanRxLock lock;
  drainRxQueue();

  const unsigned long start = millis();
  twai_message_t resp;
  while (millis() - start < timeout_ms) {
    if (twai_receive(&resp, pdMS_TO_TICKS(10)) != ESP_OK) continue;
    if (resp.rtr || resp.identifier != heartbeat_id || resp.data_length_code < 5) continue;

    uint32_t axis_error = 0;
    memcpy(&axis_error, &resp.data[0], 4);
    const uint8_t axis_state = resp.data[4];
    *out = MotorStatus{};
    out->online = true;
    out->enabled = axis_state == 8;  // CLOSED_LOOP_CONTROL
    out->fault = axis_error != 0;
    out->fault_code = axis_error;
    if (out->fault) snprintf(out->fault_text, sizeof(out->fault_text), "axis_error 0x%08lX", static_cast<unsigned long>(axis_error));
    out->updated_ms = millis();
    return true;
  }
  return false;
}

GIM8108CAN::GIM8108CAN(uint32_t id, LoopStats* stats) : can_id_(id), loop_stats_(stats) {}

void GIM8108CAN::send_cmd(uint8_t cmd, const uint8_t* payload, uint8_t payload_len) {
  if (payload_len > 7) payload_len = 7;

  twai_message_t msg = {};
  msg.identifier = can_id_;
  msg.extd = 0;
  msg.data_length_code = static_cast<uint8_t>(1 + payload_len);
  msg.data[0] = cmd;
  for (uint8_t i = 0; i < payload_len; ++i) {
    msg.data[1 + i] = payload[i];
  }

  if (twai_transmit(&msg, 0) == ESP_OK) {
    if (loop_stats_) ++loop_stats_->can_runtime_tx_ok_count;
  } else {
    if (loop_stats_) ++loop_stats_->can_runtime_tx_drop_count;
  }
}

void GIM8108CAN::clear_fault() {
  send_cmd(0xAF);
}

void GIM8108CAN::disable_motor() {
  send_cmd(0xCF);
}

void GIM8108CAN::set_position_max_speed_rpm(float rpm) {
  uint32_t value = static_cast<uint32_t>(rpm * 100.0f);
  uint8_t payload[4];
  memcpy(payload, &value, sizeof(value));
  send_cmd(0xB2, payload, sizeof(payload));
}

void GIM8108CAN::set_max_current_amp(float current_amp) {
  uint32_t value = static_cast<uint32_t>(current_amp * 1000.0f);
  uint8_t payload[4];
  memcpy(payload, &value, sizeof(value));
  send_cmd(0xB3, payload, sizeof(payload));
}

void GIM8108CAN::set_acceleration_rpm_per_sec(float accel_rpm_per_sec) {
  uint32_t value = static_cast<uint32_t>(accel_rpm_per_sec * 100.0f);
  uint8_t payload[4];
  memcpy(payload, &value, sizeof(value));
  send_cmd(0xB5, payload, sizeof(payload));
}

void GIM8108CAN::set_absolute_position_turns(float turns) {
  int32_t counts = static_cast<int32_t>(turns * 16384.0f);
  // The 20 ms steering loop calls this every tick. Every command gets a reply, and at
  // that rate the drive mostly fails to answer a status request (0xAE), so resend an
  // unchanged target only every 100 ms (as ODriveCAN::set_position).
  const unsigned long now = millis();
  if (counts == last_sent_counts_ && now - last_sent_ms_ < 100) return;
  last_sent_counts_ = counts;
  last_sent_ms_ = now;
  uint8_t payload[4];
  memcpy(payload, &counts, sizeof(counts));
  send_cmd(0xC2, payload, sizeof(payload));
}

void GIM8108CAN::move_relative_position_turns(float turns) {
  int32_t counts = static_cast<int32_t>(turns * 16384.0f);
  uint8_t payload[4];
  memcpy(payload, &counts, sizeof(counts));
  send_cmd(0xC3, payload, sizeof(payload));
}

void GIM8108CAN::go_home() {
  send_cmd(0xC4);
}

bool GIM8108CAN::read_status(MotorStatus* out, unsigned long timeout_ms) {
  CanRxLock lock;
  for (int attempt = 0; attempt < 2; ++attempt) {
    drainRxQueue();
    send_cmd(0xAE);

    const unsigned long start = millis();
    twai_message_t resp;
    while (millis() - start < timeout_ms) {
      if (twai_receive(&resp, pdMS_TO_TICKS(5)) != ESP_OK) continue;
      // Replies come from the motor's own id and echo the command byte.
      if (resp.rtr || resp.data_length_code < 8 || resp.data[0] != 0xAE) continue;
      if (resp.identifier != can_id_) continue;

      uint16_t voltage_raw = 0;
      memcpy(&voltage_raw, &resp.data[1], 2);
      const uint8_t mode = resp.data[6];
      const uint8_t fault = resp.data[7];
      *out = MotorStatus{};
      out->online = true;
      out->enabled = mode != 0;  // 0 = disabled, 1..4 = voltage/current/speed/position
      out->fault = fault != 0;
      out->fault_code = fault;
      out->voltage_v = voltage_raw * 0.01f;
      out->temp_c = resp.data[5];
      static const struct { uint8_t bit; const char* name; } kFaults[] = {
        {0x01, "voltage"}, {0x02, "current"}, {0x04, "temperature"},
        {0x08, "encoder"}, {0x40, "hardware"}, {0x80, "software"},
      };
      for (const auto& f : kFaults) {
        if (fault & f.bit) appendFault(out, "", f.name);
      }
      out->updated_ms = millis();
      return true;
    }
  }
  return false;
}

ZLAC8015D::ZLAC8015D(uint32_t id, LoopStats* stats) : node_id(id), tx_id(0x600 + id), loop_stats_(stats) {}

void ZLAC8015D::send_sdo(uint8_t cmd,
                         uint8_t idx_l,
                         uint8_t idx_h,
                         uint8_t sub,
                         uint8_t d0,
                         uint8_t d1,
                         uint8_t d2,
                         uint8_t d3) {
  twai_message_t msg = {};
  msg.identifier = tx_id;
  msg.extd = 0;
  msg.data_length_code = 8;
  msg.data[0] = cmd;
  msg.data[1] = idx_l;
  msg.data[2] = idx_h;
  msg.data[3] = sub;
  msg.data[4] = d0;
  msg.data[5] = d1;
  msg.data[6] = d2;
  msg.data[7] = d3;
  twai_transmit(&msg, pdMS_TO_TICKS(10));
  delay(30);
}

void ZLAC8015D::init_sync_velocity_mode() {
  send_sdo(0x2B, 0x0F, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00);
  send_sdo(0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00);
  send_sdo(0x23, 0x83, 0x60, 0x01, 0xC8, 0x00, 0x00, 0x00);
  send_sdo(0x23, 0x83, 0x60, 0x02, 0xC8, 0x00, 0x00, 0x00);
  send_sdo(0x23, 0x84, 0x60, 0x01, 0xC8, 0x00, 0x00, 0x00);
  send_sdo(0x23, 0x84, 0x60, 0x02, 0xC8, 0x00, 0x00, 0x00);
}

void ZLAC8015D::enable_motor() {
  send_sdo(0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00);
  send_sdo(0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00);
  send_sdo(0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00);
}

void ZLAC8015D::set_sync_speed(int16_t left_rpm, int16_t right_rpm) {
  unsigned long now = millis();
  if (left_rpm == last_left_rpm && right_rpm == last_right_rpm && (now - last_speed_sent_ms) < 100) {
    return;
  }

  twai_message_t msg = {};
  msg.identifier = tx_id;
  msg.extd = 0;
  msg.data_length_code = 8;
  msg.data[0] = 0x23;
  msg.data[1] = 0xFF;
  msg.data[2] = 0x60;
  msg.data[3] = 0x03;
  memcpy(&msg.data[4], &left_rpm, 2);
  memcpy(&msg.data[6], &right_rpm, 2);
  if (twai_transmit(&msg, 0) == ESP_OK) {
    last_left_rpm = left_rpm;
    last_right_rpm = right_rpm;
    last_speed_sent_ms = now;
    if (loop_stats_) ++loop_stats_->can_runtime_tx_ok_count;
  } else {
    if (loop_stats_) ++loop_stats_->can_runtime_tx_drop_count;
  }
}

void ZLAC8015D::trigger_e_stop() {
  send_sdo(0x2B, 0x40, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00);
}

void ZLAC8015D::clear_fault() {
  send_sdo(0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00);
}

void ZLAC8015D::release_motor() {
  send_sdo(0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00);
}

bool ZLAC8015D::read_sdo(uint16_t index, uint8_t sub, uint32_t* out, unsigned long timeout_ms) {
  CanRxLock lock;
  drainRxQueue();

  twai_message_t req = {};
  req.identifier = tx_id;
  req.extd = 0;
  req.data_length_code = 8;
  req.data[0] = 0x40;  // SDO upload request
  req.data[1] = index & 0xFF;
  req.data[2] = index >> 8;
  req.data[3] = sub;
  if (twai_transmit(&req, pdMS_TO_TICKS(10)) != ESP_OK) {
    if (loop_stats_) ++loop_stats_->can_runtime_tx_drop_count;
    return false;
  }
  if (loop_stats_) ++loop_stats_->can_runtime_tx_ok_count;

  const uint32_t reply_id = 0x580 + node_id;
  const unsigned long start = millis();
  twai_message_t resp;
  while (millis() - start < timeout_ms) {
    if (twai_receive(&resp, pdMS_TO_TICKS(5)) != ESP_OK) continue;
    if (resp.rtr || resp.identifier != reply_id || resp.data_length_code < 8) continue;
    // Skip acks of the speed writes ControlTask keeps sending (0x60) and replies for
    // other objects.
    if (resp.data[1] != (index & 0xFF) || resp.data[2] != (index >> 8) || resp.data[3] != sub) continue;
    if (resp.data[0] == 0x80) return false;  // SDO abort
    if ((resp.data[0] & 0xE0) != 0x40) continue;
    uint32_t value = 0;
    memcpy(&value, &resp.data[4], 4);
    *out = value;
    return true;
  }
  return false;
}

bool ZLAC8015D::read_status(MotorStatus* out) {
  CanRxLock lock;
  uint32_t status_word = 0, error_code = 0;
  if (!read_sdo(0x6041, 0x00, &status_word)) return false;  // offline: skip the rest

  *out = MotorStatus{};
  out->online = true;
  status_word &= 0xFFFF;
  out->enabled = (status_word & 0x000F) == 0x0007;  // after controlword 0x0F
  if (read_sdo(0x603F, 0x00, &error_code)) out->fault_code = error_code;
  out->fault = (status_word & 0x0008) || out->fault_code != 0;

  // 0x603F: the manual's object table says low 16 bits = left motor, high = right
  // (its section 3.7 says the opposite). Over/under-voltage and EEPROM are shared.
  static const struct { uint16_t bit; const char* name; } kFaults[] = {
    {0x0001, "over-voltage"}, {0x0002, "under-voltage"}, {0x0004, "over-current"},
    {0x0008, "overload"}, {0x0010, "current tolerance"}, {0x0020, "encoder tolerance"},
    {0x0040, "speed tolerance"}, {0x0080, "ref voltage"}, {0x0100, "EEPROM"},
    {0x0200, "hall"}, {0x0400, "motor overtemp"}, {0x0800, "encoder"}, {0x2000, "speed setting"},
  };
  constexpr uint16_t kSharedFaults = 0x0001 | 0x0002 | 0x0100;
  const uint16_t left = error_code & 0xFFFF;
  const uint16_t right = error_code >> 16;
  for (const auto& f : kFaults) {
    if (f.bit & kSharedFaults) {
      if ((left | right) & f.bit) appendFault(out, "", f.name);
    } else {
      if (left & f.bit) appendFault(out, "L ", f.name);
      if (right & f.bit) appendFault(out, "R ", f.name);
    }
  }
  if (out->fault && !out->fault_text[0]) appendFault(out, "", "driver alarm");

  uint32_t raw = 0;
  if (read_sdo(0x2035, 0x00, &raw)) out->voltage_v = (raw & 0xFFFF) * 0.01f;
  // 0x2032 sub 1/2 = left/right motor, sub 3 = driver; signed 0.1 degC.
  for (uint8_t sub = 1; sub <= 3; ++sub) {
    if (!read_sdo(0x2032, sub, &raw)) continue;
    const float t = static_cast<int16_t>(raw & 0xFFFF) * 0.1f;
    if (isnan(out->temp_c) || t > out->temp_c) out->temp_c = t;
  }
  out->updated_ms = millis();
  return true;
}
