#include "CanDrivers.hpp"

#include <string.h>

#include "driver/twai.h"

namespace {
constexpr uint32_t kODriveAxisStateHoming = 11;
}

ODriveCAN::ODriveCAN(uint32_t id, LoopStats* stats) : node_id(id), loop_stats_(stats) {}

void ODriveCAN::set_axis_state(uint32_t state_id) {
  twai_message_t msg;
  msg.identifier = (node_id << 5) | 0x07;
  msg.extd = 0;
  msg.data_length_code = 8;
  memcpy(&msg.data[0], &state_id, 4);
  memset(&msg.data[4], 0, 4);
  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void ODriveCAN::set_controller_mode(int32_t control_mode, int32_t input_mode) {
  twai_message_t msg;
  msg.identifier = (node_id << 5) | 0x0B;
  msg.extd = 0;
  msg.data_length_code = 8;
  memcpy(&msg.data[0], &control_mode, 4);
  memcpy(&msg.data[4], &input_mode, 4);
  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void ODriveCAN::set_position(float pos_turns, float vel_ff, float torque_ff) {
  pos_turns = pos_turns * gear_ratio;
  current_target_pos = pos_turns;

  unsigned long now = millis();
  if (fabsf(pos_turns - last_sent_pos) < 0.01f && (now - last_sent_pos_ms) < 100) {
    return;
  }

  twai_message_t msg;
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



GIM8108CAN::GIM8108CAN(uint32_t id, LoopStats* stats) : can_id_(id), loop_stats_(stats) {}

void GIM8108CAN::send_cmd(uint8_t cmd, const uint8_t* payload, uint8_t payload_len) {
  if (payload_len > 7) payload_len = 7;

  twai_message_t msg;
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

ZLAC8015D::ZLAC8015D(uint32_t id, LoopStats* stats) : node_id(id), tx_id(0x600 + id), loop_stats_(stats) {}

void ZLAC8015D::send_sdo(uint8_t cmd,
                         uint8_t idx_l,
                         uint8_t idx_h,
                         uint8_t sub,
                         uint8_t d0,
                         uint8_t d1,
                         uint8_t d2,
                         uint8_t d3) {
  twai_message_t msg;
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

  twai_message_t msg;
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
