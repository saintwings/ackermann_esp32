#pragma once

#include <Arduino.h>

#include "SystemState.hpp"

class ODriveCAN {
 public:
  uint32_t node_id;
  float current_target_pos = 0.0f;
  float last_sent_pos = 9999.0f;
  unsigned long last_sent_pos_ms = 0;

  explicit ODriveCAN(uint32_t id, LoopStats* stats = nullptr);

  void set_axis_state(uint32_t state_id);
  void set_controller_mode(int32_t control_mode, int32_t input_mode);
  void set_position(float pos_turns, float vel_ff = 0.0f, float torque_ff = 0.0f);

 private:
  LoopStats* loop_stats_;
};

class GIM8108CAN {
 public:
  explicit GIM8108CAN(uint32_t id, LoopStats* stats = nullptr);

  void clear_fault();
  void disable_motor();
  void set_position_max_speed_rpm(float rpm);
  void set_max_current_amp(float current_amp);
  void set_acceleration_rpm_per_sec(float accel_rpm_per_sec);
  void set_absolute_position_turns(float turns);

 private:
  void send_cmd(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t payload_len = 0);

  uint32_t can_id_;
  LoopStats* loop_stats_;
};

class ZLAC8015D {
 public:
  uint32_t node_id, tx_id;
  int16_t last_left_rpm = 32767;
  int16_t last_right_rpm = 32767;
  unsigned long last_speed_sent_ms = 0;

  explicit ZLAC8015D(uint32_t id, LoopStats* stats = nullptr);

  void send_sdo(uint8_t cmd,
                uint8_t idx_l,
                uint8_t idx_h,
                uint8_t sub,
                uint8_t d0,
                uint8_t d1,
                uint8_t d2,
                uint8_t d3);
  void init_sync_velocity_mode();
  void enable_motor();
  void set_sync_speed(int16_t left_rpm, int16_t right_rpm);
  void trigger_e_stop();

 private:
  LoopStats* loop_stats_;
};
