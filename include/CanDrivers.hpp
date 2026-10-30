#pragma once

#include <Arduino.h>

#include "SystemState.hpp"

// Serializes CAN request/reply exchanges across tasks: each one drains the shared RX
// queue and then waits for its reply, so two running at once would steal each other's
// frames. Recursive, so a multi-read status call can hold it across its reads. No-op
// until canRxLockInit() has run (as NavLock).
void canRxLockInit();
struct CanRxLock {
  CanRxLock();
  ~CanRxLock();
};

class ODriveCAN {
 public:
  uint32_t node_id;
  float current_target_pos = 0.0f;
  float last_sent_pos = 9999.0f;
  unsigned long last_sent_pos_ms = 0;
  float gear_ratio = 8.0;

  explicit ODriveCAN(uint32_t id, LoopStats* stats = nullptr);

  void set_axis_state(uint32_t state_id);
  void set_controller_mode(int32_t control_mode, int32_t input_mode);
  // force=true bypasses the unchanged-target throttle below — use it for one-shot
  // commands (not the 20ms steering control loop, which relies on the throttle to
  // avoid spamming the bus with an unchanged target every tick).
  void set_position(float pos_turns, float vel_ff = 0.0f, float torque_ff = 0.0f, bool force = false);
  void go_home();

  // Requests Get_Encoder_Estimates over CAN (RTR frame) and blocks up to timeout_ms
  // waiting for the reply. On success, *out_turns is the ODrive's raw pos_estimate in
  // motor/axis turns (not divided by gear_ratio). Returns false on timeout/no response.
  bool get_position_turns(float* out_turns, unsigned long timeout_ms = 200);

  // Clear_Errors; the axis then needs set_axis_state(8) to re-enter closed loop.
  void clear_errors();
  // Waits for the ODrive's periodic heartbeat (axis_error, axis_state).
  bool read_status(MotorStatus* out, unsigned long timeout_ms = 150);

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
  // Moves by <turns> relative to the drive's current position (native relative move —
  // no position readback needed, immune to any software/hardware baseline drift).
  void move_relative_position_turns(float turns);
  void go_home();
  // Read status (0xAE): voltage, current, temperature, mode, fault. Retries once.
  bool read_status(MotorStatus* out, unsigned long timeout_ms = 50);

 private:
  void send_cmd(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t payload_len = 0);

  uint32_t can_id_;
  LoopStats* loop_stats_;
  int32_t last_sent_counts_ = INT32_MIN;
  unsigned long last_sent_ms_ = 0;
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
  // Controlword fault reset (0x80); the driver then needs enable_motor().
  void clear_fault();
  // Controlword 0x06: both motors unpowered (no holding torque); enable_motor() re-enables.
  void release_motor();
  // SDO upload of one object; false on timeout or SDO abort.
  bool read_sdo(uint16_t index, uint8_t sub, uint32_t* out, unsigned long timeout_ms = 30);
  // Status word, error code (0x603F), temperatures and bus voltage.
  bool read_status(MotorStatus* out);

 private:
  LoopStats* loop_stats_;
};
