#pragma once

#include <stdint.h>

#include "CanDrivers.hpp"

class IDriveMotorPair {
 public:
  virtual ~IDriveMotorPair() = default;
  virtual void begin() = 0;
  virtual void enable() = 0;
  virtual void emergencyStop() = 0;
  virtual void setWheelRpm(int16_t left_rpm, int16_t right_rpm) = 0;
};

class ISteeringMotorPair {
 public:
  virtual ~ISteeringMotorPair() = default;
  virtual void begin() = 0;
  virtual void enable() = 0;
  virtual void emergencyStop() = 0;
  virtual void goHome() = 0;
  virtual void setSteeringTurns(float left_turns, float right_turns) = 0;
};

class ZlacDriveMotorPair final : public IDriveMotorPair {
 public:
  ZlacDriveMotorPair(uint32_t front_id, uint32_t rear_id, LoopStats* stats)
      : front_(front_id, stats), rear_(rear_id, stats) {}

  void begin() override {
    front_.init_sync_velocity_mode();
    rear_.init_sync_velocity_mode();
  }

  void enable() override {
    front_.enable_motor();
    rear_.enable_motor();
  }

  void emergencyStop() override {
    front_.trigger_e_stop();
    rear_.trigger_e_stop();
  }

  void setWheelRpm(int16_t left_rpm, int16_t right_rpm) override {
    front_.set_sync_speed(left_rpm, right_rpm);
    rear_.set_sync_speed(left_rpm, right_rpm);
  }

 private:
  ZLAC8015D front_;
  ZLAC8015D rear_;
};

class OdriveSteeringMotorPair final : public ISteeringMotorPair {
 public:
  OdriveSteeringMotorPair(uint32_t left_id, uint32_t right_id, LoopStats* stats)
      : left_(left_id, stats), right_(right_id, stats) {}

  void begin() override {
    left_.set_controller_mode(3, 1);
    right_.set_controller_mode(3, 1);
    enable();
#if ODRIVE_STARTUP_HOME_ENABLE
    goHome();
#else
    setSteeringTurns(0.0f, 0.0f);
#endif
  }

  void enable() override {
    left_.set_axis_state(8);
    right_.set_axis_state(8);
  }

  void emergencyStop() override {
    left_.set_axis_state(1);
    right_.set_axis_state(1);
  }

  void goHome() override {
    left_.go_home();
    right_.go_home();
  }

  void setSteeringTurns(float left_turns, float right_turns) override {
    left_.set_position(left_turns);
    right_.set_position(right_turns);
  }

 private:
  ODriveCAN left_;
  ODriveCAN right_;
};

class Gim8108SteeringMotorPair final : public ISteeringMotorPair {
 public:
  Gim8108SteeringMotorPair(uint32_t left_id,
                           uint32_t right_id,
                           LoopStats* stats,
                           int8_t left_sign = 1,
                           int8_t right_sign = 1)
      : left_(left_id, stats),
        right_(right_id, stats),
        left_sign_(left_sign),
        right_sign_(right_sign) {}

  void begin() override {
    left_.set_position_max_speed_rpm(100.0f);
    right_.set_position_max_speed_rpm(100.0f);
    left_.set_max_current_amp(5.0f);
    right_.set_max_current_amp(5.0f);
    left_.set_acceleration_rpm_per_sec(200.0f);
    right_.set_acceleration_rpm_per_sec(200.0f);
    enable();
    goHome();
  }

  void enable() override {
    left_.clear_fault();
    right_.clear_fault();
  }

  void emergencyStop() override {
    left_.disable_motor();
    right_.disable_motor();
  }

  void goHome() override {
    left_.go_home();
    right_.go_home();
  }

  void setSteeringTurns(float left_turns, float right_turns) override {
    left_.set_absolute_position_turns(static_cast<float>(left_sign_) * left_turns);
    right_.set_absolute_position_turns(static_cast<float>(right_sign_) * right_turns);
  }

 private:
  GIM8108CAN left_;
  GIM8108CAN right_;
  int8_t left_sign_;
  int8_t right_sign_;
};
