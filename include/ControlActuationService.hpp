#pragma once

#include <Arduino.h>
#include <math.h>

#include "MotorInterfaces.hpp"

namespace ControlActuation {

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline int16_t mpsToRpmClamped(float mps, float wheel_diameter_m, int max_rpm, int8_t direction) {
  const float circumference = static_cast<float>(M_PI) * wheel_diameter_m;
  float rpm = 0.0f;
  if (circumference > 1e-6f) {
    rpm = (mps / circumference) * 60.0f;
  }
  rpm *= static_cast<float>(direction);
  rpm = clampf(rpm, -static_cast<float>(max_rpm), static_cast<float>(max_rpm));
  return static_cast<int16_t>(rpm);
}

template <typename RobotT>
void applyDriveOutputs(bool zero_turn_mode,
                       float linear_mps,
                       float angle_deg,
                       float wheelbase_m,
                       float wheel_diameter_m,
                       int max_rpm,
                       int8_t motor_di_left,
                       int8_t motor_di_right,
                       RobotT& robot,
                       IDriveMotorPair& drive_motors,
                       ISteeringMotorPair& steering_motors) {
  if (!zero_turn_mode) {
    // ========== NORMAL ACKERMANN MODE ==========
    float R = 1e6f;
    float angle_rad = RobotT::deg2rad(angle_deg);
    if (fabsf(angle_rad) > 1e-3f) {
      R = (wheelbase_m / 2.0f) / tanf(angle_rad);
    }

    robot.setVelocity(linear_mps);
    robot.setTurnRadius(R);
    const auto& state = robot.update();

    int16_t left_rpm = mpsToRpmClamped(state.speed_fl, wheel_diameter_m, max_rpm, motor_di_left);
    int16_t right_rpm = mpsToRpmClamped(state.speed_fr, wheel_diameter_m, max_rpm, motor_di_right);

    drive_motors.setWheelRpm(left_rpm, right_rpm);

    steering_motors.setSteeringTurns(-state.angle_fl / 45.0f, -state.angle_fr / 45.0f);
    return;
  }

  // ========== ZERO-TURN (TANK DRIVE) MODE ==========
  // Steering wheels fixed at maximum turn: odrive_1 = 1.0, odrive_2 = -1.0
  steering_motors.setSteeringTurns(1.0f, -1.0f);

  // For differential drive:
  // - linear_mps controls forward/backward speed (both motors same direction)
  // - angle_deg controls rotation intensity (differential between motors)
  // 
  // Scaling angle_deg (which is in [-MAX_STEER_DEG, MAX_STEER_DEG]) to differential ratio
  // at angle_deg = 0: both motors same speed (pure forward)
  // at angle_deg = ±MAX_STEER_DEG: pure rotation (differential = 1.0)
  
  float forward_mps = linear_mps;
  float rotation_ratio = fabsf(angle_deg) / 30.0f;  // Normalize to [0, 1] assuming MAX_STEER_DEG ≈ 30
  rotation_ratio = clampf(rotation_ratio, 0.0f, 1.0f);
  
  // Create differential: when rotating, one motor increases while other decreases
  float left_mps = forward_mps;
  float right_mps = forward_mps;
  
  if (fabsf(angle_deg) > 0.1f) {
    // angle_deg > 0: rotate left (increase left motor differential)
    // angle_deg < 0: rotate right (increase right motor differential)
    float rotation_component = forward_mps * rotation_ratio;
    if (angle_deg > 0.0f) {
      left_mps += rotation_component;
      right_mps -= rotation_component;
    } else {
      left_mps -= rotation_component;
      right_mps += rotation_component;
    }
  }
  
  int16_t left_rpm = mpsToRpmClamped(left_mps, wheel_diameter_m, max_rpm, motor_di_left);
  int16_t right_rpm = mpsToRpmClamped(right_mps, wheel_diameter_m, max_rpm, motor_di_right);

  drive_motors.setWheelRpm(left_rpm, right_rpm);
}

}  // namespace ControlActuation
