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

inline float computeZeroTurnSteeringAngleDeg(float wheelbase_m, float track_width_m) {
  if (track_width_m < 1e-6f) return 0.0f;
  return atan2f(wheelbase_m, track_width_m) * 180.0f / static_cast<float>(M_PI);
}

inline float steeringAngleDegToTurns(float angle_deg) {
  return angle_deg / 360.0f;
}

template <typename RobotT>
void applyDriveOutputs(bool zero_turn_mode,
                       float linear_mps,
                       float angle_deg,
                       float wheelbase_m,
                       float track_width_m,
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

    steering_motors.setSteeringTurns(-steeringAngleDegToTurns(state.angle_fl), -steeringAngleDegToTurns(state.angle_fr));
    return;
  }

  // ========== ZERO-TURN (TANK DRIVE) MODE ==========
  // Fixed steering angle from geometry: alpha = atan(H / W)
  const float zero_turn_angle_deg = computeZeroTurnSteeringAngleDeg(wheelbase_m, track_width_m);
  const float zero_turn_turns = steeringAngleDegToTurns(zero_turn_angle_deg);
  steering_motors.setSteeringTurns(zero_turn_turns, -zero_turn_turns);


  Serial.println("!!zeroturn");
  Serial.println("H , W");
  Serial.println(wheelbase_m);
  Serial.println(track_width_m);
  Serial.printf("zero_turn_angle_deg: %f\n", zero_turn_angle_deg);



  
  float forward_mps = linear_mps;

  Serial.println("speed");
  Serial.println(forward_mps);

  
  // Create differential: when rotating, one motor increases while other decreases
  float left_mps = forward_mps;
  float right_mps = -forward_mps;
  

  
  int16_t left_rpm = mpsToRpmClamped(left_mps, wheel_diameter_m, max_rpm, motor_di_left);
  int16_t right_rpm = mpsToRpmClamped(right_mps, wheel_diameter_m, max_rpm, motor_di_right);

  drive_motors.setWheelRpm(left_rpm, right_rpm);
}

}  // namespace ControlActuation
