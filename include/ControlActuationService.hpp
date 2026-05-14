#pragma once

#include <Arduino.h>
#include <math.h>

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

template <typename RobotT, typename ZlacT, typename OdriveT>
void applyDriveOutputs(bool zero_turn_mode,
                       float linear_mps,
                       float angle_deg,
                       float wheelbase_m,
                       float wheel_diameter_m,
                       int max_rpm,
                       int8_t motor_di_left,
                       int8_t motor_di_right,
                       RobotT& robot,
                       ZlacT& zlac_front,
                       ZlacT& zlac_rear,
                       OdriveT& odrive_1,
                       OdriveT& odrive_2) {
  if (!zero_turn_mode) {
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

    zlac_front.set_sync_speed(left_rpm, right_rpm);
    zlac_rear.set_sync_speed(left_rpm, right_rpm);

    odrive_1.set_position(-state.angle_fl / 45.0f);
    odrive_2.set_position(-state.angle_fr / 45.0f);
    return;
  }

  odrive_1.set_position(1.0f);
  odrive_2.set_position(-1.0f);

  int16_t left_rpm = mpsToRpmClamped(linear_mps, wheel_diameter_m, max_rpm, motor_di_left);
  int16_t right_rpm = mpsToRpmClamped(-linear_mps, wheel_diameter_m, max_rpm, motor_di_right);

  zlac_front.set_sync_speed(left_rpm, right_rpm);
  zlac_rear.set_sync_speed(left_rpm, right_rpm);
}

}  // namespace ControlActuation
