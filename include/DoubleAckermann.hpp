#pragma once
#include <cmath>

struct AckermannState {
  float v_current{0.0f};
  float R_current{1e6f};
  float angle_fl{0.0f}, angle_fr{0.0f}, angle_rl{0.0f}, angle_rr{0.0f};
  float speed_fl{0.0f}, speed_fr{0.0f}, speed_rl{0.0f}, speed_rr{0.0f};
};

class DoubleAckermannSteering {
 public:
  DoubleAckermannSteering(float wheelbase, float track_width)
      : L_(wheelbase), W_(track_width) {}

  void setVelocity(float v) { st_.v_current = v; }
  void setTurnRadius(float R) { st_.R_current = R; }

  const AckermannState& update() {
    st_.angle_fl = computeSteeringAngle(st_.R_current, true);
    st_.angle_fr = computeSteeringAngle(st_.R_current, false);
    st_.angle_rl = -st_.angle_fl;
    st_.angle_rr = -st_.angle_fr;

    st_.speed_fl = computeWheelSpeed(true);
    st_.speed_fr = computeWheelSpeed(false);
    st_.speed_rl = computeWheelSpeed(true);
    st_.speed_rr = computeWheelSpeed(false);
    return st_;
  }

  static float deg2rad(float d) { return d * static_cast<float>(M_PI / 180.0); }
  static float rad2deg(float r) { return r * static_cast<float>(180.0 / M_PI); }

 private:
  float computeSteeringAngle(float R, bool left) const {
    float offset = left ? -W_ / 2.0f : W_ / 2.0f;
    float angle = rad2deg(std::atan((L_ / 2.0f) / (R + offset)));
    if (std::fabs(angle) < 0.1f) angle = 0.0f;
    if (angle > 30.0f) angle = 30.0f;
    if (angle < -30.0f) angle = -30.0f;
    return angle;
  }

  float computeWheelRadius(bool left) const {
    float offset = left ? -W_ / 2.0f : W_ / 2.0f;
    return std::hypot(st_.R_current + offset, L_ / 2.0f);
  }

  float computeWheelSpeed(bool left) const {
    float Rw = computeWheelRadius(left);
    float omega = st_.v_current / std::fabs(st_.R_current);
    return omega * Rw;
  }

  float L_{}; // wheelbase
  float W_{}; // track width
  AckermannState st_{};
};
