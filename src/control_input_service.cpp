#include "ControlInputService.hpp"

namespace {

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}  // namespace

namespace ControlInput {

DriveCommand resolveDriveCommand(ControlTargets& targets,
                                 bool control_enable,
                                 bool remote_active,
                                 int ly,
                                 int lx,
                                 float max_linear_mps,
                                 float max_steer_deg) {
  DriveCommand out;
  out.remote_active = remote_active;

  if (remote_active) {
    out.linear_mps = targets.linear_mps;
    out.angle_deg = targets.angle_deg;
    return out;
  }

  if (control_enable) {
    float axis_forward = -(static_cast<float>(ly) - 128.0f) / 128.0f;
    float axis_turn = -(static_cast<float>(lx) - 128.0f) / 128.0f;

    if (fabsf(axis_forward) < 0.05f) axis_forward = 0.0f;
    if (fabsf(axis_turn) < 0.05f) axis_turn = 0.0f;

    axis_forward = clampf(axis_forward, -1.0f, 1.0f);
    axis_turn = clampf(axis_turn, -1.0f, 1.0f);

    out.linear_mps = axis_forward * max_linear_mps;
    out.angle_deg = axis_turn * max_steer_deg;

    targets.linear_mps = out.linear_mps;
    targets.angle_deg = out.angle_deg;
    return out;
  }

  targets.linear_mps = 0.0f;
  targets.angle_deg = 0.0f;
  out.linear_mps = 0.0f;
  out.angle_deg = 0.0f;
  return out;
}

}  // namespace ControlInput
