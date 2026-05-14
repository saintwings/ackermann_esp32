#pragma once

#include <Arduino.h>

#include "SystemState.hpp"

struct DriveCommand {
  float linear_mps{0.0f};
  float angle_deg{0.0f};
  bool remote_active{false};
};

namespace ControlInput {
DriveCommand resolveDriveCommand(ControlTargets& targets,
                                 bool control_enable,
                                 bool remote_active,
                                 int ly,
                                 int lx,
                                 float max_linear_mps,
                                 float max_steer_deg);
}
