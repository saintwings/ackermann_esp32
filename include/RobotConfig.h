#pragma once

// ── Active robot configuration ────────────────────────────────────────────────
// Change this one line to switch between robots.
// All firmware source files include this header — never include Config.h or
// Config_2.h directly.
//
//   Config.h   = Robot "ladybug"  (ODrive steering, NET_MODE 2, SERVER_MODE 2)
//   Config_2.h = Robot "esp32-02" (GIM8108 steering, NET_MODE 1, SERVER_MODE 1)
//#include "Config.h"
#include "Config_2.h"
