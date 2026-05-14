# ESP32 LOLIN Double Ackermann Controller

This converts the Python-based double-Ackermann joystick control into an ESP32 (LOLIN32) firmware with:
- RS485 Modbus RTU for two ZLAC8015D motor drivers (left/right pairs)
- CAN bus (TWAI) for two steering actuators (ODrive-style command frames)
- PS2 gamepad input using a PS2 controller (SPI-like) via PS2X library
- WS2812B (8 LEDs) status display via Adafruit NeoPixel
- USB serial protocol for direct velocity command input over USB
 - WiFi line-based protocol (TCP) matching the USB protocol

## Features
- Modes: Normal (Ackermann) and Zero-Turn (tank spin), switched by gamepad combos
- Deadbanded joystick input and scaling to velocity/angle
- Robust RS485 driver enable handling

## Hardware & Pins
Adjust in `src/main.cpp` if needed.

- RS485 (UART2):
  - TX2 = GPIO17, RX2 = GPIO16 (default LOLIN32)
  - DE/RE = GPIO4 (controls driver direction)
  - Baudrate: 115200, 8N1
  - Both ZLAC devices share the same RS485 bus. Device IDs 1 and 2.

- CAN (TWAI): Needs external CAN transceiver (e.g., SN65HVD230, TJA1050)
  - TX = GPIO5 (TWAI_TX_PIN)
  - RX = GPIO4 (TWAI_RX_PIN)
  - Bitrate = 500 kbit/s
  - You can override pins with PlatformIO build flags or edit `MotorCANController.h` defines.

- PS2 Controller (PlayStation 2 gamepad)
  - ATT = GPIO21, CMD = GPIO23, DAT = GPIO19, CLK = GPIO18 (adjust as needed)
  - Requires 3.3V logic; power with 3.3V and GND

- WS2812B LED strip/ring (8 LEDs)
  - DIN = GPIO13 (default)
  - Power: 5V and GND (share GND with ESP32). Consider a level shifter on DIN for robustness.

- WiFi
  - STA mode: connects to `WIFI_SSID`/`WIFI_PASSWORD` from `include/Config.h`
  - If STA fails, starts AP: `AckermannESP32` / `ackermann123`
  - TCP port: 7777 (editable via `Config.h`)

## Controls
- Hold R1 to enable control (safety)
- START + TRIANGLE → Zero-Turn Mode
- START + CIRCLE → Normal Mode
- Right stick Y = forward/back speed
- Right stick X = steering angle

LEDs (8):
- 0: Control source (Joystick=green, USB=blue)
- 1: Drive mode (Normal=green, Zero=orange)
- 2: Enabled/armed (white when active)
- 3: CAN status (green OK, red fail)
- 4: RS485 status (green OK, red fail)
- 5..7: ambient

## Build & Flash
This project uses PlatformIO (VS Code extension or CLI).

- Install dependencies (one-time):
  - ModbusMaster (auto-installed via `platformio.ini`)
  - PS2X_lib: either add to `platformio.ini` lib_deps or vendor into `lib/PS2X_lib`.
  - Adafruit NeoPixel (auto-installed)

- Quick start (VS Code PlatformIO):
  1. Open this folder.
  2. Connect LOLIN32.
  3. Build and Upload.

- CLI (optional):
```bash
# Requires platformio core installed: pipx install platformio
pio run -t upload
pio device monitor -b 115200
```

If you don't have the PS2X library, the firmware will still compile but joystick input will be disabled (motors idle). Install PS2X_lib to enable input.

## WiFi Protocol
Same as USB serial, but over TCP on port 7777.

Example (netcat):
```bash
nc <esp32-ip> 7777
MODE WIFI
ROBOT NORMAL
CMD_VEL 30 12
STATUS?
```

## Behavior
- Normal mode: computes Ackermann wheel speeds and steering angles from joystick input.
- Zero-turn: fixes steering to opposing positions and drives wheels in opposite directions.
 - Control source can be switched via USB serial protocol.

## Notes
- ZLAC8015D registers/behavior mirror the Python code (velocity mode + register writes).
- CAN frames follow the `(node_id << 5) | cmd` standard-ID scheme used in the original code.
- Ensure proper termination and biasing on RS485 and CAN buses.

## File Map
- `src/main.cpp` – Bootstraps peripherals and runs the control loop
- `include/DoubleAckermann.hpp` – Ackermann math ported from Python
- `lib/ZLAC8015DController/` – Modbus RTU wrapper for ZLAC8015D
- `lib/MotorCANController/` – TWAI/CAN sender for steering actuators
- `lib/JoystickPS2/` – PS2X-based joystick adapter with fallback stub
 - `lib/LEDStatus/` – WS2812B LED status helper

## Tuning
- Speed and angle scalars live in `JoystickPS2.h` and `main.cpp`.
- Adjust `motor_di` signs if wheel directions are inverted.
- Change `WHEELBASE_M`, `TRACK_WIDTH_M` to match your platform.

## Double Ackermann Geometry
For a double Ackermann vehicle, the left and right steering wheels use different angles,
but the chassis still turns about one instantaneous center of rotation.

Definitions:
- `L` = wheelbase
- `W` = track width
- `R` = vehicle centerline turning radius
- `delta_eq` = equivalent bicycle steering angle
- `delta_in` = inner wheel steering angle
- `delta_out` = outer wheel steering angle

Equivalent bicycle model:

```text
tan(delta_eq) = L / R
R = L / tan(delta_eq)
```

Double Ackermann wheel angles for a turn:

```text
delta_in  = atan(L / (R - W/2))
delta_out = atan(L / (R + W/2))
```

### Minimum Spiral Radius
If the steering limit is treated as an equivalent steering command, the minimum centerline
spiral radius is:

```text
R_min = L / tan(delta_max)
```

If the steering limit is the inner wheel mechanical limit, the minimum centerline radius is:

```text
R_min = L / tan(delta_in_max) + W/2
```

Using the current firmware defaults in `src/main.cpp`:
- `WHEELBASE_M = 0.36`
- `TRACK_WIDTH_M = 0.36`
- `MAX_STEER_DEG = 30.0`

Equivalent steering limit:

```text
R_min = 0.36 / tan(30 deg) = 0.624 m
```

Inner wheel steering limit:

```text
R_min = 0.36 / tan(30 deg) + 0.18 = 0.804 m
```

For hardware testing, use a spiral radius of at least `0.8 m` unless you have measured the
true steering geometry and verified a smaller safe radius.

## USB Serial Protocol
Line-based text protocol at 115200 baud. End lines with `\n`.

Commands:
- `CMD_VEL <linear_mps> <angular_rad_s> [timeout_ms]` – set velocity command in SI units. Steering is computed from angular velocity and wheelbase. Timeout defaults to `250 ms`. Use `timeout_ms=0` for infinite timeout (command persists until new command received).
- `STOP` – command zero linear speed and zero steering.
- `STATUS?` – prints a single-line status summary
- `HELP` or `?` – prints command usage.

Examples:
```text
CMD_VEL 0.60 0.25 300
STATUS?
```

### Python USB GUI (CMD_VEL)
A simple desktop GUI is provided in `usb_cmd_vel_gui.py` to send velocity commands over USB serial.

Install dependency:
```bash
pip install pyserial
```

Run:
```bash
python3 usb_cmd_vel_gui.py
```

In the GUI:
- Select your ESP32 serial port (for example `/dev/ttyUSB0` or `/dev/ttyACM0`)
- Keep baud at `115200`
- Enter `linear` (m/s), `angular` (rad/s), and timeout (ms)
  - Use `0` for infinite timeout (command persists until new command)
  - Default is `250` ms for safety
- Click `Send CMD_VEL` or `STOP`
