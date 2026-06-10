# Ackermann ESP32 Robot — Project Summary

## 1. Project Overview

An outdoor Ackermann-drive autonomous robot built on an ESP32-S3. It connects to a cloud control server over either WiFi or a 4G LTE SIM card (fallback / field use), receives tele-operation commands and mission plans, and publishes sensor telemetry. The robot can also execute GPS-guided waypoint missions autonomously.

---

## 2. Hardware

| Component | Part / Config |
|---|---|
| MCU | ESP32-S3 DevKit-C (dual-core 240 MHz) |
| Drive motors | ZLAC brushless pair — CAN bus (500 kbps) |
| Steering motors | ODrive *or* GIM8108 — CAN bus (config-selectable) |
| GPS | u-blox ZED-F9P RTK (UART1, 38 400 baud) |
| IMU | Adafruit BNO085 (SPI) |
| LTE modem | FS-HCore-A7670C (SIMCOM A7670C-LNNV, firmware V11.0.01) — UART2 |
| Controller | Sony PS2 gamepad (SPI bit-bang via PS2X) |
| Status LEDs | WS2812B × 8 (NeoPixel, GPIO 8) |
| CAN | ESP32-S3 TWAI peripheral |

### Two robot configs (same codebase)

| | `Config.h` — **ladybug** | `Config_2.h` — **esp32-02** |
|---|---|---|
| Steering | ODrive | GIM8108 |
| Geometry | 0.36 m × 0.36 m | 0.70 m × 1.10 m |
| Wheel dia | 0.20 m | 0.23 m |
| Max speed | 0.2 m/s | 0.6 m/s |
| Net mode | SIM only | WiFi only |

Active config is chosen in `include/RobotConfig.h` (one `#include` line).

---

## 3. Software Architecture

### Build environment
- PlatformIO, Arduino framework, ESP-IDF TWAI driver
- C++17 (`-std=gnu++17`)
- Key libraries: `TinyGSM`, `WebSockets` (Links2004), `ArduinoJson`, `Adafruit BNO08x`, `Adafruit NeoPixel`, `PS2X_lib`

### FreeRTOS task layout

```
Core 0                          Core 1
────────────────────────────    ─────────────────────────────
commsTask  (priority 2)         controlTask  (priority 3)
  20 ms nominal loop              20 ms nominal loop
  vTaskDelay(2 ms)                vTaskDelay(1 ms)
  ─ net_manager.update()          ─ ps2x.read_gamepad()
  ─ robot_client.update()         ─ E-stop toggle (Circle btn)
  ─ handleUsbSerialControl()      ─ Mode switch (R1+R2+△/✕)
  ─ NTRIP connect/poll            ─ ControlActuation::applyDriveOutputs()
  ─ GPS + IMU update
  ─ safety guards
  ─ mission executor
  ─ telemetry publish
  ─ status LEDs
```

### Module map

```
main.cpp
├── NetManager          — WiFi + SIM bearer selection
│   └── TinyGSM         — SIM modem AT command driver
├── RobotClient         — WebSocket client (both transports)
│   ├── WebSocketsClient  (WiFi path — Links2004 library)
│   └── SimWsClient       (SIM path — custom AT+CIPOPEN driver)
├── SensorCommsService  — GPS (NMEA parser), NTRIP, BNO085 IMU
├── ControlInputService — PS2 / remote command priority
├── ControlActuationService — Ackermann / zero-turn math → CAN
├── DoubleAckermann     — steering geometry calculations
└── MotorInterfaces     — ZLAC, ODrive, GIM8108 CAN drivers
```

---

## 4. Network Architecture

### NET_MODE options (Config.h)

| Value | Behaviour |
|---|---|
| 1 | WiFi primary; SIM fallback after 15 s WiFi loss |
| 2 | SIM only (WiFi radio off) |
| 3 | WiFi only |

### WiFi path
- Static IP, auto-reconnect every 8 s
- Uses `WebSocketsClient` (Links2004) with SSL (`beginSSL`) on port 443
- Telemetry every 500 ms (`ROBOT_TELEMETRY_INTERVAL_MS`)

### SIM path (A7670C LTE modem)

**Boot sequence** (NetManager state machine):
```
IDLE → POWERING_ON (10 s boot) → BOOTING (modem.init())
     → REGISTERING (network attach) → GPRS_CONNECT (PDP context / APN)
     → READY (keep-alive every 30 s, unless WS client owns the serial)
```

**Key constraint — firmware V11.0.01:**  
`AT+CSSLCFG` and `AT+CCH` commands do not exist in this firmware version. SSL via `AT+CIPOPEN="SSL"` fails with error code 3 (cert verify failure, un-bypassable). The workaround is **plain TCP on port 80** through Cloudflare (see Server Setup below).

**SimWsClient — custom WebSocket over AT commands:**

| AT command | Purpose |
|---|---|
| `AT+CIPRXGET=1` | Enable manual receive mode (required before CIPOPEN) |
| `AT+CIPOPEN=1,"TCP","host",80` | Open plain TCP connection on mux 1 |
| `AT+CIPSEND=1,N` → `>` → data | Send N bytes |
| `AT+CIPRXGET=2,1,N` | Read up to N bytes from modem buffer |
| `AT+CIPCLOSE=1` | Close connection |

WebSocket framing follows RFC 6455: client→server frames are masked with `{0x3C,0xA1,0x55,0xF2}`. Payloads ≥ 126 bytes use the 16-bit extended length header.

**HTTP header consumption:** Cloudflare adds `CF-RAY` and `alt-svc` headers to the 101 Switching Protocols response. `wsHandshake()` accumulates all bytes until `\r\n\r\n` before returning, saving any post-header bytes directly into the WS receive buffer.

**Post-send drain:** After each `cipSend()`, a `cipRecv()` drain is called (300 ms timeout) to capture any incoming WS frames that arrived while the send was blocking. This prevents URCs from being lost and keeps the connection responsive.

- Telemetry every 1 500–2 000 ms (`SIM_TELEMETRY_INTERVAL_MS`) — slower because each AT+CIPSEND round-trip blocks 200–500 ms
- Periodic WS ping every 15 s (Cloudflare idle timeout protection)
- Auto-reconnect every 12 s on connection loss

---

## 5. Server & Cloud Setup

### Control server (`control_server.py`)
- Python WebSocket server, pure WebSocket protocol (no MQTT)
- Runs locally (or on a VPS) behind a `cloudflared` tunnel daemon
- Exposes `robot.saintwings.xyz` via Cloudflare Tunnel

### Cloudflare Tunnel setup
```
ESP32 robot ──TCP:80──► Cloudflare edge ──TLS──► cloudflared ──► control_server.py
                         (SSL termination)
```
- **WiFi robots:** connect to `wss://robot.saintwings.xyz:443` — full TLS from ESP32 to Cloudflare edge
- **SIM robots:** connect to `http://robot.saintwings.xyz:80` — plain TCP; Cloudflare re-encrypts on the Cloudflare→origin leg
- **Required Cloudflare setting:** SSL/TLS → "Always Use HTTPS" must be **OFF** to allow port 80 plain HTTP through

### WebSocket message protocol

All messages are JSON with a `type` field:

| Type | Direction | Purpose |
|---|---|---|
| 0 (`REGISTER`) | Robot → Server | Robot announce: id, name, type, max_speed |
| 1 (`SENSOR_TELEMETRY`) | Robot → Server | GPS, IMU, odometry, mission state |
| 2 (`COMMAND_ACK`) | Robot → Server | Acknowledge command receipt |
| 10 (`COMMAND_MOVE`) | Server → Robot | Velocity command (linear m/s, angular rad/s, duration s) |
| 11 (`COMMAND_WAYPOINT`) | Server → Robot | Single GPS waypoint |
| 12 (`EMERGENCY_STOP`) | Server → Robot | Halt and lock all motion |
| 13 (`CANCEL_TASK`) | Server → Robot | Cancel current task |
| 14 (`EXECUTE_MISSION`) | Server → Robot | Full mission plan JSON |
| 15 (`PAUSE_MISSION`) | Server → Robot | Toggle pause/resume |
| 16 (`CANCEL_MISSION`) | Server → Robot | Abort mission |
| 99 (`HEARTBEAT`) | Server → Robot | Keep-alive (ignored by robot) |

---

## 6. GPS & RTK

- ZED-F9P parses NMEA on UART1
- NTRIP client fetches RTCM3 corrections from `110.78.0.54:2116` (VRS_RTCM32 mountpoint, Thailand)
- Corrections are forwarded to the ZED-F9P over the same UART
- Fix quality 4 = RTK Fixed (cm accuracy), 5 = RTK Float (dm accuracy)
- NTRIP uses the active TCP client (WiFiClient or TinyGsmClient) — automatically reconnects on bearer switch

---

## 7. IMU

- BNO085 connected via SPI (SCK=4, MISO=5, MOSI=6, CS=7, INT=15, RST=16)
- Reports rotation quaternion, linear acceleration, gyroscope, magnetometer
- Heading is derived from quaternion yaw: `atan2(cosy_cosp, siny_cosp)`
- Stale timeout: 1 500 ms — invalid IMU aborts active mission after 2 s grace period

---

## 8. Drive & Steering

### Drive
- ZLAC brushless motor pair (front + rear axle), CAN IDs 1 and 2
- Speed in RPM; conversion: `RPM = (m/s / (π × diameter)) × 60`
- Direction sign configurable per-robot (`DRIVE_LEFT_SIGN`, `DRIVE_RIGHT_SIGN`)

### Steering
- **ODrive** (ladybug): field-oriented FOC, position control in turns
- **GIM8108** (esp32-02): gimbal motor, position control in turns, direction sign configurable
- Steering angle computed by `DoubleAckermannSteering` — inner/outer wheel get different angles

### Drive modes

| Mode | Behaviour |
|---|---|
| Normal | Ackermann geometry — turn by steering angle |
| Zero-turn | Both steering motors set to opposite max angles; spin in place |

Mode toggle: PS2 `R1+R2+△` (zero-turn) / `R1+R2+✕` (normal), or USB serial `MODE ZERO_TURN` / `MODE NORMAL`.

---

## 9. Mission System

Missions are JSON plans sent via `EXECUTE_MISSION`. Supported task types:

| Task type | Description |
|---|---|
| `goto_waypoint` | Drive to a single GPS waypoint |
| `coverage_path` | Drive through an ordered list of waypoints (coverage survey) |
| `wait` | Hold position for N seconds |
| `loop_start` / `loop_end` | Repeat a block N times |
| `spiral` | Time-based outward spiral (no GPS needed) |

Navigation uses haversine distance + bearing → proportional heading error → steering angle. Waypoint reached threshold: 1.5 m (goto) / 0.5 m (coverage). Mission aborts if GPS or IMU is invalid for > 2 s.

---

## 10. Control Priority

```
1. E-stop (PS2 Circle btn or EMERGENCY_STOP command) — overrides everything
2. Mission executor — sets remote_move_expire_ms = now + 150 ms each tick
3. Remote WebSocket (COMMAND_MOVE) — sets expire timestamp
4. PS2 manual (R1 held)
5. USB serial CMD_VEL command
```

A remote command expires if no refresh arrives within its timeout window; motors return to zero automatically.

---

## 11. Status LEDs (WS2812B × 8)

| LED | Meaning |
|---|---|
| 0 | System health: Green=OK, Red=fault, Blinking red=E-stop |
| 1 | Network: Green=WiFi, Blue=SIM, Blinking yellow=trying, Red=no network |
| 2 | GPS fix: Green=RTK fixed (q4), Blue=RTK float (q5), Red=no fix |
| 3 | IMU: Green=valid, Red=invalid |
| 4 | NTRIP: Green=connected, Red=disconnected, Yellow=disabled |
| 5 | Control source: Cyan=remote, Magenta=PS2 available, Yellow=none, Red=E-stop |
| 6 | Mission state: Blue=idle, Yellow=queued, Green=running, Orange=paused, Cyan=done, White=cancelled, Red=failed |
| 7 | Loop timing: Green=normal, Red=overrun |

---

## 12. USB Serial Interface

Connect via USB CDC at 115 200 baud. Commands:

```
CMD_VEL <linear_mps> <angular_rad_s> [timeout_ms]   # drive command
STOP                                                  # immediate stop
MODE <NORMAL|ZERO_TURN>                               # switch drive mode
STATUS?                                               # print current state
HELP                                                  # show commands
```

`timeout_ms=0` means the command persists indefinitely until replaced.

---

## 13. Key Configuration Defines

| Define | File | Notes |
|---|---|---|
| `NET_MODE` | Config.h | 1=WiFi+SIM, 2=SIM only, 3=WiFi only |
| `SERVER_MODE` | Config.h | 1=local, 2=Cloudflare Tunnel |
| `SIM_CONTROL_SERVER_PORT` | Config.h | 80 for SIM (plain TCP through CF) |
| `ROBOT_TELEMETRY_INTERVAL_MS` | Config.h | WiFi telemetry rate (default 500 ms) |
| `SIM_TELEMETRY_INTERVAL_MS` | Config.h | SIM telemetry rate (≥ 1 500 ms) |
| `DRIVE_LEFT_SIGN` / `DRIVE_RIGHT_SIGN` | Config.h | Invert motor direction |
| `STEER_MOTOR_TYPE` | Config.h | `STEER_MOTOR_TYPE_ODRIVE` or `_GIM8108` |

Switch between robot configs by editing the single `#include` in `include/RobotConfig.h`.
