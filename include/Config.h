#pragma once

// WiFi credentials
// Fill with your network credentials. If STA connect fails, firmware starts an AP.
//****** */
#define WIFI_SSID     "raina_robofarm_2.4G"
#define WIFI_PASSWORD "h1pp0p0tamu5"
// #define WIFI_SSID     "Book_2G"
// #define WIFI_PASSWORD "B0899228122K"
// #define WIFI_SSID     "RAINET-IOT"
// #define WIFI_PASSWORD "h1pp0p0tamu5"

// Static IP configuration
//****** */
#define WIFI_STATIC_IP      "192.168.1.41"
//#define WIFI_STATIC_IP      "192.168.1.140"

#define WIFI_STATIC_GATEWAY "192.168.1.1"
#define WIFI_STATIC_SUBNET  "255.255.255.0"

// SoftAP fallbacks
#define WIFI_AP_SSID     "AckermannESP32"
#define WIFI_AP_PASSWORD "ackermann123"

// TCP server port for line-based protocol
#define WIFI_SERVER_PORT 7777

// ── Server connection mode ────────────────────────────────────────────────────
// 1 = LOCAL   — control server on local WiFi  (ws://, no SSL)
// 2 = PUBLIC  — control server via Cloudflare Tunnel  (wss://, SSL)
#define SERVER_MODE 2

#if SERVER_MODE == 1
  #define CONTROL_SERVER_HOST    "192.168.1.166"
  #define CONTROL_SERVER_PORT    8765
  #define CONTROL_SERVER_USE_SSL 0
  #define SIM_CONTROL_SERVER_PORT 8765
#elif SERVER_MODE == 2
  #define CONTROL_SERVER_HOST    "robot.saintwings.xyz"
  #define CONTROL_SERVER_PORT    443
  #define CONTROL_SERVER_USE_SSL 1
  // SIM uses plain TCP on port 80 (Cloudflare HTTP).
  // Requires "Always Use HTTPS" = OFF in Cloudflare SSL/TLS settings.
  #define SIM_CONTROL_SERVER_PORT 80
#endif

#define CONTROL_SERVER_ENABLE 1
#define CONTROL_SERVER_PATH "/"
#define ROBOT_TELEMETRY_INTERVAL_MS 500
#define SIM_TELEMETRY_INTERVAL_MS   2000 // should not less than 1500 for SIM

#define SIM_CONTROL_ENABLE  (SERVER_MODE == 2 ? 1 : 0)

// Robot identity for control_server.py registration
#define ROBOT_ID "ladybug_001"
#define ROBOT_NAME "ladybug"
#define ROBOT_TYPE "ackermann"

// ZED-F9P UART settings (UART1)
#define GPS_UART_BAUD 38400
#define GPS_UART_RX_PIN 9
#define GPS_UART_TX_PIN 10

// NTRIP client settings (for RTK corrections)
#define NTRIP_ENABLE 1
#define NTRIP_HOST "110.78.0.54"
#define NTRIP_PORT 2116
#define NTRIP_MOUNTPOINT "VRS_RTCM32"
#define NTRIP_USER "1200100213690"
#define NTRIP_PASSWORD "EE14"

// BNO085 SPI pin settings (remapped to commonly exposed DevKit GPIOs)
#define BNO085_SPI_SCK_PIN 4
#define BNO085_SPI_MISO_PIN 5
#define BNO085_SPI_MOSI_PIN 6
#define BNO085_SPI_CS_PIN 7
#define BNO085_SPI_INT_PIN 15
#define BNO085_SPI_RST_PIN 16

#define CAN_TX_PIN GPIO_NUM_1
#define CAN_RX_PIN GPIO_NUM_2

#define PS2_ATT GPIO_NUM_14
#define PS2_CMD GPIO_NUM_12
#define PS2_DAT GPIO_NUM_13
#define PS2_CLK GPIO_NUM_11

// Mission output channels A-D (dummy GPIO — replace with real relay pins when wired)
#define OUTPUT_A_PIN 35
#define OUTPUT_B_PIN 36
#define OUTPUT_C_PIN 37
#define OUTPUT_D_PIN 38
#define OUTPUT_COUNT 4

// ── USB serial FUNC_<n> actuator mapping ────────────────────────────────────
// Lets a USB serial command drive one named actuator:
//   FUNC_<n> ON|OFF          — GPIO/relay actuator: drive it directly, no timer
//   FUNC_<n> <degrees> DEG   — CAN position actuator (ODRIVE/GIM8108):
//                              move +<degrees> from home, hold for FUNC_DEG_HOLD_MS,
//                              then return to 0 and hold there
//   FUNC_<n> <degrees> REL_DEG — CAN position actuator: move +<degrees> relative to
//                              its current position and just hold there
//
// FUNC_<n>_TYPE is one of FUNC_TYPE_GPIO / FUNC_TYPE_ODRIVE / FUNC_TYPE_GIM8108.
// FUNC_<n>_ID is:
//   - FUNC_TYPE_GPIO:    index into OUTPUT_A..D (0=A 1=B 2=C 3=D), reuses those pins
//   - FUNC_TYPE_ODRIVE / FUNC_TYPE_GIM8108: CAN node/axis id of that actuator
#define FUNC_TYPE_GPIO    0
#define FUNC_TYPE_ODRIVE  1
#define FUNC_TYPE_GIM8108 2

#define FUNC_COUNT 2

#define FUNC_0_TYPE FUNC_TYPE_GPIO
#define FUNC_0_ID   0   // OUTPUT_A_PIN


#define FUNC_1_TYPE FUNC_TYPE_ODRIVE
#define FUNC_1_ID   3   // ODrive CAN node id 3

// How long a DEG-type actuator holds at the target position before it auto-returns to 0.
#define FUNC_DEG_HOLD_MS 1000UL

// REL_DEG busy-guard: a new REL_DEG command is rejected while the previous one hasn't
// finished yet, so rapid re-presses / button-debounce glitches can't chain onto a move
// still in flight and compound into runaway motion.
//   ODRIVE:  compares live position (read over CAN) against the last commanded target —
//            "arrived" means within this many degrees of it.
#define FUNC_REL_DEG_ARRIVAL_TOLERANCE_DEG 10.0f
// If ODRIVE's arrival check keeps rejecting the same mismatch for longer than this, the
// tracked baseline is desynced from reality (not "still moving" — a normal move settles in
// well under this) and gets auto-resynced to the live reading instead of blocking forever.
#define FUNC_REL_DEG_STUCK_TIMEOUT_MS 3000UL
//   GIM8108: has no position readback here, so it uses a fixed busy window instead —
//            how long (ms) a REL_DEG move is assumed to take before another is allowed.
#define FUNC_REL_DEG_GIM_BUSY_MS 500UL

// WS2812B NeoPixel Status LEDs (8 LEDs)
#define NEOPIXEL_PIN 8
#define NEOPIXEL_COUNT 8
// 0..255 (lower is dimmer)
#define NEOPIXEL_BRIGHTNESS 32

// Motor backend selection (switch implementations without changing control code)
#define DRIVE_MOTOR_TYPE_ZLAC 1
#define DRIVE_MOTOR_TYPE DRIVE_MOTOR_TYPE_ZLAC

#define STEER_MOTOR_TYPE_ODRIVE 1
#define STEER_MOTOR_TYPE_GIM8108 2
#define STEER_MOTOR_TYPE STEER_MOTOR_TYPE_ODRIVE

// Motor CAN IDs used by backend adapters
#define DRIVE_FRONT_MOTOR_ID 1
#define DRIVE_REAR_MOTOR_ID 2
#define STEER_LEFT_MOTOR_ID 1
#define STEER_RIGHT_MOTOR_ID 2

// Drive direction signs (set to -1 to invert a side)
#define DRIVE_LEFT_SIGN 1
#define DRIVE_RIGHT_SIGN -1


// GIM8108 steering direction signs (set to -1 to invert a side)
#define STEER_GIM_LEFT_SIGN 1
#define STEER_GIM_RIGHT_SIGN 1

// Robot geometry (meters)
// W = track width, H = wheelbase/axle spacing used by zero-turn steering geometry.
#define ROBOT_GEOMETRY_W_M 0.36f
#define ROBOT_GEOMETRY_H_M 0.36f
// #define ROBOT_GEOMETRY_W_M 0.7f
// #define ROBOT_GEOMETRY_H_M 1.1f
// Drive wheel diameter used for m/s <-> RPM conversion.
#define DRIVE_WHEEL_DIAMETER_M 0.2f

// Robot motion limits
#define MAX_LINEAR_SPEED_MS 0.5f       // Maximum forward/reverse speed (m/s)
#define MAX_STEERING_ANGLE_DEG 35.0f   // Maximum steering angle (degrees)

// Heading offset (degrees) added after IMU yaw computation.
// Positive = rotate heading clockwise. Range: any value, normalised to 0-360.
#define HEADING_OFFSET_DEG 40.0f

// ── SIMCOM A7670X (CAT1-A7670X-V1.02 board) ─────────────────────────────────
// Uses ESP32-S3 UART2.  Board pins: VCC=5V rail, GND, TXD→GPIO18, RXD←GPIO17,
// EN→GPIO21 (HIGH=on), CTS=leave open, VDD=leave open.
#define SIM_UART_NUM        2
#define SIM_UART_BAUD       115200
#define SIM_TX_PIN          17      // ESP32 UART2 TX → board RXD
#define SIM_RX_PIN          18      // board TXD      → ESP32 UART2 RX
// Enable pin for modem power control. Set to -1 if the board auto-powers on.
// FS-HCore-A7670C: board powers on automatically, no EN pin needed.
#define SIM_EN_PIN          -1


// APN for your SIM card (Thailand: AIS/DTAC/TrueMove H all use "internet")
#define SIM_APN             "internet"
#define SIM_APN_USER        ""
#define SIM_APN_PASS        ""

// ── Network mode ──────────────────────────────────────────────────────────────
// 0 = Offline  — no WiFi, no SIM, no NTRIP (serial-only control, fastest loop)
// 1 = WiFi primary, SIM fallback
// 2 = SIM only (WiFi disabled)
// 3 = WiFi only (no SIM)
#define NET_MODE 0  // 0=offline, 1=WiFi+SIM fallback, 2=SIM only, 3=WiFi only

#define NET_WIFI_FAIL_TIMEOUT_MS     15000UL
#define NET_WIFI_RECOVER_TIMEOUT_MS  30000UL

// Debug log groups (1 = enabled, 0 = disabled)
#define DEBUG_LOG_SENSOR_1HZ 0
#define DEBUG_LOG_SENSOR_COMMS 0
#define DEBUG_LOG_MISSION 0
#define DEBUG_LOG_SAFETY 0
#define DEBUG_LOG_ROBOT_CLIENT 0

// -----------------------------------------------------------------------------
// Status LED Legend (WS2812B, index 0..7)
// -----------------------------------------------------------------------------
// LED0: Overall system health
//   - Green: System OK
//   - Red: Fault / Not OK
//   - Blinking Red: E-stop active
//
// LED1: Internet connection type
//   - Green:          WiFi connected (primary)
//   - Blue:           SIM / cellular connected (fallback)
//   - Blinking Yellow: No network — actively trying (WiFi lost / SIM connecting)
//   - Red:            No network and not connecting
//
// LED2: GPS fix quality
//   - Green: Fix quality = 4 (RTK Fixed)
//   - Blue:  Fix quality = 5 (RTK Float)
//   - Red:   Invalid / other fix states
//
// LED3: IMU
//   - Green: IMU ready and valid
//   - Red: IMU invalid / not ready
//
// LED4: NTRIP
//   - Green: Connected
//   - Red: Disconnected
//   - Yellow: NTRIP disabled in config
//
// LED5: Control source
//   - Cyan: Remote command active
//   - Magenta: Manual (PS2) available
//   - Yellow: No active command source
//   - Red: E-stop
//
// LED6: Mission state
//   - Blue: Idle
//   - Yellow: Queued
//   - Green: Running
//   - Orange: Paused
//   - Cyan: Completed
//   - White: Cancelled
//   - Red: Failed
//   - Blinking Red: E-stop state
//
// LED7: Control loop health
//   - Green: Normal loop timing
//   - Red: Loop overrun detected
