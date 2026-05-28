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

// Control server WebSocket settings
#define CONTROL_SERVER_ENABLE 1
//****** */
#define CONTROL_SERVER_HOST "192.168.1.152"
//#define CONTROL_SERVER_HOST "192.168.1.43"

#define CONTROL_SERVER_PORT 8765
#define CONTROL_SERVER_PATH "/"
// Robot telemetry publish interval to control_server (milliseconds)
#define ROBOT_TELEMETRY_INTERVAL_MS 500

// Robot identity for control_server.py registration
#define ROBOT_ID "esp32-02"
#define ROBOT_NAME "Robot2"
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

// GIM8108 steering direction signs (set to -1 to invert a side)
#define STEER_GIM_LEFT_SIGN 1
#define STEER_GIM_RIGHT_SIGN 1

// Robot geometry (meters)
// W = track width, H = wheelbase/axle spacing used by zero-turn steering geometry.
#define ROBOT_GEOMETRY_W_M 0.36f
#define ROBOT_GEOMETRY_H_M 0.36f
// Drive wheel diameter used for m/s <-> RPM conversion.
#define DRIVE_WHEEL_DIAMETER_M 0.20f

// Robot motion limits
#define MAX_LINEAR_SPEED_MS 0.6f       // Maximum forward/reverse speed (m/s)
#define MAX_STEERING_ANGLE_DEG 30.0f   // Maximum steering angle (degrees)

// ── SIMCOM A7670X (CAT1-A7670X-V1.02 board) ─────────────────────────────────
#define SIM_UART_NUM        2
#define SIM_UART_BAUD       115200
#define SIM_TX_PIN          17
#define SIM_RX_PIN          18
#define SIM_EN_PIN          21
#define SIM_APN             "internet"
#define SIM_APN_USER        ""
#define SIM_APN_PASS        ""
#define NET_WIFI_FAIL_TIMEOUT_MS     15000UL
#define NET_WIFI_RECOVER_TIMEOUT_MS  30000UL

// Debug log groups (1 = enabled, 0 = disabled)
#define DEBUG_LOG_SENSOR_1HZ 1
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
