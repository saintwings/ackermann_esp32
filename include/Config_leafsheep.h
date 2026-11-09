#pragma once

// =============================================================================
//  Robot configuration — "leafsheep"
// -----------------------------------------------------------------------------
//  Sections:
//    1. Robot identity
//    2. Network            (mode, WiFi, SIM modem)
//    3. Control server     (connection, telemetry)
//    4. GPS / RTK          (ZED-F9P, NTRIP)
//    5. IMU / heading      (BNO085, heading source, GPS heading fusion)
//    6. Core pin map       (CAN, PS2, mission outputs, NeoPixel)
//    7. Motors             (backends, CAN IDs, direction signs, e-stop)
//    8. Geometry & motion limits
//    9. Mission            (GPS gate, waypoints, link loss, auto zero-turn)
//   10. FUNC actuators     (USB serial FUNC_<n>, joystick combo)
//   11. Add-on I/O modules (I2C, DS3502, PCF8574 limit switches, linear actuator)
//   12. Debug logging
//   13. Status LED legend
// =============================================================================


// =============================================================================
//  1. ROBOT IDENTITY
// =============================================================================

// Robot identity for control_server.py registration
#define ROBOT_ID "leafsheep_001"
#define ROBOT_NAME "leafsheep"
#define ROBOT_TYPE "ackermann"


// =============================================================================
//  2. NETWORK
// =============================================================================

// ── Network mode ──────────────────────────────────────────────────────────────
// 0 = Offline  — no WiFi, no SIM, no NTRIP (serial-only control, fastest loop)
// 1 = WiFi primary, SIM fallback
// 2 = SIM only (WiFi disabled)
// 3 = WiFi only (no SIM)
#define NET_MODE 2  // 0=offline, 1=WiFi+SIM fallback, 2=SIM only, 3=WiFi only

#define NET_WIFI_FAIL_TIMEOUT_MS     15000UL
#define NET_WIFI_RECOVER_TIMEOUT_MS  30000UL

// ── WiFi credentials ─────────────────────────────────────────────────────────
// Fill with your network credentials. If STA connect fails, firmware starts an AP.
//****** */
#define WIFI_SSID     "raina_robofarm_2.4G"
#define WIFI_PASSWORD "h1pp0p0tamu5"
// #define WIFI_SSID     "Book_2G"
// #define WIFI_PASSWORD "B0899228122K"
// #define WIFI_SSID     "RAINET-IOT"
// #define WIFI_PASSWORD "h1pp0p0tamu5"

// ── Static IP configuration ──────────────────────────────────────────────────
//****** */
#define WIFI_STATIC_IP      "192.168.1.42"
//#define WIFI_STATIC_IP      "192.168.1.140"

#define WIFI_STATIC_GATEWAY "192.168.1.1"
#define WIFI_STATIC_SUBNET  "255.255.255.0"

// ── SoftAP fallback ──────────────────────────────────────────────────────────
#define WIFI_AP_SSID     "leafsheep"
#define WIFI_AP_PASSWORD "leafsheep001"

// TCP server port for line-based protocol
#define WIFI_SERVER_PORT 7777

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


// =============================================================================
//  3. CONTROL SERVER
// =============================================================================

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
#define SIM_CONTROL_ENABLE  (SERVER_MODE == 2 ? 1 : 0)

// ── Telemetry ────────────────────────────────────────────────────────────────
#define ROBOT_TELEMETRY_INTERVAL_MS 500
#define SIM_TELEMETRY_INTERVAL_MS   2000 // should not less than 1500 for SIM


// =============================================================================
//  4. GPS / RTK
// =============================================================================

// ── ZED-F9P UART settings (UART1) ────────────────────────────────────────────
#define GPS_UART_BAUD 38400
#define GPS_UART_RX_PIN 9
#define GPS_UART_TX_PIN 10
// ZED-F9P output config sent at every boot (RAM only): GGA+GST on, GSV/GSA/GLL/VTG/RMC off.
// Frees the 38400 link so every epoch's GGA gets through.
#define GPS_UBX_CONFIG_ENABLE 1
// Measurement period in ms (100 = 10 Hz). 0 = leave the receiver's current rate.
#define GPS_NAV_RATE_MS 0

// Echo GPS data to USB Serial as JSON, same fields as the "gps" object sent
// to the web UI in robot telemetry (valid/latitude/longitude/altitude/hdop/
// fix_quality/satellites), at the same telemetry interval.
#define GPS_SERIAL_OUTPUT_ENABLE 0

// ── NTRIP client settings (for RTK corrections) ─────────────────────────────
#define NTRIP_ENABLE 1
#define NTRIP_HOST "110.78.0.54"
#define NTRIP_PORT 2116
#define NTRIP_MOUNTPOINT "VRS_RTCM32"
#define NTRIP_USER "1100700706272"
#define NTRIP_PASSWORD "EC47"


// =============================================================================
//  5. IMU / HEADING
// =============================================================================

// ── BNO085 SPI pin settings (remapped to commonly exposed DevKit GPIOs) ─────
#define BNO085_SPI_SCK_PIN 4
#define BNO085_SPI_MISO_PIN 5
#define BNO085_SPI_MOSI_PIN 6
#define BNO085_SPI_CS_PIN 7
#define BNO085_SPI_INT_PIN 15
#define BNO085_SPI_RST_PIN 16

// ── Heading source ───────────────────────────────────────────────────────────
// 0 = BNO08x rotation vector (magnetometer north, as before);
// 1 = gyro-only game rotation vector corrected by GPS course while driving straight on
// RTK fixed. Switch at runtime with serial "HEADING MAG" / "HEADING GPS".
#define HEADING_SOURCE_DEFAULT 1

// ── GPS heading fusion (HEADING_SOURCE 1) ────────────────────────────────────
#define HEADING_GPS_MIN_BASELINE_M 0.5f      // straight distance per GPS course sample
#define HEADING_GPS_MAX_YAW_CHANGE_DEG 3.0f  // more turning than this restarts the segment
#define HEADING_GPS_GAIN 0.3f                // fraction of each course error applied
#define HEADING_GPS_MAX_HERR_M 0.05f         // GST error limit for using a sample
#define HEADING_GPS_FLOAT_MIN_BASELINE_M 2.0f  // RTK float: log-only samples (DEBUG_LOG_HEADING), never applied
#define HEADING_GPS_FLOAT_MAX_YAW_CHANGE_DEG 10.0f  // turning limit for float samples (chord = mid heading)

// ── Heading offset ───────────────────────────────────────────────────────────
// Heading offset (degrees) added after IMU yaw computation.
// Positive = rotate heading clockwise. Range: any value, normalised to 0-360.
#define HEADING_OFFSET_DEG 0.0f


// =============================================================================
//  6. CORE PIN MAP
// =============================================================================

// ── CAN bus ──────────────────────────────────────────────────────────────────
#define CAN_TX_PIN GPIO_NUM_1
#define CAN_RX_PIN GPIO_NUM_2

// ── PS2 joystick ─────────────────────────────────────────────────────────────
#define PS2_ATT GPIO_NUM_14
#define PS2_CMD GPIO_NUM_12
#define PS2_DAT GPIO_NUM_13
#define PS2_CLK GPIO_NUM_11

// ── Mission output channels A-D (relay/GPIO pins) ───────────────────────────
// Avoids: 1,2 (CAN), 4-7 (BNO085 SPI), 8 (NeoPixel), 9-10 (GPS UART), 11-14 (PS2),
// 15-16 (BNO085 INT/RST), 17-18 (SIM UART), 19-20 (USB D-/D+), 0/3/45/46 (strapping),
// 26-37 (flash/octal PSRAM), 41-42 (I2C), 3/44/47/48 (linear actuator).
#define OUTPUT_A_PIN 21
#define OUTPUT_B_PIN 38
#define OUTPUT_C_PIN 39
#define OUTPUT_D_PIN 40
#define OUTPUT_COUNT 4

// ── WS2812B NeoPixel status LEDs (8 LEDs, legend in section 13) ─────────────
#define NEOPIXEL_PIN 8
#define NEOPIXEL_COUNT 8
// 0..255 (lower is dimmer)
#define NEOPIXEL_BRIGHTNESS 32


// =============================================================================
//  7. MOTORS
// =============================================================================

// ── Motor backend selection ──────────────────────────────────────────────────
// Switch implementations without changing control code.
#define DRIVE_MOTOR_TYPE_ZLAC 1
#define DRIVE_MOTOR_TYPE DRIVE_MOTOR_TYPE_ZLAC

#define STEER_MOTOR_TYPE_ODRIVE 1
#define STEER_MOTOR_TYPE_GIM8108 2
#define STEER_MOTOR_TYPE STEER_MOTOR_TYPE_GIM8108

// ── Motor CAN IDs used by backend adapters ───────────────────────────────────
#define DRIVE_FRONT_MOTOR_ID 1
#define DRIVE_REAR_MOTOR_ID 2
#define STEER_LEFT_MOTOR_ID 1
#define STEER_RIGHT_MOTOR_ID 2

// ── Direction signs (set to -1 to invert a side) ────────────────────────────
// Drive
#define DRIVE_LEFT_SIGN -1
#define DRIVE_RIGHT_SIGN 1
// GIM8108 steering
#define STEER_GIM_LEFT_SIGN 1
#define STEER_GIM_RIGHT_SIGN 1

// ── E-stop behaviour ─────────────────────────────────────────────────────────
// What e-stop does to the motors.
// 1 = stop and hold: drive wheels ramp to 0 rpm (0x6084 decel) and stay powered, so the
//     robot holds on a slope; steering stays powered at its last angle.
// 0 = release: drive quick stop, steering unpowered (no hold).
#define ESTOP_HOLD 1


// =============================================================================
//  8. GEOMETRY & MOTION LIMITS
// =============================================================================

// ── Robot geometry (meters) ──────────────────────────────────────────────────
// W = track width, H = wheelbase/axle spacing used by zero-turn steering geometry.
#define ROBOT_GEOMETRY_W_M 0.7f
#define ROBOT_GEOMETRY_H_M 1.1f
// #define ROBOT_GEOMETRY_W_M 0.7f
// #define ROBOT_GEOMETRY_H_M 1.1f
// Drive wheel diameter used for m/s <-> RPM conversion.
#define DRIVE_WHEEL_DIAMETER_M 0.23f

// ── Robot motion limits ──────────────────────────────────────────────────────
#define MAX_LINEAR_SPEED_MS 0.8f       // Maximum forward/reverse speed (m/s)
#define MAX_STEERING_ANGLE_DEG 45.0f   // Maximum steering angle (degrees)
#define STEER_MAX_WHEEL_ANGLE_DEG 45.0f // Mechanical limit of each steered wheel; inner wheel saturates here
// Zero-turn turn-rate limit for manual / PC control (rad/s; 0.4 rad/s ~ 23 deg/s):
//   PS2:     full stick turns the robot at this rate.
//   CMD_VEL: in ZERO_TURN mode the angular value (rad/s, + = counter-clockwise) sets
//            the turn rate and is clamped to this; the linear value is ignored.
// Converted to wheel speed like MISSION_ZERO_TURN_RATE_RADS, never above
// MAX_LINEAR_SPEED_MS. Missions use MISSION_ZERO_TURN_RATE_RADS instead.
#define ZERO_TURN_MAX_RATE_RADS 0.6f


// =============================================================================
//  9. MISSION
// =============================================================================

// ── GPS accuracy gate ────────────────────────────────────────────────────────
// Mission GPS accuracy gate (GST horizontal 1-sigma error). Above the limit a running
// mission stops and holds in place, resumes when accuracy recovers, and fails after
// the timeout.
#define MISSION_GPS_MAX_HERR_M 0.10f
#define MISSION_GPS_HOLD_TIMEOUT_MS 60000UL

// ── Waypoint arrival ─────────────────────────────────────────────────────────
// Waypoint arrival radius (m): a waypoint counts as reached inside this distance.
// Coverage-path points use the tighter radius so rows stay straight.
#define MISSION_WAYPOINT_REACHED_M 0.3f
#define MISSION_COVERAGE_WAYPOINT_REACHED_M 0.3f

// ── Server link loss ─────────────────────────────────────────────────────────
// A running mission holds in place after this long without the
// control server, and resumes automatically when the link returns.
#define MISSION_LINK_LOSS_HOLD_MS 30000UL

// ── Solar rows task (driven by the nav PC over USB serial) ───────────────────
// A "solar_rows" mission task hands driving to the nav PC (SOLAR_TASK line, see
// docs/solar_rows.md). The firmware re-sends SOLAR_TASK every RESEND_MS until
// nav acknowledges, and fails the mission after ACK_TIMEOUT_MS without one.
// While running, nav sends CMD_VEL at its control rate and SOLAR_STATUS at 1 Hz:
// after QUIET_HOLD_MS with neither the robot holds at zero speed, after
// QUIET_FAIL_MS the mission fails.
#define SOLAR_TASK_RESEND_MS 1000UL
#define SOLAR_ACK_TIMEOUT_MS 10000UL
#define SOLAR_NAV_QUIET_HOLD_MS 2000UL
#define SOLAR_NAV_QUIET_FAIL_MS 30000UL
// Most rows in a "lanes" list (rows marked as waypoint pairs). Each row adds about
// 57 chars to the SOLAR_TASK line (60 rows ~ 3.5 KB) and 2 mission waypoints.
#define SOLAR_MAX_LANES 60
// Drive held at zero after nav switches MODE (pivot turns) while the wheels steer
// into place. Separate from MISSION_MODE_SWITCH_SETTLE_MS so it can be tuned for
// solar pivots; starts equal to it. Each pivot turn switches twice.
#define SOLAR_MODE_SWITCH_SETTLE_MS 1500UL

// ── Mission auto zero-turn ───────────────────────────────────────────────────
// During GotoWaypoint / CoveragePath tasks, switch to zero-turn (rotate in place)
// when |heading error| to the target exceeds ENTER_DEG, and switch back to normal
// Ackermann mode once it has rotated to within EXIT_DEG (hysteresis: EXIT < ENTER).
// Set ENABLE to 0 to always stay in normal mode during missions.
#define MISSION_AUTO_ZERO_TURN_ENABLE 1
#define MISSION_ZERO_TURN_ENTER_DEG 45.0f
#define MISSION_ZERO_TURN_EXIT_DEG 10.0f
// Turn rate while rotating in place (rad/s; 0.5 rad/s ~ 29 deg/s). Converted to wheel
// speed with this robot's size: v = rate * r, r = 0.5 * sqrt(W^2 + H^2) (wheel to
// centre); the wheel speed is capped at MAX_LINEAR_SPEED_MS.
#define MISSION_ZERO_TURN_RATE_RADS 0.4f   // = 0.5 m/s wheel speed on this robot (r = 0.65 m)
// Smooth mode transition (both directions):
//   1. ramp the drive speed down to 0 in the current mode (MISSION_SPEED_SLEW_MPS2)
//   2. switch mode, hold the drive at 0 for MISSION_MODE_SWITCH_SETTLE_MS while the
//      steering mechanism moves to its new angle
//   3. ramp the drive speed back up in the new mode
#define MISSION_MODE_SWITCH_SETTLE_MS 1500UL
// Max change of mission drive speed (m/s per second). Also smooths normal mission
// starts/stops of driving between waypoints.
#define MISSION_SPEED_SLEW_MPS2 0.3f
// Set to -1 if the robot rotates away from the target instead of toward it.
#define MISSION_ZERO_TURN_DIRECTION_SIGN 1


// =============================================================================
//  10. FUNC ACTUATORS (USB serial FUNC_<n>, joystick combo)
// =============================================================================

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

// ── Joystick combo ───────────────────────────────────────────────────────────
// R1+L1 / R1+L2 joystick combo target:
//   0 = FUNC_0 GPIO — R1+L1 triggers ON, R1+L2 triggers OFF
//   1 = FUNC_1 CAN  — R1+L1 jogs +45 REL_DEG, R1+L2 jogs -45 REL_DEG
#define JOYSTICK_COMBO_FUNC_TARGET 0

// ── DEG / REL_DEG timing ─────────────────────────────────────────────────────
// How long a DEG-type actuator holds at the target position before it auto-returns to 0.
#define FUNC_DEG_HOLD_MS 1000UL

// REL_DEG busy-guard: a new REL_DEG command is rejected while the previous one hasn't
// finished yet, so rapid re-presses / button-debounce glitches can't chain onto a move
// still in flight and compound into runaway motion.
//   ODRIVE:  compares live position (read over CAN) against the last commanded target —
//            "arrived" means within this many degrees of it.
#define FUNC_REL_DEG_ARRIVAL_TOLERANCE_DEG 2.0f
// If ODRIVE's arrival check keeps rejecting the same mismatch for longer than this, the
// tracked baseline is desynced from reality (not "still moving" — a normal move settles in
// well under this) and gets auto-resynced to the live reading instead of blocking forever.
#define FUNC_REL_DEG_STUCK_TIMEOUT_MS 3000UL
//   GIM8108: has no position readback here, so it uses a fixed busy window instead —
//            how long (ms) a REL_DEG move is assumed to take before another is allowed.
#define FUNC_REL_DEG_GIM_BUSY_MS 500UL


// =============================================================================
//  11. ADD-ON I/O MODULES
// =============================================================================

// ── Module enables: 1 = enabled, 0 = disabled ───────────────────────────────
// A disabled module is left out of the build: no init, no serial commands
// (they reply "... rejected: <NAME>_ENABLE is 0"), no telemetry fields.
// The I2C bus only starts when DS3502 or PCF8574 is enabled.
#define DS3502_ENABLE   1   // 3 x DS3502 digital pots (I2C)
#define PCF8574_ENABLE  1   // PCF8574 digital inputs (I2C)
#define ACTUATOR_ENABLE 1   // linear actuator: VNH2SP30 + feedback pot (GPIO 3/44/47/48)

// ── I2C bus ─────────────────────────────────────────────────────────────────
#define I2C_SDA_PIN 41
#define I2C_SCL_PIN 42
#define I2C_FREQ_HZ 400000

// ── DS3502 digital potentiometers (I2C, 10 kOhm, wiper 0..127) ──────────────
// USB serial: POT <ch> <0-100> (percent), POT <ch> RAW <0-127>, POT?
// Address 0x28 + A1/A0 straps: A1=GND A0=GND 0x28, GND/VCC 0x29, VCC/GND 0x2A.
#define DS3502_COUNT 3
#define DS3502_ADDRESSES {0x28, 0x29, 0x2A}
// Wiper set at boot, in percent. Until then each chip holds its EEPROM value (0x40 = ~50%).
#define DS3502_BOOT_PERCENT 0.0f

// ── PCF8574 digital inputs (I2C) — safety limit switches ───────────────────
// Inputs 1..N on P0..P(N-1), 3.3 V logic only (optocouplers for 12/24 V signals).
// USB serial: IN?
#define PCF8574_I2C_ADDR 0x20   // PCF8574A: 0x38
#define DIGITAL_INPUT_COUNT 4
#define DIGITAL_INPUT_NAMES {"Front L", "Front R", "Rear L", "Rear R"}
// 0 = input is active (switch hit) when its pin reads HIGH: normally-closed switch
//     between the pin and GND — closed in normal operation, opens when hit. Fail-safe:
//     a broken wire or unplugged switch also reads as hit.
// 1 = active when the pin is pulled LOW: normally-open switch to GND (not fail-safe).
#define DIGITAL_INPUT_ACTIVE_LOW 0
#define DIGITAL_INPUT_POLL_MS 20
// Latched e-stop when any input becomes active, or when the PCF8574 is missing or
// stops answering. Release with PS2 L1 + L2 + Circle. A switch still held after the release does
// not re-trigger (so the robot can be driven away); hitting it again does.
// Until the switches are wired, jumper P0..P3 to GND or set this to 0.
#define LIMIT_SWITCH_ESTOP_ENABLE 1
#define LIMIT_SWITCH_DEBOUNCE_POLLS 2    // consecutive active polls before e-stop (2 x 20 ms)
#define LIMIT_SWITCH_READ_FAIL_POLLS 5   // consecutive I2C read failures before e-stop

// ── Linear actuator (VNH2SP30 H-bridge + feedback pot) ──────────────────────
// USB serial: ACT <0-100> (percent of stroke), ACT STOP, ACT?, ACT JOG <-100..100> <ms>
// Calibration: ACT JOG -60 3000 to retract fully, ACT? → copy raw to ACT_ADC_RETRACTED;
// ACT JOG 60 3000 to extend fully, ACT? → copy raw to ACT_ADC_EXTENDED.
// If ACT JOG 60 retracts instead of extends, set ACT_DIRECTION_SIGN to -1.
#define ACT_INA_PIN 48
#define ACT_INB_PIN 44            // labelled "RX" on the DevKitC-1
#define ACT_PWM_PIN 47
#define ACT_POS_PIN 3            // ADC1_CH2; max ~3.1 V — divide a 5 V pot down
#define ACT_LEDC_CHANNEL 0
#define ACT_PWM_FREQ_HZ 20000
#define ACT_ADC_RETRACTED 125    // raw ADC (0..4095) at full retract — calibrate
#define ACT_ADC_EXTENDED 3170    // raw ADC at full extend — calibrate
#define ACT_ADC_FAULT_MARGIN 200 // readings further outside the range = broken wire
#define ACT_DIRECTION_SIGN 1
#define ACT_DEADBAND_PCT 1.0f    // stop within this distance of the target
#define ACT_SLOW_ZONE_PCT 10.0f  // slow down inside this distance
#define ACT_MIN_DUTY_PCT 35.0f   // lowest duty that still moves the actuator
#define ACT_MAX_DUTY_PCT 100.0f
#define ACT_STALL_TIMEOUT_MS 1000UL
#define ACT_STALL_MIN_CHANGE_PCT 0.5f


// =============================================================================
//  12. DEBUG LOGGING  (1 = enabled, 0 = disabled)
// =============================================================================
#define DEBUG_LOG_SENSOR_1HZ 0
#define DEBUG_LOG_SENSOR_COMMS 0
#define DEBUG_LOG_MISSION 0
#define DEBUG_LOG_SAFETY 0
#define DEBUG_LOG_ROBOT_CLIENT 0
#define DEBUG_LOG_HEADING 0  // [HDG_S] line per GPS course sample (mag vs fused)


// =============================================================================
//  13. STATUS LED LEGEND (WS2812B, index 0..7)
// =============================================================================
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
