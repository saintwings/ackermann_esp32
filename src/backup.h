// #include <Arduino.h>
// #include <PS2X_lib.h>

// PS2X ps2x;
// int error = 0;

// static constexpr int PS2_ATT = 21;
// static constexpr int PS2_CMD = 23;
// static constexpr int PS2_DAT = 19;
// static constexpr int PS2_CLK = 18;

// void setup() {
//   Serial.begin(115200);
//   delay(300); // wait for controller to power up

//   error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);

//   if (error == 0) Serial.println("PS2 controller connected!");
//   else Serial.printf("PS2 init error %d\n", error);
// }

// void loop() {
//   // your control code here
// }


// #include <Arduino.h>
// #include "JoystickPS2.h"
// #include "LEDStatus.h"

// // === Pin configuration ===
// // RS485 on Serial2 (UART2)
// static constexpr int RS485_DE_RE_PIN  = 1;
// static constexpr int RS485_RX_PIN     = 16;
// static constexpr int RS485_TX_PIN     = 17;

// // PS2 controller pins (ATT, CMD, DAT, CLK)
// static constexpr int PS2_ATT = 21;
// static constexpr int PS2_CMD = 23;
// static constexpr int PS2_DAT = 19;
// static constexpr int PS2_CLK = 18;

// // WS2812D LED strip
// static constexpr int LED_PIN = 13;
// static constexpr int LED_COUNT = 8;

// // === Robot parameters ===
// static constexpr float WHEELBASE_M  = 0.36f;
// static constexpr float TRACK_WIDTH_M = 0.36f;

// // === Globals ===
// LEDStatus           leds(LED_PIN, LED_COUNT);
// JoystickPS2         js(PS2_ATT, PS2_CMD, PS2_DAT, PS2_CLK);

// RobotDriveMode robot_mode = RobotDriveMode::Normal;


// void setup() {
//   Serial.begin(115200);
//   delay(100);
//   Serial.println("\n=== ESP32 Double Ackermann Controller ===");

//   // Joystick
//   js.begin();

//   // LEDs
//   // leds.begin();
//   // leds.setRobotMode(robot_mode);
//   // leds.update();

// }


// void loop() {

//   // Read joystick
//   JoyState j = js.read();
//   delay(1000);
//   // robot_mode = RobotDriveMode::ZeroTurn;
//   // leds.setRobotMode(robot_mode);
//   // leds.update();
//   // delay(1000);
//   // robot_mode = RobotDriveMode::Normal;
//   // leds.setRobotMode(robot_mode);
//   // leds.update();
//   // delay(1000);
// }



#include <Arduino.h>
#include "driver/twai.h"
#include <PS2X_lib.h>
#include <math.h>

// ==========================================
// HARDWARE PINS
// ==========================================
#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_5

#define PS2_ATT GPIO_NUM_21
#define PS2_CMD GPIO_NUM_23
#define PS2_DAT GPIO_NUM_19
#define PS2_CLK GPIO_NUM_18



// ==========================================
// 1. ODRIVE CAN CLASS (Position Control)
// ==========================================
class ODriveCAN {
  public:
    uint32_t node_id;
    float current_target_pos = 0.0; // Keep track of where we told it to go

    ODriveCAN(uint32_t id) { node_id = id; }

    void set_axis_state(uint32_t state_id) {
      twai_message_t msg;
      msg.identifier = (node_id << 5) | 0x07; 
      msg.extd = 0; msg.data_length_code = 8;
      memcpy(&msg.data[0], &state_id, 4); 
      memset(&msg.data[4], 0, 4);         
      twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    void set_controller_mode(int32_t control_mode, int32_t input_mode) {
      twai_message_t msg;
      msg.identifier = (node_id << 5) | 0x0B; 
      msg.extd = 0; msg.data_length_code = 8;
      memcpy(&msg.data[0], &control_mode, 4);
      memcpy(&msg.data[4], &input_mode, 4);
      twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    void set_position(float pos_turns, float vel_ff = 0.0, float torque_ff = 0.0) {
      current_target_pos = pos_turns; // Save for our D-pad logic later
      
      twai_message_t msg;
      msg.identifier = (node_id << 5) | 0x0C; 
      msg.extd = 0; msg.data_length_code = 8;
      
      int16_t v_ff_int = (int16_t)(vel_ff * 1000);
      int16_t t_ff_int = (int16_t)(torque_ff * 1000);
      
      memcpy(&msg.data[0], &pos_turns, 4);
      memcpy(&msg.data[4], &v_ff_int, 2);
      memcpy(&msg.data[6], &t_ff_int, 2);
      twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    void clear_errors() {
      twai_message_t msg;
      msg.identifier = (node_id << 5) | 0x18; // Clear_Errors
      msg.extd = 0;
      msg.data_length_code = 8;
      memset(msg.data, 0, 8);
      twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    bool recover_and_enter_closed_loop(uint8_t retries = 3) {
      for (uint8_t attempt = 0; attempt < retries; ++attempt) {
        set_axis_state(1); // IDLE
        delay(30);
        clear_errors();
        delay(30);
        set_controller_mode(3, 1); // Position + passthrough
        delay(30);
        set_axis_state(8); // Closed loop
        delay(40);
        set_position(current_target_pos);
        delay(20);
      }
      return true;
    }
};

// ==========================================
// 2. ZLAC8015D CANOPEN CLASS (Velocity Control)
// ==========================================
class ZLAC8015D {
  public:
    uint32_t node_id, tx_id;

    ZLAC8015D(uint32_t id) {
      node_id = id;
      tx_id = 0x600 + node_id;
    }

    void send_sdo(uint8_t cmd, uint8_t idx_l, uint8_t idx_h, uint8_t sub, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
      twai_message_t msg;
      msg.identifier = tx_id; msg.extd = 0; msg.data_length_code = 8;
      msg.data[0] = cmd; msg.data[1] = idx_l; msg.data[2] = idx_h; msg.data[3] = sub;
      msg.data[4] = d0;  msg.data[5] = d1;    msg.data[6] = d2;    msg.data[7] = d3;
      twai_transmit(&msg, pdMS_TO_TICKS(10));
      delay(30); 
    }

    void init_sync_velocity_mode() {
      send_sdo(0x2B, 0x0F, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00); 
      send_sdo(0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00); 
      send_sdo(0x23, 0x83, 0x60, 0x01, 0xC8, 0x00, 0x00, 0x00); 
      send_sdo(0x23, 0x83, 0x60, 0x02, 0xC8, 0x00, 0x00, 0x00); 
      send_sdo(0x23, 0x84, 0x60, 0x01, 0xC8, 0x00, 0x00, 0x00); 
      send_sdo(0x23, 0x84, 0x60, 0x02, 0xC8, 0x00, 0x00, 0x00); 
    }

    void enable_motor() {
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00); 
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00); 
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00); 
    }

    void fault_reset() {
      // CANopen controlword fault reset
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00);
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00);
    }

    bool recover_and_enable_velocity_mode(uint8_t retries = 2) {
      for (uint8_t attempt = 0; attempt < retries; ++attempt) {
        trigger_e_stop();
        delay(30);
        fault_reset();
        delay(30);
        init_sync_velocity_mode();
        delay(30);
        enable_motor();
        delay(30);
        set_sync_speed(0, 0);
        delay(20);
      }
      return true;
    }

    void set_sync_speed(int16_t left_rpm, int16_t right_rpm) {
      twai_message_t msg;
      msg.identifier = tx_id; msg.extd = 0; msg.data_length_code = 8;
      msg.data[0] = 0x23; msg.data[1] = 0xFF; msg.data[2] = 0x60; msg.data[3] = 0x03;
      memcpy(&msg.data[4], &left_rpm, 2);
      memcpy(&msg.data[6], &right_rpm, 2);
      twai_transmit(&msg, pdMS_TO_TICKS(10)); 
    }

    void trigger_e_stop() {
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00);
    }
};

// ==========================================
// 3. GLOBAL INSTANCES
// ==========================================
ZLAC8015D zlac_front(1);
ZLAC8015D zlac_rear(2);
ODriveCAN odrive_1(1);
ODriveCAN odrive_2(2);

PS2X ps2x; // Create PS2 Controller Class
int ps2_error = 0;

// Maximum speed settings
const int MAX_RPM = 150;
const float WHEELBASE_M = 0.36f;
const float TRACK_WIDTH_M = 0.36f;
const float WHEEL_DIAMETER_M = 0.20f;
const float MAX_LINEAR_MPS = 0.60f;
const float MAX_STEER_DEG = 30.0f;

// Match Python motor direction mapping: [1, -1]
const int8_t MOTOR_DI_LEFT = 1;
const int8_t MOTOR_DI_RIGHT = -1;

bool e_stop_active = false;

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float mps_to_rpm(float mps) {
  float circumference = (float)M_PI * WHEEL_DIAMETER_M;
  if (circumference <= 1e-6f) return 0.0f;
  return (mps / circumference) * 60.0f;
}

// ==========================================
// 4. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- Initialize PS2 Controller ---
  Serial.println("Connecting to PS2 Controller...");
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
  if(ps2_error == 0) {
    Serial.println("PS2 Controller found and configured!");
  } else {
    Serial.println("Error configuring PS2 Controller. Check wiring!");
  }

  // --- Initialize TWAI (CAN) ---
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  // --- Initialize Motors ---
  Serial.println("Configuring ZLACs...");
  zlac_front.recover_and_enable_velocity_mode();
  zlac_rear.recover_and_enable_velocity_mode();

  Serial.println("Configuring ODrives...");
  odrive_1.recover_and_enter_closed_loop();
  odrive_2.recover_and_enter_closed_loop();

  odrive_1.set_position(0.0);
  odrive_2.set_position(0.0);
  
  Serial.println("Setup Complete! Ready to Drive.");
}

// ==========================================
// 5. MAIN LOOP
// ==========================================
unsigned long last_control_time = 0;

void loop() {
  // We only read the PS2 controller and update motors every 20ms (50Hz)
  if (millis() - last_control_time >= 20) {
    last_control_time = millis();

    // If PS2 didn't connect properly, skip the loop
    if (ps2_error != 0) return;

    // Read gamepad state
    ps2x.read_gamepad();

    // --- E-STOP LOGIC ---
    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
      e_stop_active = !e_stop_active; // Toggle E-Stop on/off
      
      if (e_stop_active) {
        Serial.println("!!! E-STOP ACTIVATED !!!");
        zlac_front.trigger_e_stop();
        zlac_rear.trigger_e_stop();
        odrive_1.set_axis_state(1); // Drop to IDLE
        odrive_2.set_axis_state(1);
      } else {
        Serial.println(">>> E-STOP RELEASED <<<");
        zlac_front.recover_and_enable_velocity_mode();
        zlac_rear.recover_and_enable_velocity_mode();
        odrive_1.recover_and_enter_closed_loop();
        odrive_2.recover_and_enter_closed_loop();
      }
    }

    // If E-Stop is active, don't process any driving commands
    if (e_stop_active) return;

    // --- DOUBLE ACKERMANN LOGIC ---
    // Read Joysticks (0 to 255. 128 is center)
    // Up is 0, Down is 255. Left is 0, Right is 255.
    int ly = ps2x.Analog(PSS_LY); 
    int rx = ps2x.Analog(PSS_RX);

    Serial.print("ly: ");
    Serial.println(ly);

    Serial.print("rx: ");
    Serial.println(rx);

    // Apply a small "deadband" so the robot doesn't drift if the sticks aren't perfectly centered
    if (abs(ly - 128) < 15) ly = 128;
    if (abs(rx - 128) < 15) rx = 128;

    // Convert to normalized command
    float forward_axis = -(ly - 128.0f) / 128.0f; // Y=0 is UP
    float turn_axis    =  (rx - 128.0f) / 128.0f;
    forward_axis = clampf(forward_axis, -1.0f, 1.0f);
    turn_axis = clampf(turn_axis, -1.0f, 1.0f);

    // Map to physical commands (same behavior as Python path)
    float linear_mps = forward_axis * MAX_LINEAR_MPS;
    float steer_deg = turn_axis * MAX_STEER_DEG;
    float steer_rad = steer_deg * ((float)M_PI / 180.0f);

    // R from steering angle: tan(delta) = (L/2) / R => R = (L/2)/tan(delta)
    float R = 1e6f;
    if (fabsf(steer_rad) > 1e-3f) {
      R = (WHEELBASE_M / 2.0f) / tanf(steer_rad);
    }

    // Steering angles for front pair (rear is mirrored mechanically, not commanded here)
    float angle_fl_deg = 0.0f;
    float angle_fr_deg = 0.0f;
    if (fabsf(R) < 1e5f) {
      float angle_fl_rad = atanf((WHEELBASE_M / 2.0f) / (R - TRACK_WIDTH_M / 2.0f));
      float angle_fr_rad = atanf((WHEELBASE_M / 2.0f) / (R + TRACK_WIDTH_M / 2.0f));
      angle_fl_deg = clampf(angle_fl_rad * 180.0f / (float)M_PI, -MAX_STEER_DEG, MAX_STEER_DEG);
      angle_fr_deg = clampf(angle_fr_rad * 180.0f / (float)M_PI, -MAX_STEER_DEG, MAX_STEER_DEG);
    }

    // Wheel linear speeds from Ackermann geometry
    float speed_left_mps = linear_mps;
    float speed_right_mps = linear_mps;
    if (fabsf(R) < 1e5f && fabsf(R) > 1e-3f) {
      float omega = linear_mps / fabsf(R);
      float rw_left = hypotf(R - TRACK_WIDTH_M / 2.0f, WHEELBASE_M / 2.0f);
      float rw_right = hypotf(R + TRACK_WIDTH_M / 2.0f, WHEELBASE_M / 2.0f);
      speed_left_mps = omega * rw_left;
      speed_right_mps = omega * rw_right;
      if (linear_mps < 0.0f) {
        speed_left_mps = -speed_left_mps;
        speed_right_mps = -speed_right_mps;
      }
    }

    // Convert to RPM and clamp for ZLAC command range
    int16_t left_rpm = (int16_t)clampf(mps_to_rpm(speed_left_mps) * MOTOR_DI_LEFT, -MAX_RPM, MAX_RPM);
    int16_t right_rpm = (int16_t)clampf(mps_to_rpm(speed_right_mps) * MOTOR_DI_RIGHT, -MAX_RPM, MAX_RPM);

    // Send same left/right pair to front and rear drive controllers
    zlac_front.set_sync_speed(left_rpm, right_rpm);
    zlac_rear.set_sync_speed(left_rpm, right_rpm);

    // ODrive steering position command scaling follows Python mapping: -angle/45
    odrive_1.set_position(-angle_fl_deg / 45.0f);
    odrive_2.set_position(-angle_fr_deg / 45.0f);
  }
}



#include <Arduino.h>
#include "driver/twai.h"
#include <PS2X_lib.h>

// ==========================================
// HARDWARE PINS
// ==========================================
#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_5

#define PS2_ATT GPIO_NUM_21
#define PS2_CMD GPIO_NUM_23
#define PS2_DAT GPIO_NUM_19
#define PS2_CLK GPIO_NUM_18



// ==========================================
// 1. ODRIVE CAN CLASS (Position Control)
// ==========================================
class ODriveCAN {
  public:
    uint32_t node_id;
    float current_target_pos = 0.0; // Keep track of where we told it to go

    ODriveCAN(uint32_t id) { node_id = id; }

    void set_axis_state(uint32_t state_id) {
      twai_message_t msg;
      msg.identifier = (node_id << 5) | 0x07; 
      msg.extd = 0; msg.data_length_code = 8;
      memcpy(&msg.data[0], &state_id, 4); 
      memset(&msg.data[4], 0, 4);         
      twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    void set_controller_mode(int32_t control_mode, int32_t input_mode) {
      twai_message_t msg;
      msg.identifier = (node_id << 5) | 0x0B; 
      msg.extd = 0; msg.data_length_code = 8;
      memcpy(&msg.data[0], &control_mode, 4);
      memcpy(&msg.data[4], &input_mode, 4);
      twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    void set_position(float pos_turns, float vel_ff = 0.0, float torque_ff = 0.0) {
      current_target_pos = pos_turns; // Save for our D-pad logic later
      
      twai_message_t msg;
      msg.identifier = (node_id << 5) | 0x0C; 
      msg.extd = 0; msg.data_length_code = 8;
      
      int16_t v_ff_int = (int16_t)(vel_ff * 1000);
      int16_t t_ff_int = (int16_t)(torque_ff * 1000);
      
      memcpy(&msg.data[0], &pos_turns, 4);
      memcpy(&msg.data[4], &v_ff_int, 2);
      memcpy(&msg.data[6], &t_ff_int, 2);
      twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    void clear_errors() {
      twai_message_t msg;
      msg.identifier = (node_id << 5) | 0x18; // Clear_Errors
      msg.extd = 0;
      msg.data_length_code = 8;
      memset(msg.data, 0, 8);
      twai_transmit(&msg, pdMS_TO_TICKS(10));
    }

    bool recover_and_enter_closed_loop(uint8_t retries = 3) {
      for (uint8_t attempt = 0; attempt < retries; ++attempt) {
        set_axis_state(1); // IDLE
        delay(30);
        clear_errors();
        delay(30);
        set_controller_mode(3, 1); // Position + passthrough
        delay(30);
        set_axis_state(8); // Closed loop
        delay(40);
        set_position(current_target_pos);
        delay(20);
      }
      return true;
    }
};

// ==========================================
// 2. ZLAC8015D CANOPEN CLASS (Velocity Control)
// ==========================================
class ZLAC8015D {
  public:
    uint32_t node_id, tx_id;

    ZLAC8015D(uint32_t id) {
      node_id = id;
      tx_id = 0x600 + node_id;
    }

    void send_sdo(uint8_t cmd, uint8_t idx_l, uint8_t idx_h, uint8_t sub, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
      twai_message_t msg;
      msg.identifier = tx_id; msg.extd = 0; msg.data_length_code = 8;
      msg.data[0] = cmd; msg.data[1] = idx_l; msg.data[2] = idx_h; msg.data[3] = sub;
      msg.data[4] = d0;  msg.data[5] = d1;    msg.data[6] = d2;    msg.data[7] = d3;
      twai_transmit(&msg, pdMS_TO_TICKS(10));
      delay(30); 
    }

    void init_sync_velocity_mode() {
      send_sdo(0x2B, 0x0F, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00); 
      send_sdo(0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00); 
      send_sdo(0x23, 0x83, 0x60, 0x01, 0xC8, 0x00, 0x00, 0x00); 
      send_sdo(0x23, 0x83, 0x60, 0x02, 0xC8, 0x00, 0x00, 0x00); 
      send_sdo(0x23, 0x84, 0x60, 0x01, 0xC8, 0x00, 0x00, 0x00); 
      send_sdo(0x23, 0x84, 0x60, 0x02, 0xC8, 0x00, 0x00, 0x00); 
    }

    void enable_motor() {
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00); 
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00); 
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00); 
    }

    void fault_reset() {
      // CANopen controlword fault reset
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00);
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00);
    }

    bool recover_and_enable_velocity_mode(uint8_t retries = 2) {
      for (uint8_t attempt = 0; attempt < retries; ++attempt) {
        trigger_e_stop();
        delay(30);
        fault_reset();
        delay(30);
        init_sync_velocity_mode();
        delay(30);
        enable_motor();
        delay(30);
        set_sync_speed(0, 0);
        delay(20);
      }
      return true;
    }

    void set_sync_speed(int16_t left_rpm, int16_t right_rpm) {
      twai_message_t msg;
      msg.identifier = tx_id; msg.extd = 0; msg.data_length_code = 8;
      msg.data[0] = 0x23; msg.data[1] = 0xFF; msg.data[2] = 0x60; msg.data[3] = 0x03;
      memcpy(&msg.data[4], &left_rpm, 2);
      memcpy(&msg.data[6], &right_rpm, 2);
      twai_transmit(&msg, pdMS_TO_TICKS(10)); 
    }

    void trigger_e_stop() {
      send_sdo(0x2B, 0x40, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00);
    }
};

// ==========================================
// 3. GLOBAL INSTANCES
// ==========================================
ZLAC8015D zlac_front(1);
ZLAC8015D zlac_rear(2);
ODriveCAN odrive_1(0);
ODriveCAN odrive_2(1);

PS2X ps2x; // Create PS2 Controller Class
int ps2_error = 0;

// Maximum speed settings
const int MAX_RPM = 150; 
bool e_stop_active = false;

// ==========================================
// 4. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- Initialize PS2 Controller ---
  Serial.println("Connecting to PS2 Controller...");
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
  if(ps2_error == 0) {
    Serial.println("PS2 Controller found and configured!");
  } else {
    Serial.println("Error configuring PS2 Controller. Check wiring!");
  }

  // --- Initialize TWAI (CAN) ---
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  // --- Initialize Motors ---
  Serial.println("Configuring ZLACs...");
  zlac_front.recover_and_enable_velocity_mode();
  zlac_rear.recover_and_enable_velocity_mode();

  Serial.println("Configuring ODrives...");
  odrive_1.recover_and_enter_closed_loop();
  odrive_2.recover_and_enter_closed_loop();

  odrive_1.set_position(0.0);
  odrive_2.set_position(0.0);
  
  Serial.println("Setup Complete! Ready to Drive.");
}

// ==========================================
// 5. MAIN LOOP
// ==========================================
unsigned long last_control_time = 0;

void loop() {
  // We only read the PS2 controller and update motors every 20ms (50Hz)
  if (millis() - last_control_time >= 20) {
    last_control_time = millis();

    // If PS2 didn't connect properly, skip the loop
    if (ps2_error != 0) return;

    // Read gamepad state
    ps2x.read_gamepad();

    // --- E-STOP LOGIC ---
    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
      e_stop_active = !e_stop_active; // Toggle E-Stop on/off
      
      if (e_stop_active) {
        Serial.println("!!! E-STOP ACTIVATED !!!");
        zlac_front.trigger_e_stop();
        zlac_rear.trigger_e_stop();
        odrive_1.set_axis_state(1); // Drop to IDLE
        odrive_2.set_axis_state(1);
      } else {
        Serial.println(">>> E-STOP RELEASED <<<");
        zlac_front.recover_and_enable_velocity_mode();
        zlac_rear.recover_and_enable_velocity_mode();
        odrive_1.recover_and_enter_closed_loop();
        odrive_2.recover_and_enter_closed_loop();
      }
    }

    // If E-Stop is active, don't process any driving commands
    if (e_stop_active) return;

    // --- ZLAC ARCADE DRIVE LOGIC ---
    // Read Joysticks (0 to 255. 128 is center)
    // Up is 0, Down is 255. Left is 0, Right is 255.
    int ly = ps2x.Analog(PSS_LY); 
    int rx = ps2x.Analog(PSS_RX);

    // Apply a small "deadband" so the robot doesn't drift if the sticks aren't perfectly centered
    if (abs(ly - 128) < 15) ly = 128;
    if (abs(rx - 128) < 15) rx = 128;

    // Convert to a -1.0 to 1.0 multiplier
    float forward_speed = -(ly - 128.0) / 128.0; // Negative because Y=0 is UP
    float turn_speed    =  (rx - 128.0) / 128.0; 

    // Mix for differential drive
    int16_t left_rpm  = (forward_speed + turn_speed) * MAX_RPM;
    int16_t right_rpm = (forward_speed - turn_speed) * MAX_RPM;

    // Send to ZLAC
    zlac_front.set_sync_speed(-left_rpm, right_rpm);
    zlac_rear.set_sync_speed(-left_rpm, right_rpm);

    // --- ODRIVE POSITION LOGIC ---
    // Move ODrives +0.1 turns when Up is pressed, -0.1 when Down is pressed
    if (ps2x.Button(PSB_PAD_UP)) {
      odrive_1.set_position(odrive_1.current_target_pos + 0.05);
      odrive_2.set_position(odrive_2.current_target_pos + 0.05);
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      odrive_1.set_position(odrive_1.current_target_pos - 0.05);
      odrive_2.set_position(odrive_2.current_target_pos - 0.05);
    }
  }
}