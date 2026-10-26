#pragma once

#include <Arduino.h>

// Linear actuator on a VNH2SP30 H-bridge with an analog feedback pot.
//
// Position is in percent of stroke: 0 = retracted (adc_retracted), 100 = extended
// (adc_extended). setTarget() moves to a position and stops inside the deadband;
// the actuator is not held afterwards (lead-screw actuators are self-locking).
//
// Faults stop the motor and stay latched until the next setTarget()/jog():
//   Stall      — driven for stall_timeout_ms without moving stall_min_change_pct
//   AdcRange   — feedback reading outside the calibrated range (broken pot wire)
class LinearActuator {
 public:
  struct Config {
    int pin_ina;
    int pin_inb;
    int pin_pwm;
    int pin_pos;           // ADC1 pin
    uint8_t ledc_channel;
    uint32_t pwm_freq_hz;
    int adc_retracted;     // raw ADC at 0 %
    int adc_extended;      // raw ADC at 100 %
    int adc_fault_margin;  // raw counts allowed outside the calibrated range
    int direction_sign;    // -1 if the motor extends when it should retract
    float deadband_pct;
    float slow_zone_pct;   // duty ramps down to min_duty inside this distance
    float min_duty_pct;
    float max_duty_pct;
    unsigned long stall_timeout_ms;
    float stall_min_change_pct;
  };

  enum class State : uint8_t { Idle, Moving, Jogging, Fault };
  enum class Fault : uint8_t { None, Stall, AdcRange };

  explicit LinearActuator(const Config& cfg);

  void begin();
  // Call periodically (every few ms). Reads the pot and runs the position loop.
  void update(unsigned long now);

  void setTarget(float pct);
  // Open loop: duty_pct in -100..100 (+ = extend) for duration_ms, then stop.
  void jog(float duty_pct, unsigned long duration_ms);
  void stop();

  float positionPct() const { return position_pct_; }
  int rawAdc() const { return raw_adc_; }
  float targetPct() const { return target_pct_; }
  State state() const { return state_; }
  Fault fault() const { return fault_; }

  static const char* stateName(State s);
  static const char* faultName(Fault f);

 private:
  void readPosition();
  void drive(float duty_pct);  // signed, + = extend
  void brake();
  void setFault(Fault f);
  void resetStallWatch(unsigned long now);

  Config cfg_;
  State state_{State::Idle};
  Fault fault_{Fault::None};
  int raw_adc_{0};
  float position_pct_{0.0f};
  float target_pct_{0.0f};
  float jog_duty_pct_{0.0f};
  unsigned long jog_until_ms_{0};
  unsigned long stall_check_ms_{0};
  float stall_ref_pct_{0.0f};
};
