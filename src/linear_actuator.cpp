#include "LinearActuator.hpp"

namespace {
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1u << PWM_BITS) - 1;
constexpr int ADC_SAMPLES = 8;
}  // namespace

LinearActuator::LinearActuator(const Config& cfg) : cfg_(cfg) {}

void LinearActuator::begin() {
  pinMode(cfg_.pin_ina, OUTPUT);
  pinMode(cfg_.pin_inb, OUTPUT);
  digitalWrite(cfg_.pin_ina, LOW);
  digitalWrite(cfg_.pin_inb, LOW);
  ledcSetup(cfg_.ledc_channel, cfg_.pwm_freq_hz, PWM_BITS);
  ledcAttachPin(cfg_.pin_pwm, cfg_.ledc_channel);
  ledcWrite(cfg_.ledc_channel, 0);

  analogReadResolution(12);
  analogSetPinAttenuation(cfg_.pin_pos, ADC_11db);  // ~0..3.1 V
  readPosition();
  target_pct_ = position_pct_;
}

void LinearActuator::readPosition() {
  int sum = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) sum += analogRead(cfg_.pin_pos);
  raw_adc_ = sum / ADC_SAMPLES;
  const float span = static_cast<float>(cfg_.adc_extended - cfg_.adc_retracted);
  position_pct_ = (span != 0.0f) ? (raw_adc_ - cfg_.adc_retracted) * 100.0f / span : 0.0f;
}

void LinearActuator::drive(float duty_pct) {
  const float d = duty_pct * cfg_.direction_sign;
  if (d == 0.0f) {
    brake();
    return;
  }
  // VNH2SP30: INA=1 INB=0 → clockwise, INA=0 INB=1 → counter-clockwise.
  digitalWrite(cfg_.pin_ina, d > 0.0f ? HIGH : LOW);
  digitalWrite(cfg_.pin_inb, d > 0.0f ? LOW : HIGH);
  const float mag = constrain(fabsf(d), 0.0f, 100.0f);
  ledcWrite(cfg_.ledc_channel, static_cast<uint32_t>(mag * PWM_MAX / 100.0f));
}

void LinearActuator::brake() {
  // INA=INB=0 with PWM off: both low-side switches → motor shorted to GND (brake).
  ledcWrite(cfg_.ledc_channel, 0);
  digitalWrite(cfg_.pin_ina, LOW);
  digitalWrite(cfg_.pin_inb, LOW);
}

void LinearActuator::setFault(Fault f) {
  brake();
  fault_ = f;
  state_ = State::Fault;
}

void LinearActuator::resetStallWatch(unsigned long now) {
  stall_check_ms_ = now;
  stall_ref_pct_ = position_pct_;
}

void LinearActuator::setTarget(float pct) {
  target_pct_ = constrain(pct, 0.0f, 100.0f);
  fault_ = Fault::None;
  state_ = State::Moving;
  resetStallWatch(millis());
}

void LinearActuator::jog(float duty_pct, unsigned long duration_ms) {
  jog_duty_pct_ = constrain(duty_pct, -100.0f, 100.0f);
  jog_until_ms_ = millis() + duration_ms;
  fault_ = Fault::None;
  state_ = State::Jogging;
  resetStallWatch(millis());
}

void LinearActuator::stop() {
  brake();
  if (state_ != State::Fault) state_ = State::Idle;
}

void LinearActuator::update(unsigned long now) {
  readPosition();

  const int lo = min(cfg_.adc_retracted, cfg_.adc_extended) - cfg_.adc_fault_margin;
  const int hi = max(cfg_.adc_retracted, cfg_.adc_extended) + cfg_.adc_fault_margin;
  const bool adc_ok = raw_adc_ >= lo && raw_adc_ <= hi;

  if (state_ == State::Idle || state_ == State::Fault) return;

  if (state_ == State::Jogging) {
    // Jog is for calibration, so it ignores the ADC range check; stall still applies.
    if (static_cast<long>(now - jog_until_ms_) >= 0) {
      stop();
      return;
    }
    drive(jog_duty_pct_);
  } else {  // Moving
    if (!adc_ok) {
      setFault(Fault::AdcRange);
      return;
    }
    const float err = target_pct_ - position_pct_;
    if (fabsf(err) <= cfg_.deadband_pct) {
      stop();
      return;
    }
    const float ramp = (cfg_.slow_zone_pct > 0.0f) ? min(1.0f, fabsf(err) / cfg_.slow_zone_pct) : 1.0f;
    const float duty = cfg_.min_duty_pct + (cfg_.max_duty_pct - cfg_.min_duty_pct) * ramp;
    drive(err > 0.0f ? duty : -duty);
  }

  if (now - stall_check_ms_ >= cfg_.stall_timeout_ms) {
    if (fabsf(position_pct_ - stall_ref_pct_) < cfg_.stall_min_change_pct) {
      // A jog reaching the end stop is expected during calibration, not a fault.
      if (state_ == State::Jogging) stop();
      else setFault(Fault::Stall);
      return;
    }
    resetStallWatch(now);
  }
}

const char* LinearActuator::stateName(State s) {
  switch (s) {
    case State::Idle: return "idle";
    case State::Moving: return "moving";
    case State::Jogging: return "jogging";
    case State::Fault: return "fault";
  }
  return "?";
}

const char* LinearActuator::faultName(Fault f) {
  switch (f) {
    case Fault::None: return "none";
    case Fault::Stall: return "stall";
    case Fault::AdcRange: return "adc_range";
  }
  return "?";
}
