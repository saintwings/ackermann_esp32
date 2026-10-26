#pragma once

#include <Arduino.h>
#include <Wire.h>

// PCF8574 I2C 8-bit I/O expander, used here as inputs only.
//
// The pins are quasi-bidirectional: writing 1 turns on a weak pull-up so the pin
// can be read; an external switch/transistor pulls it LOW. begin() writes 0xFF so
// every pin is an input. Address 0x20..0x27 (PCF8574) or 0x38..0x3F (PCF8574A).
class Pcf8574 {
 public:
  explicit Pcf8574(TwoWire& wire = Wire);

  bool begin(uint8_t address);
  bool present() const { return present_; }

  // Reads all 8 pins (raw levels, 1 = HIGH). Returns false on I2C error.
  bool read(uint8_t& levels);

 private:
  TwoWire& wire_;
  uint8_t address_{0x20};
  bool present_{false};
};
