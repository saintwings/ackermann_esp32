#pragma once

#include <Arduino.h>
#include <Wire.h>

// DS3502 I2C digital potentiometer (10 kOhm, 128 wiper steps, 0..127).
//
// begin() puts the chip in mode 1 (CR bit 7 = 1): wiper writes go to the volatile
// WR register only, so frequent changes don't wear the EEPROM. At power-up the chip
// loads the wiper from its EEPROM (IVR, factory default 0x40) until begin() runs.
class Ds3502 {
 public:
  static constexpr uint8_t kMaxWiper = 127;

  explicit Ds3502(TwoWire& wire = Wire);

  // address: 0x28 + A1/A0 straps (0x28..0x2B).
  bool begin(uint8_t address);
  uint8_t address() const { return address_; }
  bool present() const { return present_; }

  bool setWiper(uint8_t value);
  // Reads the wiper back from the chip; returns -1 on I2C error.
  int readWiper();
  // Last value written with setWiper(), or the value read at begin().
  uint8_t wiper() const { return wiper_; }

 private:
  bool writeReg(uint8_t reg, uint8_t value);
  int readReg(uint8_t reg);

  TwoWire& wire_;
  uint8_t address_{0x28};
  bool present_{false};
  uint8_t wiper_{0};
};
