#include "Pcf8574.hpp"

Pcf8574::Pcf8574(TwoWire& wire) : wire_(wire) {}

bool Pcf8574::begin(uint8_t address) {
  address_ = address;
  present_ = false;
  wire_.beginTransmission(address_);
  wire_.write(0xFF);  // all pins weak-HIGH = inputs
  if (wire_.endTransmission() != 0) return false;
  present_ = true;
  return true;
}

bool Pcf8574::read(uint8_t& levels) {
  if (!present_) return false;
  if (wire_.requestFrom(address_, static_cast<uint8_t>(1)) != 1) return false;
  levels = static_cast<uint8_t>(wire_.read());
  return true;
}
