#include "Ds3502.hpp"

namespace {
constexpr uint8_t REG_WIPER = 0x00;    // WR (and IVR in mode 0)
constexpr uint8_t REG_CONTROL = 0x02;  // CR
constexpr uint8_t CR_MODE1_WR_ONLY = 0x80;
}  // namespace

Ds3502::Ds3502(TwoWire& wire) : wire_(wire) {}

bool Ds3502::begin(uint8_t address) {
  address_ = address;
  present_ = false;
  wire_.beginTransmission(address_);
  if (wire_.endTransmission() != 0) return false;
  if (!writeReg(REG_CONTROL, CR_MODE1_WR_ONLY)) return false;
  const int w = readReg(REG_WIPER);
  if (w < 0) return false;
  wiper_ = static_cast<uint8_t>(w) & kMaxWiper;
  present_ = true;
  return true;
}

bool Ds3502::setWiper(uint8_t value) {
  if (!present_) return false;
  if (value > kMaxWiper) value = kMaxWiper;
  if (!writeReg(REG_WIPER, value)) return false;
  wiper_ = value;
  return true;
}

int Ds3502::readWiper() {
  if (!present_) return -1;
  const int w = readReg(REG_WIPER);
  return w < 0 ? -1 : (w & kMaxWiper);
}

bool Ds3502::writeReg(uint8_t reg, uint8_t value) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  wire_.write(value);
  return wire_.endTransmission() == 0;
}

int Ds3502::readReg(uint8_t reg) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return -1;
  if (wire_.requestFrom(address_, static_cast<uint8_t>(1)) != 1) return -1;
  return wire_.read();
}
