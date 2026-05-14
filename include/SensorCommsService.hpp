#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_BNO08x.h>

#include "SystemState.hpp"

struct SensorCommsContext {
  HardwareSerial& gps_uart;
  WiFiClient& ntrip_client;
  Adafruit_BNO08x& bno08x;
  sh2_SensorValue_t& bno_sensor_value;
  GpsFix& gps_fix;
  ImuState& imu_state;
  String& gps_line_buffer;
  unsigned long& last_ntrip_reconnect_ms;
  unsigned long& last_ntrip_gga_send_ms;
  unsigned long& ntrip_reconnect_backoff_ms;
  uint8_t& ntrip_fail_count;
  bool& ntrip_connected;
  bool& wifi_connected;
  bool& imu_ready;
};

namespace SensorComms {
void connectWifiIfNeeded(SensorCommsContext& ctx);
void initBno085(SensorCommsContext& ctx);
void updateImu(SensorCommsContext& ctx);
void connectNtripIfNeeded(SensorCommsContext& ctx);
void updateGpsAndNtrip(SensorCommsContext& ctx);
}
