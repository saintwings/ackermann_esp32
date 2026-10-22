#pragma once

#include <Arduino.h>
#include <Client.h>
#include <Adafruit_BNO08x.h>

#include "SystemState.hpp"

struct SensorCommsContext {
  HardwareSerial& gps_uart;
  // Pointer to the active TCP transport (WiFiClient or TinyGsmClient).
  // Set by NetManager each comms tick. nullptr = no network available.
  Client* ntrip_client;
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
  bool& wifi_connected;   // true only when WiFi is the active bearer (used by LED1)
  bool& imu_ready;
};

namespace SensorComms {
// Configure the ZED-F9P NMEA output over the GPS UART (RAM layer, every boot):
// GGA + GST on, GSV/GSA/GLL/VTG/RMC off. Returns true if the receiver ACKed.
bool configureGnssOutput(SensorCommsContext& ctx);
void initBno085(SensorCommsContext& ctx);
void updateImu(SensorCommsContext& ctx);
void connectNtripIfNeeded(SensorCommsContext& ctx);
// Drain + parse the GPS UART into ctx.gps_fix (NavTask; caller holds the nav lock).
void readGps(SensorCommsContext& ctx);
// Send the latest GGA to the VRS caster (1 Hz) and forward one RTCM read to the
// receiver (CommsTask). latest_gga = "" while the fix is invalid.
void forwardNtrip(SensorCommsContext& ctx, const String& latest_gga);
}
