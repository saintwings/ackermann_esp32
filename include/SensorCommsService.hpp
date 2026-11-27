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

enum class GnssResetType : uint8_t { Hot, Warm, Cold };

namespace SensorComms {
// Configure the ZED-F9P NMEA output over the GPS UART (RAM layer, every boot):
// GGA + GST on, GSV/GSA/GLL/VTG/RMC off. Returns true if the receiver ACKed.
bool configureGnssOutput(SensorCommsContext& ctx);
void initBno085(SensorCommsContext& ctx);
void updateImu(SensorCommsContext& ctx);
void connectNtripIfNeeded(SensorCommsContext& ctx);
// Drain + parse the GPS UART into ctx.gps_fix (NavTask; caller holds the nav lock).
void readGps(SensorCommsContext& ctx);
// True when the fix is good enough to give to the VRS caster (NTRIP_GGA_* gate).
// Caller holds the nav lock (reads gps_fix).
bool ggaGoodForVrs(const GpsFix& fix, unsigned long now_ms);
// Send the latest GGA to the VRS caster (1 Hz) and forward one RTCM read to the
// receiver (CommsTask). latest_gga = "" while the fix fails ggaGoodForVrs.
// Also reconnects NTRIP when the fix turns good after a long weak spell (VRS refresh).
void forwardNtrip(SensorCommsContext& ctx, const String& latest_gga);
// What forwardNtrip did with GGA on the last tick: sent, held-weak-fix,
// held-vrs-refresh, vrs-refresh, no-session, none.
const char* ntripGgaStatus();
// Restart the receiver's GNSS engine (UBX-CFG-RST, GNSS-only). Hot keeps ephemeris and
// gets satellites back in seconds; clears a stuck RTK float. Caller holds the nav lock.
bool resetGnss(SensorCommsContext& ctx, GnssResetType type);
}
