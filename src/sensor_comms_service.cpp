#include "SensorCommsService.hpp"

#include <SPI.h>
#include <cmath>
#include <cstdlib>

#include "RobotConfig.h"

// ZED-F9P boot-time configuration (override in Config*.h)
#ifndef GPS_UBX_CONFIG_ENABLE
#define GPS_UBX_CONFIG_ENABLE 1
#endif
#ifndef GPS_NAV_RATE_MS
#define GPS_NAV_RATE_MS 0  // 0 = leave the receiver's measurement rate unchanged
#endif

namespace {

#if DEBUG_LOG_SENSOR_COMMS
#define SENSOR_COMMS_LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define SENSOR_COMMS_LOG_PRINTLN(msg) Serial.println(msg)
#else
#define SENSOR_COMMS_LOG_PRINTF(...)
#define SENSOR_COMMS_LOG_PRINTLN(msg)
#endif

String base64Encode(const String& input) {
  static const char* kAlphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out;
  int val = 0;
  int valb = -6;
  for (size_t i = 0; i < input.length(); ++i) {
    val = (val << 8) + static_cast<uint8_t>(input[i]);
    valb += 8;
    while (valb >= 0) {
      out += kAlphabet[(val >> valb) & 0x3F];
      valb -= 6;
    }
  }
  if (valb > -6) {
    out += kAlphabet[((val << 8) >> (valb + 8)) & 0x3F];
  }
  while ((out.length() % 4) != 0) {
    out += '=';
  }
  return out;
}

double parseNmeaCoord(const String& value, const String& hemisphere) {
  if (value.length() < 4) return 0.0;
  char* end_ptr = nullptr;
  double raw = std::strtod(value.c_str(), &end_ptr);
  if (end_ptr == value.c_str()) return 0.0;
  int degrees = static_cast<int>(raw / 100.0);
  double minutes = raw - (degrees * 100.0);
  double decimal = static_cast<double>(degrees) + (minutes / 60.0);
  if (hemisphere == "S" || hemisphere == "W") decimal = -decimal;
  return decimal;
}

// XOR of every char between '$' and '*' must match the two hex digits after '*'.
// Rejects sentences spliced together when UART bytes are dropped (RX overflow),
// which otherwise parse into shifted fields (fix_quality=66, lat/lon=0, ...).
bool nmeaChecksumOk(const String& line) {
  int star = line.lastIndexOf('*');
  if (star < 1 || star + 3 > static_cast<int>(line.length())) return false;
  uint8_t sum = 0;
  for (int i = 1; i < star; ++i) sum ^= static_cast<uint8_t>(line[i]);
  String hex = line.substring(star + 1, star + 3);
  char* end_ptr = nullptr;
  long expected = std::strtol(hex.c_str(), &end_ptr, 16);
  if (end_ptr != hex.c_str() + 2) return false;
  return sum == static_cast<uint8_t>(expected);
}

bool parseGgaLine(const String& line, GpsFix& out) {
  if (!(line.startsWith("$GNGGA") || line.startsWith("$GPGGA"))) return false;
  if (!nmeaChecksumOk(line)) {  // keep the previous fix; stale timeout covers long outages
    ++out.gga_bad_count;
    return false;
  }
  ++out.gga_ok_count;

  String fields[15];
  int field_count = 0;
  int start = 0;
  for (int i = 0; i <= line.length() && field_count < 15; ++i) {
    if (i == line.length() || line[i] == ',') {
      fields[field_count++] = line.substring(start, i);
      start = i + 1;
    }
  }
  if (field_count < 10) return false;

  int fix_quality = fields[6].toInt();
  if (fix_quality < 0 || fix_quality > 8) return false;  // GGA quality is 0..8
  out.has_gga = true;
  out.fix_quality = fix_quality;
  out.satellites = fields[7].toInt();
  out.hdop = fields[8].toFloat();
  out.altitude = fields[9].toFloat();
  out.last_gga = line;
  out.last_update_ms = millis();
  if (fields[2].length() > 0 && fields[4].length() > 0) {
    out.latitude = parseNmeaCoord(fields[2], fields[3]);
    out.longitude = parseNmeaCoord(fields[4], fields[5]);
  }
  out.valid = (fix_quality > 0);
  return out.valid;
}

// $GNGST,time,rms,smaj,smin,orient,lat_std,lon_std,alt_std*cs
bool parseGstLine(const String& line, GpsFix& out) {
  if (!(line.startsWith("$GNGST") || line.startsWith("$GPGST"))) return false;
  if (!nmeaChecksumOk(line)) {
    ++out.gga_bad_count;  // same symptom (dropped bytes) — count together
    return false;
  }
  int star = line.lastIndexOf('*');
  String body = line.substring(0, star);
  String fields[9];
  int field_count = 0;
  int start = 0;
  for (int i = 0; i <= body.length() && field_count < 9; ++i) {
    if (i == body.length() || body[i] == ',') {
      fields[field_count++] = body.substring(start, i);
      start = i + 1;
    }
  }
  if (field_count < 9 || fields[6].length() == 0 || fields[7].length() == 0) return false;
  out.std_lat_m = fields[6].toFloat();
  out.std_lon_m = fields[7].toFloat();
  out.std_alt_m = fields[8].toFloat();
  out.h_err_m = sqrtf(out.std_lat_m * out.std_lat_m + out.std_lon_m * out.std_lon_m);
  out.has_gst = true;
  out.last_gst_ms = millis();
  return true;
}

// ── UBX-CFG-VALSET (u-blox F9 configuration interface) ───────────────────────

// CFG-MSGOUT-NMEA_ID_<msg>_UART1 keys (U1 value = output rate per epoch).
// The UART2 key is always UART1 key + 1.
struct NmeaRateKey { uint32_t uart1_key; uint8_t rate; const char* name; };
constexpr NmeaRateKey kNmeaRates[] = {
  {0x209100bb, 1, "GGA"},  // position + fix quality: navigation and NTRIP VRS
  {0x209100d4, 1, "GST"},  // position error estimate
  {0x209100c5, 0, "GSV"},  // satellites in view — biggest, ~1 KB/epoch
  {0x209100c0, 0, "GSA"},
  {0x209100ca, 0, "GLL"},
  {0x209100b1, 0, "VTG"},
  {0x209100ac, 0, "RMC"},
};
constexpr uint32_t kCfgRateMeasKey = 0x30210001;  // CFG-RATE-MEAS (U2, ms)

size_t ubxBuild(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, uint8_t* out) {
  out[0] = 0xB5; out[1] = 0x62; out[2] = cls; out[3] = id;
  out[4] = len & 0xFF; out[5] = len >> 8;
  memcpy(out + 6, payload, len);
  uint8_t ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < 6u + len; ++i) { ck_a += out[i]; ck_b += ck_a; }
  out[6 + len] = ck_a; out[7 + len] = ck_b;
  return 8u + len;
}

// Wait for UBX-ACK-ACK / ACK-NAK for CFG-VALSET (0x06 0x8A), skipping the NMEA
// stream. Returns 1 = ACK, 0 = NAK, -1 = timeout.
int ubxWaitAck(HardwareSerial& uart, uint32_t timeout_ms) {
  // Match: B5 62 05 <01|00> 02 00 06 8A
  static const uint8_t kPrefix[] = {0xB5, 0x62, 0x05};
  uint8_t state = 0;
  uint8_t ack_id = 0;
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (!uart.available()) { delay(1); continue; }
    uint8_t b = static_cast<uint8_t>(uart.read());
    if (state < 3) {
      state = (b == kPrefix[state]) ? state + 1 : (b == 0xB5 ? 1 : 0);
    } else if (state == 3) {
      if (b == 0x01 || b == 0x00) { ack_id = b; state = 4; } else state = 0;
    } else if (state == 4) { state = (b == 0x02) ? 5 : 0;
    } else if (state == 5) { state = (b == 0x00) ? 6 : 0;
    } else if (state == 6) { state = (b == 0x06) ? 7 : 0;
    } else if (state == 7) {
      if (b == 0x8A) return ack_id == 0x01 ? 1 : 0;
      state = 0;
    }
  }
  return -1;
}

}  // namespace

namespace SensorComms {

// WiFi connection is now managed by NetManager — see net_manager.cpp.
// sensor_comms.wifi_connected and sensor_comms.ntrip_client are updated
// in commsTask (main.cpp) before connectNtripIfNeeded is called.

void initBno085(SensorCommsContext& ctx) {
  SENSOR_COMMS_LOG_PRINTF("[IMU] Init SPI pins SCK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d\n",
                BNO085_SPI_SCK_PIN,
                BNO085_SPI_MISO_PIN,
                BNO085_SPI_MOSI_PIN,
                BNO085_SPI_CS_PIN,
                BNO085_SPI_INT_PIN,
                BNO085_SPI_RST_PIN);

  pinMode(BNO085_SPI_RST_PIN, OUTPUT);
  digitalWrite(BNO085_SPI_RST_PIN, LOW);
  delay(20);
  digitalWrite(BNO085_SPI_RST_PIN, HIGH);
  delay(120);

  pinMode(BNO085_SPI_INT_PIN, INPUT_PULLUP);
  SPI.begin(BNO085_SPI_SCK_PIN, BNO085_SPI_MISO_PIN, BNO085_SPI_MOSI_PIN, BNO085_SPI_CS_PIN);
  ctx.imu_ready = false;
  for (int attempt = 1; attempt <= 5; ++attempt) {
    ctx.imu_ready = ctx.bno08x.begin_SPI(BNO085_SPI_CS_PIN, BNO085_SPI_INT_PIN, &SPI);
    if (ctx.imu_ready) {
      SENSOR_COMMS_LOG_PRINTF("[IMU] BNO085 detected on attempt %d\n", attempt);
      break;
    }
    SENSOR_COMMS_LOG_PRINTF("[IMU] BNO085 init attempt %d failed\n", attempt);
    delay(120);
  }
  if (!ctx.imu_ready) {
    SENSOR_COMMS_LOG_PRINTLN("[IMU] BNO085 init failed after retries");
    return;
  }

  ctx.bno08x.enableReport(SH2_ACCELEROMETER, 10000);
  ctx.bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  ctx.bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
  SENSOR_COMMS_LOG_PRINTLN("[IMU] BNO085 ready (SPI)");
}

void updateImu(SensorCommsContext& ctx) {
  if (!ctx.imu_ready) return;
  if (digitalRead(BNO085_SPI_INT_PIN) != LOW) return;

  for (int i = 0; i < 6; ++i) {
    if (!ctx.bno08x.getSensorEvent(&ctx.bno_sensor_value)) {
      break;
    }
    if (ctx.bno_sensor_value.sensorId == SH2_ACCELEROMETER) {
      ctx.imu_state.ax = ctx.bno_sensor_value.un.accelerometer.x;
      ctx.imu_state.ay = ctx.bno_sensor_value.un.accelerometer.y;
      ctx.imu_state.az = ctx.bno_sensor_value.un.accelerometer.z;
      ctx.imu_state.valid = true;
      ctx.imu_state.last_update_ms = millis();
    } else if (ctx.bno_sensor_value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      ctx.imu_state.gx = ctx.bno_sensor_value.un.gyroscope.x;
      ctx.imu_state.gy = ctx.bno_sensor_value.un.gyroscope.y;
      ctx.imu_state.gz = ctx.bno_sensor_value.un.gyroscope.z;
      ctx.imu_state.valid = true;
      ctx.imu_state.last_update_ms = millis();
    } else if (ctx.bno_sensor_value.sensorId == SH2_ROTATION_VECTOR) {
      ctx.imu_state.qw = ctx.bno_sensor_value.un.rotationVector.real;
      ctx.imu_state.qx = ctx.bno_sensor_value.un.rotationVector.i;
      ctx.imu_state.qy = ctx.bno_sensor_value.un.rotationVector.j;
      ctx.imu_state.qz = ctx.bno_sensor_value.un.rotationVector.k;
      ctx.imu_state.valid = true;
      ctx.imu_state.last_update_ms = millis();
    }

    if (digitalRead(BNO085_SPI_INT_PIN) != LOW) {
      break;
    }
  }
}

void connectNtripIfNeeded(SensorCommsContext& ctx) {
#if NTRIP_ENABLE
  // Guard: no network transport available yet (set by NetManager in commsTask)
  if (!ctx.ntrip_client) {
    ctx.ntrip_connected = false;
    return;
  }

  if (ctx.ntrip_client->connected()) {
    ctx.ntrip_connected = true;
    ctx.ntrip_reconnect_backoff_ms = 3000;
    ctx.ntrip_fail_count = 0;
    return;
  }

  unsigned long now = millis();
  if (now - ctx.last_ntrip_reconnect_ms < ctx.ntrip_reconnect_backoff_ms) return;
  ctx.last_ntrip_reconnect_ms = now;

  ctx.ntrip_client->stop();
  ctx.ntrip_client->setTimeout(1200);
  SENSOR_COMMS_LOG_PRINTF("[NTRIP] Connecting to %s:%d (retry=%u, backoff=%lums)\n",
                NTRIP_HOST,
                NTRIP_PORT,
                static_cast<unsigned>(ctx.ntrip_fail_count),
                ctx.ntrip_reconnect_backoff_ms);
  if (!ctx.ntrip_client->connect(NTRIP_HOST, NTRIP_PORT)) {
    ctx.ntrip_connected = false;
    ++ctx.ntrip_fail_count;
    ctx.ntrip_reconnect_backoff_ms = min<unsigned long>(ctx.ntrip_reconnect_backoff_ms * 2UL, 60000UL);
    SENSOR_COMMS_LOG_PRINTF("[NTRIP] TCP connect failed, next retry in %lums\n", ctx.ntrip_reconnect_backoff_ms);
    return;
  }

  String credentials = String(NTRIP_USER) + ":" + String(NTRIP_PASSWORD);
  String auth = base64Encode(credentials);
  String request;
  request += "GET /";
  request += NTRIP_MOUNTPOINT;
  request += " HTTP/1.1\r\n";
  request += "Host: ";
  request += NTRIP_HOST;
  request += ":";
  request += String(NTRIP_PORT);
  request += "\r\n";
  request += "Ntrip-Version: Ntrip/2.0\r\n";
  request += "User-Agent: ESP32-NTRIP/1.0\r\n";
  request += "Authorization: Basic ";
  request += auth;
  request += "\r\nConnection: keep-alive\r\n\r\n";
  ctx.ntrip_client->print(request);

  unsigned long t0 = millis();
  String response;
  while (millis() - t0 < 1200) {
    while (ctx.ntrip_client->available()) {
      response += static_cast<char>(ctx.ntrip_client->read());
      if (response.endsWith("\r\n\r\n")) break;
    }
    if (response.endsWith("\r\n\r\n")) break;
    delay(10);
  }

  if (response.indexOf("200 OK") >= 0 || response.indexOf("ICY 200 OK") >= 0) {
    ctx.ntrip_connected = true;
    ctx.ntrip_client->setTimeout(20);
    ctx.ntrip_reconnect_backoff_ms = 3000;
    ctx.ntrip_fail_count = 0;
    SENSOR_COMMS_LOG_PRINTLN("[NTRIP] Connected");
  } else {
    ctx.ntrip_connected = false;
    ++ctx.ntrip_fail_count;
    ctx.ntrip_reconnect_backoff_ms = min<unsigned long>(ctx.ntrip_reconnect_backoff_ms * 2UL, 60000UL);
    int eol = response.indexOf("\r\n");
    String status_line = (eol > 0) ? response.substring(0, eol) : response;
    SENSOR_COMMS_LOG_PRINTF("[NTRIP] Handshake failed: %s\n", status_line.c_str());
    SENSOR_COMMS_LOG_PRINTF("[NTRIP] Next retry in %lums\n", ctx.ntrip_reconnect_backoff_ms);
    ctx.ntrip_client->stop();
  }
#else
  (void)ctx;
#endif
}

bool configureGnssOutput(SensorCommsContext& ctx) {
#if !GPS_UBX_CONFIG_ENABLE
  (void)ctx;
  return false;
#else
  // Payload: version=0, layers=RAM (0x01), reserved x2, then key/value pairs.
  // Applied to UART1 and UART2 since either may be the one wired to the ESP32.
  uint8_t payload[4 + 2 * (sizeof(kNmeaRates) / sizeof(kNmeaRates[0])) * 5 + 6];
  size_t n = 0;
  payload[n++] = 0x00;
  payload[n++] = 0x01;
  payload[n++] = 0x00;
  payload[n++] = 0x00;
  for (const NmeaRateKey& k : kNmeaRates) {
    for (uint32_t key : {k.uart1_key, k.uart1_key + 1}) {
      memcpy(payload + n, &key, 4);  // little-endian on ESP32
      n += 4;
      payload[n++] = k.rate;
    }
  }
#if GPS_NAV_RATE_MS > 0
  uint32_t rate_key = kCfgRateMeasKey;
  uint16_t rate_ms = GPS_NAV_RATE_MS;
  memcpy(payload + n, &rate_key, 4);
  n += 4;
  memcpy(payload + n, &rate_ms, 2);
  n += 2;
#endif

  uint8_t frame[8 + sizeof(payload)];
  size_t frame_len = ubxBuild(0x06, 0x8A, payload, static_cast<uint16_t>(n), frame);

  // The F9P may still be booting if it powered up with the ESP32 — retry.
  for (int attempt = 1; attempt <= 5; ++attempt) {
    ctx.gps_uart.write(frame, frame_len);
    ctx.gps_uart.flush();
    int r = ubxWaitAck(ctx.gps_uart, 600);
    if (r == 1) {
      Serial.printf("[GPS] UBX config ACK (GGA+GST on, GSV/GSA/GLL/VTG/RMC off, rate=%d ms)\n",
                    GPS_NAV_RATE_MS);
      return true;
    }
    Serial.printf("[GPS] UBX config %s (attempt %d)\n", r == 0 ? "NAK" : "no reply", attempt);
    delay(300);
  }
  Serial.println("[GPS] UBX config failed — receiver keeps its stored NMEA settings");
  return false;
#endif
}

void readGps(SensorCommsContext& ctx) {
  // Drain everything waiting (bounded so a babbling UART can't starve the
  // caller). Runs in NavTask, which is never blocked by SIM modem AT traffic.
  for (int i = 0; i < 4096 && ctx.gps_uart.available(); ++i) {
    char c = static_cast<char>(ctx.gps_uart.read());
    ctx.gps_fix.last_rx_ms = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      if (ctx.gps_line_buffer.length() > 0) {
        if (!parseGgaLine(ctx.gps_line_buffer, ctx.gps_fix)) {
          parseGstLine(ctx.gps_line_buffer, ctx.gps_fix);
        }
        ctx.gps_line_buffer = "";
      }
    } else if (ctx.gps_line_buffer.length() < 220) {
      ctx.gps_line_buffer += c;
    }
  }
}

void forwardNtrip(SensorCommsContext& ctx, const String& latest_gga) {
#if NTRIP_ENABLE
  if (ctx.ntrip_connected && ctx.ntrip_client && ctx.ntrip_client->connected()) {
    if (latest_gga.length() > 0 && (millis() - ctx.last_ntrip_gga_send_ms) > 1000) {
      ctx.ntrip_client->print(latest_gga);
      ctx.ntrip_client->print("\r\n");
      ctx.last_ntrip_gga_send_ms = millis();
    }

    // One read per tick: on SIM each TinyGsmClient available()/read() can cost
    // an AT+CIPRXGET round-trip, so looping here stalls the comms task for seconds.
    int avail = ctx.ntrip_client->available();
    if (avail > 0) {
      uint8_t rtcm_buf[256];
      int to_read = (avail > static_cast<int>(sizeof(rtcm_buf))) ? static_cast<int>(sizeof(rtcm_buf)) : avail;
      int read_n = ctx.ntrip_client->read(rtcm_buf, to_read);
      if (read_n > 0) {
        ctx.gps_uart.write(rtcm_buf, read_n);
      }
    }
  } else {
    ctx.ntrip_connected = false;
  }
#else
  (void)ctx;
  (void)latest_gga;
#endif
}

}  // namespace SensorComms
