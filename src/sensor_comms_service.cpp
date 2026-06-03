#include "SensorCommsService.hpp"

#include <SPI.h>
#include <cstdlib>

#include "RobotConfig.h"

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

bool parseGgaLine(const String& line, GpsFix& out) {
  if (!(line.startsWith("$GNGGA") || line.startsWith("$GPGGA"))) return false;

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

void updateGpsAndNtrip(SensorCommsContext& ctx) {
  for (int i = 0; i < 128 && ctx.gps_uart.available(); ++i) {
    char c = static_cast<char>(ctx.gps_uart.read());
    ctx.gps_fix.last_rx_ms = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      if (ctx.gps_line_buffer.length() > 0) {
        parseGgaLine(ctx.gps_line_buffer, ctx.gps_fix);
        ctx.gps_line_buffer = "";
      }
    } else if (ctx.gps_line_buffer.length() < 220) {
      ctx.gps_line_buffer += c;
    }
  }

#if NTRIP_ENABLE
  if (ctx.ntrip_connected && ctx.ntrip_client && ctx.ntrip_client->connected()) {
    if (ctx.gps_fix.valid && (millis() - ctx.last_ntrip_gga_send_ms) > 1000 && ctx.gps_fix.last_gga.length() > 0) {
      ctx.ntrip_client->print(ctx.gps_fix.last_gga);
      ctx.ntrip_client->print("\r\n");
      ctx.last_ntrip_gga_send_ms = millis();
    }

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
#endif
}

}  // namespace SensorComms
