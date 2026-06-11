#include "SimWsClient.hpp"
#include "RobotConfig.h"

// Masking key — client-to-server frames must be masked (RFC 6455).
static const uint8_t kMask[4] = {0x3C, 0xA1, 0x55, 0xF2};
static const char*   kWsKey   = "dGhlIHNhbXBsZSBub25jZQ==";

// SSL mux index used for our WebSocket connection.
// Mux 0 is owned by TinyGSM (NTRIP plain TCP); mux 1 is ours.
static constexpr uint8_t kMux = 1;

// ─────────────────────────────────────────────────────────────────────────────

SimWsClient::SimWsClient(Stream& simSerial) : _stream(simSerial) {}

// ── Internal helper ───────────────────────────────────────────────────────────

bool SimWsClient::waitStr(const char* expected, uint32_t timeoutMs) {
  String acc;
  unsigned long deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    while (_stream.available()) {
      char c = (char)_stream.read();
      acc += c;
      if (acc.indexOf(expected) >= 0) return true;
      if (acc.length() > 256) acc = acc.substring(acc.length() - 128);
    }
    delay(1);
  }
  return false;
}

// ── SSL TCP layer — uses AT+CIPOPEN "SSL" (CIPOPEN stack, mux kMux) ───────────

// Read and log every line from the modem until 'needle' is found or timeout.
// Returns true if needle was found. Always prints every non-empty line received.
bool SimWsClient::waitLine(const char* needle, uint32_t timeoutMs, const char* logTag) {
  String line;
  unsigned long deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    while (_stream.available()) {
      char c = (char)_stream.read();
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          Serial.printf("[SIM_WS] %s: %s\n", logTag, line.c_str());
          if (line.indexOf(needle) >= 0) return true;
        }
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
    delay(1);
  }
  return false;
}

bool SimWsClient::sslStart() {
  delay(50);
  while (_stream.available()) _stream.read();  // drain stale bytes

  // Enable manual-receive mode — required before AT+CIPOPEN.
  _stream.print("AT+CIPRXGET=1\r\n");
  waitLine("OK", 2000, "CIPRXGET=1");

  return true;
}

bool SimWsClient::sslOpen(const char* host, uint16_t port) {
  // AT+CSSLCFG is completely absent in firmware V11.0.01 — SSL cannot be
  // configured. Use plain TCP instead; caller must supply an HTTP (non-SSL) port.
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "AT+CIPOPEN=%u,\"TCP\",\"%s\",%u\r\n", kMux, host, port);
  Serial.printf("[SIM_WS] > %s", cmd);
  _stream.print(cmd);

  // Read responses line-by-line: log everything, act on +CIPOPEN URC or ERROR.
  String line;
  char openUrc[20];
  snprintf(openUrc, sizeof(openUrc), "+CIPOPEN: %u,", kMux);
  unsigned long deadline = millis() + 35000;

  while (millis() < deadline) {
    while (_stream.available()) {
      char c = (char)_stream.read();
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          Serial.printf("[SIM_WS] CIPOPEN: %s\n", line.c_str());
          if (line.startsWith(openUrc)) {
            int code = line.substring(strlen(openUrc)).toInt();
            if (code != 0)
              Serial.printf("[SIM_WS] CIPOPEN error code: %d\n", code);
            return (code == 0);
          }
          if (line.indexOf("ERROR") >= 0) return false;
        }
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
    delay(1);
  }
  Serial.println("[SIM_WS] CIPOPEN timeout (no URC)");
  return false;
}

bool SimWsClient::cipSend(const uint8_t* data, size_t len) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%u\r\n", kMux, (unsigned)len);
  _stream.print(cmd);

  if (!waitStr(">", 3000)) return false;

  _stream.write(data, len);
  _stream.flush();

  // Wait for send confirmation: +CIPSEND: mux,req,actual
  char cipsend[16];
  snprintf(cipsend, sizeof(cipsend), "+CIPSEND: %u,", kMux);
  return waitStr(cipsend, 5000);
}

int SimWsClient::cipRecv(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
  size_t ask = (maxLen < 1460) ? maxLen : 1460;

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CIPRXGET=2,%u,%u\r\n", kMux, (unsigned)ask);
  _stream.print(cmd);

  // Response (line-based, binary-safe):
  //   "+CIPRXGET: 2,<mux>,<actual_len>,<remain>\r\n" + <actual_len bytes> + "\r\nOK\r\n"
  //   "+CIPRXGET: 2,<mux>,0,0\r\n" + "\r\nOK\r\n"   (no data)
  //   "ERROR\r\n"                                    (mux not open / bad state)
  String line;
  unsigned long deadline = millis() + timeoutMs;
  char prefix[24];
  snprintf(prefix, sizeof(prefix), "+CIPRXGET: 2,%u,", kMux);

  while (millis() < deadline) {
    while (_stream.available()) {
      char c = (char)_stream.read();
      if (c == '\n') {
        line.trim();
        if (line.indexOf("ERROR") >= 0) return 0;

        if (line.startsWith(prefix)) {
          String rest     = line.substring(strlen(prefix));  // "actual,remain"
          int    comma    = rest.indexOf(',');
          int    dataLen  = (comma > 0) ? rest.substring(0, comma).toInt() : rest.toInt();
          if (dataLen <= 0) { waitStr("OK", 500); return 0; }

          size_t toRead       = (size_t)dataLen < maxLen ? (size_t)dataLen : maxLen;
          size_t got          = 0;
          unsigned long dDl   = millis() + 3000;
          while (got < toRead && millis() < dDl) {
            if (_stream.available()) buf[got++] = (uint8_t)_stream.read();
            else delay(1);
          }
          int excess = dataLen - (int)maxLen;
          if (excess > 0) {
            unsigned long eDl = millis() + 2000;
            while (excess-- > 0 && millis() < eDl) {
              while (!_stream.available() && millis() < eDl) delay(1);
              if (_stream.available()) _stream.read();
            }
          }
          waitStr("OK", 1000);
          return (int)got;
        }
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
    delay(1);
  }
  return 0;
}

// ── WebSocket layer ───────────────────────────────────────────────────────────

bool SimWsClient::wsHandshake(const char* host, const char* path) {
  String req = "GET ";
  req += path;
  req += " HTTP/1.1\r\nHost: ";
  req += host;
  req += "\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: ";
  req += kWsKey;
  req += "\r\nSec-WebSocket-Version: 13\r\n\r\n";

  if (!cipSend((const uint8_t*)req.c_str(), req.length())) return false;

  // Accumulate HTTP response until the last \r\n\r\n (end of all headers).
  // Cloudflare appends CF-RAY + alt-svc after the standard 101 headers in the
  // same TCP chunk, producing two \r\n\r\n sequences.  We loop until the first
  // one appears, then drain once more to pick up trailing CF headers, and use
  // lastIndexOf to find the true end.
  //
  // Post-header bytes are intentionally NOT saved to _rxBuf here — they consist
  // of modem AT framing leakage (OK\r\n) that corrupts WS frame parsing.
  // connect() clears _rxLen after this returns, so we start frame parsing clean.
  String hdrs;
  uint8_t chunk[512];
  unsigned long deadline = millis() + 10000;

  while (millis() < deadline) {
    delay(200);
    int n = cipRecv(chunk, sizeof(chunk), 1500);
    if (n > 0) {
      for (int i = 0; i < n; i++) hdrs += (char)chunk[i];
    }
    if (hdrs.indexOf("\r\n\r\n") < 0) continue;  // headers not complete yet

    // One more drain to capture any Cloudflare trailing headers (CF-RAY, alt-svc)
    // that arrive in a second TCP segment after the initial 101.
    delay(500);
    n = cipRecv(chunk, sizeof(chunk), 800);
    if (n > 0) {
      for (int i = 0; i < n; i++) hdrs += (char)chunk[i];
    }

    bool ok = hdrs.indexOf("101") >= 0;
    if (!ok) Serial.printf("[SIM_WS] Unexpected HTTP: %.80s\n", hdrs.c_str());
    return ok;
  }
  Serial.println("[SIM_WS] Handshake timeout");
  return false;
}

bool SimWsClient::sendWsFrame(uint8_t opcode, const uint8_t* payload, size_t payloadLen) {
  uint8_t header[8];
  size_t  headerLen;

  header[0] = 0x80 | opcode;

  if (payloadLen < 126) {
    header[1] = 0x80 | (uint8_t)payloadLen;
    headerLen = 2;
  } else {
    header[1] = 0xFE;
    header[2] = (uint8_t)(payloadLen >> 8);
    header[3] = (uint8_t)(payloadLen & 0xFF);
    headerLen = 4;
  }
  header[headerLen + 0] = kMask[0];
  header[headerLen + 1] = kMask[1];
  header[headerLen + 2] = kMask[2];
  header[headerLen + 3] = kMask[3];
  headerLen += 4;

  size_t   frameLen = headerLen + payloadLen;
  uint8_t* frame    = new (std::nothrow) uint8_t[frameLen];
  if (!frame) return false;

  memcpy(frame, header, headerLen);
  for (size_t i = 0; i < payloadLen; i++) {
    frame[headerLen + i] = payload[i] ^ kMask[i & 3];
  }

  bool ok = cipSend(frame, frameLen);
  delete[] frame;

  if (!ok) {
    Serial.println("[SIM_WS] cipSend failed — marking disconnected");
    _connected = false;
    return false;
  }

  // Drain any incoming data that arrived while cipSend() was blocking.
  // waitStr() consumes bytes including +CIPRXGET URCs, but the modem still
  // buffers the data — an explicit AT+CIPRXGET=2 call retrieves it.
  if (_rxLen < kRxBufSize) {
    int n = cipRecv(_rxBuf + _rxLen, kRxBufSize - _rxLen, 300);
    if (n > 0) {
      Serial.printf("[SIM_WS] Drained %d bytes after send\n", n);
      _rxLen += (size_t)n;
    }
  }
  return ok;
}

int SimWsClient::parseWsFrame(String& payloadOut) {
  if (_rxLen < 2) return -1;

  uint8_t opcode = _rxBuf[0] & 0x0F;
  uint8_t b1     = _rxBuf[1];
  bool    masked = (b1 & 0x80) != 0;
  size_t  plen   = b1 & 0x7F;
  size_t  hlen   = 2;

  if (plen == 126) {
    if (_rxLen < 4) return -1;
    plen = ((uint16_t)_rxBuf[2] << 8) | _rxBuf[3];
    hlen = 4;
  } else if (plen == 127) {
    return -1;
  }
  if (masked) hlen += 4;

  size_t total = hlen + plen;
  if (_rxLen < total) return -1;

  const uint8_t* maskPtr = masked ? &_rxBuf[hlen - 4] : nullptr;
  const uint8_t* data    = &_rxBuf[hlen];

  // Log every non-text frame opcode for diagnostics
  if (opcode != 0x01 && opcode != 0x00) {
    if (opcode == 0x0A && _pingSentMs > 0) {
      Serial.printf("[SIM_WS] pong RTT=%lums\n", millis() - _pingSentMs);
      _pingSentMs = 0;
    } else {
      Serial.printf("[SIM_WS] ctrl op=0x%02X plen=%u\n", opcode, (unsigned)plen);
    }
  }

  if (opcode == 0x01 || opcode == 0x00) {
    String frame;
    for (size_t i = 0; i < plen; i++) {
      frame += (char)(maskPtr ? (data[i] ^ maskPtr[i & 3]) : data[i]);
    }
    Serial.printf("[SIM_WS] text: %.80s\n", frame.c_str());
    payloadOut += frame;
  } else if (opcode == 0x09) {           // ping → pong
    sendWsFrame(0x0A, data, plen);
  }

  memmove(_rxBuf, _rxBuf + total, _rxLen - total);
  _rxLen -= total;
  return (int)opcode;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool SimWsClient::connect(const char* host, uint16_t port, const char* path) {
  disconnect();
  _rxLen = 0;

  sslStart();  // configure SSL / manual-receive mode; errors non-fatal

  if (!sslOpen(host, port)) {
    Serial.println("[SIM_WS] SSL open failed");
    disconnect();
    return false;
  }
  Serial.printf("[SIM_WS] SSL open OK → %s:%u\n", host, port);

  if (!wsHandshake(host, path)) {
    Serial.println("[SIM_WS] WebSocket handshake failed");
    disconnect();
    return false;
  }

  // Discard any bytes that leaked into the buffer during the HTTP handshake
  // (modem AT framing: OK\r\n).  The server sends nothing until after the robot
  // sends its register message, so nothing useful is lost here.
  _rxLen = 0;

  _connected   = true;
  _lastProbeMs = millis();
  Serial.println("[SIM_WS] WebSocket connected via SIM");
  return true;
}

void SimWsClient::disconnect() {
  // Always send AT+CIPCLOSE regardless of internal state.
  // If the channel is already closed the modem returns ERROR — harmless.
  // This guarantees a clean slate before any reconnect attempt.
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%u\r\n", kMux);
  _stream.print(cmd);
  waitStr("OK", 2000);
  _sslStarted = false;
  _connected  = false;
  _rxLen      = 0;
}

bool SimWsClient::sendText(const String& text) {
  if (!_connected) return false;
  return sendWsFrame(0x01, (const uint8_t*)text.c_str(), text.length());
}

void SimWsClient::sendPing() {
  if (!_connected) return;
  _pingSentMs = millis();
  sendWsFrame(0x09, nullptr, 0);
}

bool SimWsClient::poll(String& textOut) {
  if (!_connected) return false;

  // --- Read any bytes already in the serial buffer (URCs, etc.) ---
  String urcBuf;
  while (_stream.available()) {
    char c = (char)_stream.read();
    urcBuf += c;
  }

  // +CIPRXGET: 1,<mux>  →  data available (no length in URC)
  char ciprxUrc[20];
  snprintf(ciprxUrc, sizeof(ciprxUrc), "+CIPRXGET: 1,%u", kMux);

  bool fetchNow = urcBuf.indexOf(ciprxUrc) >= 0;

  // +CIPCLOSE: <mux>  →  connection dropped by server
  char closeUrc[20];
  snprintf(closeUrc, sizeof(closeUrc), "+CIPCLOSE: %u", kMux);
  if (urcBuf.indexOf(closeUrc) >= 0) {
    Serial.println("[SIM_WS] CIPCLOSE URC: TCP closed by modem");
    _connected = false;
    return false;
  }

  // Periodic probe every 1 s in case we missed a URC
  unsigned long now = millis();
  if (!fetchNow && (now - _lastProbeMs >= 1000)) {
    fetchNow     = true;
    _lastProbeMs = now;
  }

  // --- Fetch data ---
  if (fetchNow) {
    _lastProbeMs = now;
    size_t space = kRxBufSize - _rxLen;
    if (space > 0) {
      int n = cipRecv(_rxBuf + _rxLen, space, 2000);
      if (n > 0) {
        // Serial.printf("[SIM_WS] rx %d bytes (buf=%u)\n", n, (unsigned)(_rxLen + n));
        _rxLen += (size_t)n;
      }
    }
  }

  // --- Parse WebSocket frames ---
  bool gotText = false;
  while (_rxLen > 0) {
    int opcode = parseWsFrame(textOut);
    if (opcode < 0) break;
    if (opcode == 0x08) {
      Serial.println("[SIM_WS] WS close frame received");
      _connected = false;
      return false;
    }
    if (opcode == 0x01 || opcode == 0x00) gotText = true;
  }
  return gotText;
}
