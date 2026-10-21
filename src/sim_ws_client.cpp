#include "SimWsClient.hpp"
#include "RobotConfig.h"

// Masking key — client-to-server frames must be masked (RFC 6455).
static const uint8_t kMask[4] = {0x3C, 0xA1, 0x55, 0xF2};
static const char*   kWsKey   = "dGhlIHNhbXBsZSBub25jZQ==";

// SSL mux index used for our WebSocket connection.
// Mux 0 is owned by TinyGSM (NTRIP plain TCP); mux 1 is ours.
static constexpr uint8_t kMux = 1;
#define SIM_WS_MUX_STR "1"  // must match kMux — used to build URC prefixes

// URCs for our mux. Remote close is reported as +IPCLOSE (not +CIPCLOSE, which
// is only the response to our own AT+CIPCLOSE).
static const char* kUrcData    = "+CIPRXGET: 1," SIM_WS_MUX_STR;
static const char* kUrcIpClose = "+IPCLOSE: " SIM_WS_MUX_STR ",";

// Probe the modem for data this often even without a URC. The URC can be
// swallowed by TinyGSM (NTRIP on mux 0) when it reads the shared UART, so this
// bounds worst-case receive latency. AT+CIPRXGET=2 is a local modem query, cheap.
static constexpr unsigned long kProbeIntervalMs = 250;

// ─────────────────────────────────────────────────────────────────────────────

SimWsClient::SimWsClient(Stream& simSerial) : _stream(simSerial) {}

// ── Internal helpers ──────────────────────────────────────────────────────────

// Record URCs that concern our socket so they are not lost when another
// reader consumes them.
void SimWsClient::scanUrc(const String& line) {
  if (line.startsWith(kUrcData)) {
    _dataPending = true;
  } else if (line.startsWith(kUrcIpClose)) {
    Serial.printf("[SIM_WS] %s\n", line.c_str());
    _peerClosed = true;
  }
}

// Consume whatever is sitting in the UART (URCs between commands) so the next
// command starts on a clean stream and stale "ERROR"/"OK" lines don't confuse it.
void SimWsClient::drainUrcs() {
  while (_stream.available()) {
    char c = (char)_stream.read();
    if (c == '\n') {
      _urcLine.trim();
      if (_urcLine.length() > 0) scanUrc(_urcLine);
      _urcLine = "";
    } else if (c != '\r') {
      if (_urcLine.length() < 128) _urcLine += c;
    }
  }
}

// Wait for 'expected' anywhere in the stream (works for '>' prompts with no
// newline). Returns false early on a bare "ERROR" line.
bool SimWsClient::waitStr(const char* expected, uint32_t timeoutMs) {
  String acc;
  String line;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (_stream.available()) {
      char c = (char)_stream.read();
      acc += c;
      if (acc.indexOf(expected) >= 0) return true;
      if (acc.length() > 256) acc = acc.substring(acc.length() - 128);
      if (c == '\n') {
        line.trim();
        if (line == "ERROR") return false;
        scanUrc(line);
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
    delay(1);
  }
  return false;
}

// ── SSL TCP layer — uses AT+CIPOPEN "SSL" (CIPOPEN stack, mux kMux) ───────────

// Read lines from the modem until one contains 'needle' or timeout.
// Returns true if needle was found (line copied to lineOut if given); false on
// timeout or a bare "ERROR" line. Logs every line when logTag is non-null.
bool SimWsClient::waitLine(const char* needle, uint32_t timeoutMs, const char* logTag, String* lineOut) {
  String line;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (_stream.available()) {
      char c = (char)_stream.read();
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          if (logTag) Serial.printf("[SIM_WS] %s: %s\n", logTag, line.c_str());
          if (line.indexOf(needle) >= 0) {
            if (lineOut) *lineOut = line;
            return true;
          }
          if (line == "ERROR") return false;
          scanUrc(line);
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
  unsigned long deadline = millis() + 15000;

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
  drainUrcs();

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%u\r\n", kMux, (unsigned)len);
  _stream.print(cmd);

  if (!waitStr(">", 3000)) return false;  // ERROR → socket gone, fail fast

  _stream.write(data, len);
  _stream.flush();

  // Wait for send confirmation: +CIPSEND: mux,req,cnf
  // cnf < req (modem buffer full) or -1 (link down) means the WebSocket byte
  // stream is now broken — the caller must reconnect.
  char cipsend[16];
  snprintf(cipsend, sizeof(cipsend), "+CIPSEND: %u,", kMux);
  String line;
  if (!waitLine(cipsend, 5000, nullptr, &line)) return false;
  int cnf = line.substring(line.lastIndexOf(',') + 1).toInt();
  if (cnf != (int)len) {
    Serial.printf("[SIM_WS] Short send %d/%u\n", cnf, (unsigned)len);
    return false;
  }
  return true;
}

// Returns bytes read (>0), 0 when the modem has no data, or -1 when the byte
// stream is corrupted (short read) and the connection must be dropped.
int SimWsClient::cipRecv(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
  drainUrcs();

  size_t ask = (maxLen < 1460) ? maxLen : 1460;

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CIPRXGET=2,%u,%u\r\n", kMux, (unsigned)ask);
  _stream.print(cmd);

  // Response (line-based, binary-safe):
  //   "+CIPRXGET: 2,<mux>,<actual_len>,<remain>\r\n" + <actual_len bytes> + "\r\nOK\r\n"
  //   "+CIPRXGET: 2,<mux>,0,0\r\n" + "\r\nOK\r\n"   (no data)
  //   "+IP ERROR: No data\r\n\r\nERROR\r\n"         (no data, some firmwares)
  //   "ERROR\r\n"                                    (mux not open / bad state)
  String line;
  unsigned long start = millis();
  char prefix[24];
  snprintf(prefix, sizeof(prefix), "+CIPRXGET: 2,%u,", kMux);

  while (millis() - start < timeoutMs) {
    while (_stream.available()) {
      char c = (char)_stream.read();
      if (c == '\n') {
        line.trim();
        if (line == "ERROR") return 0;
        if (line.startsWith("+IP ERROR")) {
          waitLine("ERROR", 50, nullptr);  // consume the trailing ERROR if any
          return 0;
        }

        if (line.startsWith(prefix)) {
          String rest     = line.substring(strlen(prefix));  // "actual,remain"
          int    comma    = rest.indexOf(',');
          int    dataLen  = (comma > 0) ? rest.substring(0, comma).toInt() : rest.toInt();
          if (dataLen <= 0) { waitStr("OK", 500); return 0; }
          if ((size_t)dataLen > ask) {
            Serial.printf("[SIM_WS] CIPRXGET overrun %d > %u\n", dataLen, (unsigned)ask);
            return -1;
          }

          size_t got          = 0;
          unsigned long dStart = millis();
          while (got < (size_t)dataLen && millis() - dStart < 3000) {
            if (_stream.available()) buf[got++] = (uint8_t)_stream.read();
            else delay(1);
          }
          if (got < (size_t)dataLen) {
            Serial.printf("[SIM_WS] Short read %u/%d — stream lost\n", (unsigned)got, dataLen);
            return -1;
          }
          waitStr("OK", 1000);
          _lastRxMs = millis();
          return (int)got;
        }
        scanUrc(line);
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
    if (n < 0) return false;
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
  // Incoming data that arrived during the send is flagged by scanUrc()
  // (_dataPending) and fetched on the next poll().
  return ok;
}

// Returns the opcode of one complete frame, -1 if more bytes are needed, or
// -2 if the buffer does not start with a valid frame (stream desynced).
int SimWsClient::parseWsFrame(String& payloadOut) {
  if (_rxLen < 2) return -1;

  uint8_t b0     = _rxBuf[0];
  uint8_t opcode = b0 & 0x0F;
  uint8_t b1     = _rxBuf[1];
  bool    masked = (b1 & 0x80) != 0;
  size_t  plen   = b1 & 0x7F;
  size_t  hlen   = 2;

  bool validOp = opcode == 0x00 || opcode == 0x01 || opcode == 0x02 ||
                 opcode == 0x08 || opcode == 0x09 || opcode == 0x0A;
  if ((b0 & 0x70) != 0 || !validOp) return -2;  // RSV bits set / unknown opcode

  if (plen == 126) {
    if (_rxLen < 4) return -1;
    plen = ((uint16_t)_rxBuf[2] << 8) | _rxBuf[3];
    hlen = 4;
  } else if (plen == 127) {
    return -2;  // >64 KB frames are never sent by our server
  }
  if (masked) hlen += 4;

  size_t total = hlen + plen;
  if (total > kRxBufSize) return -2;
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
    frame.reserve(plen);
    for (size_t i = 0; i < plen; i++) {
      frame += (char)(maskPtr ? (data[i] ^ maskPtr[i & 3]) : data[i]);
    }
    Serial.printf("[SIM_WS] text: %.80s\n", frame.c_str());
    payloadOut = frame;
  } else if (opcode == 0x09) {           // ping → pong
    sendWsFrame(0x0A, data, plen);
  }

  memmove(_rxBuf, _rxBuf + total, _rxLen - total);
  _rxLen -= total;
  return (int)opcode;
}

// Parse buffered frames until one text message is extracted.
// Returns 1 = text in textOut, 0 = need more bytes, -1 = connection unusable.
int SimWsClient::nextTextFrame(String& textOut) {
  while (_rxLen > 0) {
    int opcode = parseWsFrame(textOut);
    if (opcode == -1) return 0;
    if (opcode == -2) {
      Serial.printf("[SIM_WS] Bad frame header 0x%02X — stream desynced\n", _rxBuf[0]);
      return -1;
    }
    if (opcode == 0x08) {
      Serial.println("[SIM_WS] WS close frame received");
      return -1;
    }
    if (opcode == 0x01 || opcode == 0x00) return 1;
  }
  return 0;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool SimWsClient::connect(const char* host, uint16_t port, const char* path) {
  disconnect();
  _rxLen       = 0;
  _dataPending = false;
  _peerClosed  = false;
  _urcLine     = "";

  sslStart();  // configure SSL / manual-receive mode; errors non-fatal

  if (!sslOpen(host, port)) {
    Serial.println("[SIM_WS] SSL open failed");
    disconnect();
    return false;
  }
  Serial.printf("[SIM_WS] SSL open OK → %s:%u\n", host, port);

  if (!wsHandshake(host, path) || _peerClosed) {
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
  _lastRxMs    = millis();
  Serial.println("[SIM_WS] WebSocket connected via SIM");
  return true;
}

void SimWsClient::disconnect() {
  // Always send AT+CIPCLOSE regardless of internal state.
  // If the channel is already closed the modem returns ERROR — harmless.
  // This guarantees a clean slate before any reconnect attempt.
  // Wait for the "+CIPCLOSE: <mux>,<err>" result (it follows OK) so it can't
  // leak into the next command's response.
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%u\r\n", kMux);
  _stream.print(cmd);
  char closeRsp[16];
  snprintf(closeRsp, sizeof(closeRsp), "+CIPCLOSE: %u,", kMux);
  waitLine(closeRsp, 2000, nullptr);
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

  // --- Deliver a message already buffered from a previous fetch ---
  int r = nextTextFrame(textOut);
  if (r != 0) {
    if (r < 0) _connected = false;
    return r > 0;
  }

  // --- Pick up URCs that arrived since the last command ---
  drainUrcs();
  if (_peerClosed) {
    Serial.println("[SIM_WS] TCP closed by server");
    _connected = false;
    return false;
  }

  // Fetch on data URC, or periodically in case the URC was missed
  unsigned long now = millis();
  bool fetchNow = _dataPending || (now - _lastProbeMs >= kProbeIntervalMs);

  // --- Fetch data ---
  // Loop to drain all modem-buffered chunks in one poll() call.
  // Each cipRecv gets up to 1460 bytes; large frames (e.g. mission JSON ~11 KB)
  // need multiple consecutive reads.  Returns immediately once the modem
  // reports no data.
  if (fetchNow) {
    _dataPending = false;
    _lastProbeMs = now;
    for (;;) {
      size_t space = kRxBufSize - _rxLen;
      if (space == 0) break;
      int n = cipRecv(_rxBuf + _rxLen, space, 500);
      if (n < 0) { _connected = false; return false; }
      if (n == 0) break;
      _rxLen += (size_t)n;
    }
  }

  // --- Parse WebSocket frames (one text message per call) ---
  r = nextTextFrame(textOut);
  if (r < 0) _connected = false;
  return r > 0;
}
