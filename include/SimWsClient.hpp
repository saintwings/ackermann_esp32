#pragma once

#ifndef TINY_GSM_MODEM_SIM7600
#define TINY_GSM_MODEM_SIM7600
#endif
#include <Arduino.h>
#include <TinyGsmClient.h>

// SimWsClient — WebSocket over SSL TCP using AT+CIPOPEN "SSL" on the A7670C.
//
// No separate SSL library is needed on the ESP32 — TLS is handled by the modem.
//
// Usage:
//   SimWsClient ws(net.simStream());
//   ws.connect("robot.saintwings.xyz", 443, "/");
//   ws.sendText(json);
//   String rx; if (ws.poll(rx)) { /* handle rx */ }
//
// Call poll() every comms loop tick. It is non-blocking when no data is waiting.
class SimWsClient {
 public:
  explicit SimWsClient(Stream& simSerial);

  // Establish SSL TCP then WebSocket upgrade. Blocking (~5-30 s first call).
  bool connect(const char* host, uint16_t port, const char* path);

  void disconnect();

  bool connected() const { return _connected; }

  // Send a UTF-8 text payload as a masked WebSocket text frame.
  bool sendText(const String& text);

  // Check for an incoming text frame. Returns true and fills textOut with
  // exactly ONE text message if available; further buffered messages are
  // returned by subsequent calls. Non-blocking when no data is waiting.
  bool poll(String& textOut);

  // Send a WebSocket ping frame (keep-alive).
  void sendPing();

  // millis() of the last byte received from the server (pong, ping, text...).
  // Used by the caller as a dead-link watchdog.
  unsigned long lastRxMs() const { return _lastRxMs; }

  // Round trip of the last answered ping (ms), or -1 if none since connect.
  long lastPongRttMs() const { return _lastPongRttMs; }

 private:
  bool sslStart();  // configure SSL context (CSSLCFG) + CIPRXGET=1
  bool sslOpen(const char* host, uint16_t port);
  bool wsHandshake(const char* host, const char* path);

  bool cipSend(const uint8_t* data, size_t len);
  int  cipRecv(uint8_t* buf, size_t maxLen, uint32_t timeoutMs = 1500);

  bool sendWsFrame(uint8_t opcode, const uint8_t* payload, size_t payloadLen);
  int  parseWsFrame(String& payloadOut);
  int  nextTextFrame(String& textOut);
  bool waitStr(const char* expected, uint32_t timeoutMs);
  bool waitLine(const char* needle, uint32_t timeoutMs, const char* logTag, String* lineOut = nullptr);
  void scanUrc(const String& line);
  void drainUrcs();

  Stream& _stream;
  bool    _connected   = false;
  bool    _sslStarted  = false;
  bool    _dataPending = false;  // "+CIPRXGET: 1,<mux>" URC seen
  bool    _peerClosed  = false;  // "+IPCLOSE: <mux>," URC seen
  String  _urcLine;              // partial URC line carried between poll() calls

  // Buffer must fit the largest single WebSocket frame we expect to receive.
  // A 200-waypoint coverage mission JSON is ~11 KB after WS framing.
  static constexpr size_t kRxBufSize = 16384;
  uint8_t  _rxBuf[kRxBufSize];
  size_t   _rxLen = 0;

  unsigned long _lastProbeMs = 0;  // for periodic poll-when-no-URC
  unsigned long _pingSentMs  = 0;  // millis() when last ping was sent
  unsigned long _lastRxMs    = 0;  // millis() of last received payload bytes
  long          _lastPongRttMs = -1;
};
