#pragma once

// A7670X is AT-command compatible with SIM7600 — use that driver.
// Must be defined before any TinyGSM header is included.
#ifndef TINY_GSM_MODEM_SIM7600
#define TINY_GSM_MODEM_SIM7600
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <TinyGsmClient.h>

#include "Config_2.h"

// Which bearer is currently providing internet
enum class NetSource { NONE, WIFI, SIM };

// NetManager — owns WiFi + SIM fallback logic.
// Call begin() once in setup(), update() every comms loop tick.
// activeClient() returns the live Client* (WiFiClient or TinyGsmClient),
// or nullptr when no network is available yet.
class NetManager {
 public:
  NetManager();
  void begin();
  void update();

  // Returns a pointer to the active TCP client (WiFiClient or TinyGsmClient).
  // Returns nullptr if no network is currently available.
  Client* activeClient();

  bool        isConnected() const { return _source != NetSource::NONE; }
  NetSource   source()      const { return _source; }
  const char* sourceName()  const;

 private:
  // ── WiFi tick ────────────────────────────────────────────────────────────
  void wifiTick();

  // ── SIM tick (state machine) ──────────────────────────────────────────────
  void simTick();
  void simPowerOn();

  enum class SimState {
    IDLE,          // not started yet
    POWERING_ON,   // EN pin asserted, waiting for modem to respond
    BOOTING,       // modem on, waiting for init()
    REGISTERING,   // waiting for network registration
    GPRS_CONNECT,  // connecting PDP/GPRS
    READY,         // GPRS up, data available
  };

  NetSource     _source       = NetSource::NONE;
  unsigned long _wifiLostMs   = 0;   // when WiFi was last seen as connected
  unsigned long _wifiBackMs   = 0;   // when WiFi returned (for recovery delay)

  // Declare in this exact order — each depends on the one above it
  HardwareSerial _simSerial;         // UART2
  TinyGsm        _modem;
  TinyGsmClient  _simClient;
  WiFiClient     _wifiClient;

  SimState      _simState     = SimState::IDLE;
  unsigned long _simStateMs   = 0;
  uint8_t       _simRetries   = 0;
};
