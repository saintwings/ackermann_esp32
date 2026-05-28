#include "NetManager.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Constructor — bind serial → modem → client in the right order
// ─────────────────────────────────────────────────────────────────────────────
NetManager::NetManager()
    : _simSerial(SIM_UART_NUM),
      _modem(_simSerial),
      _simClient(_modem) {}

// ─────────────────────────────────────────────────────────────────────────────
// begin() — call once in setup()
// ─────────────────────────────────────────────────────────────────────────────
void NetManager::begin() {
  // EN pin: idle LOW (modem off) until SIM fallback is needed
  pinMode(SIM_EN_PIN, OUTPUT);
  digitalWrite(SIM_EN_PIN, LOW);

  // Start WiFi using the credentials and static IP from Config
  WiFi.mode(WIFI_STA);
  IPAddress ip, gw, sn;
  ip.fromString(WIFI_STATIC_IP);
  gw.fromString(WIFI_STATIC_GATEWAY);
  sn.fromString(WIFI_STATIC_SUBNET);
  WiFi.config(ip, gw, sn);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[NET] WiFi connecting to SSID: %s\n", WIFI_SSID);
}

// ─────────────────────────────────────────────────────────────────────────────
// update() — call every comms loop tick
// ─────────────────────────────────────────────────────────────────────────────
void NetManager::update() {
  wifiTick();

  // Only spin up SIM after WiFi has been absent long enough
  if (_source != NetSource::WIFI &&
      _wifiLostMs > 0 &&
      millis() - _wifiLostMs >= NET_WIFI_FAIL_TIMEOUT_MS) {
    simTick();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// wifiTick — tracks WiFi state, switches source back when WiFi recovers
// ─────────────────────────────────────────────────────────────────────────────
void NetManager::wifiTick() {
  const bool ok = (WiFi.status() == WL_CONNECTED);
  const unsigned long now = millis();

  if (ok) {
    _wifiLostMs = 0;  // reset "lost" timer

    if (_source == NetSource::WIFI) return;  // already on WiFi, nothing to do

    // WiFi came back while on SIM — wait recovery timeout before switching
    if (_source == NetSource::SIM) {
      if (_wifiBackMs == 0) _wifiBackMs = now;
      if (now - _wifiBackMs < NET_WIFI_RECOVER_TIMEOUT_MS) return;
      Serial.println("[NET] Switching back to WiFi");
    } else {
      Serial.printf("[NET] WiFi connected: %s\n", WiFi.localIP().toString().c_str());
    }
    _source = NetSource::WIFI;
    _wifiBackMs = 0;
    return;
  }

  // WiFi is down
  _wifiBackMs = 0;
  if (_wifiLostMs == 0) {
    _wifiLostMs = now;
    if (_source == NetSource::WIFI) {
      _source = NetSource::NONE;
      Serial.println("[NET] WiFi lost — will try SIM in 15 s");
    }
  }

  // Reconnect attempt every 8 s
  static unsigned long last_reconnect_ms = 0;
  if (now - last_reconnect_ms > 8000) {
    last_reconnect_ms = now;
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// simPowerOn — assert EN pin HIGH; modem boots automatically
// ─────────────────────────────────────────────────────────────────────────────
void NetManager::simPowerOn() {
  Serial.println("[SIM] EN → HIGH (powering on modem)");
  digitalWrite(SIM_EN_PIN, HIGH);
}

// ─────────────────────────────────────────────────────────────────────────────
// simTick — state machine that brings up GPRS connectivity
// ─────────────────────────────────────────────────────────────────────────────
void NetManager::simTick() {
  const unsigned long now = millis();

  switch (_simState) {

    // ── IDLE: first time entering SIM mode ───────────────────────────────────
    case SimState::IDLE:
      _simSerial.begin(SIM_UART_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
      simPowerOn();
      _simState   = SimState::POWERING_ON;
      _simStateMs = now;
      _simRetries = 0;
      break;

    // ── POWERING_ON: wait for modem to boot and respond ──────────────────────
    case SimState::POWERING_ON:
      if (now - _simStateMs < 6000) return;  // give modem 6 s to boot
      _simState   = SimState::BOOTING;
      _simStateMs = now;
      break;

    // ── BOOTING: send AT, check modem is alive ───────────────────────────────
    case SimState::BOOTING:
      if (now - _simStateMs < 2000) return;
      if (_modem.init()) {
        // Disable CTS/RTS hardware flow control (board has CTS pin, we leave it open)
        _modem.sendAT("+IFC=0,0");
        _modem.waitResponse(1000);
        Serial.printf("[SIM] Modem ready: %s\n", _modem.getModemInfo().c_str());
        _simState   = SimState::REGISTERING;
        _simStateMs = now;
        _simRetries = 0;
      } else {
        ++_simRetries;
        Serial.printf("[SIM] Init failed (attempt %u), retrying...\n", _simRetries);
        _simStateMs = now;  // retry in 2 s
        if (_simRetries > 10) {
          // Hard reset: cycle EN pin
          Serial.println("[SIM] Too many init failures — cycling power");
          digitalWrite(SIM_EN_PIN, LOW);
          delay(500);
          simPowerOn();
          _simRetries = 0;
          _simStateMs = now;
          _simState   = SimState::POWERING_ON;
        }
      }
      break;

    // ── REGISTERING: wait for cellular network registration ──────────────────
    case SimState::REGISTERING:
      if (now - _simStateMs < 2000) return;
      _simStateMs = now;
      if (_modem.waitForNetwork(5000)) {
        Serial.printf("[SIM] Registered on: %s\n", _modem.getOperator().c_str());
        _simState   = SimState::GPRS_CONNECT;
        _simRetries = 0;
      } else {
        ++_simRetries;
        Serial.printf("[SIM] Not registered yet (%u)...\n", _simRetries);
        if (_simRetries > 30) {
          Serial.println("[SIM] Registration timeout — restarting modem");
          digitalWrite(SIM_EN_PIN, LOW);
          delay(500);
          simPowerOn();
          _simRetries = 0;
          _simState   = SimState::POWERING_ON;
          _simStateMs = now;
        }
      }
      break;

    // ── GPRS_CONNECT: bring up data connection ────────────────────────────────
    case SimState::GPRS_CONNECT:
      Serial.printf("[SIM] Connecting GPRS (APN=%s)...\n", SIM_APN);
      if (_modem.gprsConnect(SIM_APN, SIM_APN_USER, SIM_APN_PASS)) {
        Serial.printf("[SIM] GPRS up — IP: %s\n", _modem.localIP().toString().c_str());
        _source     = NetSource::SIM;
        _simState   = SimState::READY;
        _simStateMs = now;
        _simRetries = 0;
      } else {
        ++_simRetries;
        Serial.printf("[SIM] GPRS connect failed (%u), retrying...\n", _simRetries);
        _simState   = SimState::REGISTERING;  // re-check registration first
        _simStateMs = now;
      }
      break;

    // ── READY: periodic keep-alive check ─────────────────────────────────────
    case SimState::READY:
      if (now - _simStateMs < 30000) return;  // check every 30 s
      _simStateMs = now;
      if (!_modem.isGprsConnected()) {
        Serial.println("[SIM] GPRS dropped — reconnecting");
        _source   = NetSource::NONE;
        _simState = SimState::GPRS_CONNECT;
      }
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// activeClient — returns the live transport, or nullptr
// ─────────────────────────────────────────────────────────────────────────────
Client* NetManager::activeClient() {
  if (_source == NetSource::WIFI) return &_wifiClient;
  if (_source == NetSource::SIM)  return &_simClient;
  return nullptr;
}

const char* NetManager::sourceName() const {
  switch (_source) {
    case NetSource::WIFI: return "WiFi";
    case NetSource::SIM:  return "SIM";
    default:              return "None";
  }
}
