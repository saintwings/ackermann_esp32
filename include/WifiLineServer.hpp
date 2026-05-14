#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include "Config.h"

class WifiLineServer {
 public:
  bool begin() {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(200);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi STA connected, IP: ");
      Serial.println(WiFi.localIP());
      server_.begin();
      return true;
    }

    // Fallback to AP
    WiFi.mode(WIFI_MODE_AP);
    bool ap = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, 1, false, 2);
    delay(200);
    if (ap) {
      Serial.print("WiFi AP started, IP: ");
      Serial.println(WiFi.softAPIP());
      server_.begin();
      return true;
    }
    Serial.println("WiFi init failed");
    return false;
  }

  // Non-blocking line reader; returns true if a full line was read into out.
  bool readLine(String& out) {
    if (!client_ || !client_.connected()) {
      WiFiClient newClient = server_.available();
      if (newClient) {
        client_ = newClient;
        client_.setNoDelay(true);
        buf_ = "";
        Serial.println("WiFi client connected");
      } else {
        return false;
      }
    }

    while (client_.available() > 0) {
      char c = (char)client_.read();
      if (c == '\n' || c == '\r') {
        if (buf_.length() > 0) {
          out = buf_;
          buf_ = "";
          return true;
        }
      } else {
        if (buf_.length() < 256) buf_ += c; // cap length
      }
    }
    return false;
  }

  void println(const String& s) {
    if (client_ && client_.connected()) client_.println(s);
  }

  bool hasClient() { return client_ && client_.connected(); }

 private:
  WiFiServer server_{WIFI_SERVER_PORT};
  WiFiClient client_{};
  String buf_;
};
