#pragma once

// Include before <TinyGsmClient.h>.
//
// TinyGSM's wait loops (waitResponse, stop, ...) call TINY_GSM_YIELD(), which defaults
// to delay(0). On ESP32 delay(0) only yields to tasks of equal or higher priority, so
// a long wait (stop() waits up to 15 s for +CIPCLOSE) starved IDLE0 on core 0 and the
// task watchdog rebooted the robot (2026-10-22 16:08, NTRIP reconnect).
//
// Sleep one tick at most every 50 ms so the idle task always runs, without adding a
// 1 ms delay per received character (TINY_GSM_YIELD_MS=1 would, since the macro is
// also called inside the per-byte read loop).
#include <Arduino.h>

#ifndef TINY_GSM_YIELD
#define TINY_GSM_YIELD()                                \
  {                                                     \
    static uint32_t tiny_gsm_last_sleep_ms = 0;         \
    const uint32_t tiny_gsm_now_ms = millis();          \
    if (tiny_gsm_now_ms - tiny_gsm_last_sleep_ms >= 50) { \
      tiny_gsm_last_sleep_ms = tiny_gsm_now_ms;         \
      delay(1);                                         \
    } else {                                            \
      delay(0);                                         \
    }                                                   \
  }
#endif
