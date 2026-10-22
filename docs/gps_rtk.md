# GPS / RTK on the Ackermann ESP32 Robot

As of 2026-10-22. Online version: https://claude.ai/artifact/9FUbpPtKwmQfRg7Uc2BeM7
Related: [robot_control.md](robot_control.md)

## Summary

GPS on the robot now holds RTK fixed with a 1.4 cm error estimate and about 9 position
updates per second, with no dropouts. On 2026-10-21 the GPS had no current position
about half the time. The cause was the ESP32 not reading the GPS UART fast enough while
the comms task waited on the SIM modem; it was not the receiver or NTRIP.

Since 2026-10-22, GPS reading and the mission executor run in their own task (NavTask),
so modem delays no longer stall them, and a running mission holds in place when the GPS
error estimate exceeds 10 cm.

- **Before:** ~20–28 `no-gga-yet` per minute, RTK fixed rarely held, many corrupted GGA
  lines.
- **After:** 0 `no-gga-yet`, 0 bad GGA lines, RTK fixed throughout, NavTask never waits
  more than 13 ms between passes.
- **Measured on:** robot leafsheep_001 (`Config_2.h`), SIM-only network, NTRIP caster
  110.78.0.54:2116, mount point VRS_RTCM32.

## How GPS data flows

GPS reading runs in NavTask, apart from everything that talks to the network, so a slow
modem no longer delays it.

```
                       ESP32, core 0
                  ┌───────────────────────────────────┐
                  │ NavTask · priority 3 · every 10 ms │
 ┌──────────┐ NMEA│  ┌─────────────────────────────┐   │
 │ ZED-F9P  ├─────┼─►│ Read GPS UART, parse GGA/GST│   │
 │ GPS      │     │  │ IMU, safety, mission        │   │
 │          │     │  └─────────────────────────────┘   │
 │ GGA+GST  │     └────────────────┬──────────────────┘
 │ out,     │       shared state behind nav_mutex
 │ RTCM in  │     ┌────────────────┴──────────────────┐        ┌────────────────┐
 │          │RTCM │ CommsTask · priority 2            │  TCP   │ NTRIP VRS      │
 │          │◄────┼──Forward RTCM (one read) ◄────────┼─mux 0─►│ caster :2116   │
 └──────────┘     │  WebSocket, commands, telemetry ◄─┼─mux 1─►│ Control server │
                  └───────────────────────────────────┘ A7670C │ WebSocket :80  │
                                                        modem  └────────────────┘
```

The ESP32 reads GGA and GST from the receiver and writes RTCM corrections back on the
same UART. NTRIP (mux 0, through TinyGSM) and the server WebSocket (mux 1) share the
modem, and the robot also sends its GGA to the caster once a second over mux 0. Both
tasks run on core 0; NavTask has the higher priority, so it preempts CommsTask while
CommsTask waits on the modem. Until 2026-10-22 all of this ran in one comms task, which
is what starved GPS reading.

## Problems found and their causes

Every problem traced back to one fact: GPS reading, NTRIP, the SIM WebSocket and
mission logic all ran in one comms task, and each SIM modem command blocks that task
for 100–500 ms or more.

| Problem | What we saw | Cause |
| --- | --- | --- |
| GPS position dropouts | `no-gga-yet` on 30–60% of status lines; the fix marked stale after 3 s | Only 128 bytes read from the GPS UART per loop pass. The receiver sends up to ~3.8 KB/s, so while the loop waited on the modem the 2 KB RX buffer overflowed and GGA lines failed their checksum |
| RTK fixed degrading to float | Fix 4 held at first, then mostly 5 | Knock-on effect: the robot only sends its GGA to the VRS caster while its fix is valid, so a stale fix degraded the corrections |
| Regression from the first fix attempt | Comms loop passes of 3–6 s, pongs of 3–6 s, more bad GGA | Forwarding RTCM in a loop of up to 16 reads. Each TinyGSM `available()`/`read()` can be a full AT round trip (64-byte RX buffer), so the loop stalled. Reverted |
| GGA rate below 10 Hz | ~7.6 GGA per second | Default NMEA set (~1.5 KB per epoch, mostly GSV) overfilled the 38400-baud link (~3.8 KB/s), so the receiver dropped output |
| Reboot by task watchdog (2026-10-22 11:24) | `task_wdt: IDLE0 (CPU 0)` with CommsTask running, then reset | TinyGSM's `AT+CIPCLOSE?` socket check parses 10 numbers with `Stream::parseInt()`, which busy-waits without yielding for the serial timeout (1 s default). An incomplete modem reply spun CommsTask for up to 10 s and starved core 0's idle task |
| NavTask stalls of ~70 ms | NavTask longest pass 65–74 ms once NTRIP was connected, 12 ms before | Writing each 256 B RTCM chunk to the GPS UART took ~67 ms at 38400 baud with no TX buffer, holding the UART lock that NavTask's GPS read also needs |
| Garbled debug log lines | Characters missing mid-line | USB serial output drops bytes under many small prints. Logging only; the robot is not affected |

## Fixes applied

Committed as ad4c350, 532e164 and 1a721c2.

1. **Drain the GPS UART every pass.** The read loop takes everything waiting, capped at
   4096 bytes, instead of 128 (`readGps()` in `src/sensor_comms_service.cpp`). This
   removed the dropouts.
2. **Forward RTCM with one read per pass** (`forwardNtrip()`). Kept as originally
   written; the looped version was tried and reverted because of the TinyGSM round
   trips described above.
3. **Configure the receiver's NMEA output at every boot.** `configureGnssOutput()` sends
   one UBX-CFG-VALSET to the ZED-F9P, RAM layer only, on UART1 and UART2: GGA and GST
   on; GSV, GSA, GLL, VTG and RMC off. It waits for UBX-ACK and retries up to 5 times.
   If the receiver never acknowledges, it keeps its stored settings.
4. **Parse GST and report the error estimate.** `parseGstLine()` stores the latitude,
   longitude and altitude 1-sigma errors and `h_err_m`, the combined horizontal error.
   It shows as `herr=` on the 1 Hz debug line, goes to the server as telemetry field
   `gps.h_err_m` (-1 when unknown), and appears as "GPS Error" in the web UI.
5. **NavTask.** GPS reading, IMU, safety guards and the mission and function executors
   moved out of CommsTask into a new task: core 0, priority 3, every 10 ms. A mutex
   (`nav_mutex`) guards GPS, IMU and mission state. Server commands are applied under
   it and acknowledged after it is released; telemetry and the 1 Hz log copy what they
   need under it.
6. **Watchdog fix.** SIM serial timeout set to 100 ms (`src/net_manager.cpp`), capping
   TinyGSM's `parseInt` busy-wait at ~1 s.
7. **GPS UART TX buffer.** 1 KB (`gps_uart.setTxBufferSize`), so RTCM writes return at
   once instead of holding the UART lock for ~67 ms.
8. **Mission accuracy gate.** Above 10 cm GST error a running mission stops and holds,
   resumes below 8 cm, and fails after 60 s. See
   [robot_control.md](robot_control.md#mission-safety-holds).
9. **Configurable arrival radii.** Waypoint (1.5 m) and coverage (0.5 m) arrival radii
   moved from `main.cpp` into the config files, unchanged.

Diagnostics on the 1 Hz debug line: `[COMMS] max_dt` and `[NAV] max_dt` (longest loop
pass of each task), `cycles`, and `[GGA] ok/bad` (lines accepted or rejected by
checksum).

## NMEA messages: kept and disabled

Only GGA and GST are output now, about 140 B per epoch instead of ~1.5 KB, so the link
carries ~1.4 KB/s of its ~3.8 KB/s at 38400 baud. Sizes are typical F9P figures, not
measured on this receiver.

| Message | Contains | Size per epoch | Setting | Why |
| --- | --- | --- | --- | --- |
| GGA | Position, fix type, satellites, HDOP | ~75 B | On | Navigation; also sent to the VRS caster |
| GST | 1-sigma error of latitude, longitude, altitude | ~60 B | On | Accuracy check for missions |
| GSV | Every visible satellite, elevation, azimuth, SNR | ~1 KB | Off | Unused; the main cause of link saturation |
| GSA | Satellites used and DOP, one line per constellation | ~260 B | Off | Unused |
| GLL | Latitude and longitude only | ~50 B | Off | Duplicates GGA |
| RMC | Position, speed over ground, course, date | ~70 B | Off | Unused; IMU and wheels are better for heading at low speed |
| VTG | Speed and course | ~35 B | Off | Unused |

To use RMC or GSV again, set its rate to 1 in `kNmeaRates` in
`src/sensor_comms_service.cpp`.

## Measured results

All figures come from the robot's serial log, with the robot stationary.

| Metric | Original firmware (2026-10-21, 17:20–17:35) | After GPS drain (2026-10-21, 17:42–17:51) | After NMEA config (2026-10-22, 10:45–10:49) | NavTask + UART and watchdog fixes (2026-10-22, 11:32–11:41) |
| --- | --- | --- | --- | --- |
| `no-gga-yet` per minute | 13–28 | 0 | 0 | 0 |
| Bad GGA lines | Not measured | 0 | 0 | 0 |
| GGA per second | Not measured | 7.6 | 8.9–9.3 | 8.6–9.7 |
| Fix type | Mostly 5 (float) or none | 5, then 4 (fixed) after ~4.5 min | 4 (fixed) throughout | 4 (fixed) throughout |
| Horizontal error (GST) | Not available | Not available | 0.014 m | 0.014 m |
| Longest comms loop pass | Not measured | 212–231 ms, one 1.9 s and one 2.5 s | 187–225 ms | 116–164 ms |
| Longest NavTask pass | No NavTask | No NavTask | No NavTask | 12–13 ms (65–74 ms before the TX buffer fix) |
| Pong round trip | 290–340 ms | 158–303 ms | ~140 ms | Not measured |
| NTRIP drops | 0 | 0 | 0 | 0 |

The intermediate build with looped RTCM forwarding is left out: it ran 17:38–17:41 with
3–6 s loop passes and was reverted.

## Config reference

GPS settings live in `include/Config_2.h` (leafsheep); `include/Config.h` (ladybug)
defines the same GPS and mission settings.

| Define | Value | Meaning |
| --- | --- | --- |
| `GPS_UART_BAUD` | 38400 | ESP32 ↔ ZED-F9P UART speed |
| `GPS_UART_RX_PIN` / `GPS_UART_TX_PIN` | 9 / 10 | ESP32 pins for the GPS UART |
| `GPS_UBX_CONFIG_ENABLE` | 1 | Send the NMEA output config to the receiver at boot |
| `GPS_NAV_RATE_MS` | 0 | Receiver measurement period in ms; 0 leaves it unchanged, 100 = 10 Hz |
| `MISSION_GPS_MAX_HERR_M` | 0.10 | GST error above which a running mission holds; resumes below 80% of it |
| `MISSION_GPS_HOLD_TIMEOUT_MS` | 60000 | A hold longer than this fails the mission |
| `MISSION_WAYPOINT_REACHED_M` | 1.5 | Waypoint arrival radius (m) |
| `MISSION_COVERAGE_WAYPOINT_REACHED_M` | 0.5 | Coverage-path point arrival radius (m) |
| `NTRIP_ENABLE` | 1 | Forward RTCM corrections from the caster |
| `NTRIP_HOST` / `NTRIP_PORT` | 110.78.0.54 / 2116 | VRS caster |
| `NTRIP_MOUNTPOINT` | VRS_RTCM32 | Caster mount point |
| `DEBUG_LOG_SENSOR_1HZ` | 1 | 1 Hz status line: NET, GPS, NTRIP, COMMS, NAV, GGA |
| `DEBUG_LOG_SENSOR_COMMS` | 1 | NTRIP connect and retry messages |
| `DEBUG_LOG_MISSION` | 1 | Mission state changes, waypoints reached |

Hard-coded in `src/main.cpp`: `GPS_STALE_TIMEOUT_MS` = 3000 (no valid GGA for this long
marks the fix invalid). GPS UART buffers are 2048 B RX and 1024 B TX, the SIM serial
timeout is 100 ms, and NavTask runs every 10 ms (`NAV_PERIOD_MS`). The three debug logs
were switched on for this investigation; consider switching them off for normal use.

## Remaining risks and next steps

No known blocking issue remains; what is left is field testing.

- [x] Move GPS reading and mission logic into their own task (NavTask).
- [x] Use the IMU for heading: already the case, via `estimateHeadingDeg()` from the
      BNO08x.
- [x] Gate missions on GPS accuracy (`gps.h_err_m`).
- [x] Web UI: show `gps.h_err_m` and a "holding" mission state.
- [x] Make the waypoint arrival radii configurable.
- [ ] Field test: move the robot mid-mission and confirm `Holding`, then `Resumed`.
- [ ] Field test: run 30+ minutes and confirm no watchdog reset.
- [ ] Choose waypoint arrival radii for RTK; 1.5 m is loose next to ~2 cm accuracy.
- [ ] Switch debug logs off for normal use, then flash ladybug with the same build.
- [ ] Low priority: find out why GGA is ~9 Hz, not 10.
