# Robot Control: Priority, Serial Protocol and Safety Holds

As of 2026-10-22. Online version: https://claude.ai/artifact/KgouQZDqdQQ2zYA4wZoRnL
Related: [gps_rtk.md](gps_rtk.md)

## Summary

Four sources can drive the robot, in a fixed priority: e-stop, the PS2 joystick, a
companion computer over USB serial, then the mission executor. The companion computer
(e.g. a lidar obstacle-avoidance program) can take over a running mission by sending
`CMD_VEL` and hands control back simply by stopping. The robot also stops a mission on
its own when GPS accuracy is poor, when the control server has been unreachable for
30 s, or when GPS or IMU becomes invalid.

All of this was tested on leafsheep_001 on 2026-10-22, except the field tests listed at
the end. The PS2 joystick and USB serial both work without the control server.

## Control priority

Each control cycle the robot takes its drive command from the highest source that is
currently active.

```
 ┌──────────────────────────────┐
 │ 1. E-stop                    │  Always wins; drive and steering motors stop
 │    PS2 Circle or web UI      │  until the e-stop is released
 └──────────────┬───────────────┘
                │ if released
 ┌──────────────▼───────────────┐
 │ 2. PS2 joystick              │  Overrides everything below while R1 is held
 │    hold R1, drive w/ sticks  │  No server needed; works during holds
 └──────────────┬───────────────┘
                │ if R1 not held
 ┌──────────────▼───────────────┐
 │ 3. Companion computer        │  In control while commands keep arriving
 │    USB serial CMD_VEL / STOP │  Mission stays running and yields; NAV override:true
 └──────────────┬───────────────┘
                │ if no command within its timeout
 ┌──────────────▼───────────────┐
 │ 4. Mission executor          │  Drives toward its current target
 │    GPS waypoints, coverage   │  Safety holds stop it: GPS error, link loss, sensors
 └──────────────────────────────┘
```

Web UI move commands share the mission's drive channel, so during a mission they have
no lasting effect: the mission rewrites it every 10 ms. The joystick runs in
ControlTask on core 1 and serial commands in SerialCmdTask, so neither depends on the
network.

## Serial protocol

The companion computer talks to the ESP32 over its USB serial port at 115200 baud, one
command per line ending in `\n`. Replies and debug logs come back on the same port;
parse only lines that start with `NAV `.

| Command | During a mission | Otherwise | Reply |
| --- | --- | --- | --- |
| `CMD_VEL <linear_m/s> <angular_rad/s> [timeout_ms]` | Overrides the mission until the timeout; a timeout of 0 is capped at 250 ms | Drives the robot until the timeout; 0 = until the next command | `[USB_SERIAL] CMD_VEL accepted ...` |
| `STOP` | Stops the robot and holds the mission off for 250 ms | Stops the robot | `[USB_SERIAL] STOP accepted` |
| `MISSION_PAUSE` | Pauses a running mission (same as the web Pause) | Rejected | `MISSION_PAUSE accepted` / `rejected` |
| `MISSION_RESUME` | Resumes a paused mission | Rejected | `MISSION_RESUME accepted` / `rejected` |
| `NAV ON` / `NAV OFF` | Starts or stops the NAV line | Same | `[USB_SERIAL] NAV output ON` / `OFF` |
| `MODE NORMAL` / `MODE ZERO_TURN` | Changes steering mode | Same | `MODE set to ...` |
| `STATUS?` | One status line | Same | `[USB_SERIAL] status mode=... mission=...` |

`CMD_VEL` timeouts are clamped to 50–5000 ms; the default is 250 ms. Commands are
rejected while the e-stop is active. In zero-turn mode the angular value sets rotation
intensity rather than an Ackermann curve.

With `NAV ON` the robot sends one line every 100 ms:

```
NAV {"t":123456,"lat":13.6576527,"lon":100.4674490,"fix":4,"herr":0.014,"heading":87.3,"mission":"running","holding":false,"override":false,"wp":1,"target_lat":13.6577012,"target_lon":100.4675011,"dist_m":4.21,"bearing":92.5,"speed_cmd":0.30}
```

| Field | Meaning |
| --- | --- |
| `t` | Robot uptime (ms) |
| `lat`, `lon` | Current position (degrees) |
| `fix` | GGA fix type: 4 = RTK fixed, 5 = RTK float, 0 = no valid fix |
| `herr` | GST horizontal error estimate (m); -1 = unknown |
| `heading` | IMU heading (degrees) |
| `mission` | Mission state: idle, queued, running, paused, completed, cancelled, failed, e_stop |
| `holding` | True while a safety hold stops the mission |
| `override` | True while serial `CMD_VEL`/`STOP` has control |
| `wp` | Current target waypoint index; -1 = none, and the target fields are left out |
| `target_lat`, `target_lon` | Target waypoint position |
| `dist_m`, `bearing` | Distance (m) and bearing (degrees) from the robot to the target |
| `speed_cmd` | Speed the mission is commanding (m/s) |

## How a high-level program should use it

The pattern is: read NAV, and send `CMD_VEL` only while you need control.

1. **Connect** to the robot's USB serial port at 115200 baud and send `NAV ON`.
2. **Read NAV lines** at 10 Hz. While `mission` is `running` and the path is clear,
   send nothing; the mission drives.
3. **Take control** when an obstacle needs avoiding: send `CMD_VEL` every 100 ms with a
   250 ms timeout. NAV shows `override:true` while you have control. Use `bearing` and
   `dist_m` to steer around the obstacle toward the target.
4. **Hand back** by simply stopping. Within the timeout the mission resumes toward its
   current waypoint from wherever the robot is.
5. **Stop in place** with `STOP` or `CMD_VEL 0 0` repeated every 100 ms. One `STOP` only
   holds for 250 ms.
6. **Pause the whole mission** with `MISSION_PAUSE` when the robot should wait longer,
   and `MISSION_RESUME` to continue.

If the program crashes, its commands stop and the robot is back under mission control
within 250 ms. The PS2 joystick and e-stop always beat the program.
`usb_cmd_vel_gui.py` in this repo does all of the above by hand: continuous `CMD_VEL`,
the mission buttons and a NAV panel.

## Mission safety holds

A hold stops the wheels but keeps the mission and its target; the mission state stays
`running`. The serial override and the PS2 joystick can still drive during any hold.

| Hold | Trigger | Recovery | Gives up | Log |
| --- | --- | --- | --- | --- |
| GPS accuracy | GST horizontal error above 10 cm (`MISSION_GPS_MAX_HERR_M`) | Resumes when the error drops below 8 cm | Mission fails after 60 s on hold | `[MISSION] Holding: GPS error ...`, `Resumed`, `Failed` |
| Server link loss | No control server for 30 s (`MISSION_LINK_LOSS_HOLD_MS`) | Resumes as soon as the link is back | Never; waits indefinitely | `[MISSION] Holding: no server link ...`, `Resumed: server link back` |
| Invalid GPS or IMU | Fix lost or GPS/IMU data stale | Continues when valid again within 2 s | Mission fails after 2 s | `[SAFETY] Mission aborted ...` (with `DEBUG_LOG_SAFETY`) |

The robot stops the moment a hold starts; it no longer keeps its last speed during the
2 s invalid-sensor grace period. Spiral tasks skip the GPS checks but still hold on
link loss. A receiver that never sends GST is not checked for accuracy.

All three exist because, without them, a robot that can't see where it is, or can't be
stopped remotely, would keep driving. Local control (joystick, serial) is still allowed
during a hold because the person or program on site has its own view of the robot.

## Web UI indicators

`mission_planner_v2.html` shows three things the operator needs to judge whether the
robot can be trusted right now. The server forwards telemetry unchanged, so all of it
comes from the robot.

| Indicator | Where | Shows | Colours | Telemetry field |
| --- | --- | --- | --- | --- |
| GPS Error | GPS panel, under Fix Quality | GST error in cm | Red above 10 cm | `gps.h_err_m` |
| Mission holding | Mission status | Amber "MISSION HOLDING — WAITING FOR GPS ACCURACY" banner, current error, resume threshold | Amber | `mission.holding` |
| Link health | Robot list, under each robot | "Updated N s ago • RTT N ms • SIM" | Age: green < 3 s, amber 3–6 s, red after. RTT: grey < 1 s, amber 1–2 s, red after | `link.rtt_ms`, `link.source` |

"Updated N s ago" uses the time the browser received the message, so it keeps counting
when telemetry stops. RTT is measured from the robot's WebSocket ping every 15 s, so it
can trail a sudden slowdown by up to 15 s. During a link-loss hold the page shows a
red, growing "Updated" age rather than the banner, since no telemetry can reach it.

## Config reference

Settings live in `include/Config_2.h` (leafsheep) and `include/Config.h` (ladybug),
with the same values.

| Define | Value | Meaning |
| --- | --- | --- |
| `MISSION_GPS_MAX_HERR_M` | 0.10 | GPS error above which a mission holds; resumes below 80% of it |
| `MISSION_GPS_HOLD_TIMEOUT_MS` | 60000 | GPS hold longer than this fails the mission |
| `MISSION_LINK_LOSS_HOLD_MS` | 30000 | Time without the server before a mission holds |
| `MISSION_WAYPOINT_REACHED_M` | 1.5 | Waypoint arrival radius (m) |
| `MISSION_COVERAGE_WAYPOINT_REACHED_M` | 0.5 | Coverage-path point arrival radius (m) |

Hard-coded in `src/main.cpp`: `CMD_VEL` timeout default 250 ms, range 50–5000 ms
(`USB_CMD_VEL_*_TIMEOUT_MS`); NAV period 100 ms (`NAV_SERIAL_PERIOD_MS`);
invalid-sensor grace 2000 ms (`MISSION_SENSOR_GRACE_MS`).

## Open items

- [ ] Field test: e-stop from the web UI during a running mission.
- [ ] Field test: move the robot mid-mission and confirm the GPS hold and resume.
- [ ] Field test: stop the control server for 30+ s during a mission and confirm the
      link-loss hold and resume.
- [ ] Field test: 30+ minute run with no watchdog reset.
- [ ] Decide whether the companion computer gets its own UART instead of sharing USB
      with debug logs.
- [ ] Choose waypoint arrival radii for RTK; 1.5 m is loose next to ~2 cm accuracy.
- [ ] Switch debug logs off for normal use, then flash ladybug with the same build.
