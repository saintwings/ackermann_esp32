# Solar rows task

A `solar_rows` mission task drives a route through a block of solar-panel rows. The ESP32 doesn't drive it itself. It hands the route to the nav PC (`rainav-fast-lio`), which drives it with LiDAR and RTK GPS. The ESP32 supervises the run and moves on to the next mission task when nav reports done.

The task is **route only**: a free route through marked waypoints, in driving order, any shape (headland, aisle entrance, aisle end, next aisle, …). Nav centres with LiDAR inside aisles, follows the GPS line elsewhere, and pivots at corners over 20° after checking there is room to turn. It waits for obstacles in aisles and detours around them with A* on open ground. If the robot is more than 0.5 m from the first point, nav drives there first, so no approach Go-to tasks are needed. Nav merges points closer than 0.3 m. See nav's `docs/solar_lanes.md` ("Solar route").

The older lane-based forms are no longer supported: lane 0 + count/pitch/side, and a `lanes` list of marked row pairs. The planner flags such tasks in saved missions and won't upload them. The server rejects them, and the firmware fails the mission when it reaches one. Nav refuses any `SOLAR_TASK` without `route`. To fix an old mission, delete the old task and add a new Solar Rows task through the same waypoints.

## Requirements

- The nav PC runs `fastlio_nav` with `solar.enabled: true` and `gps_fusion.enabled: true` (e.g. `config/navigation_solar.yaml`). It must hold this USB port. Stop nav before flashing, because it holds the port.
- The heading-ready gate applies at mission start as usual. In GPS heading mode, drive 3–5 m straight on RTK fixed first.

## Mission JSON

```json
{"type": "solar_rows",
 "route": [{"lat": 13.7563067, "lon": 100.5018185},
           {"lat": 13.7563067, "lon": 100.5020035},
           {"lat": 13.7563202, "lon": 100.5020035}],
 "route_waypoints": [3, 5],
 "speed": 0.3}
```

| Field | Meaning |
|---|---|
| `route` | The route: `[{lat,lon}, …]` (or `[lat,lon]` arrays), 2–`SOLAR_MAX_ROUTE_POINTS` (60) points in driving order, at least 1 m long in total |
| `speed` | Row speed, 0.05–2.0 m/s. It is clamped to the firmware limits and to nav's `robot.max_speed`. |
| `route_waypoints` | Planner only: the `[first, last]` mission waypoints the route was copied from. Used when the task is edited. |

In the planner, add a **Solar Rows** task and pick the first and last waypoint. The planner checks 2–60 points and at least 1 m in total. It copies the points into the task, so moving the waypoints later doesn't change the task; edit and re-save it instead. It also offers to remove the Go-to tasks for those waypoints. The map draws the route.

`control_server.py` validates the task and turns each route point into one mission waypoint, so batched uploads work. The firmware task then carries `route_wp_start` / `route_count`. The firmware also accepts an embedded `route` list directly (not with batched uploads).

If the task is invalid, the firmware keeps it and fails the mission when it reaches it ("solar_rows: invalid task"). This covers no route, fewer than 2 points, or more than 60. It never silently skips a block.

## USB-serial protocol

One line per message. Every reply from nav is JSON carrying the `run` number of the task. The ESP32 ignores lines with any other run.

### ESP32 → nav

| Line | When |
|---|---|
| `SOLAR_TASK {"run":R,"task_idx":i,"speed":v,"route":[[lat,lon],…]}` | At task start. Re-sent every `SOLAR_TASK_RESEND_MS` until `SOLAR_ACK` or `SOLAR_STATUS` arrives. `run` goes up on every task start. Points have 8 decimals, about 27 characters each, so 60 points make a line of about 1.7 KB. |
| `SOLAR_PAUSE pause` / `link_lost` / `estop` | Mission paused, server link lost for `MISSION_LINK_LOSS_HOLD_MS`, or an e-stop that leaves the mission running (PS2). Sent once per pause; e-stop and link loss are one combined pause, so clearing one doesn't resume while the other holds. |
| `SOLAR_RESUME` | The pause ended. Sent once. |
| `SOLAR_STOP <reason>` | The run ended on the ESP32 side. Reasons: `cancel`, `estop`, `mission_failed`, `mission_ended`, `new_mission`, `task_changed`, `no_ack`, `nav_silent`. Repeated every second for 3 s in case nav missed it. Nav also ends a run on its own when the `NAV` line's `mission` stops being `running` / `paused`. |

### nav → ESP32

| Line | Effect |
|---|---|
| `SOLAR_ACK {"run","task_idx"}` | Nav accepted the task. |
| `SOLAR_STATUS {"run","task_idx","phase","visit","visits","s","len","d","src","conf","rtk","obst"}` | Sent at 1 Hz. `visit`/`visits` count route legs. `phase` is one of `FOLLOW`, `CORNER`, `CREEP`, `DETOUR`, `BACKOFF`, `WATCH`, `FAULT` or `DONE`. Stored (up to 383 chars) and sent to the web UI in telemetry. |
| `SOLAR_DONE {"run","task_idx"}` | The route is done. The mission goes to the next task. |
| `SOLAR_FAIL {"run","task_idx","why"}` | The mission fails with "solar: \<why\>". |
| `CMD_VEL …`, `MODE ZERO_TURN|NORMAL` | Driving, as usual. Pivots use the existing zero-turn mode, where `CMD_VEL 0 <rate>` turns at `<rate>` rad/s (+ = counter-clockwise, clamped to `ZERO_TURN_MAX_RATE_RADS`; the linear value is ignored). During a run, `MODE` switches directly (no 200 ms `applyModeInit` delay under the nav lock) and `CMD_VEL` is held at zero for `SOLAR_MODE_SWITCH_SETTLE_MS` while the wheels steer. |

## Firmware behaviour during the task

- **Not applied:** the GPS accuracy gate and the GPS/IMU-invalid abort. Nav handles RTK loss itself (LiDAR inside aisles; see nav's docs). Auto zero-turn and the mission speed slew are also off, because nav owns the motion.
- **Still applied:**
  - E-stop and limit switches. `CMD_VEL` is rejected. Limit-switch and server e-stops end the mission, so nav gets `SOLAR_STOP estop`. The PS2 e-stop leaves the mission running, so nav gets `SOLAR_PAUSE estop`, then `SOLAR_RESUME` after release.
  - Server link-loss hold, as a pause.
  - The heading gate at mission start.
- **No motion until accepted or while paused:** until nav has accepted the task, and while paused, nav's `CMD_VEL` is applied as zero.
- **Nav silence:** no `CMD_VEL` and no `SOLAR_*` line for `SOLAR_NAV_QUIET_HOLD_MS` (2 s) makes the robot hold at zero. After `SOLAR_NAV_QUIET_FAIL_MS` (30 s) the mission fails.
- **No answer to `SOLAR_TASK`:** the mission fails after `SOLAR_ACK_TIMEOUT_MS` (10 s).
- **After a run ends:** for 3 s, nonzero `CMD_VEL` and `MODE ZERO_TURN` from serial are dropped, so a late pivot line can't leave the next task in zero-turn.
- **Every exit forces NORMAL mode:** done, fail, cancel, e-stop, timeouts and a new mission all force `MODE NORMAL` and zero targets, and clear the serial override.

The settings are in section 9 of `include/Config_*.h`.

## Telemetry

Sent only while a run is active: `payload.solar = {"active": true, "run": R, "state": "idle|waiting_nav|running|paused|nav_quiet", "status": {…last SOLAR_STATUS…}}`

The mission planner shows this under the mission status ("Leg: visit / visits"). The route is drawn on the map, with its first leg in yellow.
