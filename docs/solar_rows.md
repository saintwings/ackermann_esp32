# Solar rows task

A `solar_rows` mission task drives a block of solar-panel lanes. The ESP32 doesn't drive it itself: it hands the block to the nav PC (`rainav-fast-lio`), which steers along each lane with LiDAR and uses RTK GPS for lane positions, row ends and turns. The ESP32 supervises the run and moves on to the next mission task when nav reports done.

A typical mission:

1. `goto_waypoint` to a point 5 m before lane 0
2. `goto_waypoint` to a point 2 m before lane 0, so the robot arrives facing along the lane
3. `solar_rows`
4. `goto_waypoint` home

The web planner adds steps 1 and 2 when you tick "Drive to the lane entrance first".

## Requirements

- The nav PC runs `fastlio_nav` with `solar.enabled: true` and `gps_fusion.enabled: true` (e.g. `config/navigation_solar.yaml`). It must hold this USB port. Stop nav before flashing, because it holds the port.
- The heading-ready gate applies at mission start as usual. In GPS heading mode, drive 3–5 m straight on RTK fixed first.
- When the task starts, the robot must be within about 2 m of lane 0's centre line, at most 4 m before A, and facing within 50° of A→B. Otherwise nav refuses and the mission fails with the reason.

## Mission JSON

```json
{"type": "solar_rows",
 "lane0_a": {"lat": 13.7563067, "lon": 100.5018185},
 "lane0_b": {"lat": 13.7563067, "lon": 100.5020035},
 "lane_count": 4, "pitch_m": 1.5, "side": "left",
 "turn_mode": "pivot", "speed": 0.3}
```

| Field | Meaning |
|---|---|
| `lane0_a`, `lane0_b` | Centre line of the first lane: entrance (A) and far end (B), at least 1 m apart |
| `lane_count` | Lanes to drive, 1–200. The robot alternates direction in each lane. |
| `pitch_m` | Row centre-to-centre spacing, 0.5–10 m |
| `side` | `left` / `right`: where lane 1 is, looking from A to B |
| `turn_mode` | `pivot` (zero-turn, about 0.8 m headland) or `ackermann` (U-turn, about 1.6 m headland) |
| `speed` | Row speed (m/s). It is clamped to the firmware limits and to nav's `robot.max_speed`. |
| `lanes` | Optional. Each row's entrance and end, in driving order: `[[{lat,lon},{lat,lon}], …]` (`[lat,lon]` arrays also work). 1–`SOLAR_MAX_LANES` (60) rows. When present, nav uses it instead of `lane0_a/b`, `lane_count` and `side`, so rows can differ in position, length and spacing. Keep sending `lane0_a/b` etc. too (set from the first rows) for older nav versions. |

### Rows from marked waypoints

In the planner, choose **Rows from: Marked waypoints** and pick the first and last waypoint. The waypoints must come in pairs, in driving order: WP0 = row 1 entrance, WP1 = row 1 end, WP2 = row 2 entrance (beside row 1's end), and so on. The planner checks what it can before adding the task: an even count, each row at least 1 m, and each row running back the other way within 30°. Nav makes the rest of its checks at task start (each entrance within 3 m along and at least 0.5 m across from the previous row's end) and fails with `SOLAR_FAIL` and a reason. The planner fills `lane0_a/b`, `lane_count`, `pitch_m` and `side` from the first rows, copies the rows into the task (moving the waypoints later doesn't change it; edit and re-save the task), and offers to remove Go-to tasks for those waypoints. See also nav's `docs/solar_lanes.md`, section "Rows from marked waypoints".

`control_server.py` validates the task and turns A and B into two mission waypoints, so batched uploads work. The firmware task then carries `waypoint_a` / `waypoint_b` indices. A `lanes` list becomes two more waypoints per row, and the task carries `lanes_wp_start` / `lanes_count`. The firmware also accepts the embedded `lane0_a` / `lane0_b` objects and `lanes` list directly (not with batched uploads).

If the settings are invalid, the firmware keeps the task and fails the mission when it reaches it ("solar_rows: invalid task"). It never silently skips a block.

## USB-serial protocol

One line per message. Every reply from nav is JSON carrying the `run` number of the task. The ESP32 ignores lines with any other run.

### ESP32 → nav

| Line | When |
|---|---|
| `SOLAR_TASK {"run":R,"task_idx":i,"lane0_a":[lat,lon],"lane0_b":[lat,lon],"lane_count":N,"pitch_m":x,"side":"left","turn_mode":"pivot","speed":v[,"lanes":[[[lat,lon],[lat,lon]],…]]}` | At task start. Re-sent every `SOLAR_TASK_RESEND_MS` until `SOLAR_ACK` or `SOLAR_STATUS` arrives. `run` goes up on every task start. `lanes` is included only when the task has one (8 decimals, about 57 characters per row, so 60 rows make a line of about 3.6 KB). |
| `SOLAR_PAUSE pause` / `link_lost` / `estop` | Mission paused, server link lost for `MISSION_LINK_LOSS_HOLD_MS`, or an e-stop that leaves the mission running (PS2). Sent once per pause; e-stop and link loss are one combined pause, so clearing one doesn't resume while the other holds. |
| `SOLAR_RESUME` | The pause ended. Sent once. |
| `SOLAR_STOP <reason>` | The run ended on the ESP32 side. Reasons: `cancel`, `estop`, `mission_failed`, `mission_ended`, `new_mission`, `task_changed`, `no_ack`, `nav_silent`. Repeated every second for 3 s in case nav missed it. Nav also ends a run on its own when the `NAV` line's `mission` stops being `running` / `paused`. |

### nav → ESP32

| Line | Effect |
|---|---|
| `SOLAR_ACK {"run","task_idx"}` | Nav accepted the task. |
| `SOLAR_STATUS {"run","task_idx","phase","lane","visit","visits","s","len","d","src","conf","rtk","obst"}` | Sent at 1 Hz. Stored (up to 383 chars) and sent to the web UI in telemetry. |
| `SOLAR_DONE {"run","task_idx"}` | All lanes done. The mission goes to the next task. |
| `SOLAR_FAIL {"run","task_idx","why"}` | The mission fails with "solar: \<why\>". |
| `CMD_VEL …`, `MODE ZERO_TURN|NORMAL` | Driving, as usual. Pivots use the existing zero-turn mode, where `CMD_VEL 0 <rate>` turns at `<rate>` rad/s (+ = counter-clockwise, clamped to `ZERO_TURN_MAX_RATE_RADS`; the linear value is ignored). During a run, `MODE` switches directly (no 200 ms `applyModeInit` delay under the nav lock) and `CMD_VEL` is held at zero for `SOLAR_MODE_SWITCH_SETTLE_MS` while the wheels steer. |

## Firmware behaviour during the task

- **Not applied:** the GPS accuracy gate and the GPS/IMU-invalid abort. Nav handles RTK loss itself: it keeps following the LiDAR inside a row, waits at the row end for RTK fixed, and stops after 1.5 m blind. Auto zero-turn and the mission speed slew are also off, because nav owns the motion.
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

The mission planner shows this under the mission status. The lanes from the task are drawn on the map, with lane 0 in yellow.
