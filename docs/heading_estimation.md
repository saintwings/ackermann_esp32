# Heading Estimation on the Ackermann ESP32 Robot

As of 2026-11-03 (commit e19e882). Code: `updateHeadingFusion()`, `estimateHeadingDeg()`,
`headingReadyForMission()` in [src/main.cpp](../src/main.cpp).
Related: [gps_rtk.md](gps_rtk.md), [robot_control.md](robot_control.md)

## Summary

The robot works out its heading from the gyro and the RTK GPS together:

- The **gyro** gives a heading that is smooth, follows every turn correctly and drifts
  only slowly, but it does not know where north is.
- The **RTK GPS** gives the true direction of travel, accurate to about 1–2°, but only
  while the robot is driving.

The firmware keeps a single correction angle, the **offset**, between the gyro heading
and true north. It refines the offset every time the robot drives straight forward on
RTK fixed.

| Heading source | Mean error | Scatter (sd) | Max error | Notes |
|---|---|---|---|---|
| Magnetometer (rotation vector) | 2.5° | 10.2° | 26.4° | Jumps of 20° on the same course; a different bias every boot |
| Fused, before first RTK fixed correction | 5.5° | 1.9° | 11.6° | Still on the startup value from the magnetometer |
| **Fused, after first RTK fixed correction** | **1.1°** | **1.5°** | 5.3° | |

The data comes from field tests on robot leafsheep_001 on 2026-11-02 and 2026-11-03 (54
samples), in [tools/heading.csv](../tools/heading.csv) and
[tools/heading2.csv](../tools/heading2.csv). Because of these results, leafsheep
(`Config_2.h`) uses `HEADING_SOURCE_DEFAULT 1` (GPS-fused heading).

## Two headings from the BNO08x

The BNO08x sends two orientation reports at 100 Hz:

| Report | Sensors used | Firmware function | Strength | Weakness |
|---|---|---|---|---|
| Rotation vector | gyro + accel + **magnetometer** | `magHeadingDeg()` | Knows true north directly | Disturbed by motors, steel and the sensor's own recalibration |
| Game rotation vector | gyro + accel only | `gameYawDeg()` | Smooth; not affected by magnetic fields | 0° is wherever the robot pointed at power-on |

## Equations

All angles are in degrees. Compass headings use 0° = north, positive clockwise.

### Helper functions

Wrap to the range 0–360° (`wrap360`):

$$
\operatorname{wrap}(\theta) = \theta \bmod 360 \quad\in [0, 360)
$$

Signed shortest difference in the range −180…+180° (`normalizeAngleDeg`):

$$
\operatorname{norm}(\theta) = \theta - 360k, \quad k \in \mathbb{Z} \text{ chosen so the result} \in [-180, 180]
$$

For example, $\operatorname{norm}(1 - 359) = +2$, not −358.

### 1. Quaternion to compass heading

The BNO08x reports a quaternion $q = (w, x, y, z)$. The standard yaw $\psi$, measured
from east and counter-clockwise, is:

$$
\psi = \operatorname{atan2}\!\big(2(wz + xy),\; 1 - 2(y^2 + z^2)\big)
$$

A compass heading is $90° - \psi$. The code computes it directly by swapping the atan2
arguments (`quatCompassYawDeg`):

$$
\text{yaw}_{\text{compass}}(q) = \operatorname{atan2}\!\big(1 - 2(y^2 + z^2),\; 2(wz + xy)\big) = 90° - \psi
$$

Applied to both reports:

$$
h_{\text{mag}} = \operatorname{wrap}\big(\text{yaw}_{\text{compass}}(q_{\text{rv}}) + \texttt{HEADING\_OFFSET\_DEG}\big)
$$

$$
g = \text{yaw}_{\text{compass}}(q_{\text{game}})
$$

where $h_{\text{mag}}$ is the magnetometer heading and $g$ is the game (gyro) yaw.

### 2. Heading model

$$
h = \operatorname{wrap}(g + \text{offset})
$$

The gyro yaw $g$ already follows every turn, so the offset is the only value the firmware
has to estimate. It changes only as fast as the gyro drifts.

### 3. Startup value

The first time the IMU is valid after boot, the offset is set from the magnetometer:

$$
\text{offset}_0 = \operatorname{norm}(h_{\text{mag}} - g)
$$

At this point the fused heading equals the magnetometer heading, including its error
(5° to more than 20° in the tests). The heading is not yet ready for a mission.

### 4. When a GPS position can be used

The firmware checks every new GPS position. It is used only if all of these hold:

$$
\text{usable} = (\text{fix} = 4) \;\wedge\; (\text{herr} \le 0.05\,\text{m}) \;\wedge\; (\text{mode} = \text{NORMAL}) \;\wedge\; (t - t_{\text{fwd}} < 1\,\text{s})
$$

- `fix = 4` means RTK fixed.
- `herr` is the GPS horizontal error estimate, taken from the GST message:
  $\text{herr} = \sqrt{\sigma_{\text{lat}}^2 + \sigma_{\text{lon}}^2}$.
- NORMAL mode is required because in zero-turn mode the robot spins in place and has no
  direction of travel.
- $t_{\text{fwd}}$ is the last time the drive target was above +0.05 m/s. The 1 s
  allowance covers PS2 read glitches, which briefly set the target to 0 while the robot
  keeps rolling.

A target below −0.05 m/s (reversing) clears $t_{\text{fwd}}$ and discards the current
segment, because reversing would give a course 180° away from the heading.

If any condition fails, the segment is discarded.

### 5. Straight segments

When a segment starts, the firmware stores the start point A: position $(\varphi_A,
\lambda_A)$ and gyro yaw $g_A$. At each new usable GPS position B it checks two things.

**Turning check.** If the robot has turned too much, the segment restarts at B:

$$
\left|\operatorname{norm}(g_B - g_A)\right| \le 3°
$$

**Distance** from A to B (haversine formula, `distanceMeters`), with
$R = 6\,371\,000$ m, $\Delta\varphi = \varphi_B - \varphi_A$ and
$\Delta\lambda = \lambda_B - \lambda_A$:

$$
a = \sin^2\!\frac{\Delta\varphi}{2} + \cos\varphi_A \cos\varphi_B \sin^2\!\frac{\Delta\lambda}{2}
$$

$$
d = 2R \cdot \operatorname{atan2}\!\big(\sqrt{a},\; \sqrt{1 - a}\big)
$$

A sample is taken once $d \ge 0.5$ m.

### 6. GPS course

The true direction of travel from A to B (great-circle bearing, `bearingDeg`):

$$
Y = \sin\Delta\lambda \cos\varphi_B
$$

$$
X = \cos\varphi_A \sin\varphi_B - \sin\varphi_A \cos\varphi_B \cos\Delta\lambda
$$

$$
c = \operatorname{wrap}\big(\operatorname{atan2}(Y, X)\big)
$$

### 7. Heading error at the midpoint

The fused heading is compared with the GPS course at the middle of the segment:

$$
g_{\text{mid}} = g_A + \tfrac{1}{2}\operatorname{norm}(g_B - g_A)
$$

$$
e = \operatorname{norm}\big(c - (g_{\text{mid}} + \text{offset})\big)
$$

The midpoint is used because on an arc of constant curvature, the direction from start
to end equals the heading halfway along. A gently curving segment therefore does not bias
the result.

### 8. Offset update

The offset moves by a fraction $K$ of the error, where $K$ is `HEADING_GPS_GAIN` = 0.3:

$$
\text{offset} \leftarrow \operatorname{norm}(\text{offset} + K \cdot e)
$$

B then becomes the start point A of the next segment, so samples keep coming every 0.5 m
while the robot drives straight.

### 9. Convergence and noise

If the true offset is constant, the error left after $n$ samples is:

$$
e_n = (1 - K)^n \, e_0 = 0.7^n \, e_0
$$

| Samples $n$ | 1 | 2 | 3 | 4 | 6 | 10 |
|---|---|---|---|---|---|---|
| $e_n / e_0$ | 0.70 | 0.49 | 0.34 | 0.24 | 0.12 | 0.03 |

This works like an exponential average over about $1/K \approx 3$ samples, roughly 1.5 m
of straight driving. It reduces the GPS course noise $\sigma_c$ to:

$$
\sigma_{\text{offset}} = \sigma_c \sqrt{\frac{K}{2 - K}} \approx 0.42\,\sigma_c
$$

The course noise comes from the position error over the segment length $d$:

$$
\sigma_c \approx \arctan\!\left(\frac{\sqrt{2}\,\text{herr}}{d}\right)
$$

With $\text{herr} = 0.014$ m and $d = 0.5$ m, this gives $\sigma_c \approx 2.3°$ and
$\sigma_{\text{offset}} \approx 1.0°$. That is close to the 1.5° scatter measured in the
field.

### 10. Mission readiness

MAG mode:

$$
\text{heading\_ready} = \text{imu\_valid}
$$

GPS mode:

$$
\text{heading\_ready} = \text{imu\_valid} \;\wedge\; \text{game\_valid} \;\wedge\; (\text{corrections} \ge 1)
$$

`corrections` counts the RTK fixed samples applied since boot.

### 11. Test log errors

Each `[HDG_S]` line reports these errors. `fused_err` is taken before the sample's offset
update:

$$
\text{fused\_err} = \operatorname{norm}\big(c - (g_{\text{mid}} + \text{offset})\big)
$$

$$
\text{mag\_err} = \operatorname{norm}(c - h_{\text{mag}})
$$

## In operation

1. **Boot.** The offset is set from the magnetometer, `heading_ready` is false, and the
   web UI shows the robot as a **dashed circle**. Mission start commands are rejected
   with "heading not calibrated: drive straight forward 3-5 m on RTK fixed first".
2. **Calibrate.** Drive straight forward 3–5 m with the PS2 controller on RTK fixed. The
   first sample comes after 0.5 m, and after that `heading_ready` is true and the marker
   becomes an **arrow**.
3. **Run the mission.** Every straight forward stretch of the mission adds more
   corrections, so gyro drift keeps being removed.

At runtime, these serial commands are available:

- `HEADING GPS` / `HEADING MAG` switch the heading source. The offset is learned in both
  modes, so switching back to GPS takes effect immediately.
- `HEADING?` prints the source, heading, magnetometer heading, offset, number of
  corrections and the last errors.

## Settings

All of these are in `Config_2.h` (leafsheep) and `Config.h`.

| Setting | Value | Meaning |
|---|---|---|
| `HEADING_SOURCE_DEFAULT` | 1 (leafsheep), 0 (`Config.h`) | 0 = magnetometer, 1 = GPS-fused |
| `HEADING_GPS_MIN_BASELINE_M` | 0.5 | Straight distance per RTK fixed sample |
| `HEADING_GPS_MAX_YAW_CHANGE_DEG` | 3.0 | More turning than this restarts the segment |
| `HEADING_GPS_GAIN` | 0.3 | $K$, the fraction of each error applied to the offset |
| `HEADING_GPS_MAX_HERR_M` | 0.05 | GPS error limit for using a sample |
| `HEADING_GPS_FLOAT_MIN_BASELINE_M` | 2.0 | Baseline for RTK float test samples |
| `HEADING_GPS_FLOAT_MAX_YAW_CHANGE_DEG` | 10.0 | Turning limit for RTK float test samples |
| `HEADING_OFFSET_DEG` | 0.0 (leafsheep) | Mounting correction added to the magnetometer heading |
| `DEBUG_LOG_HEADING` | 0 | 1 = print one `[HDG_S]` line per sample, and log RTK float samples |
| `HEADING_FORWARD_HOLD_MS` (in `main.cpp`) | 1000 | How long a zero drive target still counts as driving forward |

## RTK float samples (test mode only)

With `DEBUG_LOG_HEADING 1`, RTK float positions (`fix = 5`) also produce samples. Because
float error is a few centimetres, these use a longer 2 m baseline and allow up to 10° of
turning. They are **only logged and never change the offset**, so a float fix can never
make the navigation heading worse. With the flag at 0, float positions are ignored.

## Running the heading test

1. Set `DEBUG_LOG_HEADING 1` and flash the robot.
2. Start the logger:
   `~/.platformio/penv/bin/python tools/heading_test.py log heading.csv`
3. Drive straight forward runs of 4–5 m in several directions, with the steering centred.
4. Stop with Ctrl+C and run:
   `~/.platformio/penv/bin/python tools/heading_test.py report heading.csv`

The report shows the mean, sd, rms and maximum error for both headings, and the
magnetometer error for each 45° course sector.

## Limitations

- **One straight run is needed after every boot** before missions can start, and it
  needs RTK fixed.
- **No corrections without straight forward driving on RTK fixed.** Turning in place,
  reversing or driving on RTK float does not correct the offset, so gyro drift builds up.
  In the tests the drift was small: the error stayed steady for over 20 minutes without
  any correction.
- **Crabbing.** If the robot slides sideways slightly (on a slope, or with misaligned
  wheels), the GPS course is not exactly where the robot points. The offset then learns
  the direction of travel, which is what path following needs anyway.
- **Not covered by the tests:** north, south and north-west courses.
- **PS2 read glitches** briefly set the drive target to 0. The 1 s forward allowance
  hides this from the heading fusion, but the glitch itself is not fixed yet.
