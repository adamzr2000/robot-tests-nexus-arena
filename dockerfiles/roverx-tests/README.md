# roverx-tests

Automated RoverX floor tests T1–T4
([protocol v4](../../docs/protocol/Astibot_Floor_Experimental_Test_Protocol_v4.xlsx)). One API
(:8090) runs each trial, records its data and keeps the run log. Every test value is in
[scripts/protocol.yaml](scripts/protocol.yaml) (see [Settings](#settings-protocolyaml)).

## The tests

T1–T3 drive the same lane from rest and are judged at the same **8 m line**; T4 turns in place:

| Test                          | Robot                                                                             | Variable (5 runs each)            |
| ----------------------------- | --------------------------------------------------------------------------------- | --------------------------------- |
| **T1** straight-line baseline | speed up at 0.5 m/s² → cruise at `speed` → slow down at 0.5 m/s² → stop at 8 m    | `speed` 0.2 / 0.5 / 1.0 / 1.5 m/s |
| **T2** acceleration response  | speed up at `accel` → cruise at 1.0 m/s → slow down at 0.5 m/s² → stop at 8 m     | `accel` 0.2 / 0.5 / 1.0 m/s²      |
| **T3** stopping response      | speed up at 1.5 m/s² → cruise at `speed` → **sudden stop at 8 m** (v = 0 at once) | `speed` 0.2 / 0.5 / 1.0 / 1.5 m/s |
| **T4** in-place rotation      | v = 0; turn rate up at 1.0 rad/s² → hold `yaw_rate` → **stop when the IMU heading reaches 90°** | `yaw_rate` +0.3 / −0.3 / +0.6 / −0.6 rad/s (+ = left) |

"Stop at 8 m" = when wheel odometry reads 8.00 m. After every run you tape **Δx, Δy**: at the
8 m line for T1–T3 (for T3, Δx is roughly the stopping distance), and the displacement of the
robot's centre for T4.

## Session

```bash
make -C dockerfiles roverx-tests                       # once, or after changing the Dockerfile
./start_roverx_tests.sh --session final-2026-11-03    # results go to results/final-2026-11-03/
docker logs -f roverx-tests                            # live output, keep it open
./stop_roverx_tests.sh                                 # at the end
```

Lane (≥ 12 m, straight, the same for all tests):

```text
start line                                       8 m line          run-out / T3 braking
    |───────────────────────────────────────────────|────────────────────────|
   0 m                                             8 m                     ≥ 12 m
   ═══════════════════════════════ centerline ═════════════════════════════════
```

Mark once:

- Floor: centerline (taut string or chalk line), start line and 8 m line perpendicular to it,
  a tape measure fixed at 0 along the centerline.
- Robot: two pointers on the chassis centreline, **front** (the reference point) and **rear**, as
  far apart as possible, tips 1–2 cm above the floor (stiff wire or a taped pen). The robot's
  centre (T4) is the midpoint between the two tips.

Nothing else may command the robot during a session (teleop, Nav2 goals, the manufacturer UI).

## Trials

Every run:

1. Robot at rest, front tip over the start line, both tips over the centerline
   (1° off = 14 cm sideways at 8 m).
2. Run the command once; the response shows the trial ID (e.g. `T1_v0.50_r3_101530`).
3. Wait for `[api] <trial id> finished: DONE` in the logs.
4. Mark the floor under the front tip and tape, with a tape measure (not a phone app):
   - `dx_cm`: along the lane from the 8 m line, + past it, − short.
   - `dy_cm`: from the centerline, + left, − right (seen from behind the robot, facing travel).

   Enter them in `results/<session>/manual.csv`, row with that trial ID.
5. Bring the robot back to the start. Each run is independent: own ID, bag and results.

T4 differs in steps 1 and 4: before the run mark the floor under both tips (or start on the
lane's start line and centerline as above); after it mark under both tips again. The centre is the
midpoint of each pair: `dx_cm` = centre displacement along the original heading (+ forward),
`dy_cm` = across it (+ left). Optional, in `observations`: the final heading from the tip marks.

Run each command 5 times, in the order listed. Only `DONE` runs count; an `ABORTED` run (the
response and logs say why) is simply run again. Progress: `curl -s localhost:8090/status | jq`
(`"done"`, e.g. `"speed 0.5": "3/5"`). Stop the robot at any time:
`curl -s -X POST localhost:8090/stop | jq`.

### T1 — straight-line baseline (20 runs)

```bash
curl -s -X POST localhost:8090/t1 -d '{"speed": 0.2}' | jq    # T1_v0.20_r1 … r5
curl -s -X POST localhost:8090/t1 -d '{"speed": 0.5}' | jq    # T1_v0.50_r1 … r5
curl -s -X POST localhost:8090/t1 -d '{"speed": 1.0}' | jq    # T1_v1.00_r1 … r5
curl -s -X POST localhost:8090/t1 -d '{"speed": 1.5}' | jq    # T1_v1.50_r1 … r5
```

### T2 — acceleration response (15 runs)

```bash
curl -s -X POST localhost:8090/t2 -d '{"accel": 0.2}' | jq    # T2_a0.20_r1 … r5
curl -s -X POST localhost:8090/t2 -d '{"accel": 0.5}' | jq    # T2_a0.50_r1 … r5
curl -s -X POST localhost:8090/t2 -d '{"accel": 1.0}' | jq    # T2_a1.00_r1 … r5
```

### T3 — stopping response (20 runs)

Check the braking of each speed before moving to the next. If a result shows
`steady_ok: false` (not at steady speed before the stop), set `override_status` to `INVALID` and
run it again.

```bash
curl -s -X POST localhost:8090/t3 -d '{"speed": 0.2}' | jq    # T3_v0.20_r1 … r5
curl -s -X POST localhost:8090/t3 -d '{"speed": 0.5}' | jq    # T3_v0.50_r1 … r5
curl -s -X POST localhost:8090/t3 -d '{"speed": 1.0}' | jq    # T3_v1.00_r1 … r5
curl -s -X POST localhost:8090/t3 -d '{"speed": 1.5}' | jq    # T3_v1.50_r1 … r5
```

Optional, separate condition: COAST stop, `-d '{"speed": 0.2, "stop_mode": "coast"}'`. At 8 m
it commands v = 0 **and** requests wheel power off; the relays open ~0.5 s later (measured), so
the robot brakes actively until then and is unpowered after. Not a pure coast.

### T4 — in-place rotation (20 runs)

Free area: a clear circle around the robot of at least its diagonal plus a margin (it turns
about its centre and may drift).

`+` turns counter-clockwise (left), `−` clockwise (right). The heading comes from the IMU gyro;
the first run checks its sign and aborts if the robot turns against the command.

```bash
curl -s -X POST localhost:8090/t4 -d '{"yaw_rate": 0.3}' | jq     # T4_w+0.30_r1 … r5
curl -s -X POST localhost:8090/t4 -d '{"yaw_rate": -0.3}' | jq    # T4_w-0.30_r1 … r5
curl -s -X POST localhost:8090/t4 -d '{"yaw_rate": 0.6}' | jq     # T4_w+0.60_r1 … r5
curl -s -X POST localhost:8090/t4 -d '{"yaw_rate": -0.6}' | jq    # T4_w-0.60_r1 … r5
```

Skid-steer scrubs its wheels when turning, so the robot turns slower than commanded (~85 %
measured in 2026-10-28 calibration): T4 reports the achieved rate and its ratio to the command.

### Short lane / quick checks

```bash
./start_roverx_tests.sh --max-travel 4      # refuses any run with distance over 4 m
curl -s -X POST localhost:8090/t1 -d '{"speed": 0.5, "distance": 3, "record": false}' | jq
curl -s -X POST localhost:8090/t2 -d '{"accel": 1.0, "distance": 3, "record": false}' | jq
curl -s -X POST localhost:8090/t3 -d '{"speed": 0.5, "distance": 3, "record": false}' | jq
curl -s -X POST localhost:8090/t4 -d '{"yaw_rate": 0.3, "record": false}' | jq
```

- `"record": false`: nothing saved, not counted.
- `"dry_run": true`: only print the plan, no motion.
- Any test setting can be overridden for one run (`"distance"`, `"accel"`…); it is recorded.
- `"notes": "..."` is copied to the run log.

## manual.csv

A row per recorded run appears automatically (`trial_id`, `test`, `setpoint`, `repeat`,
`status`). Fill in with Excel/LibreOffice (`,` or `;`, decimal commas fine) and close the file
before the next run:

| Column                  | Value                                                                           |
| ----------------------- | ------------------------------------------------------------------------------- |
| `dx_cm`, `dy_cm`        | T1–T3: offset at the 8 m line (signs as in step 4); T4: centre displacement      |
| `override_status`       | empty, or `INVALID` / `SKIPPED` / `NOT SUPPORTED`: the run no longer counts and the next run takes its repeat number |
| `observations`, `notes` | slip / scrub, floor marks, rear-tip Δy, anything else                           |

`status` is set by the system (`DONE`, `ABORTED`, `FAILED`); `override_status` is yours and wins.

## Results

```text
results/<session>/
  run_log.csv            one row per trial: results + your tape values + computed columns
  manual.csv             your tape values
  <trial id>/
    trial.json           settings used, events, metrics, driver calibration, clock offset, IMU at rest
    samples.csv          50 Hz: phase, commands (linear, angular), wheel speed, IMU yaw rate, heading, odometry
    bag/                 rosbag (sqlite3), topics below
```

### Recorded data (rosbag)

Each trial records its own bag, started before the robot is prepared, so it covers the whole
run. Topics (`bag_topics` in protocol.yaml):

| Topic            | Type                  | Rate     | Content                                                                    |
| ---------------- | --------------------- | -------- | -------------------------------------------------------------------------- |
| `/unilidar/imu`  | sensor_msgs/Imu       | ~250 Hz  | raw IMU: acceleration ax/ay/az, gyro gx/gy/gz, orientation (lidar's IMU)   |
| `/odom_raw`      | nav_msgs/Odometry     | ~20 Hz   | wheel odometry: position, measured speed (the distance the tests stop on) |
| `/odom`          | nav_msgs/Odometry     | ~45 Hz   | the robot's fused estimate (wheel speed + IMU heading)                     |
| `/cmd_vel`       | geometry_msgs/Twist   | 50 Hz    | every speed command sent: the exact profile (ramps, cruise, stop)         |
| `/relay_status`  | std_msgs/Bool         | 10 Hz    | wheel power on/off                                                         |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | varies   | the robot's coordinate frames                                              |
| `/trial/events`  | std_msgs/String       | 3 per run | JSON: `motion_start`, `stop_command` (time, odometry, speed, heading), `at_rest` |

Inspect one (ROS is in the container):
`docker exec roverx-tests /ros_entrypoint.sh ros2 bag info /home/robot/results/<session>/<trial id>/bag`.
Message times are the laptop's, header stamps the robot's (offset: `clock_offset_s` in `trial.json`).

### run_log.csv columns

Besides the identification (`trial_id`, `test`, `setpoint`, `repeat`, `status`, `reason`,
`started`, `overrides`, `notes`, `bag`):

| Tests       | Columns |
| ----------- | ------- |
| all         | `motion_time_s`, `heading_change_deg` (IMU), `clock_offset_s` |
| T1–T3       | `odom_end_m` (odometry at rest), `time_to_speed_s`, `speed_held_s`, `speed_held_mean` (time and mean at target speed), `max_speed` |
| T1–T3       | `imu_fwd_speedup_mean`, `imu_fwd_slowdown_mean`, `imu_fwd_braking_mean`: mean IMU acceleration along the robot's forward axis per phase; `imu_*_dir_deg`: its direction (~111° speeding up, ~−69° slowing down; a change means the IMU mounting moved) |
| T1–T3 taped | `dx_cm`, `dy_cm`, `odom_error_cm` = (8 m + Δx) − odometry at rest |
| T1, T2      | `endpoint_error_cm` = √(Δx² + Δy²), `coast_after_stop_m` (odometry after the stop command) |
| T2          | `t10_s`, `t90_s` vs `planned_t90_s`, `accel_measured` (from wheel speed, 10–90 %) |
| T3          | `stop_mode`, `stop_odom_m` (odometry where v = 0 was commanded), `speed_at_stop` (1 s average), `steady_before_stop_s`, `steady_ok`, `braking_odom_m`, `braking_t95_s` (time to 95 % of the braking distance), `braking_time_s` (until rest is confirmed), `wheels_powered_at_rest` (false = power was cut while still braking), `stop_distance_cm` = (8 m + Δx) − `stop_odom_m` |
| T4          | `yaw_rate_cruise_mean` / `_std` (IMU, while commanded at full rate, first 0.5 s left out), `yaw_rate_ratio` (achieved / commanded), `yaw_rate_peak`, `time_to_rate_s` (to 90 % of the achieved rate), `heading_at_stop_deg`, `heading_final_deg`, `overshoot_deg` (final − 90°), `coast_after_stop_deg`, `stop_time_s`, `odom_heading_change_deg` and `wheel_imu_heading_ratio` (wheel odometry vs IMU: turning slip), `center_drift_odom_m` (wheel odometry's view, ~0 by construction in a skid-steer turn: the real drift is the tape); taped: `dx_cm`, `dy_cm`, `center_drift_cm` = √(Δx² + Δy²) |

## Settings (protocol.yaml)

[scripts/protocol.yaml](scripts/protocol.yaml) holds every value. Edit it to change the protocol
for all following runs: it is read at every run, no restart needed (the scripts folder is
mounted into the container). To change a value for one run only, pass it in the request instead
(e.g. `"distance": 3`). Each `trial.json` records the values actually used.

| Section                  | Key                 | Current           | Meaning                                                               |
| ------------------------ | ------------------- | ----------------- | --------------------------------------------------------------------- |
| `tests.T1`               | `values`            | 0.2 0.5 1.0 1.5   | `speed` conditions (m/s)                                              |
|                          | `repetitions`       | 5                 | DONE runs per condition                                               |
|                          | `distance`          | 8.0               | stop when odometry reads this (m)                                     |
|                          | `accel`             | 0.5               | speed-up and slow-down (m/s²)                                         |
|                          | `overshoot_guard`   | 0.5               | abort beyond `distance` + this (m)                                    |
| `tests.T2`               | `values`            | 0.2 0.5 1.0       | `accel` conditions (m/s²)                                             |
|                          | `speed`             | 1.0               | cruise speed (m/s)                                                    |
|                          | `distance`, `decel`, `overshoot_guard` | 8.0, 0.5, 0.5 | as T1                                                  |
| `tests.T3`               | `values`            | 0.2 0.5 1.0 1.5   | `speed` conditions (m/s)                                              |
|                          | `accel`             | 1.5               | speed-up (m/s²)                                                       |
|                          | `distance`          | 8.0               | where v = 0 is commanded (m)                                          |
|                          | `min_steady_s`      | 1.0               | time at steady speed required before the stop (s)                     |
|                          | `stop_mode`         | active            | `active` (v = 0 held) or `coast` (v = 0 + wheel power off, ~0.5 s later) |
|                          | `braking_guard`     | 4.0               | abort beyond `distance` + this (m)                                    |
| `tests.T4`               | `values`            | 0.3 −0.3 0.6 −0.6 | `yaw_rate` conditions (rad/s, + counter-clockwise)                     |
|                          | `angle_deg`         | 90.0              | stop when the IMU heading has turned this much (°)                    |
|                          | `ang_accel`         | 1.0               | turn-rate ramp (rad/s²)                                               |
|                          | `heading_guard_deg`, `drift_guard` | 45.0, 0.5 | abort past `angle_deg` + this (°) / if the centre moves more than this (m) |
| `control`                | `rate_hz`, `settle_s`, `crawl_speed`, `steady_*`, … | | command rate, still period, final crawl, "at speed" definition, abort timeouts |
| `robot`                  | topics, `imu_forward_deg`, … |          | topic names, IMU mounting, preparation services                       |
| `bag_topics`             |                     | see above         | what each bag records                                                 |

Adding a value to `values` (e.g. a new speed) adds a condition; `/status` counts it.

## How each trial runs

1. Lidar on (it carries the IMU) and wheels powered, checked on `/relay_status`.
2. 2 s still: gyro bias, gravity and robot–laptop clock offset are measured. The run aborts if
   the wheels or the gyro show the robot moved.
3. The motion, commanded at 50 Hz. All times and distances count from its start.
4. Zero held until the wheels report rest and the IMU no turning, then wheel power is cut.

The robot is stopped and the run marked `ABORTED` on: `/stop`, odometry or IMU silent for 0.5 s,
wheels not moving 1.5 s after a clear command, odometry past the guard distance (8.5 m for T1/T2,
12 m for T3), timeout, and for T4: heading past 135°, centre moved over 0.5 m, or IMU heading
turning against the command (wrong `imu_z_sign`).

## RoverX facts the tests rely on (measured 2026-10-30)

- Wheel odometry `/odom_raw`: speed from the motor controllers' Hall loop, each controller at
  10 Hz; between samples the distance is extrapolated with the measured speed. The speed can be
  up to 0.1 s old, so T3's stop position includes that latency; distances at rest don't.
- Driver: `wheel_speed_scale` 0.29, `friction_ff` 0, speed limits out of reach (10 m/s); every
  trial records them. Measured wheel speed is −3 % under command at 0.2 m/s, −1 % at 0.5.
- Response: 0.2–0.3 s from the first command to wheel motion, then ~0.12 s lag; speed-ups up to
  1.0 m/s² are followed.
- IMU `/unilidar/imu`: 250 Hz, m/s², upside down (z ≈ −9.8 at rest; the robot's TF wrongly says
  identity), robot forward at 111° in its x-y plane. ±1–2 m/s² vibration while moving
  (spinning lidar), hence the per-phase means.
- Robot clock ~2.8 s ahead of the laptop and drifting; every trial records the offset.
- Relays: a zero `/cmd_vel` cuts wheel power 3 s later. No battery topic: note the charge level.
