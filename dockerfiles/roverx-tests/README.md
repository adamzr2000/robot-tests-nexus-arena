# roverx-tests

Automated RoverX floor tests T1–T3
([protocol v4](../../docs/protocol/Astibot_Floor_Experimental_Test_Protocol_v4.xlsx)). One API
(:8090) runs each trial, records its data and keeps the run log. Settings:
[scripts/protocol.yaml](scripts/protocol.yaml).

## The tests

All three drive the same lane from rest and are judged at the same **8 m line**:

| Test | Robot | Variable (5 runs each) |
|------|-------|------------------------|
| **T1** straight-line baseline | speed up at 0.5 m/s² → cruise at `speed` → slow down at 0.5 m/s² → stop at 8 m | `speed` 0.2 / 0.5 / 1.0 / 1.5 m/s |
| **T2** acceleration response | speed up at `accel` → cruise at 1.0 m/s → slow down at 0.5 m/s² → stop at 8 m | `accel` 0.2 / 0.5 / 1.0 m/s² |
| **T3** stopping response | speed up at 0.5 m/s² → cruise at `speed` → **sudden stop at 8 m** (v = 0 at once) | `speed` 0.2 / 0.5 / 1.0 / 1.5 m/s |

"Stop at 8 m" = when wheel odometry reads 8.00 m. After every run you tape **Δx, Δy** at the
8 m line; for T3, Δx is roughly the stopping distance.

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

- Floor: centerline (taut string or chalk line), start line and 8 m line perpendicular to it
  (large square or 3-4-5 rule), a tape measure fixed at 0 along the centerline.
- Robot: two pointers on the chassis centreline, **front** (the reference point) and **rear**, as
  far apart as possible, tips 1–2 cm above the floor (stiff wire or a taped pen).

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

Optional, separate condition: COAST stop (wheel power cut at 8 m instead of v = 0),
`-d '{"speed": 0.2, "stop_mode": "coast"}'`.

### Short lane / quick checks

```bash
./start_roverx_tests.sh --max-travel 4      # refuses any run with distance over 4 m
curl -s -X POST localhost:8090/t1 -d '{"speed": 0.5, "distance": 3, "record": false}' | jq
curl -s -X POST localhost:8090/t2 -d '{"accel": 1.0, "distance": 3, "record": false}' | jq
curl -s -X POST localhost:8090/t3 -d '{"speed": 0.5, "distance": 3, "record": false}' | jq
```

- `"record": false`: nothing saved, not counted.
- `"dry_run": true`: only print the plan, no motion.
- Any protocol.yaml setting can be overridden per run (`"distance"`, `"accel"`…); it is recorded.
- `"notes": "..."` is copied to the run log.

## manual.csv

A row per recorded run appears automatically (`trial_id`, `test`, `setpoint`, `repeat`,
`status`). Fill in with Excel/LibreOffice (`,` or `;`, decimal commas fine) and close the file
before the next run:

| Column | Value |
|--------|-------|
| `dx_cm`, `dy_cm` | offset at the 8 m line (signs as in step 4) |
| `override_status` | `INVALID`, `SKIPPED` or `NOT SUPPORTED`: the run no longer counts |
| `observations`, `notes` | slip / scrub, floor marks, rear-tip Δy, anything else |

## Results

```text
results/<session>/
  run_log.csv            one row per trial: results + your tape values + computed columns
  manual.csv             your tape values
  <trial id>/
    trial.json           settings used, events, metrics, driver calibration, clock offset, IMU at rest
    samples.csv          50 Hz: phase, commanded and measured speed, odometry distance
    bag/                 /unilidar/imu, /odom_raw, /odom, /cmd_vel, /relay_status, /tf, /tf_static, /trial/events
```

Recording starts before the robot is prepared, so it always covers the whole motion.

`run_log.csv` columns:

| Tests | Columns |
|-------|---------|
| all | `odom_end_m` (odometry at rest), `time_to_speed_s`, `speed_held_s`, `speed_held_mean` (time and mean at target speed), `max_speed`, `heading_change_deg` (IMU) |
| all | `imu_fwd_speedup_mean`, `imu_fwd_slowdown_mean`, `imu_fwd_braking_mean`: mean IMU acceleration along the robot's forward axis per phase; `imu_*_dir_deg`: its direction (~111° speeding up, ~−69° slowing down; a change means the IMU mounting moved) |
| all (taped) | `odom_error_cm` = (8 m + Δx) − odometry at rest |
| T1, T2 | `endpoint_error_cm` = √(Δx² + Δy²) |
| T2 | `t10_s`, `t90_s` vs `planned_t90_s`, `accel_measured` (from wheel speed, 10–90 %) |
| T3 | `stop_odom_m` (odometry where v = 0 was commanded), `speed_at_stop` (1 s average), `steady_ok`, `braking_odom_m`, `braking_t95_s` (time to 95 % of the braking distance), `braking_time_s` (until rest is confirmed), `wheels_powered_at_rest` (false = power was cut while still braking), `stop_distance_cm` = (8 m + Δx) − `stop_odom_m` |

## How each trial runs

1. Lidar on (it carries the IMU) and wheels powered, checked on `/relay_status`.
2. 2 s still: gyro bias, gravity and robot–laptop clock offset are measured. The run aborts if
   the wheels or the gyro show the robot moved.
3. The motion, commanded at 50 Hz. All times and distances count from its start.
4. v = 0 held until the wheels report rest, then wheel power is cut.

The robot is stopped and the run marked `ABORTED` on: `/stop`, odometry or IMU silent for 0.5 s,
wheels not moving 1.5 s after a clear command, odometry past the guard distance (8.5 m for T1/T2,
12 m for T3), or timeout.

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
