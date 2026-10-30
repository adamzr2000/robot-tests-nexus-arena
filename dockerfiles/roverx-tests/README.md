# roverx-tests

Automated RoverX floor tests T1–T3 from
[docs/protocol/Astibot_Floor_Experimental_Test_Protocol_v4.xlsx](../../docs/protocol/Astibot_Floor_Experimental_Test_Protocol_v4.xlsx).
One API (:8090) runs each trial, records its data and keeps the run log. All settings are in
[scripts/protocol.yaml](scripts/protocol.yaml).

| Test | Robot does | Conditions (5 runs each) | You tape |
|------|------------|--------------------------|----------|
| T1 straight-line baseline | Accelerate at 0.50 m/s² to `speed`, cruise, slow down at 0.50 m/s², stop when wheel odometry reads **8.00 m** | `speed` 0.2 / 0.5 / 1.0 / 1.5 m/s | Δx, Δy at the 8 m line |
| T2 acceleration response | Ramp the command at `accel` to 1.00 m/s, hold 2.0 s, slow down at 0.50 m/s² | `accel` 0.2 / 0.5 / 1.0 m/s² | nothing |
| T3 stopping response | Accelerate at 0.50 m/s² to `speed`, cruise; at **6.00 m** of wheel odometry command v = 0 and hold it (ACTIVE) | `speed` 0.2 / 0.5 / 1.0 / 1.5 m/s | start line → robot at rest |

## Session setup

```bash
make -C dockerfiles roverx-tests                       # once, or after changing the Dockerfile
./start_roverx_tests.sh --session final-2026-11-03    # final area (results/final-2026-11-03/)
docker logs -f roverx-tests                            # live output, keep it open
./stop_roverx_tests.sh                                 # at the end
```

Floor (one straight lane, ≥ 11 m):

```text
start line            T3 trigger mark         T1 finish line            run-out
    |──────────────────────────|──────────────────────|────────────────────|
   0 m                        6 m                    8 m                ≥ 11 m
   ═════════════════════════ centerline ═════════════════════════════════════
```

Marking (once):

- Floor: centerline as a straight taut string or chalk line (or tape along a string), start line,
  6 m and 8 m lines perpendicular to it (use a large square or the 3-4-5 rule). A tape measure
  fixed at 0 along the centerline lets you read distances directly.
- Robot: two pointers on the chassis centre plane, one at the **front** (the reference point) and
  one at the **rear**, as far apart as possible: e.g. a stiff wire or a pen taped so its tip is
  1–2 cm above the floor. Measure from the floor point straight below each tip.

Every run:

- Before: robot at rest, front tip over the start line, both tips over the centerline
  (1° off = 14 cm sideways at 8 m).
- After (T1): mark the floor below the front tip, then Δx = along the lane from the 8 m line
  (+ past it, − short) and Δy = square distance from the centerline (+ left, − right, seen from
  behind the robot facing travel). Optional, in `observations`: Δy of the rear tip too; with the
  pointer spacing L, the final heading is atan((Δy_front − Δy_rear) / L).
- After (T3): `total_cm` = start line → floor point below the front tip, read on the tape.
- Use a tape measure, not a phone app (± a few cm is the size of what T1 measures).
- During a session nothing else may command the robot (teleop, Nav2 goals, the manufacturer UI):
  `/cmd_vel` is shared.

## Trials

For every run:

1. Align the robot at the start (as above).
2. Run the command once. The response shows the trial ID (`T1_v0.50_r3_101530`).
3. Watch `docker logs -f roverx-tests` until `[api] <trial id> finished: DONE`.
4. T1 / T3: tape and enter the values in `results/<session>/manual.csv` (row with that trial ID).
5. Drive or carry the robot back to the start. Each run is independent: own ID, bag and results.

Only `DONE` runs count as repetitions; an `ABORTED` run (with its reason) is simply run again
with the same command. `curl -s -X POST localhost:8090/stop | jq` stops the robot at any time.
`curl -s localhost:8090/status | jq` shows progress (`"done"`: e.g. `"speed 0.5": "3/5"`).

### T1 — straight-line baseline (20 runs, lane ≥ 11 m)

Run each command 5 times, in this order. After each run tape `dx_cm`, `dy_cm`.

```bash
curl -s -X POST localhost:8090/t1 -d '{"speed": 0.2}' | jq    # T1_v0.20_r1 … r5
curl -s -X POST localhost:8090/t1 -d '{"speed": 0.5}' | jq    # T1_v0.50_r1 … r5
curl -s -X POST localhost:8090/t1 -d '{"speed": 1.0}' | jq    # T1_v1.00_r1 … r5
curl -s -X POST localhost:8090/t1 -d '{"speed": 1.5}' | jq    # T1_v1.50_r1 … r5
```

### T2 — acceleration response (15 runs, lane ≥ 7 m)

Run each command 5 times, in this order. Nothing to tape (note slip if seen).

```bash
curl -s -X POST localhost:8090/t2 -d '{"accel": 0.2}' | jq    # T2_a0.20_r1 … r5   (5.5 m)
curl -s -X POST localhost:8090/t2 -d '{"accel": 0.5}' | jq    # T2_a0.50_r1 … r5   (4.0 m)
curl -s -X POST localhost:8090/t2 -d '{"accel": 1.0}' | jq    # T2_a1.00_r1 … r5   (3.5 m)
```

### T3 — stopping response (20 runs, lane ≥ 11 m)

Run each command 5 times, in this order, and look at the braking of each speed before going to
the next. After each run tape `total_cm` (start line → reference point at rest). Check
`steady_ok: true` in the result; if `false`, mark the run `INVALID` and run it again.

```bash
curl -s -X POST localhost:8090/t3 -d '{"speed": 0.2}' | jq    # T3_v0.20_r1 … r5
curl -s -X POST localhost:8090/t3 -d '{"speed": 0.5}' | jq    # T3_v0.50_r1 … r5
curl -s -X POST localhost:8090/t3 -d '{"speed": 1.0}' | jq    # T3_v1.00_r1 … r5
curl -s -X POST localhost:8090/t3 -d '{"speed": 1.5}' | jq    # T3_v1.50_r1 … r5
```

COAST stop (wheel power cut at the trigger) is a separate, optional condition:
`-d '{"speed": 0.2, "stop_mode": "coast"}'`.

### Indoor checks (short lane, nothing recorded)

```bash
./start_roverx_tests.sh --max-travel 4      # refuses runs planned longer than 4 m
curl -s -X POST localhost:8090/t1 -d '{"speed": 0.5, "distance": 3, "record": false}' | jq
curl -s -X POST localhost:8090/t2 -d '{"accel": 1.0, "hold_s": 1, "record": false}' | jq
curl -s -X POST localhost:8090/t3 -d '{"speed": 0.5, "trigger": 3, "record": false}' | jq
```

Any request can add `"dry_run": true` (only print the plan) or `"notes": "..."`, and override a
protocol.yaml setting (`"distance"`, `"trigger"`, `"hold_s"`…); the override is recorded.

## Tape values: manual.csv

After each recorded run a row appears in `results/<session>/manual.csv` with `trial_id`,
`test`, `setpoint`, `repeat`, `status`. Fill in (Excel/LibreOffice, `,` or `;`, decimal commas
fine), and close the file before the next run:

| Column | Value |
|--------|-------|
| `dx_cm`, `dy_cm` | T1: offset at the 8 m line (signs as above) |
| `total_cm` | T3: start line → reference point at rest |
| `override_status` | `INVALID`, `SKIPPED` or `NOT SUPPORTED` (the run then doesn't count) |
| `observations`, `notes` | slip / scrub, floor marks, anything else |

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

The bag starts before the robot is prepared, so logging always begins before motion.

Main `run_log.csv` columns:

- All: `time_to_speed_s` (first measured speed within 5 %), `speed_held_s` / `speed_held_mean`
  (time and mean at target speed), `odom_end_m`, `max_speed`, `heading_change_deg` (IMU),
  `imu_fwd_*_mean` (mean IMU acceleration along the robot's forward axis while speeding up /
  slowing down / braking) with `imu_*_dir_deg` (direction in the IMU plane, ~111° speeding up,
  ~−69° slowing down; a change means the IMU mounting changed).
- T1: `endpoint_error_cm` = √(Δx² + Δy²); `odom_error_cm` = (8 m + Δx) − odometry at rest.
- T2: `t10_s`, `t90_s` vs `planned_t90_s`, `accel_measured` (wheel speed, 10–90 %).
- T3: `trigger_odom_m`, `speed_at_trigger` (1 s average), `steady_ok`, `wheels_powered_at_rest`
  (false: wheel power was cut before rest, so not a pure ACTIVE stop), `braking_odom_m`, `braking_t95_s`
  (time to 95 % of the braking distance), `braking_time_s` (until rest is confirmed),
  `stop_distance_cm` = `total_cm` − trigger.

## How each trial runs

Lidar on (it carries the IMU), wheels powered (checked on `/relay_status`), 2 s still (gyro
bias, gravity, robot–laptop clock offset; the run aborts if the wheels or the gyro show the
robot moved meanwhile), then the motion at 50 Hz; all times and distances
count from motion start. At the end v = 0 is held until the wheels report rest, then wheel power
is cut. The run aborts (robot stopped) on: operator `/stop`, odometry or IMU silent 0.5 s,
wheels not moving 1.5 s after a clear command, odometry past the guard distance, timeout.

## RoverX facts the tests rely on (measured 2026-10-30)

- Wheel odometry `/odom_raw`: speed from the motor controllers' Hall loop, each controller at
  10 Hz; between samples the distance is extrapolated with the measured speed. Speed can be up
  to 0.1 s old, so T3's trigger position includes that latency. Distances at rest are unaffected.
- Driver: `wheel_speed_scale` 0.29, `friction_ff` 0, speed limits out of reach (10 m/s);
  each trial records them. Measured wheel speed is −3 % under command at 0.2 m/s, −1 % at 0.5.
- Response: ~0.2–0.3 s from first command to wheel motion, then ~0.12 s lag; T2 ramps up to
  1.0 m/s² are followed.
- IMU `/unilidar/imu`: 250 Hz, m/s², upside down (z ≈ −9.8 at rest; the robot's TF says identity,
  which is wrong), robot forward at 111° in its x-y plane. ±1–2 m/s² vibration while moving
  (spinning lidar), so the metrics use phase means.
- Robot clock is ~2.8 s ahead of the laptop and drifts; each trial records the offset.
- Relays: zero `/cmd_vel` cuts wheel power 3 s later. No battery topic: note the charge level.
