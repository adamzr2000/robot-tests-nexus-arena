# robot-trajectories

Trajectory scripts (`scripts/*_walk.py`, plain `rclpy`) behind a small HTTP API
(`scripts/main.py`, port 8080, started with the container).

## API

| Method | Path                       | Description                                  |
|--------|----------------------------|----------------------------------------------|
| GET    | `/trajectories`            | List trajectory names                        |
| POST   | `/trajectories/<name>/run` | Start `<name>`; JSON body = its CLI flags    |
| POST   | `/stop`                    | Stop the running trajectory                  |
| GET    | `/status`                  | Running trajectory and bag; `last` run exit code (0 = completed) |

Body keys map to CLI flags (`"angular_speed": 0.5` -> `--angular-speed 0.5`;
booleans become a flag or nothing); see `--help` on each script. Starting a run
stops the previous one.

API-only keys:
- `"record_bag": true` records a bag via `bags-collector` (`BAGS_URL`, default
  `http://localhost:8081`) for exactly the run, plus 1 s. Only the profile's
  `bag_topics` (commands and pose estimates) and any explicit `topic`/`odom_topic`.
  If recording can't start, the run doesn't either (HTTP 502).
- `"tag"`: bag name suffix, `<time>_<robot>_<trajectory>_<tag>`.

```bash
curl -X POST localhost:8080/trajectories/circle/run \
     -d '{"robot": "go2", "radius": 1.0, "angular_speed": 0.5, "laps": 2, "use_odom": true}'
curl -X POST localhost:8080/trajectories/square/run \
     -d '{"robot": "roverx", "side": 1.0, "speed": 0.3, "turn_speed": 0.5, "laps": 1,
          "use_odom": true, "record_bag": true, "tag": "exp1"}'
```

## Robot profiles

`"robot"` sets topics, preparation and bag topics (`scripts/robot_profiles.py`), not
speed caps. Explicit `topic` and `odom_topic` override it.

| Profile  | Distance              | Heading              | Bag topics                                        |
|----------|-----------------------|----------------------|---------------------------------------------------|
| `go2`    | `/utlidar/robot_odom` | same odometry        | `/cmd_vel`, `/utlidar/robot_odom`                 |
| `roverx` | `/odom_raw` (wheels)  | `/unilidar/imu` gyro | `/cmd_vel`, `/odom_raw`, `/odom`, `/unilidar/imu` |

Both command `/cmd_vel`. `roverx` turns the lidar on (it carries the IMU) and the
wheel power on before moving, stays still 2 s to measure gyro bias, and cuts
wheel power at the end.

Topic rates (measured):

| Robot  | Topic                 | Rate         | Role                                   |
|--------|-----------------------|--------------|----------------------------------------|
| both   | `/cmd_vel`            | 10 Hz        | Trajectory commands (`--rate`)         |
| go2    | `/utlidar/robot_odom` | ~175-195 Hz  | Odometry: position and heading         |
| roverx | `/odom_raw`           | ~22 Hz       | Wheel odometry: distance               |
| roverx | `/odom`               | ~50 Hz       | Robot's fused estimate (wheels + IMU)  |
| roverx | `/unilidar/imu`       | ~250 Hz      | Gyro: heading                          |

RoverX's driver applies commands at 50 Hz and stops after 0.5 s without one.

## Command profile (timed runs)

Timed runs (the default) are open loop and send the identical command stream to
any robot, so execution error (slip) can be compared against ground truth:

- `/cmd_vel` at 10 Hz; 0.3 m/s and 0.3 rad/s; circle and figure-8 radius 1 m; square side 1 m.
- Linear ramp 0 -> full speed over the first 1 s and back to 0 over the last 1 s,
  applied to linear and angular speed alike (circle radius stays exact). The run
  is lengthened by the ramp time so the commanded distance/angle is exact:
  one circle lap = 21.9 s, one square lap = 35.3 s, one figure-8 lap = 42.9 s.
- Square: instantaneous switch between straight and turning phases.
- Figure-8: two tangent loops of the same radius from the crossing point, first
  left (or right with `clockwise`), constant forward speed; the turn direction
  flips instantly at the crossing. One lap = both loops; it needs 4 x radius
  sideways and 2 x radius along the start heading.
- Robot preparation (RoverX: lidar, wheel power, 2 s still) happens before the
  profile starts and doesn't affect it. The exact commands are in the bag's `/cmd_vel`.

## Laps: timed vs measured

`laps` alone is timed from the commanded speed, so real laps drift. With
`"use_odom": true` they're measured (feedback from the robot's own odometry):
- **circle**: a lap is 360 degrees of heading.
- **square**: sides by distance, corners toward absolute k * 90 degree headings,
  so heading error doesn't accumulate. `laps` rounds up to whole sides.
- **figure8**: each loop ends at an absolute heading (+360 degrees, then back to
  0). `laps` rounds up to whole loops.

Measured runs wait up to 10 s for odometry before moving, stop if it goes silent
for 1 s, and stop at 2x the timed estimate if the target isn't reached.

## Safety

- No speed caps unless the request sets `max_linear_speed` / `max_angular_speed`
  (then it is refused above them); numeric flags must be finite.
- Ramps up over `--ramp-time` (default 1 s), and down at the end of timed runs.
- On any exit (done, Ctrl+C, `/stop`) sends a zero `Twist` 5 times, then the
  profile's stop services.

## Adding a trajectory

Subclass `trajectory_common.TrajectoryNode` with a `twist(t)` (see
`circle_walk.py`), use `add_common_args`/`validate_common_args`/`init`/`run`,
and add one line to `TRAJECTORIES` in `scripts/main.py`.
