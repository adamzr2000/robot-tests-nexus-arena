# robot-trajectories

Container for running robot trajectory scripts (circle, square, figure-eight, ...)
over a `cmd_vel`-style topic. Plain `rclpy` scripts under `scripts/`, no ROS2
package, no build step. `scripts/main.py` is a small HTTP API that triggers
them, started by default when the container runs.

## Endpoints

| Method | Path                      | Description                                   |
|--------|---------------------------|------------------------------------------------|
| GET    | `/trajectories`           | List available trajectory names                |
| POST   | `/trajectories/<name>/run`| Start `<name>`; JSON body = its CLI flags       |
| POST   | `/stop`                   | Stop whatever trajectory is currently running   |
| GET    | `/status`                 | Currently running trajectory, if any            |

Starting a trajectory stops any trajectory already running, since two scripts
can't share a `cmd_vel` topic.

JSON body keys map 1:1 to CLI flags (`"angular_speed": 0.5` -> `--angular-speed 0.5`,
booleans become a flag or nothing). See `--help` on a script for its full flag
list, e.g. `scripts/circle_walk.py --help` or `scripts/square_walk.py --help`.

### Example

```bash
curl -X POST localhost:8080/trajectories/circle/run \
     -d '{"radius": 1.0, "angular_speed": 0.5, "laps": 2}'

curl -X POST localhost:8080/trajectories/square/run \
     -d '{"side": 1.0, "speed": 0.3, "turn_speed": 0.5, "laps": 1}'

curl localhost:8080/status

curl -X POST localhost:8080/stop
```

## Safety

All trajectories are open loop (no odometry feedback) and share a few guards
in `scripts/trajectory_common.py`:

- **Speed caps**: refuses to run if the commanded linear or angular speed
  exceeds `--max-linear-speed` (default 1.5 m/s) or `--max-angular-speed`
  (default 2.0 rad/s). Raise them explicitly if you really mean it.
- **NaN/Infinity rejected**: every numeric flag is checked for a finite value.
- **Ramp-up**: `--ramp-time` (default 1.0s) linearly ramps from 0 to full
  speed at start instead of commanding it instantly from a standstill.
- **Always stops on exit**: Ctrl+C, `/stop`, or `--duration`/`--laps` running
  out all publish a zero `Twist` before the script exits.

## Adding a new trajectory

1. Add a script under `scripts/` following `circle_walk.py`'s or
   `square_walk.py`'s pattern: plain `rclpy`, flags as `--snake-case-with-dashes`,
   call `trajectory_common.add_common_args`/`validate_common_args`/`init`/`run`
   for the shared stop condition, rate, topic, ramp-up, and safety caps.
2. Add one line to `TRAJECTORIES` in `scripts/main.py`.
