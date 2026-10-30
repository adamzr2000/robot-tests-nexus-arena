# RoverX 2: changes to the robot's software

What we changed on `roverx2-PC` (192.168.5.2) compared to the manufacturer's
code, why, and how to undo it. Changed 2026-10-28; driver caps raised out of
reach 2026-10-29; friction feed-forward off 2026-10-30.

## How it is deployed

The robot runs everything in one Docker container (`robot_ros2`), started at
boot by `configura_can.service` with `docker-compose` in `~/robot_startup`.
The container mounts a workspace at `/root/roverx2_ws` and runs its `start.sh`.

| Path on the robot           | What it is                                                   |
|-----------------------------|--------------------------------------------------------------|
| `~/roverx2_ws`              | Original workspace, **untouched**                            |
| `~/roverx2_ws_v2`           | Patched copy; the robot runs this one                        |
| `~/robot_startup/.env`      | `ROVERX_WS=/home/roverx2/roverx2_ws_v2` selects the copy     |
| `~/robot_startup/docker-compose.yml` | One line changed: mount source is `${ROVERX_WS:-/home/roverx2/roverx2_ws}`; original kept as `docker-compose.yml.orig` |
| `~/roverx_backups/2026-10-28/` | Tar backups of both original folders                      |

The Docker image and everything outside these folders are unchanged.

```bash
ssh roverx2@192.168.5.2
rm ~/robot_startup/.env && sudo systemctl restart configura_can     # run the original again
echo "ROVERX_WS=/home/roverx2/roverx2_ws_v2" > ~/robot_startup/.env && sudo systemctl restart configura_can
cd ~/roverx2_ws_v2 && git log --oneline && git diff 7dc69a7          # every change since the shipped baseline
```

`~/roverx2_ws_v2` has git history: the shipped baseline, then one commit per step.

## Problems found in the original

1. **Wrong speed units.** The driver converts m/s to motor commands with a
   0.10 m wheel radius and assumes the controller takes wheel rpm. The wheels
   are 12-inch (0.1524 m, per the manufacturer's own readme), and the robot
   actually moved ~0.29x the commanded and reported distance. So `/cmd_vel`
   and `/odom_raw` were ~3.5x off, and the UI and Nav2 limits were tuned
   around that.
2. **No command timeout.** The driver held the last `/cmd_vel` forever: if the
   sender crashed or the network dropped, the robot kept driving.
3. **`/odom` position was not measured.** `imu_to_odom_stable.py` integrated
   the *commanded* speed; only its heading (lidar IMU) was real. SLAM and Nav2
   use this `/odom`.
4. **Turning slip.** 4-wheel skid-steer: wheels slide sideways when turning, so
   the robot turns ~36% of what wheel speeds predict. Physical, not a bug.

## Changes (in `~/roverx2_ws_v2`)

**`src/MotoreCan_ws/src/rover_sim_control/src/rover_driver.cpp`** (rebuilt with
`colcon build --packages-select rover_sim_control`)
- `cmd_timeout` (0.5 s): stops the wheels if `/cmd_vel` goes silent. `<= 0` disables.
- `max_linear` / `max_angular`: commands are clamped to these. NaN/Inf commands are ignored.
  Wheel commands are also saturated at ±3000 controller units (~9 m/s real), only so
  they can't overflow int16 and reverse.
- `wheel_speed_scale`: real wheel speed / nominal, applied to both commands and
  wheel odometry (1.0 = original behaviour). The original friction feed-forward
  is applied in the original units, so motor behaviour at low speed is unchanged.
- `track_width` is now read live, so it can serve as an effective (slip-calibrated) width.
- All of the above can be changed live with `ros2 param set /smart_rover_driver ...`
  (reset on restart). Original kinematics parameters (`wheel_radius 0.10`) untouched.

**`start.sh`**: the driver starts with the calibration, caps set high on
purpose (no practical limit; whoever sends `/cmd_vel` decides the speed), and
the friction feed-forward off
```
-p wheel_speed_scale:=0.29 -p track_width:=1.2 -p max_linear:=10.0 -p max_angular:=20.0 -p friction_ff:=0.0
```
(Caps 0.6 and 1.0 until 2026-10-29. `friction_ff` 0.04 until 2026-10-30: it was added
to every non-zero wheel command, so the robot drove +0.0116 m/s faster than
commanded, +5.8% at 0.2 m/s. The motor controllers already hold speed on their own
Hall-sensor loop, so commanded and driven speed now match; the floor tests confirm it
with the measured wheel speed.)

Wheel speed feedback: each motor controller reports at 10 Hz (CAN `0x1801E604`
front, `0x1801E002` rear); `/odom_raw` is published on every frame (~20 Hz), so the
speed it carries can be up to 0.1 s old. Distances at rest are unaffected; positions
during motion trail the real ones by about speed × 0.05 s.

**`src/unilidar_sdk/.../lidar_rover_x/scripts/imu_to_odom_stable.py`**: `/odom`
position now integrates the measured wheel speed from `/odom_raw` (0 if it goes
stale) instead of the command. Heading is still the IMU (Madgwick filter).

**`src/roverx_slam/config/my_nav_params_real.yaml`**: Nav2 limits in real units:
velocity smoother max 0.5 m/s and 1.0 rad/s (was 1.2 and 4.0), accelerations
1.0 m/s² and 2.0 rad/s² (was 4.0 and 5.0), path follower 0.4 m/s (was 1.2),
rotate-to-heading 1.0 rad/s (was 3.5).

Not changed: relay manager, lidar manager (lidar still starts in standby),
cameras, SLAM, arm, manufacturer UI.

## Calibration

Measured on the lab floor with `/scripts/roverx_diag.py` (laptop repo) and a
tape measure / phone:

| Value                     | How                                                        |
|---------------------------|------------------------------------------------------------|
| `wheel_speed_scale` 0.29  | Real distance / driver distance over 5 straight runs (0.27-0.30) |
| `track_width` 1.2         | 0.44 / 0.36 turning efficiency, checked with spins          |

Result: straight runs within ~5% (limited by measuring), wheel and IMU heading
within ~1-2% (IMU) and ~2-6% (wheels, depends on turn direction).

To recalibrate: run `roverx_diag.py straight --speed 0.3 --duration 10`, then
`wheel_speed_scale = current * real_distance / odom_raw_distance`; spins give
`track_width = current * odom_raw_angle / real_angle`. Try live with
`ros2 param set`, then update `start.sh`.

## Known limits

- Turning in place reaches ~85% of the commanded rate (motors don't hold speed
  under skid load). Timed trajectories show this as error; measured mode
  (`use_odom`, IMU heading) compensates it.
- `/odom` heading reads ~3.5% low (manufacturer's IMU filter); the raw IMU
  integrated directly is ~1%.
- The lidar (and its IMU) starts in standby at boot: `/toggle_lidar true`.
  robot-trajectories does this automatically.
- When the timeout stops the robot, the wheels stay powered (held at zero) until
  a zero `/cmd_vel` or `/toggle_wheels false`.
- Commands now mean real speeds: the manufacturer UI's speed values move the
  robot ~3.5x faster than before, and the driver no longer limits them.
- Nav2's own limits (above) still apply to Nav2, not to `/cmd_vel` from other senders.
