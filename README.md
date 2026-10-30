# Robot tests

Same trajectories on Go2 and RoverX through one HTTP API (port 8080), with optional bag
recording. Build the images once (and after changing a Dockerfile):

```bash
make -C dockerfiles
```

## Trajectories

Defaults (identical for both robots):

| Trajectory | Keys and defaults                                     | Forward               | Turn                 | 1 lap (timed) |
|------------|-------------------------------------------------------|-----------------------|----------------------|---------------|
| `circle`   | `radius` 1.0 m, `angular_speed` 0.3 rad/s             | 0.3 m/s (r x angular) | 0.3 rad/s            | 21.9 s        |
| `figure8`  | `radius` 1.0 m, `angular_speed` 0.3 rad/s             | 0.3 m/s (r x angular) | ±0.3 rad/s           | 42.9 s        |
| `square`   | `side` 1.0 m, `speed` 0.3 m/s, `turn_speed` 0.3 rad/s | 0.3 m/s on sides      | 0.3 rad/s at corners | 35.3 s        |

Common keys: `robot` (`go2` / `roverx`), `laps` or `duration` (s; neither = run until
`/stop`), `clockwise` false, `use_odom` false, `ramp_time` 1 s, `rate` 10 Hz.
No speed caps: any speed you request is sent, unless you add `max_linear_speed` /
`max_angular_speed` to that request.

```bash
curl -X POST localhost:8080/trajectories/circle/run  -d '{"robot": "roverx", "laps": 1}' | jq
curl -X POST localhost:8080/trajectories/square/run  -d '{"robot": "go2", "laps": 1}' | jq
curl -X POST localhost:8080/trajectories/figure8/run -d '{"robot": "roverx", "laps": 1}' | jq

curl -X POST localhost:8080/trajectories/circle/run \
     -d '{"robot": "roverx", "radius": 2.0, "angular_speed": 0.4, "laps": 1}' | jq   # 0.8 m/s
curl -X POST localhost:8080/trajectories/square/run \
     -d '{"robot": "go2", "side": 2.0, "speed": 0.8, "turn_speed": 0.8, "laps": 1}' | jq

curl -X POST localhost:8080/stop | jq
curl localhost:8080/status | jq                  # last.exit_code 0 = completed
docker logs --tail 20 robot-trajectories
```

- **Timed (default)**: open loop, so both robots get exactly the same `/cmd_vel` stream.
- **`"use_odom": true`**: laps, sides and corners are measured from the robot's odometry.
- **`"record_bag": true, "tag": "exp1"`**: records that run's commands and pose estimates to
  `bags/<time>_<robot>_<trajectory>_exp1`.

Details (command profile, lap logic, adding a trajectory):
[dockerfiles/robot-trajectories/README.md](dockerfiles/robot-trajectories/README.md).

## Go2

Robot PC `192.168.123.161`, ROS domain 0, CycloneDDS. Locally, everything runs on domain 10
through `go2-domain-bridge`. Below, `eno1` is the laptop port wired to the Go2.

```bash
sudo ip addr flush dev eno1 && sudo ip addr add 192.168.123.51/24 dev eno1
./start_go2.sh --robot-iface eno1
./stop_go2.sh
./run_ros_dev.sh --ros-domain-id 10 /scripts/teleop.sh   # --speed 0.3 --turn 0.5
```

## RoverX

Robot PC `192.168.5.2` (`ssh roverx2@192.168.5.2`), ROS domain 0, FastDDS. Below,
`enp3s0` is the laptop port wired to the RoverX.

```bash
sudo ip addr flush dev enp3s0 && sudo ip addr add 192.168.5.51/24 dev enp3s0
./start_roverx.sh --iface enp3s0
./stop_roverx.sh
```

Each run turns the lidar on (it carries the IMU used for heading) and powers the wheels,
then stays still for 2 s. All of this happens before the timed motion starts, about 3-6 s
in total. Wheel power is cut at the end. Keep the radius at 1 m or more: tighter circles
make the inner wheels stop or reverse (skid-steer).

Floor test protocol (T1 straight-line baseline, T2 acceleration, T3 stopping) runs in its own
stack, `./start_roverx_tests.sh` (API on :8090, stop this one first):
[dockerfiles/roverx-tests/README.md](dockerfiles/roverx-tests/README.md).

Dev shell, teleop, motion diagnostic (scripts mounted at `/scripts`):

```bash
./run_ros_dev.sh --dds fast --iface enp3s0 --ros-domain-id 0
./run_ros_dev.sh --dds fast --iface enp3s0 --ros-domain-id 0 /scripts/roverx_teleop.sh   # --speed 0.15 --turn 0.4
./run_ros_dev.sh --dds fast --iface enp3s0 --ros-domain-id 0 /scripts/roverx_diag.py straight   # or: spin
```

Use `roverx_teleop.sh`, not `teleop.sh`. The robot stops after 0.5 s without `/cmd_vel`,
so this script keeps repeating the current key. Ctrl+C stops the robot and cuts wheel power.

Manual services (lidar is in standby after every boot):

```bash
ros2 service call /toggle_lidar  std_srvs/srv/SetBool "{data: true}"
ros2 service call /toggle_wheels std_srvs/srv/SetBool "{data: true}"   # false cuts power
```

The robot runs a patched copy of its workspace: real m/s and rad/s, a 0.5 s command timeout,
measured `/odom`, and driver speed limits set out of reach (10 m/s, 20 rad/s). What changed
and how to revert: [docs/roverx2-robot-changes.md](docs/roverx2-robot-changes.md).
