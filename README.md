# Robot tests

Build images (first time, or after changing a Dockerfile)
```bash
make -C dockerfiles
```

Trajectories run through one HTTP API (port 8080) for both robots; options in
[dockerfiles/robot-trajectories/README.md](dockerfiles/robot-trajectories/README.md).
By default they are timed (open loop): both robots get exactly the same command
stream, 0.3 m/s and 0.3 rad/s, circle radius 1 m, square side 1 m. Add
`"use_odom": true` to measure laps from the robot's own odometry instead.
Add `"record_bag": true, "tag": "exp1"` to any run to record its commands and pose
estimates to `bags/<time>_<robot>_<trajectory>_exp1`.

```bash
curl -X POST localhost:8080/stop | jq
curl localhost:8080/status | jq
docker logs --tail 20 robot-trajectories
```

## Go2

Network: robot PC is `192.168.123.161`
```bash
sudo ip addr flush dev eno1 && sudo ip addr add 192.168.123.51/24 dev eno1
```

Stack
```bash
./start_go2.sh   # optional: --robot-iface eno1 (wired to the Go2) --iface lo (local DDS)
./stop_go2.sh
```

Teleop
```bash
./run_ros_dev.sh --ros-domain-id 10 /scripts/teleop.sh   # optional: --speed 0.3 --turn 0.5
```

Trajectories
```bash
curl -X POST localhost:8080/trajectories/circle/run -d '{"robot": "go2", "laps": 1}' | jq
curl -X POST localhost:8080/trajectories/square/run -d '{"robot": "go2", "laps": 1}' | jq
curl -X POST localhost:8080/trajectories/circle/run \
     -d '{"robot": "go2", "radius": 1.0, "angular_speed": 0.3, "duration": 30, "clockwise": true}' | jq
```

## RoverX

Network: robot PC is `192.168.5.2` (ROS domain 0, FastDDS)
```bash
sudo ip addr flush dev eno1 && sudo ip addr add 192.168.5.51/24 dev eno1
```

Stack
```bash
./start_roverx.sh   # optional: --iface eno1 --ros-domain-id 0
./stop_roverx.sh
```

Trajectories: same requests with `"robot": "roverx"`. Each run first turns the lidar on
and powers the wheels (~3-6 s, before the timed motion starts) and cuts wheel power at
the end. Caps: 0.6 m/s, 1.0 rad/s. Prefer radius >= 1 m: tighter circles make the inner
wheels stop or reverse (skid-steer).
```bash
curl -X POST localhost:8080/trajectories/circle/run -d '{"robot": "roverx", "laps": 1}' | jq
curl -X POST localhost:8080/trajectories/square/run -d '{"robot": "roverx", "laps": 1}' | jq
```

Dev shell, teleop and motion diagnostic (scripts in
[dockerfiles/ros-humble-dev/scripts](dockerfiles/ros-humble-dev/scripts), mounted at `/scripts`)
```bash
./run_ros_dev.sh --dds fast --iface eno1 --ros-domain-id 0
./run_ros_dev.sh --dds fast --iface eno1 --ros-domain-id 0 /scripts/roverx_teleop.sh   # --speed 0.15 --turn 0.4
./run_ros_dev.sh --dds fast --iface eno1 --ros-domain-id 0 /scripts/roverx_diag.py straight   # or: spin
```
Use `roverx_teleop.sh`, not `teleop.sh`: the robot stops after 0.5 s without `/cmd_vel`,
so it repeats the current key and keeps the wheels powered. Ctrl+C stops and cuts power.

Services (from the dev shell). The lidar starts in standby at every boot.
```bash
ros2 service call /toggle_wheels std_srvs/srv/SetBool "{data: true}"   # false cuts power
ros2 service call /toggle_lidar std_srvs/srv/SetBool "{data: true}"
```

### Robot-side changes

The robot runs a patched, calibrated copy of its workspace (0.5 s command timeout, real
m/s and rad/s, measured `/odom`). What changed, why, and how to revert:
[docs/roverx2-robot-changes.md](docs/roverx2-robot-changes.md).
