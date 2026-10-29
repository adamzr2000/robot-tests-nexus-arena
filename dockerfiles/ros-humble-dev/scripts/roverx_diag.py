#!/usr/bin/env python3
"""RoverX motion diagnostic: drives one fixed command and compares every pose source.

    /scripts/roverx_diag.py straight [--speed 0.2] [--duration 5]
    /scripts/roverx_diag.py spin     [--speed 0.5] [--duration 6.28]

Powers the wheels, waits for them to be ready, drives, stops, cuts wheel power
and prints what each source thinks happened. Compare it with the tape measure.
Ctrl+C stops the robot at any time.
"""
import argparse
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

POWER_SETTLE_S = 2.0
COAST_S = 1.5
CMD_RATE = 20.0
TF_PERIOD_S = 0.1


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Track:
    def __init__(self):
        self.count = 0
        self.start = None
        self.x = self.y = self.yaw = self.heading = 0.0

    def update(self, x, y, yaw):
        self.count += 1
        if self.start is None:
            self.start = (x, y)
        else:
            self.heading += wrap(yaw - self.yaw)
        self.x, self.y, self.yaw = x, y, yaw

    def reset(self):
        self.start = (self.x, self.y) if self.count else None
        self.heading = 0.0
        self.count = 0

    def report(self, name, seconds=None):
        if self.start is None:
            return f"  {name:<16} no data"
        dist = math.hypot(self.x - self.start[0], self.y - self.start[1])
        rate = f"   ({self.count / seconds:5.1f} Hz)" if seconds else ""
        return f"  {name:<16} moved {dist:5.2f} m, turned {math.degrees(self.heading):+7.1f} deg{rate}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["straight", "spin"])
    parser.add_argument("--speed", type=float, help="m/s (straight) or rad/s (spin)")
    parser.add_argument("--duration", type=float, help="seconds")
    args = parser.parse_args()
    straight = args.mode == "straight"
    speed = args.speed if args.speed is not None else (0.2 if straight else 0.5)
    duration = args.duration if args.duration is not None else (5.0 if straight else math.pi / abs(speed))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = rclpy.create_node("roverx_diag")
    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    wheels = node.create_client(SetBool, "/toggle_wheels")
    tf = Buffer()
    TransformListener(tf, node)

    odom, odom_raw, slam = Track(), Track(), Track()
    imu = {"count": 0, "bias": [], "yaw": 0.0, "last": None, "moving": False}
    relays = {"on": None}
    driving = False
    raw_speeds = []

    def on_odom(msg, track):
        p = msg.pose.pose
        track.update(p.position.x, p.position.y, yaw_of(p.orientation))

    def on_raw(msg):
        on_odom(msg, odom_raw)
        if driving:
            raw_speeds.append((msg.twist.twist.linear.x, msg.twist.twist.angular.z))

    def on_imu(msg):
        now = time.monotonic()
        gz = -msg.angular_velocity.z  # the lidar IMU is mounted with z inverted
        imu["count"] += 1
        if imu["moving"] and imu["last"] is not None:
            bias = sum(imu["bias"]) / len(imu["bias"]) if imu["bias"] else 0.0
            imu["yaw"] += (gz - bias) * (now - imu["last"])
        elif not imu["moving"]:
            imu["bias"].append(gz)
        imu["last"] = now

    node.create_subscription(Odometry, "/odom", lambda m: on_odom(m, odom), 10)
    node.create_subscription(Odometry, "/odom_raw", on_raw, 10)
    node.create_subscription(Imu, "/unilidar/imu", on_imu, qos_profile_sensor_data)
    node.create_subscription(Bool, "/relay_status", lambda m: relays.update(on=m.data), 10)

    def spin_for(seconds, cmd=None):
        end = time.monotonic() + seconds
        next_pub = next_tf = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if cmd is not None and now >= next_pub:
                cmd_pub.publish(cmd)
                next_pub = now + 1.0 / CMD_RATE
            if now >= next_tf:
                next_tf = now + TF_PERIOD_S
                try:
                    t = tf.lookup_transform("map", "base_link", Time()).transform
                    slam.update(t.translation.x, t.translation.y, yaw_of(t.rotation))
                except TransformException:
                    pass
            rclpy.spin_once(node, timeout_sec=0.005)

    def set_wheels(on, attempts=3):
        for _ in range(attempts):
            future = wheels.call_async(SetBool.Request(data=on))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if future.done():
                res = future.result()
                if not res.success:
                    print(f"WARNING: /toggle_wheels {on} answered success=false ('{res.message}')")
                return True
            wheels.remove_pending_request(future)
        return False

    def wait_relays(on, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and relays["on"] != on:
            rclpy.spin_once(node, timeout_sec=0.05)
        return relays["on"] == on

    cmd = Twist()
    if straight:
        cmd.linear.x = speed
    else:
        cmd.angular.z = speed
    try:
        if not wheels.wait_for_service(timeout_sec=5.0):
            print("ERROR: /toggle_wheels not found (wrong domain/DDS/interface?)")
            return 1
        if not set_wheels(True):
            print("ERROR: /toggle_wheels did not answer")
            return 1
        if not wait_relays(True):
            print(f"ERROR: wheels not powered (/relay_status: {relays['on']})")
            return 1
        print(f"Wheels powered, settling {POWER_SETTLE_S:.0f} s...")
        spin_for(POWER_SETTLE_S)
        for track in (odom, odom_raw, slam):
            track.reset()
        imu["count"] = 0

        print(f"Driving {args.mode} at {speed} for {duration:.2f} s")
        imu["moving"] = driving = True
        start = time.monotonic()
        spin_for(duration, cmd)
        driving = False
        for _ in range(10):
            cmd_pub.publish(Twist())
            spin_for(0.05)
        spin_for(COAST_S)
        elapsed = time.monotonic() - start
    except KeyboardInterrupt:
        print("Interrupted")
        return 130
    finally:
        for _ in range(10):
            cmd_pub.publish(Twist())
            time.sleep(0.05)
        if wheels.service_is_ready():
            set_wheels(False)
        node.destroy_node()
        rclpy.shutdown()

    if straight:
        print(f"\nCommanded: {speed * duration:.2f} m")
    else:
        print(f"\nCommanded: {math.degrees(speed * duration):.0f} deg")
    print(odom.report("/odom", elapsed))
    print(odom_raw.report("/odom_raw", elapsed))
    print(slam.report("map->base_link"))
    print(f"  {'/unilidar/imu':<16} turned {math.degrees(imu['yaw']):+7.1f} deg"
          f"   ({imu['count'] / elapsed:5.1f} Hz, gyro bias {sum(imu['bias']) / max(len(imu['bias']), 1):+.4f} rad/s)")
    if not imu["count"]:
        print("  No IMU data: the lidar is in standby, so /odom heading and SLAM are blind too. Turn it on with\n"
              "    ros2 service call /toggle_lidar std_srvs/srv/SetBool \"{data: true}\"")
    steady = raw_speeds[len(raw_speeds) // 2:]
    if steady:
        v = sum(s[0] for s in steady) / len(steady)
        w = sum(s[1] for s in steady) / len(steady)
        print(f"  /odom_raw speed  {v:+.3f} m/s, {w:+.3f} rad/s (wheel feedback, second-half average)")
    print("\nNow measure: real distance / angle, and how far the centre of the robot drifted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
