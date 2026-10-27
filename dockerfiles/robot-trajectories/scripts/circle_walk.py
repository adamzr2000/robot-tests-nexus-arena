#!/usr/bin/env python3
"""Drive the robot in a circle by publishing a constant Twist.

ROS2 port of an old ROS1 circle_walk.py, generalized for flexible runs:
  - --radius and --angular-speed are independent; linear speed is derived
    from them (v = radius * angular_speed), so --radius means an actual
    circle radius in meters instead of silently doubling as a speed knob.
  - --clockwise flips direction without touching the radius/speed math.
  - --duration stops the robot automatically after N seconds; --laps stops
    it after an estimated N full revolutions instead (duration = laps * 2pi
    / angular_speed). Both are open loop, no odometry feedback: --laps is a
    command-side estimate of when a lap *should* be done, not a measurement
    of when it actually was, so real completion will drift from it with
    acceleration ramp-up, foot-placement lag, or slip. Omit both to run
    until Ctrl+C. Either way, a zero Twist is published on exit so the
    robot doesn't keep coasting after the script stops.
  - --ramp-time smooths the start from 0 to full speed instead of
    commanding it instantly from a standstill.
  - --max-linear-speed/--max-angular-speed are safety ceilings: the script
    refuses to run if radius * angular-speed or angular-speed alone would
    exceed them, rather than silently sending whatever was asked for.
  - --topic lets this point at whatever /cmd_vel-equivalent a given robot's
    driver exposes (e.g. --topic /go2/cmd_vel), since this container isn't
    tied to one robot.

Examples:
  python3 circle_walk.py --radius 1.0
  python3 circle_walk.py --radius 0.5 --angular-speed 0.8 --clockwise --duration 20
  python3 circle_walk.py --radius 1.0 --laps 2
  python3 circle_walk.py --radius 1.0 --topic /go2/cmd_vel
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import trajectory_common as tc


class CircleWalk(Node):
    def __init__(self, radius, angular_speed, clockwise, rate_hz, topic, ramp_time):
        super().__init__("circle_walk")
        self._linear_speed = radius * angular_speed
        self._angular_speed = -angular_speed if clockwise else angular_speed
        self._ramp_time = ramp_time
        self._pub = self.create_publisher(Twist, topic, 10)
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"Circling: radius={radius:g} m, linear={self._linear_speed:.3f} m/s, "
            f"angular={self._angular_speed:.3f} rad/s, rate={rate_hz:g} Hz, "
            f"topic='{topic}'"
        )

    def _ramp_scale(self):
        if self._ramp_time <= 0:
            return 1.0
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        return min(1.0, elapsed / self._ramp_time)

    def _tick(self):
        scale = self._ramp_scale()
        msg = Twist()
        msg.linear.x = self._linear_speed * scale
        msg.angular.z = self._angular_speed * scale
        self._pub.publish(msg)

    def stop(self):
        self._pub.publish(Twist())  # all-zero Twist


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--radius", type=float, default=1.0,
                         help="Circle radius in meters (default: 1.0)")
    parser.add_argument("--angular-speed", type=float, default=1.0,
                         help="Turn rate in rad/s (default: 1.0)")
    parser.add_argument("--clockwise", action="store_true",
                         help="Turn clockwise instead of the default counter-clockwise")
    tc.add_common_args(parser)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    tc.require_positive("--radius", args.radius)
    tc.require_positive("--angular-speed", args.angular_speed)
    tc.validate_common_args(args)

    linear_speed = args.radius * args.angular_speed
    tc.check_speed_caps(linear_speed, args.angular_speed, args.max_linear_speed, args.max_angular_speed)

    duration = args.duration
    if args.laps is not None:
        duration = args.laps * 2 * math.pi / args.angular_speed

    tc.init()
    try:
        node = CircleWalk(args.radius, args.angular_speed, args.clockwise, args.rate, args.topic, args.ramp_time)
        if args.laps is not None:
            node.get_logger().info(
                f"--laps {args.laps:g} -> estimated duration {duration:.1f}s "
                "(open loop, not measured: actual lap completion will drift from this)"
            )
        tc.run(node, duration)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
