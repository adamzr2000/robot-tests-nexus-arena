#!/usr/bin/env python3
"""Drive the robot in a circle by publishing a constant Twist.

--radius and --angular-speed set the circle; linear speed is derived from
them (v = radius * angular_speed). Stop with --duration or --laps, or omit
both to run until Ctrl+C. --laps is timed from the commanded turn rate by
default; with --use-odom it is measured as total heading change from
odometry, which also accounts for lag and slip. A zero Twist is published
on exit.

Examples:
  python3 circle_walk.py --robot go2 --radius 1.0 --duration 30
  python3 circle_walk.py --robot go2 --radius 0.5 --angular-speed 0.8 --clockwise --laps 2
  python3 circle_walk.py --robot go2 --radius 1.0 --laps 2 --use-odom
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist

import trajectory_common as tc


class CircleWalk(tc.TrajectoryNode):
    def __init__(self, args, measure_laps):
        super().__init__("circle_walk", args, args.odom_topic if measure_laps else None)
        self._direction = -1.0 if args.clockwise else 1.0
        self._linear_speed = args.radius * args.angular_speed
        self._angular_speed = self._direction * args.angular_speed
        if measure_laps:
            self._target = args.laps * 2 * math.pi
            self.deadline = tc.ODOM_TIME_MARGIN * tc.ramped_time(
                self._target / args.angular_speed, args.ramp_time)
        self.get_logger().info(
            f"Circling: radius={args.radius:g} m, linear={self._linear_speed:.3f} m/s, "
            f"angular={self._angular_speed:.3f} rad/s, rate={args.rate:g} Hz, topic='{args.topic}'"
        )

    def laps_turned(self):
        return self._direction * self.odom.heading / (2 * math.pi)

    def progress(self):
        return f"{self.laps_turned():.2f} laps"

    def twist(self, _t):
        if self.odom and self._direction * self.odom.heading >= self._target:
            self.finish(f"Measured {self.laps_turned():.2f} laps")
            return None
        msg = Twist()
        msg.linear.x = self._linear_speed
        msg.angular.z = self._angular_speed
        return msg


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--radius", type=float, default=1.0,
                         help="Circle radius in meters (default: 1.0)")
    parser.add_argument("--angular-speed", type=float, default=0.3,
                         help="Turn rate in rad/s (default: 0.3, i.e. 0.3 m/s at the default radius)")
    parser.add_argument("--clockwise", action="store_true",
                         help="Turn clockwise instead of the default counter-clockwise")
    tc.add_common_args(parser)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    tc.require_positive("--radius", args.radius)
    tc.require_positive("--angular-speed", args.angular_speed)
    tc.validate_common_args(args)
    tc.check_speed_caps(args.radius * args.angular_speed, args.angular_speed,
                        args.max_linear_speed, args.max_angular_speed)

    measure_laps = args.use_odom and args.laps is not None
    duration = args.duration
    if args.laps is not None and not measure_laps:
        duration = tc.ramped_time(args.laps * 2 * math.pi / args.angular_speed, args.ramp_time)

    tc.init()
    try:
        node = CircleWalk(args, measure_laps)
        if measure_laps:
            node.get_logger().info(f"--laps {args.laps:g} measured from '{args.odom_topic}' "
                                   f"(safety limit {node.deadline:.1f}s)")
        elif args.laps is not None:
            node.get_logger().info(f"--laps {args.laps:g} -> timed {duration:.1f}s "
                                   "(open loop: actual laps will drift with lag and slip)")
        return tc.run(node, duration)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
