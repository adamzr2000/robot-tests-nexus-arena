#!/usr/bin/env python3
"""Drive the robot in a figure eight: two tangent circles of the same radius.

Starting at the crossing point, the robot drives one full loop turning left,
then one full loop turning right, back to the start. Forward speed is
constant (v = radius * angular_speed); only the turn direction flips, at the
crossing point. One lap is the whole eight (both loops). --clockwise starts
with the right-hand loop.

Stop with --duration or --laps, or omit both to run until Ctrl+C. By default
loops are timed from the commanded turn rate. With --use-odom each loop ends
when the measured heading reaches an absolute target (+360 degrees, then back
to 0), so heading error doesn't accumulate; --laps then rounds up to whole
loops. A zero Twist is published on exit.

Examples:
  python3 figure8_walk.py --robot go2 --laps 1
  python3 figure8_walk.py --robot roverx --radius 1.5 --angular-speed 0.2 --laps 2
  python3 figure8_walk.py --robot roverx --laps 1 --use-odom
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist

import trajectory_common as tc


class Figure8Walk(tc.TrajectoryNode):
    def __init__(self, args, measure_laps):
        super().__init__("figure8_walk", args, args.odom_topic if measure_laps else None)
        self._first = -1.0 if args.clockwise else 1.0
        self._linear_speed = args.radius * args.angular_speed
        self._angular_speed = args.angular_speed
        self._loop_time = 2 * math.pi / args.angular_speed
        self._loops = 0
        self._target_loops = None
        if measure_laps:
            self._target_loops = math.ceil(args.laps * 2 - 1e-9)
            self.deadline = tc.ODOM_TIME_MARGIN * tc.ramped_time(
                self._target_loops * self._loop_time, args.ramp_time)
        self.get_logger().info(
            f"Figure eight: radius={args.radius:g} m, linear={self._linear_speed:.3f} m/s, "
            f"angular=+/-{self._angular_speed:.3f} rad/s (first loop "
            f"{'right' if args.clockwise else 'left'}), rate={args.rate:g} Hz, topic='{args.topic}'"
        )

    def progress(self):
        return f"{self._loops}/{self._target_loops} loops"

    def twist(self, t):
        loop = self._measured_loop() if self.odom else int(t // self._loop_time)
        if self.finished:
            return None
        msg = Twist()
        msg.linear.x = self._linear_speed
        msg.angular.z = (self._first if loop % 2 == 0 else -self._first) * self._angular_speed
        return msg

    def _measured_loop(self):
        # Even loops turn the first way up to +360 degrees, odd loops back to 0.
        heading = self._first * self.odom.heading
        done = heading >= 2 * math.pi if self._loops % 2 == 0 else heading <= 0.0
        if done:
            self._loops += 1
            if self._target_loops is not None and self._loops >= self._target_loops:
                self.finish(f"Measured {self._loops} loops")
        return self._loops


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--radius", type=float, default=1.0,
                         help="Radius of each loop in meters (default: 1.0)")
    parser.add_argument("--angular-speed", type=float, default=0.3,
                         help="Turn rate in rad/s (default: 0.3, i.e. 0.3 m/s at the default radius)")
    parser.add_argument("--clockwise", action="store_true",
                         help="Start with the right-hand (clockwise) loop instead of the left one")
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
        duration = tc.ramped_time(args.laps * 2 * 2 * math.pi / args.angular_speed, args.ramp_time)

    tc.init()
    try:
        node = Figure8Walk(args, measure_laps)
        if measure_laps:
            node.get_logger().info(f"--laps {args.laps:g} measured from '{args.odom_topic}' "
                                   f"(safety limit {node.deadline:.1f}s)")
        elif args.laps is not None:
            node.get_logger().info(f"--laps {args.laps:g} -> timed {duration:.1f}s "
                                   "(open loop: actual loops will drift with lag and slip)")
        return tc.run(node, duration)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
