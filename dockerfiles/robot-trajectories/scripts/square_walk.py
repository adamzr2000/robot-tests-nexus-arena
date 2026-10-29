#!/usr/bin/env python3
"""Drive the robot in a square: straight, pivot 90 degrees, repeat.

--side is the length of each side, --speed the forward speed along it and
--turn-speed the pivot rate at corners. One lap is 4 sides and 4 corners.

By default sides and corners are timed from the commanded speeds. With
--use-odom, sides are measured as distance walked and corners as heading
change from odometry. Corner targets are absolute (k * 90 degrees from the
start heading), so heading error doesn't accumulate around the square; in
this mode --laps rounds up to whole sides.

Examples:
  python3 square_walk.py --robot go2 --side 1.0
  python3 square_walk.py --robot go2 --side 0.5 --speed 0.2 --turn-speed 0.4 --laps 2
  python3 square_walk.py --robot go2 --side 1.0 --laps 1 --use-odom
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist

import trajectory_common as tc


class SquareWalk(tc.TrajectoryNode):
    def __init__(self, args):
        super().__init__("square_walk", args, args.odom_topic if args.use_odom else None)
        self._side = args.side
        self._speed = args.speed
        self._direction = -1.0 if args.clockwise else 1.0
        self._turn_speed = self._direction * args.turn_speed
        self._straight_time = args.side / args.speed
        self._phase_time = self._straight_time + (math.pi / 2) / args.turn_speed
        self._corners = 0
        self._turning = False
        self._side_start = None
        self._target_corners = None
        if self.odom and args.laps is not None:
            self._target_corners = math.ceil(args.laps * 4 - 1e-9)
            self.deadline = tc.ODOM_TIME_MARGIN * tc.ramped_time(
                self._target_corners * self._phase_time, args.ramp_time)
        self.get_logger().info(
            f"Square: side={args.side:g} m, speed={args.speed:.3f} m/s, "
            f"turn_speed={self._turn_speed:.3f} rad/s, rate={args.rate:g} Hz, topic='{args.topic}', "
            + (f"measured from '{args.odom_topic}'" if self.odom else
               f"timed {self._straight_time:.2f}s/side, {self._phase_time - self._straight_time:.2f}s/corner")
        )

    def progress(self):
        return f"{self._corners}/{self._target_corners} corners"

    def twist(self, t):
        in_turn = self._measured_turn() if self.odom else (t % self._phase_time) >= self._straight_time
        if self.finished:
            return None
        msg = Twist()
        if in_turn:
            msg.angular.z = self._turn_speed
        else:
            msg.linear.x = self._speed
        return msg

    def _measured_turn(self):
        odom = self.odom
        if self._side_start is None:
            self._side_start = (odom.x, odom.y)
        if not self._turning:
            walked = math.hypot(odom.x - self._side_start[0], odom.y - self._side_start[1])
            self._turning = walked >= self._side
        elif self._direction * odom.heading >= (self._corners + 1) * math.pi / 2:
            self._corners += 1
            self._turning = False
            self._side_start = (odom.x, odom.y)
            if self._target_corners is not None and self._corners >= self._target_corners:
                self.finish(f"Measured {self._corners} corners")
        return self._turning


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--side", type=float, default=1.0,
                         help="Side length in meters (default: 1.0)")
    parser.add_argument("--speed", type=float, default=0.3,
                         help="Forward speed while driving a side, in m/s (default: 0.3)")
    parser.add_argument("--turn-speed", type=float, default=0.3,
                         help="Pivot rate at each corner, in rad/s (default: 0.3)")
    parser.add_argument("--clockwise", action="store_true",
                         help="Turn right at each corner instead of the default left")
    tc.add_common_args(parser)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    tc.require_positive("--side", args.side)
    tc.require_positive("--speed", args.speed)
    tc.require_positive("--turn-speed", args.turn_speed)
    tc.validate_common_args(args)
    tc.check_speed_caps(args.speed, args.turn_speed, args.max_linear_speed, args.max_angular_speed)

    duration = args.duration
    if args.laps is not None and not args.use_odom:
        lap_time = 4 * (args.side / args.speed + (math.pi / 2) / args.turn_speed)
        duration = tc.ramped_time(args.laps * lap_time, args.ramp_time)

    tc.init()
    try:
        node = SquareWalk(args)
        if args.laps is not None:
            node.get_logger().info(
                f"--laps {args.laps:g} measured, safety limit {node.deadline:.1f}s" if node.odom else
                f"--laps {args.laps:g} -> timed {duration:.1f}s (open loop: corners will drift with lag and slip)")
        return tc.run(node, duration)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
