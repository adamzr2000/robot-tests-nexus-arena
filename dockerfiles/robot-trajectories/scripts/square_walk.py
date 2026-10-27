#!/usr/bin/env python3
"""Drive the robot in a square: straight, pivot 90 degrees, repeat 4 times.

Open loop, same as circle_walk.py: there's no odometry feedback, so each
side and turn is timed from the commanded speed (side length / speed,
90deg / turn-speed), not measured from actual motion. Expect real corners
to drift from exactly 90 degrees with slip or lag, same caveat as
circle_walk.py's --laps.

  - --side is the length of each side in meters; --speed is the forward
    speed while driving a side; --turn-speed is the pivot rate at corners.
  - --clockwise turns right at each corner instead of the default left.
  - --duration/--laps/--ramp-time/--max-*-speed/--topic/--rate behave the
    same as in circle_walk.py (one lap here = all 4 sides and corners).
  - Transitions between a straight side and a corner turn are instantaneous
    (no ramp), only the very start of the run is ramped. Standard for
    velocity-tracking locomotion/drive controllers, which are built to
    track changing cmd_vel, unlike the initial standstill-to-motion step.

Examples:
  python3 square_walk.py --side 1.0
  python3 square_walk.py --side 0.5 --speed 0.2 --turn-speed 0.4 --laps 2
  python3 square_walk.py --side 1.0 --topic /go2/cmd_vel
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import trajectory_common as tc


class SquareWalk(Node):
    def __init__(self, side, speed, turn_speed, clockwise, rate_hz, topic, ramp_time):
        super().__init__("square_walk")
        self._speed = speed
        self._turn_speed = -turn_speed if clockwise else turn_speed
        self._straight_time = side / speed
        self._turn_time = (math.pi / 2) / turn_speed
        self._phase_time = self._straight_time + self._turn_time  # one side + its corner
        self._ramp_time = ramp_time
        self._pub = self.create_publisher(Twist, topic, 10)
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"Square: side={side:g} m, speed={speed:.3f} m/s, turn_speed={self._turn_speed:.3f} rad/s, "
            f"{self._straight_time:.2f}s/side, {self._turn_time:.2f}s/corner, "
            f"rate={rate_hz:g} Hz, topic='{topic}'"
        )

    def _ramp_scale(self, elapsed):
        if self._ramp_time <= 0:
            return 1.0
        return min(1.0, elapsed / self._ramp_time)

    def _tick(self):
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        in_turn = (elapsed % self._phase_time) >= self._straight_time
        scale = self._ramp_scale(elapsed)

        msg = Twist()
        if in_turn:
            msg.angular.z = self._turn_speed * scale
        else:
            msg.linear.x = self._speed * scale
        self._pub.publish(msg)

    def stop(self):
        self._pub.publish(Twist())  # all-zero Twist


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--side", type=float, default=1.0,
                         help="Side length in meters (default: 1.0)")
    parser.add_argument("--speed", type=float, default=0.3,
                         help="Forward speed while driving a side, in m/s (default: 0.3)")
    parser.add_argument("--turn-speed", type=float, default=0.5,
                         help="Pivot rate at each corner, in rad/s (default: 0.5)")
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

    straight_time = args.side / args.speed
    turn_time = (math.pi / 2) / args.turn_speed
    lap_time = 4 * (straight_time + turn_time)

    duration = args.duration
    if args.laps is not None:
        duration = args.laps * lap_time

    tc.init()
    try:
        node = SquareWalk(args.side, args.speed, args.turn_speed, args.clockwise,
                           args.rate, args.topic, args.ramp_time)
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
