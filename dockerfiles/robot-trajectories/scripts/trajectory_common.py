"""Shared helpers for trajectory scripts: validation, safety caps, and the
common rclpy run loop. Not a script itself, imported by circle_walk.py,
square_walk.py, etc.
"""
import math
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions

# Safety ceilings applied to every trajectory unless explicitly raised via
# --max-linear-speed/--max-angular-speed. Conservative on purpose: these
# scripts run open loop, with no obstacle or stability feedback.
DEFAULT_MAX_LINEAR_SPEED = 1.5   # m/s
DEFAULT_MAX_ANGULAR_SPEED = 2.0  # rad/s
MAX_RATE_HZ = 50.0               # sanity cap on publish rate, not a robot limit


def require_finite(flag, value):
    if not math.isfinite(value):
        sys.exit(f"ERROR: {flag} must be a finite number, got {value}")


def require_positive(flag, value):
    require_finite(flag, value)
    if value <= 0:
        sys.exit(f"ERROR: {flag} must be positive")


def check_speed_caps(linear_speed, angular_speed, max_linear_speed, max_angular_speed):
    if abs(angular_speed) > max_angular_speed:
        sys.exit(f"ERROR: angular speed {angular_speed:.3f} rad/s exceeds --max-angular-speed "
                 f"{max_angular_speed:g}; raise it explicitly if you really mean this")
    if abs(linear_speed) > max_linear_speed:
        sys.exit(f"ERROR: linear speed {linear_speed:.3f} m/s exceeds --max-linear-speed "
                 f"{max_linear_speed:g}; raise it explicitly if you really mean this")


def add_common_args(parser):
    """Flags shared by every trajectory script: stop condition, rate, topic,
    ramp-up, and safety caps. Trajectory-specific flags (radius, side, ...)
    are added separately by each script."""
    stop_group = parser.add_mutually_exclusive_group()
    stop_group.add_argument("--duration", type=float, default=None,
                             help="Stop automatically after N seconds (default: run until Ctrl+C)")
    stop_group.add_argument("--laps", type=float, default=None,
                             help="Stop after an estimated N full laps of the shape (open loop, "
                                  "not measured from actual motion)")
    parser.add_argument("--rate", type=float, default=10.0,
                         help="Publish rate in Hz (default: 10)")
    parser.add_argument("--topic", type=str, default="/cmd_vel",
                         help="Velocity command topic (default: /cmd_vel)")
    parser.add_argument("--ramp-time", type=float, default=1.0,
                         help="Seconds to linearly ramp from 0 up to full speed at start, instead "
                              "of commanding it instantly from a standstill (default: 1.0; 0 disables)")
    parser.add_argument("--max-linear-speed", type=float, default=DEFAULT_MAX_LINEAR_SPEED,
                         help=f"Safety cap on linear speed in m/s; refuses to run above it unless "
                              f"raised explicitly (default: {DEFAULT_MAX_LINEAR_SPEED:g})")
    parser.add_argument("--max-angular-speed", type=float, default=DEFAULT_MAX_ANGULAR_SPEED,
                         help=f"Safety cap on angular speed in rad/s; refuses to run above it unless "
                              f"raised explicitly (default: {DEFAULT_MAX_ANGULAR_SPEED:g})")


def validate_common_args(args):
    require_positive("--rate", args.rate)
    if args.rate > MAX_RATE_HZ:
        sys.exit(f"ERROR: --rate must be <= {MAX_RATE_HZ:g} Hz")
    require_finite("--ramp-time", args.ramp_time)
    if args.ramp_time < 0:
        sys.exit("ERROR: --ramp-time must be >= 0")
    if args.duration is not None:
        require_positive("--duration", args.duration)
    if args.laps is not None:
        require_positive("--laps", args.laps)
    require_finite("--max-linear-speed", args.max_linear_speed)
    require_finite("--max-angular-speed", args.max_angular_speed)


def init():
    """rclpy.init with its own SIGINT handling disabled: by default it tears
    down the context as part of handling the signal, which races with run()
    below trying to publish a stop command through that same context. With
    it off, SIGINT just raises the normal Python KeyboardInterrupt and run()
    controls the shutdown order, so the stop command actually gets sent."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)


def run(node, duration):
    """Spin `node` until `duration` seconds pass (None = forever), always
    publishing a stop command on the way out, however the loop ends."""
    start = node.get_clock().now()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if duration is not None:
                elapsed_s = (node.get_clock().now() - start).nanoseconds / 1e9
                if elapsed_s >= duration:
                    node.get_logger().info(f"Reached estimated stop time ({duration:.1f}s), stopping.")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception as exc:
            print(f"WARNING: failed to publish stop command: {exc}", file=sys.stderr)
        node.destroy_node()
