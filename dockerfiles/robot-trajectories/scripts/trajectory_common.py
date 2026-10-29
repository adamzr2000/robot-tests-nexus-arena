"""Shared helpers for trajectory scripts: flags, robot profiles (see
robot_profiles.py), safety caps, odometry tracking and the common run loop. Imported by circle_walk.py,
square_walk.py, etc.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool

from robot_profiles import NO_PROFILE, ROBOT_PROFILES

# Safety ceilings applied to every trajectory unless the robot profile sets
# lower ones or they're explicitly raised via --max-linear-speed/
# --max-angular-speed. Conservative on purpose: there is no obstacle or
# stability feedback.
DEFAULT_MAX_LINEAR_SPEED = 1.5   # m/s
DEFAULT_MAX_ANGULAR_SPEED = 2.0  # rad/s
MAX_RATE_HZ = 50.0               # sanity cap on publish rate, not a robot limit

ODOM_WAIT_S = 10.0      # refuse to move if no odometry arrives within this
ODOM_STALE_S = 1.0      # stop if odometry goes silent mid-run for this long
ODOM_TIME_MARGIN = 2.0  # odometry-measured runs still stop at this x the timed estimate
ODOM_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
IMU_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)

SERVICE_WAIT_S = 15.0     # discovering a service on a big robot graph takes up to several s
SERVICE_TIMEOUT_S = 10.0  # per attempt; turning the RoverX lidar on takes ~4.5 s
SERVICE_ATTEMPTS = 2      # a reply can occasionally be lost right after discovery
SETTLE_S = 2.0            # still after on_start: motors power up, IMU bias is measured
STOP_REPEATS = 5          # zero Twists sent on exit, in case one is lost


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
    """Flags shared by every trajectory script: robot profile, stop condition,
    odometry, rate, topics, ramps, and safety caps."""
    parser.add_argument("--robot", choices=sorted(ROBOT_PROFILES), default=None,
                         help="Robot profile: topics, speed caps and robot preparation "
                              "(see robot_profiles.py)")
    stop_group = parser.add_mutually_exclusive_group()
    stop_group.add_argument("--duration", type=float, default=None,
                             help="Stop automatically after N seconds (default: run until Ctrl+C)")
    stop_group.add_argument("--laps", type=float, default=None,
                             help="Stop after N full laps of the shape; timed from the commanded "
                                  "speed unless --use-odom")
    parser.add_argument("--use-odom", action="store_true",
                         help="Measure laps, sides and corners from odometry instead of timing them")
    parser.add_argument("--rate", type=float, default=10.0,
                         help="Publish rate in Hz (default: 10)")
    parser.add_argument("--topic", type=str, default=None,
                         help=f"Velocity command topic (default: from --robot, else {NO_PROFILE['cmd_vel']})")
    parser.add_argument("--odom-topic", type=str, default=None,
                         help="nav_msgs/Odometry topic for --use-odom (default: from --robot)")
    parser.add_argument("--ramp-time", type=float, default=1.0,
                         help="Seconds of linear ramp from 0 to full speed at start and, on timed "
                              "runs, back to 0 at the end; timed laps are lengthened so the commanded "
                              "distance/angle stays exact (default: 1.0; 0 disables)")
    parser.add_argument("--max-linear-speed", type=float, default=None,
                         help=f"Safety cap on linear speed in m/s; refuses to run above it unless "
                              f"raised explicitly (default: from --robot, else {DEFAULT_MAX_LINEAR_SPEED:g})")
    parser.add_argument("--max-angular-speed", type=float, default=None,
                         help=f"Safety cap on angular speed in rad/s; refuses to run above it unless "
                              f"raised explicitly (default: from --robot, else {DEFAULT_MAX_ANGULAR_SPEED:g})")


def validate_common_args(args):
    """Validate shared flags and fill topics and caps from the --robot profile."""
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

    profile = ROBOT_PROFILES[args.robot] if args.robot else NO_PROFILE
    args.profile = profile
    if args.max_linear_speed is None:
        args.max_linear_speed = profile.get("max_linear_speed", DEFAULT_MAX_LINEAR_SPEED)
    if args.max_angular_speed is None:
        args.max_angular_speed = profile.get("max_angular_speed", DEFAULT_MAX_ANGULAR_SPEED)
    require_finite("--max-linear-speed", args.max_linear_speed)
    require_finite("--max-angular-speed", args.max_angular_speed)
    args.topic = args.topic or profile["cmd_vel"]
    args.odom_topic = args.odom_topic or profile["odom"]
    if not args.topic:
        sys.exit(f"ERROR: robot profile '{args.robot}' has no cmd_vel topic yet; pass --topic")
    if args.use_odom and not args.odom_topic:
        sys.exit("ERROR: --use-odom needs an odometry topic: pass --odom-topic or a --robot "
                 "profile that has one")


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)


def ramp_scale(elapsed, ramp_time, end=None):
    """Speed factor at `elapsed`: linear 0->1 over the first `ramp_time` and,
    when the run's `end` is known, 1->0 over the last `ramp_time`."""
    if ramp_time <= 0:
        return 1.0
    scale = min(1.0, elapsed / ramp_time)
    if end is not None:
        scale = min(scale, (end - elapsed) / ramp_time)
    return max(0.0, scale)


def full_speed_time(elapsed, ramp_time, end=None):
    """Seconds at full speed that cover the same ground as `elapsed` seconds
    of the ramped run (the integral of ramp_scale)."""
    if ramp_time <= 0:
        return elapsed
    if end is not None and end < 2 * ramp_time:  # ramps meet in the middle
        half = end / 2
        if elapsed <= half:
            return elapsed * elapsed / (2 * ramp_time)
        left = end - min(elapsed, end)
        return (2 * half * half - left * left) / (2 * ramp_time)
    up = elapsed * elapsed / (2 * ramp_time) if elapsed < ramp_time else elapsed - ramp_time / 2
    if end is None:
        return up
    down = max(0.0, min(elapsed, end) - (end - ramp_time))
    return min(up, end - ramp_time / 2) - down * down / (2 * ramp_time)


def ramped_time(full_time, ramp_time):
    """Run length with ramps at both ends that covers `full_time` seconds of
    full-speed motion (inverse of full_speed_time at the end of the run)."""
    if ramp_time <= 0:
        return full_time
    if full_time >= ramp_time:
        return full_time + ramp_time
    return 2 * math.sqrt(ramp_time * full_time)


class ImuHeading:
    """Cumulative heading (rad) integrated from an IMU's z gyro. Samples until
    calibrate() measure the gyro bias, so the robot must be still until then."""

    def __init__(self, node, topic, z_sign=1.0):
        self.heading = 0.0
        self._z_sign = z_sign
        self._bias_samples = []
        self._bias = None
        self._last_rx = None
        self._sub = node.create_subscription(Imu, topic, self._on_imu, IMU_QOS)

    def _on_imu(self, msg):
        now = time.monotonic()
        rate = self._z_sign * msg.angular_velocity.z
        if self._bias is None:
            self._bias_samples.append(rate)
        elif self._last_rx is not None:
            self.heading += (rate - self._bias) * (now - self._last_rx)
        self._last_rx = now

    def calibrate(self):
        self._bias = sum(self._bias_samples) / len(self._bias_samples) if self._bias_samples else 0.0

    @property
    def ready(self):
        return self._last_rx is not None

    def stale(self):
        return time.monotonic() - self._last_rx > ODOM_STALE_S


class OdomTracker:
    """Cumulative unwrapped heading (rad, since the first sample) and latest
    x/y position from a nav_msgs/Odometry topic. With `imu`, heading comes
    from that IMU's gyro instead of the odometry's orientation."""

    def __init__(self, node, topic, imu=None, imu_z_sign=1.0):
        self._heading = 0.0
        self.x = 0.0
        self.y = 0.0
        self._last_yaw = None
        self._last_rx = None
        self._imu = ImuHeading(node, imu, imu_z_sign) if imu else None
        self._sub = node.create_subscription(Odometry, topic, self._on_odom, ODOM_QOS)

    def _on_odom(self, msg):
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        if self._last_yaw is not None:
            self._heading += wrap_angle(yaw - self._last_yaw)
        self._last_yaw = yaw
        self.x = pose.position.x
        self.y = pose.position.y
        self._last_rx = time.monotonic()

    @property
    def heading(self):
        return self._imu.heading if self._imu else self._heading

    def calibrate(self):
        if self._imu:
            self._imu.calibrate()

    @property
    def ready(self):
        return self._last_rx is not None and (self._imu is None or self._imu.ready)

    def stale(self):
        return time.monotonic() - self._last_rx > ODOM_STALE_S or (self._imu is not None and self._imu.stale())


class TrajectoryNode(Node):
    """Publishes the subclass's `twist(t)` at --rate once started, ramped up
    over --ramp-time and, when the end time is known (timed runs), ramped down
    over the last --ramp-time. `t` is full_speed_time, so timed phases stay
    exact through the ramps. twist() returns None after calling finish()."""

    def __init__(self, name, args, odom_topic=None):
        super().__init__(name)
        profile = args.profile
        self.odom = (OdomTracker(self, odom_topic, profile.get("imu"), profile.get("imu_z_sign", 1.0))
                     if odom_topic else None)
        self.on_start = profile.get("on_start", [])
        self.on_stop = profile.get("on_stop", [])
        self.deadline = None
        self.finished = False
        self._pub = self.create_publisher(Twist, args.topic, 10)
        self._period = 1.0 / args.rate
        self._ramp_time = args.ramp_time
        self._start = None
        self._end = None
        self._timer = None

    def start(self, end=None):
        self._start = self.get_clock().now()
        self._end = end
        self._timer = self.create_timer(self._period, self._tick)

    def elapsed(self):
        return (self.get_clock().now() - self._start).nanoseconds / 1e9

    def finish(self, reason):
        self.finished = True
        self.get_logger().info(f"{reason}, stopping.")

    def progress(self):
        return "unknown progress"

    def _tick(self):
        if self.finished:
            return
        elapsed = self.elapsed()
        msg = self.twist(full_speed_time(elapsed, self._ramp_time, self._end))
        if msg is None:
            return
        scale = ramp_scale(elapsed, self._ramp_time, self._end)
        msg.linear.x *= scale
        msg.linear.y *= scale
        msg.angular.z *= scale
        self._pub.publish(msg)

    def stop(self):
        # Halt the twist timer first: on_stop service calls spin the node,
        # which would otherwise let it publish motion after the zeros.
        self.finished = True
        if self._timer is not None:
            self._timer.cancel()
        for _ in range(STOP_REPEATS):
            self._pub.publish(Twist())
            time.sleep(0.05)
        self.call_services(self.on_stop)

    def call_services(self, calls, stop_at_failure=False):
        """Call each (std_srvs/SetBool service, value); True if all succeeded."""
        ok = True
        for name, value in calls:
            if not self._call_service(name, value):
                ok = False
                if stop_at_failure:
                    break
        return ok

    def _call_service(self, name, value):
        log = self.get_logger()
        client = self.create_client(SetBool, name)
        try:
            if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
                log.error(f"{name} not found within {SERVICE_WAIT_S:g}s")
                return False
            for _ in range(SERVICE_ATTEMPTS):
                future = client.call_async(SetBool.Request(data=value))
                rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT_S)
                if future.done():
                    result = future.result()
                    log_fn = log.info if result.success else log.error
                    log_fn(f"{name} {value}: {result.message}")
                    return result.success
                client.remove_pending_request(future)
                log.warning(f"{name} {value}: no reply within {SERVICE_TIMEOUT_S:g}s")
            return False
        finally:
            self.destroy_client(client)


def init():
    """rclpy.init with its own SIGINT handling disabled: by default it tears
    down the context as part of handling the signal, which races with run()
    below trying to publish a stop command through that same context. With
    it off, SIGINT just raises the normal Python KeyboardInterrupt and run()
    controls the shutdown order, so the stop command actually gets sent."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)


def _wait_for_odom(node):
    deadline = time.monotonic() + ODOM_WAIT_S
    while rclpy.ok() and not node.odom.ready and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.odom.ready


def _spin_for(node, seconds):
    end = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def run(node, duration=None):
    """Prepare the robot (profile on_start services), start `node` and spin
    until it finishes, `duration` seconds pass (None = no limit) or Ctrl+C,
    always publishing stop commands and calling on_stop services on the way
    out. Returns a process exit code."""
    log = node.get_logger()
    try:
        if not node.call_services(node.on_start, stop_at_failure=True):
            log.error("Robot preparation failed, not moving.")
            return 1
        if node.odom and not _wait_for_odom(node):
            log.error(f"No odometry received within {ODOM_WAIT_S:g}s, not moving.")
            return 1
        if node.on_start:
            _spin_for(node, SETTLE_S)
        if node.odom:
            node.odom.calibrate()
        node.start(duration)
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.elapsed()
            if duration is not None and elapsed >= duration:
                log.info(f"Reached stop time ({duration:.1f}s), stopping.")
                break
            if node.deadline is not None and elapsed >= node.deadline:
                log.warning(f"Safety time limit ({node.deadline:.1f}s) reached at "
                            f"{node.progress()}, stopping.")
                return 1
            if node.odom and node.odom.stale():
                log.error(f"Odometry silent for over {ODOM_STALE_S:g}s, stopping.")
                return 1
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception as exc:
            print(f"WARNING: failed to stop the robot cleanly: {exc}", file=sys.stderr)
        node.destroy_node()
    return 0
