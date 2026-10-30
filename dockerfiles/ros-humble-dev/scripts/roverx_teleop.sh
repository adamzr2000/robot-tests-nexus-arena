#!/usr/bin/env bash
# RoverX keyboard teleop that keeps the wheels powered between key presses.
# Runs inside the ros-dev container:
#   ./run_ros_dev.sh --dds fast --iface eno1 --ros-domain-id 0 /scripts/roverx_teleop.sh
#
# The robot's driver stops if /cmd_vel is silent for 0.5 s, but the keyboard
# teleop only publishes on key presses. So teleop publishes on its own topic and
# a background guard forwards every key to /cmd_vel immediately, repeating the
# current motion at 10 Hz. The guard also re-powers the wheels 1 s after each
# stop (the robot cuts power 3 s after a zero cmd_vel). On exit it sends a few
# zero cmd_vel and cuts wheel power, leaving the robot stopped.
set -eo pipefail

SPEED=0.15
TURN=0.4
TELEOP_TOPIC=/roverx_teleop/cmd_vel

usage() {
  echo "Usage: $0 [--speed <m/s>] [--turn <rad/s>]"
  echo "Defaults: --speed 0.15, --turn 0.4. Quit with Ctrl+C: stops the robot and cuts wheel power."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --speed)   SPEED="${2:?--speed needs a value}"; shift 2 ;;
    --turn)    TURN="${2:?--turn needs a value}";   shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

TELEOP_TOPIC="$TELEOP_TOPIC" python3 - <<'EOF' &
import os
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool

REPEAT_S = 0.1          # the driver stops after 0.5 s without /cmd_vel
REENABLE_DELAY_S = 1.0  # robot cuts power 3 s after a zero cmd_vel
MOVING = 0.01           # same threshold as the robot's rele_manager
STOP_REPEATS = 5

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
signal.signal(signal.SIGTERM, signal.default_int_handler)
node = rclpy.create_node("roverx_teleop_guard")
log = node.get_logger()
client = node.create_client(SetBool, "/toggle_wheels")
cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
latest = Twist()
stops_left = 0
reenable_at = None
pending = None


def moving(msg):
    return abs(msg.linear.x) > MOVING or abs(msg.angular.z) > MOVING


def on_teleop(msg):
    global latest, stops_left, reenable_at
    latest = msg
    cmd_pub.publish(msg)
    if not moving(msg):
        stops_left = STOP_REPEATS
        reenable_at = time.monotonic() + REENABLE_DELAY_S


def request_wheels(on):
    global pending
    pending = client.call_async(SetBool.Request(data=on))


node.create_subscription(Twist, os.environ["TELEOP_TOPIC"], on_teleop, 10)
powered = False
warn_at = time.monotonic() + 5.0
next_repeat = 0.0
try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic()
        if now >= next_repeat:
            next_repeat = now + REPEAT_S
            if moving(latest):
                cmd_pub.publish(latest)
            elif stops_left:
                cmd_pub.publish(latest)
                stops_left -= 1
        if pending is not None and pending.done():
            if pending.result() is None or not pending.result().success:
                log.warning("/toggle_wheels did not succeed")
            pending = None
        if not client.service_is_ready():
            if warn_at is not None and now >= warn_at:
                log.warning("/toggle_wheels not found yet; still waiting")
                warn_at = None
            continue
        if not powered:
            request_wheels(True)
            powered = True
            log.info("Wheels powered; they stay on until you quit (Ctrl+C).")
        if reenable_at is not None and now >= reenable_at and pending is None:
            reenable_at = None
            request_wheels(True)
except KeyboardInterrupt:
    pass
finally:
    for _ in range(5):
        cmd_pub.publish(Twist())
        time.sleep(0.05)
    if client.service_is_ready():
        future = client.call_async(SetBool.Request(data=False))
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        log.info("Robot stopped, wheel power cut.")
    node.destroy_node()
    rclpy.shutdown()
EOF
GUARD_PID=$!

stop_guard() {
  kill -TERM "$GUARD_PID" 2>/dev/null || true
  wait "$GUARD_PID" 2>/dev/null || true
}
trap stop_guard EXIT

"$(dirname "$0")/teleop.sh" --speed "$SPEED" --turn "$TURN" --topic "$TELEOP_TOPIC"
