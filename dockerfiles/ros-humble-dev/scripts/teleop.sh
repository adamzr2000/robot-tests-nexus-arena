#!/usr/bin/env bash
# Generic keyboard teleop on /cmd_vel. Runs inside the ros-dev container:
#   ./run_ros_dev.sh [DDS options] /scripts/teleop.sh [--speed <m/s>] [--turn <rad/s>] [--topic <name>]
set -eo pipefail

ROS_ARGS=()

usage() {
  echo "Usage: $0 [--speed <m/s>] [--turn <rad/s>] [--topic <name>]"
  echo "Defaults: teleop_twist_keyboard's own (speed 0.5, turn 1.0), topic /cmd_vel"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --speed)   ROS_ARGS+=( -p "speed:=${2:?--speed needs a value}" ); shift 2 ;;
    --turn)    ROS_ARGS+=( -p "turn:=${2:?--turn needs a value}" );   shift 2 ;;
    --topic)   ROS_ARGS+=( -r "cmd_vel:=${2:?--topic needs a value}" ); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

if [[ ${#ROS_ARGS[@]} -gt 0 ]]; then
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args "${ROS_ARGS[@]}"
else
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
fi
