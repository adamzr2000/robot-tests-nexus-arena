#!/usr/bin/env bash
set -euo pipefail

# Look at the lidar test data (ROS 1) on this laptop:
#   ./lidar_viz.sh rviz                 RViz on the live /scan (start ./start_rplidar_tests.sh first)
#   ./lidar_viz.sh rviz <run folder>    RViz replaying that run's bag in a loop, e.g.
#                                       results/lidar-benchmark/L2_box_d2.00_r1_140502
#   ./lidar_viz.sh plot [session]       figures from the bags into results/<session>/figures
#                                       (default session: lidar-benchmark)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="rplidar-viz"
TESTS_DIR="${SCRIPT_DIR}/dockerfiles/rplidar-tests"

usage() { sed -n '4,9p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
[[ $# -ge 1 ]] || usage

docker image inspect "${IMAGE}:latest" >/dev/null 2>&1 \
  || docker build -t "$IMAGE" -f "${TESTS_DIR}/Dockerfile.viz" "${TESTS_DIR}"

case "$1" in
  rviz)
    [[ -n "${DISPLAY:-}" ]] || { echo "ERROR: no DISPLAY; run this from the laptop's desktop session"; exit 1; }
    xhost +SI:localuser:root >/dev/null          # let the container's root use this display...
    trap 'xhost -SI:localuser:root >/dev/null' EXIT  # ...only while RViz is open
    X11=( -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:ro
          -v "${TESTS_DIR}/rviz:/rviz:ro" )
    [[ -e /dev/dri ]] && X11+=( --device /dev/dri )
    if [[ $# -ge 2 ]]; then
      RUN="$(cd "$2" && pwd)"
      [[ -f "${RUN}/bag/scan.bag" ]] || { echo "ERROR: ${RUN}/bag/scan.bag not found"; exit 1; }
      echo "Replaying $(basename "$RUN") in a loop; close the RViz window to stop."
      # own ROS master inside the container (no clash with a running test stack)
      docker run --rm "${X11[@]}" -v "${RUN}/bag:/bag:ro" "$IMAGE" bash -c '
        source /opt/ros/noetic/setup.bash
        roscore >/dev/null 2>&1 & sleep 3
        rosparam set use_sim_time true
        rosbag play --clock --loop -q /bag/scan.bag >/dev/null 2>&1 &
        rviz -d /rviz/scan.rviz'
    else
      docker ps --format '{{.Names}}' | grep -qx rplidar-lidar \
        || { echo "ERROR: the lidar driver is not running; ./start_rplidar_tests.sh first"; exit 1; }
      echo "Live /scan; close the RViz window to stop."
      docker run --rm "${X11[@]}" --net host -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
        "$IMAGE" bash -c 'source /opt/ros/noetic/setup.bash && rviz -d /rviz/scan.rviz'
    fi
    ;;
  plot)
    SESSION="${2:-lidar-benchmark}"
    docker run --rm --user "$(id -u):$(id -g)" -e MPLCONFIGDIR=/tmp \
      -v "${SCRIPT_DIR}/results:/results" -v "${TESTS_DIR}/scripts:/scripts:ro" "$IMAGE" \
      bash -c "source /opt/ros/noetic/setup.bash && python3 /scripts/plot_scans.py /results/${SESSION}"
    ;;
  *) usage ;;
esac
