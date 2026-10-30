#!/usr/bin/env bash
set -euo pipefail

DDS_INTERFACE="enp3s0"
DOMAIN_ID=0
SESSION_NAME=""
MAX_TRAVEL=""

usage() {
  echo "Usage: $0 [--iface IFNAME] [--ros-domain-id ID] [--session NAME] [--max-travel METERS]"
  echo "  --iface          Laptop interface wired to the RoverX (default: enp3s0). FastDDS is"
  echo "                   restricted to its IPv4 address, so it must be up with one."
  echo "  --ros-domain-id  The robot's ROS_DOMAIN_ID (default: 0)"
  echo "  --session        Results folder name under results/ (default: today's date)"
  echo "  --max-travel     Space available here: runs planned longer are refused and"
  echo "                   odometry beyond it + 1 m aborts (default: no limit)"
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface)         [[ $# -ge 2 ]] || usage; DDS_INTERFACE="$2"; shift 2 ;;
    --ros-domain-id) [[ $# -ge 2 ]] || usage; DOMAIN_ID="$2"; shift 2 ;;
    --session)       [[ $# -ge 2 ]] || usage; SESSION_NAME="$2"; shift 2 ;;
    --max-travel)    [[ $# -ge 2 ]] || usage; MAX_TRAVEL="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "Unknown argument: $1"; usage ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Only one thing may command /cmd_vel
if docker ps --format '{{.Names}}' | grep -qx robot-trajectories; then
  echo "ERROR: the trajectories stack is running; stop it first with ./stop_roverx.sh or ./stop_go2.sh"
  exit 1
fi

mkdir -p "${SCRIPT_DIR}/.generated" "${SCRIPT_DIR}/results"
"${SCRIPT_DIR}/dockerfiles/fastdds_profile.sh" "$DDS_INTERFACE" > "${SCRIPT_DIR}/.generated/fastdds_roverx.xml"
chmod 644 "${SCRIPT_DIR}/.generated/fastdds_roverx.xml"
echo "Using DDS interface: ${DDS_INTERFACE}, ROS_DOMAIN_ID=${DOMAIN_ID}, session: ${SESSION_NAME:-<date>}, max travel: ${MAX_TRAVEL:-none}"

ROVERX_DOMAIN_ID="$DOMAIN_ID" SESSION="$SESSION_NAME" MAX_TRAVEL_M="$MAX_TRAVEL" \
  docker compose -f "${SCRIPT_DIR}/docker-compose-roverx-tests.yml" up -d

echo "Done. API on :8090, live output: docker logs -f roverx-tests"
