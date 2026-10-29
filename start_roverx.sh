#!/usr/bin/env bash
set -euo pipefail

DDS_INTERFACE="eno1"
DOMAIN_ID=0

usage() {
  echo "Usage: $0 [--iface IFNAME] [--ros-domain-id ID]"
  echo "  --iface          Interface on the robot network (default: eno1). FastDDS is"
  echo "                   restricted to its IPv4 address, so it must be up with one."
  echo "  --ros-domain-id  The robot's ROS_DOMAIN_ID (default: 0)"
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface)         [[ $# -ge 2 ]] || usage; DDS_INTERFACE="$2"; shift 2 ;;
    --ros-domain-id) [[ $# -ge 2 ]] || usage; DOMAIN_ID="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "Unknown argument: $1"; usage ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Same container names and ports as the Go2 stack: only one robot at a time
if docker ps -a --format '{{.Names}}' | grep -qx go2-driver; then
  echo "ERROR: the Go2 stack is running; stop it first with ./stop_go2.sh"
  exit 1
fi

mkdir -p "${SCRIPT_DIR}/.generated"
"${SCRIPT_DIR}/dockerfiles/fastdds_profile.sh" "$DDS_INTERFACE" > "${SCRIPT_DIR}/.generated/fastdds_roverx.xml"
chmod 644 "${SCRIPT_DIR}/.generated/fastdds_roverx.xml"
echo "Using DDS interface: ${DDS_INTERFACE}, ROS_DOMAIN_ID=${DOMAIN_ID}"

ROVERX_DOMAIN_ID="$DOMAIN_ID" docker compose -f "${SCRIPT_DIR}/docker-compose-roverx.yml" up -d

echo "Done. Use 'docker compose -f ${SCRIPT_DIR}/docker-compose-roverx.yml logs -f' to follow logs."
