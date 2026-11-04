#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="lidar-benchmark"

usage() {
  echo "Usage: $0 [--session NAME]"
  echo "  --session  Results folder under results/ (default: lidar-benchmark, the lidar campaign;"
  echo "             use another name only for data that must stay apart)"
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) [[ $# -ge 2 ]] || usage; SESSION_NAME="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "Unknown argument: $1"; usage ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -e /dev/rplidar ]]; then
  echo "ERROR: /dev/rplidar not found. Plug in the lidar USB; if it is plugged in, install the udev"
  echo "       rule once: (cd dockerfiles/rplidar && ./apply_lidar_rules.sh)"
  exit 1
fi

# The driver and master use fixed container names; stop copies started by hand (run_example.sh)
for name in rplidar-lidar ros-master; do
  if docker ps --format '{{.Names}} {{.Label "com.docker.compose.project"}}' | grep -qx "${name} "; then
    echo "Stopping ${name} started outside this stack"
    docker stop "${name}" >/dev/null
  fi
done

# Images: the lidar driver is built from dockerfiles/rplidar, the tests from dockerfiles/rplidar-tests
docker image inspect rplidar-lidar:latest >/dev/null 2>&1 || docker build -t rplidar-lidar "${SCRIPT_DIR}/dockerfiles/rplidar"
docker image inspect rplidar-tests:latest >/dev/null 2>&1 || docker build -t rplidar-tests "${SCRIPT_DIR}/dockerfiles/rplidar-tests"

mkdir -p "${SCRIPT_DIR}/results"
echo "Session: ${SESSION_NAME}"
SESSION="$SESSION_NAME" HOST_UID="$(id -u)" HOST_GID="$(id -g)" \
  docker compose -f "${SCRIPT_DIR}/docker-compose-rplidar-tests.yml" up -d
# The API's code is mounted: restart it so a stack that was already up runs the current scripts/api.py
docker restart rplidar-tests >/dev/null

echo "Done. API on :8091 (curl -s localhost:8091/peek), live output: docker logs -f rplidar-tests"
