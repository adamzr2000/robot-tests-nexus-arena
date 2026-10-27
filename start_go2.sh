#!/usr/bin/env bash
set -euo pipefail

DDS_INTERFACE="lo"

usage() {
  echo "Usage: $0 [--iface IFNAME]"
  echo "  --iface  IFNAME  Network interface for DDS (default: lo). go2-driver,"
  echo "                   robot-trajectories, and bags-collector all share this"
  echo "                   same config, and the domain-10 side of go2-domain-bridge"
  echo "                   follows it too, so all four stay reachable when DDS"
  echo "                   traffic moves off loopback."
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface|--if|--interface)
      [[ $# -ge 2 ]] || usage
      DDS_INTERFACE="$2"
      shift 2
      ;;
    -h|--help) usage ;;
    *)
      echo "Unknown argument: $1"
      usage
      ;;
  esac
done

# ── Loopback multicast (local mode only) ─────────────────────────────────────
if [[ "$DDS_INTERFACE" == "lo" ]]; then
  if ! ip link show lo | grep -q MULTICAST; then
    echo "Enabling multicast on loopback..."
    if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
      ip link set lo multicast on
    else
      sudo ip link set lo multicast on
    fi
  fi
fi

# ── DDS kernel tuning ─────────────────────────────────────────────────────────
REQUIRED=10485760
CURRENT_RMEM="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
CURRENT_WMEM="$(sysctl -n net.core.wmem_max 2>/dev/null || echo 0)"
if (( CURRENT_RMEM < REQUIRED || CURRENT_WMEM < REQUIRED )); then
  echo "Applying DDS kernel tuning..."
  SUDO=()
  [[ "${EUID:-$(id -u)}" -ne 0 ]] && SUDO=(sudo)
  "${SUDO[@]}" sysctl -w net.core.rmem_max=2147483647          >/dev/null
  # Send-side analog of rmem_max above
  "${SUDO[@]}" sysctl -w net.core.wmem_max=2147483647          >/dev/null
  "${SUDO[@]}" sysctl -w net.ipv4.ipfrag_time=3                >/dev/null
  "${SUDO[@]}" sysctl -w net.ipv4.ipfrag_high_thresh=134217728 >/dev/null
fi

echo "Using DDS interface: ${DDS_INTERFACE}"

# ── DDS config ────────────────────────────────────────────────────────────────
if [[ "$DDS_INTERFACE" == "lo" ]]; then
  CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine='false' name='lo' priority='default' multicast='true'/></Interfaces><AllowMulticast>true</AllowMulticast><MaxMessageSize>65500B</MaxMessageSize></General><Internal><SocketReceiveBufferSize min='10MB'/><SocketSendBufferSize min='10MB'/><Watermarks><WhcHigh>500kB</WhcHigh></Watermarks></Internal></Domain></CycloneDDS>"
else
  CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine='false' name='${DDS_INTERFACE}' priority='default' multicast='default'/></Interfaces><AllowMulticast>default</AllowMulticast><MaxMessageSize>65500B</MaxMessageSize></General><Internal><SocketReceiveBufferSize min='10MB'/><SocketSendBufferSize min='10MB'/><Watermarks><WhcHigh>500kB</WhcHigh></Watermarks></Internal></Domain></CycloneDDS>"
fi

export CYCLONEDDS_URI

# ── DDS config for go2-domain-bridge ─────────────────────────────────────────
# The bridge's "robot0" interface (the robot's own internal LAN NIC, for the
# domain-0 side) is fixed on purpose and left alone here. Only the domain-10
# side — the one the bridge uses to reach go2-driver — follows DDS_INTERFACE,
# so the two containers stay on the same NIC instead of silently falling out
# of discovery when driver traffic moves off loopback (e.g. --iface for a
# NAT/edge setup). Left unset for the lo default, which keeps the bridge on
# its built-in file:///home/go2/config/cyclonedds.xml — unchanged behavior.
CYCLONEDDS_URI_BRIDGE=""
if [[ "$DDS_INTERFACE" != "lo" ]]; then
  CYCLONEDDS_URI_BRIDGE="<CycloneDDS><Domain Id='any'><General><Interfaces><NetworkInterface name='robot0' multicast='default'/><NetworkInterface name='${DDS_INTERFACE}' multicast='default'/></Interfaces><DontRoute>true</DontRoute></General></Domain></CycloneDDS>"
fi
export CYCLONEDDS_URI_BRIDGE

# ── Launch ────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker compose -f "${SCRIPT_DIR}/docker-compose-go2.yml" up -d

echo "Done. Use 'docker compose -f ${SCRIPT_DIR}/docker-compose-go2.yml logs -f' to follow logs."
