#!/usr/bin/env bash
set -euo pipefail

DDS_INTERFACE="lo"
ROBOT_INTERFACE="eno1"

usage() {
  echo "Usage: $0 [--iface IFNAME] [--robot-iface IFNAME]"
  echo "  --iface        Interface for the local DDS graph, domain 10 (default: lo)."
  echo "                 go2-driver, robot-trajectories, bags-collector and the"
  echo "                 domain-10 side of go2-domain-bridge all use it."
  echo "  --robot-iface  Interface wired to the Go2 (default: eno1). The bridge's"
  echo "                 domain-0 side and the camera stream use it."
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface|--if|--interface)
      [[ $# -ge 2 ]] || usage
      DDS_INTERFACE="$2"
      shift 2
      ;;
    --robot-iface)
      [[ $# -ge 2 ]] || usage
      ROBOT_INTERFACE="$2"
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

if ! ip link show dev "$ROBOT_INTERFACE" >/dev/null 2>&1; then
  echo "ERROR: robot interface '$ROBOT_INTERFACE' not found (see --robot-iface)"
  exit 1
fi

echo "Using DDS interface: ${DDS_INTERFACE}, robot interface: ${ROBOT_INTERFACE}"

# ── DDS config ────────────────────────────────────────────────────────────────
if [[ "$DDS_INTERFACE" == "lo" ]]; then
  CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine='false' name='lo' priority='default' multicast='true'/></Interfaces><AllowMulticast>true</AllowMulticast><MaxMessageSize>65500B</MaxMessageSize></General><Internal><SocketReceiveBufferSize min='10MB'/><SocketSendBufferSize min='10MB'/><Watermarks><WhcHigh>500kB</WhcHigh></Watermarks></Internal></Domain></CycloneDDS>"
else
  CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine='false' name='${DDS_INTERFACE}' priority='default' multicast='default'/></Interfaces><AllowMulticast>default</AllowMulticast><MaxMessageSize>65500B</MaxMessageSize></General><Internal><SocketReceiveBufferSize min='10MB'/><SocketSendBufferSize min='10MB'/><Watermarks><WhcHigh>500kB</WhcHigh></Watermarks></Internal></Domain></CycloneDDS>"
fi

export CYCLONEDDS_URI

# ── DDS config for go2-domain-bridge ─────────────────────────────────────────
# Domain 0 (the robot) on ROBOT_INTERFACE, domain 10 (go2-driver & co.) on
# DDS_INTERFACE. Generated so neither interface is hard-coded in a file.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$DDS_INTERFACE" == "lo" ]]; then
  LOCAL_IFACE="<NetworkInterface address=\"127.0.0.1\" multicast=\"true\"/>"
else
  LOCAL_IFACE="<NetworkInterface name=\"${DDS_INTERFACE}\" multicast=\"default\"/>"
fi
BRIDGE_CONFIG="${SCRIPT_DIR}/.generated/cyclonedds_go2_bridge.xml"
mkdir -p "${SCRIPT_DIR}/.generated"
cat > "$BRIDGE_CONFIG" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="${ROBOT_INTERFACE}" multicast="default"/>
        ${LOCAL_IFACE}
      </Interfaces>
      <DontRoute>true</DontRoute>
    </General>
  </Domain>
</CycloneDDS>
XML
chmod 644 "$BRIDGE_CONFIG"
export GO2_LAN_PORT="$ROBOT_INTERFACE"

# ── Launch ────────────────────────────────────────────────────────────────────
docker compose -f "${SCRIPT_DIR}/docker-compose-go2.yml" up -d

echo "Done. Use 'docker compose -f ${SCRIPT_DIR}/docker-compose-go2.yml logs -f' to follow logs."
