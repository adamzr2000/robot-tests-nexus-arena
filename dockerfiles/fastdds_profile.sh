#!/usr/bin/env bash
# Print a FastDDS XML profile that restricts DDS to one network interface:
#   dockerfiles/fastdds_profile.sh <iface> > profile.xml
# UDP only, whitelisted to the interface's IPv4 address (Fast DDS 2.6 in
# Humble accepts addresses, not interface names). Disabling the builtin
# transports also drops shared memory, which fails across containers.
set -euo pipefail

iface="${1:?usage: $0 <iface>}"
iface_ip="$(ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1 || true)"
if [[ -z "$iface_ip" ]]; then
  echo "ERROR: interface '$iface' has no IPv4 address (missing, down, or not configured)" >&2
  exit 1
fi

cat <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!-- interface: ${iface} (${iface_ip}) -->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_iface_only</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>${iface_ip}</address></interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="default" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp_iface_only</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
