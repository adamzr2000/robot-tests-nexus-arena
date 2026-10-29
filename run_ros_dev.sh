#!/bin/bash
set -e

container_image="ros-humble-dev:latest"
container_name="ros-dev"
ros_domain_id=0

dds="cyclone"
iface="lo"
gui=0        # default: no GUI/X11
gpu=0        # default: no GPU access

# Helper scripts (teleop, ...) are always available in the container at /scripts
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="${REPO_DIR}/dockerfiles/ros-humble-dev/scripts"
VOLUME_ARGS=( --volume="${SCRIPTS_DIR}:/scripts:ro" )

usage() {
  echo "Usage: $0 [--ros-domain-id <id>] [--dds cyclone|fast] [--iface <iface>] [--gui] [--gpu] [--] [command ...]"
  echo "Defaults: --ros-domain-id 0, --dds cyclone (UDP), --iface lo, --gui off, --gpu off"
  echo "  --dds cyclone|fast   DDS vendor (default: cyclone). Both run their standard UDP"
  echo "                       discovery."
  echo "  --iface <iface>      The only network interface DDS may use, for either vendor."
  echo "                       FastDDS is restricted by the interface's IPv4 address, so"
  echo "                       the interface must be up with an address."
  echo "  command ...          Run this instead of an interactive bash shell (ROS is"
  echo "                       already sourced). Everything after the first non-option"
  echo "                       argument, or after '--', is passed through as-is, e.g.:"
  echo "                         $0 --ros-domain-id 10 /scripts/teleop.sh"
  echo "                         $0 --ros-domain-id 10 -- ros2 topic echo /cmd_vel --once"
  echo "Scripts in dockerfiles/ros-humble-dev/scripts are mounted read-only at /scripts."
}

need_arg() {
  local opt="$1"
  local val="$2"
  if [[ -z "$val" || "$val" == --* ]]; then
    echo "ERROR: $opt requires an argument"
    usage
    exit 1
  fi
}

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-domain-id) need_arg "$1" "${2:-}"; ros_domain_id="$2"; shift 2 ;;
    --dds)           need_arg "$1" "${2:-}"; dds="$2";           shift 2 ;;
    --iface)         need_arg "$1" "${2:-}"; iface="$2";        shift 2 ;;
    --gui)           gui=1; shift ;;
    --gpu)           gpu=1; shift ;; # Added GPU flag
    -h|--help)       usage; exit 0 ;;
    --)              shift; break ;;
    -*) echo "Unknown option: $1"; usage; exit 1 ;;
    *)               break ;; # first non-option: start of the command
  esac
done

# Remaining args are the command to run; default to an interactive shell
cmd=( "$@" )
[[ ${#cmd[@]} -eq 0 ]] && cmd=( bash )

# Only allocate a TTY when we have one, so piped/scripted commands still work
TTY_ARGS=( -i )
[[ -t 0 && -t 1 ]] && TTY_ARGS+=( -t )

# Validate DDS choice + set RMW
case "$dds" in
  fast)    RMW="rmw_fastrtps_cpp" ;;
  cyclone) RMW="rmw_cyclonedds_cpp" ;;
  *) echo "ERROR: invalid --dds value: '$dds' (use 'fast' or 'cyclone')"; exit 1 ;;
esac

# Extra docker args
EXTRA_ENV_ARGS=( --env="RMW_IMPLEMENTATION=${RMW}" )
GUI_ARGS=()
GPU_ARGS=() # Initialize GPU args array

# GPU Configuration
if [[ "$gpu" -eq 1 ]]; then
  GPU_ARGS+=( --gpus all )
fi

# GUI/X11 only if --gui
if [[ "$gui" -eq 1 ]]; then
  if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null
  else
    echo "WARNING: xhost not found. Install: sudo apt-get install -y x11-xserver-utils"
  fi

  GUI_ARGS+=(
    --env="DISPLAY=$DISPLAY"
    --env="QT_X11_NO_MITSHM=1"
    --env="QT_QPA_PLATFORM=xcb"
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
  )
fi

# lo needs multicast enabled on the host for DDS discovery; CycloneDDS can't
# even create a participant on it otherwise ("not multicast-capable").
if [[ "$iface" == "lo" ]] && ! ip link show lo | grep -q MULTICAST; then
  echo "Enabling multicast on loopback..."
  if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
    ip link set lo multicast on
  else
    sudo ip link set lo multicast on
  fi
fi

# FastDDS: restricted to the interface's IPv4 address (see fastdds_profile.sh)
if [[ "$dds" == "fast" ]]; then
  fastdds_profile="$(mktemp --suffix=.xml)"
  trap 'rm -f "$fastdds_profile"' EXIT
  "${REPO_DIR}/dockerfiles/fastdds_profile.sh" "$iface" > "$fastdds_profile"
  iface_ip="$(grep -o '<address>[^<]*' "$fastdds_profile" | cut -d'>' -f2)"
  chmod 644 "$fastdds_profile"
  VOLUME_ARGS+=( --volume="${fastdds_profile}:/fastdds_profile.xml:ro" )
  EXTRA_ENV_ARGS+=( --env="FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_profile.xml" )
fi

# Only for cyclone: set CYCLONEDDS_URI inline — no host dir or mount needed
if [[ "$dds" == "cyclone" ]]; then
  # The config below asks for a 10MB socket receive buffer; the kernel caps
  # it at net.core.rmem_max, which defaults well under that on most distros.
  REQUIRED=10485760
  CURRENT_RMEM="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
  if (( CURRENT_RMEM < REQUIRED )); then
    echo "Applying DDS kernel tuning..."
    SUDO=()
    [[ "${EUID:-$(id -u)}" -ne 0 ]] && SUDO=(sudo)
    "${SUDO[@]}" sysctl -w net.core.rmem_max=2147483647 >/dev/null
    "${SUDO[@]}" sysctl -w net.core.wmem_max=2147483647 >/dev/null
  fi

  CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine='false' name='${iface}' priority='default' multicast='default'/></Interfaces><AllowMulticast>default</AllowMulticast><MaxMessageSize>65500B</MaxMessageSize></General><Internal><SocketReceiveBufferSize min='10MB'/><Watermarks><WhcHigh>500kB</WhcHigh></Watermarks></Internal></Domain></CycloneDDS>"
  EXTRA_ENV_ARGS+=( --env="CYCLONEDDS_URI=${CYCLONEDDS_URI}" )
fi

echo "Running container with:"
echo "  image:          $container_image"
echo "  name:           $container_name"
echo "  ROS_DOMAIN_ID:  $ros_domain_id"
echo "  DDS:            $dds"
echo "  iface:          $iface${iface_ip:+ ($iface_ip)}"
echo "  GUI:            $([[ "$gui" -eq 1 ]] && echo on || echo off)"
echo "  GPU:            $([[ "$gpu" -eq 1 ]] && echo on || echo off)"
echo "  command:        ${cmd[*]}"


docker run \
  "${TTY_ARGS[@]}" \
  --rm \
  --name "${container_name}" \
  --env="ROS_DOMAIN_ID=${ros_domain_id}" \
  --net host \
  --privileged \
  "${GPU_ARGS[@]}" \
  "${GUI_ARGS[@]}" \
  "${VOLUME_ARGS[@]}" \
  "${EXTRA_ENV_ARGS[@]}" \
  "${container_image}" \
  "${cmd[@]}"