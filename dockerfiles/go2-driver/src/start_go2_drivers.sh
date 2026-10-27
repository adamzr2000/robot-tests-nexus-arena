#!/bin/bash

CONFIG_FILE="/home/go2/ros2_ws/src/go2_base/config/go2_base.yaml"

TARGET_PORT=${GO2_LAN_PORT:-"enp92s0"}

echo "Configuring YAML: Setting lan_port to '$TARGET_PORT' in $CONFIG_FILE"
sed -i "s/lan_port: .*/lan_port: '$TARGET_PORT'/" "$CONFIG_FILE"

if [ -n "${IMAGE_WIDTH}" ]; then
    echo "Configuring YAML: Setting image_width to '$IMAGE_WIDTH' in $CONFIG_FILE"
    sed -i "s/image_width: .*/image_width: $IMAGE_WIDTH/" "$CONFIG_FILE"
fi

if [ -n "${IMAGE_HEIGHT}" ]; then
    echo "Configuring YAML: Setting image_height to '$IMAGE_HEIGHT' in $CONFIG_FILE"
    sed -i "s/image_height: .*/image_height: $IMAGE_HEIGHT/" "$CONFIG_FILE"
fi

if [ -n "${PUBLISH_WIDTH}" ]; then
    echo "Configuring YAML: Setting publish_width to '$PUBLISH_WIDTH' in $CONFIG_FILE"
    sed -i "s/publish_width: .*/publish_width: $PUBLISH_WIDTH/" "$CONFIG_FILE"
fi

if [ -n "${PUBLISH_HEIGHT}" ]; then
    echo "Configuring YAML: Setting publish_height to '$PUBLISH_HEIGHT' in $CONFIG_FILE"
    sed -i "s/publish_height: .*/publish_height: $PUBLISH_HEIGHT/" "$CONFIG_FILE"
fi

source /opt/ros/humble/setup.bash
source /home/go2/ros2_ws/install/setup.bash

# Disable optional accessory models (e.g., top open manipulator) for plain GO2.
export GO2_ACCESSORIES=0

if [ "${GO2_CAMERA_ONLY:-0}" = "1" ]; then
    echo "GO2_CAMERA_ONLY=1: starting go2_camerapublisher node only"
    exec ros2 run go2_base go2_camerapublisher --ros-args \
        --params-file "$CONFIG_FILE" \
        -r __ns:=/go2 \
        -r __node:=go2_camerapublisher
fi

exec ros2 launch go2_bringup system.launch.py