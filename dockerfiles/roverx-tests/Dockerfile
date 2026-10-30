ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# rosbag2 (sqlite3 storage) for the per-trial bags; rclpy, std_srvs, nav_msgs,
# sensor_msgs and PyYAML already come with the base image.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
 && rm -rf /var/lib/apt/lists/*

RUN useradd -m robot
WORKDIR /home/robot
COPY scripts/ /home/robot/scripts/
RUN mkdir -p /home/robot/results && chown -R robot:robot /home/robot
USER robot
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/robot/.bashrc

# HTTP API (scripts/api.py); results/ is meant to be bind-mounted to the host.
EXPOSE 8090
CMD ["python3", "/home/robot/scripts/api.py"]
