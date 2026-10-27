ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# rosbag2 + its default storage plugins (sqlite3), needed for `ros2 bag record`
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# Create a non-root user with sudo privileges
RUN useradd -m robot && \
    echo "robot:robot" | chpasswd && adduser robot sudo

WORKDIR /home/robot

# Control API (scripts/main.py) + default bag output directory. Mount a host
# volume over bags/ to export recordings (see docker-compose-go2.yml).
RUN mkdir -p /home/robot/scripts /home/robot/bags
COPY scripts/ /home/robot/scripts/
RUN find /home/robot/scripts -name '*.py' -exec chmod +x {} +

RUN chown -R robot:robot /home/robot
USER robot

# Source overlays in bash sessions
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/robot/.bashrc

# Default: start the HTTP API so recordings can be triggered on request
# (see scripts/main.py). Port 8081, not 8080: with network_mode: host this
# runs alongside robot-trajectories' API, which already owns 8080. For an
# interactive shell instead, override at run time:
# `docker run ... bags-collector bash`.
EXPOSE 8081
CMD ["python3", "/home/robot/scripts/main.py"]
