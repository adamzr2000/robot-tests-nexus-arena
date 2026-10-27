ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    cmake \
    build-essential \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# rosdep init (ok if already initialized in base)
RUN rosdep init || true && rosdep update

# Create a non-root user with sudo privileges
RUN useradd -m robot && \
    echo "robot:robot" | chpasswd && adduser robot sudo

WORKDIR /home/robot

# Trajectory scripts (circle, square, figure-eight, etc.) — plain rclpy
# scripts, not ROS2 packages. They only need rclpy + standard message types,
# both already in the base image, so there's nothing to rosdep/colcon build;
# just `python3 ~/scripts/<name>.py`. Robot-agnostic by design: point --topic
# at whatever /cmd_vel-equivalent the target robot's driver exposes.
RUN mkdir -p /home/robot/scripts
COPY scripts/ /home/robot/scripts/
RUN find /home/robot/scripts -name '*.py' -exec chmod +x {} +

RUN chown -R robot:robot /home/robot
USER robot

# Source overlays in bash sessions
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/robot/.bashrc

# Default: start the HTTP API so trajectories can be triggered on request
# (see scripts/main.py). For an interactive shell instead, override at run
# time: `docker run ... robot-trajectories bash`.
EXPOSE 8080
CMD ["python3", "/home/robot/scripts/main.py"]
