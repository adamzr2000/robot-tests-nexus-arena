# ROS 2 distro as build arg
ARG ROS_DISTRO=humble

# Official ROS 2 base image
FROM ros:${ROS_DISTRO}

LABEL maintainer="azahir@pa.uc3m.es"

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///home/go2/config/cyclonedds.xml

SHELL ["/bin/bash", "-c"]

# System + build tools + rosdep + colcon + rviz
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    ros-${ROS_DISTRO}-domain-bridge \
    libyaml-cpp-dev \
    python3-rosdep \
    python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

RUN useradd -m -s /bin/bash go2 && \
    echo "go2:go2" | chpasswd && \
    apt-get update && apt-get install -y --no-install-recommends sudo && \
    adduser go2 sudo && \
    rm -rf /var/lib/apt/lists/*

# rosdep init (ok if already initialized in base)
RUN rosdep init || true && rosdep update

WORKDIR /home/go2

# Copy workspace
RUN mkdir -p /home/go2/ros2_ws/src
COPY src/ /home/go2/ros2_ws/src/

# Bridge runtime files
COPY config/domain_bridge.yaml /home/go2/config/domain_bridge.yaml
COPY config/cyclonedds.xml /home/go2/config/cyclonedds.xml

# Install package dependencies (cached unless src changes)
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /home/go2/ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /home/go2/ros2_ws && \
    colcon build --symlink-install

# Switch to user for runtime
RUN chown -R go2:go2 /home/go2
USER go2

# Source overlays in bash sessions
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/go2/.bashrc && \
    echo "source /home/go2/ros2_ws/install/setup.bash" >> /home/go2/.bashrc

CMD ["/bin/bash", "-lc", \
  "source /opt/ros/${ROS_DISTRO}/setup.bash && \
   source /home/go2/ros2_ws/install/setup.bash && \
    ros2 run domain_bridge domain_bridge /home/go2/config/domain_bridge.yaml"]