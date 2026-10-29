ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    wget \
    libpcl-dev \
    libglfw3-dev \
    libglew-dev \
    libeigen3-dev \
    libjsoncpp-dev \
    libtclap-dev \
    libboost-all-dev \
    libbullet-dev \
    libgtk-3-dev \
    # Gstreamer stack
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    # FFmpeg/Video libs
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libswresample-dev \
    # Python tools
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-gi \
    python3-gst-1.0 \
    gir1.2-gstreamer-1.0 \
    clang-format \
    # ROS 2 Packages
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-robot-upstart \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-dynamixel-workbench \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# rosdep init (ok if already initialized in base)
RUN rosdep init || true && rosdep update

# Create a new user 'go2' with sudo privileges and set a password
RUN useradd -m go2 && \
    echo "go2:go2" | chpasswd && adduser go2 sudo

WORKDIR /home/go2

# Copy workspace
RUN mkdir -p /home/go2/ros2_ws/src
COPY src/ /home/go2/ros2_ws/src/

RUN chmod +x /home/go2/ros2_ws/src/start_go2_drivers.sh \
             /home/go2/ros2_ws/src/go2_base/go2_base/go2_statepublisher \
             /home/go2/ros2_ws/src/go2_base/go2_base/go2_scanpublisher \
             /home/go2/ros2_ws/src/go2_base/go2_base/go2_camerapublisher

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

CMD ["/home/go2/ros2_ws/src/start_go2_drivers.sh"]