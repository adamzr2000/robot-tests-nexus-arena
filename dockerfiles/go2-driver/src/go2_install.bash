#!/usr/bin/env bash

function color_echo () {
    echo "$(tput setaf 1)$1$(tput sgr0)"
}

function install_binary_packages () {
    color_echo "Installing build pacakges."
    sudo apt-get install libpcl-dev build-essential\
                         cmake\
                         libglfw3-dev\
                         libglew-dev\
                         libeigen3-dev\
                         libjsoncpp-dev\
                         libtclap-dev\
                         libeigen3-dev\
                         libboost-all-dev\
                         gstreamer1.0-plugins-base\
                         gstreamer1.0-plugins-good\
                         gstreamer1.0-plugins-bad\
                         gstreamer1.0-plugins-ugly\
                         gstreamer1.0-libav\
                         gstreamer1.0-doc\
                         gstreamer1.0-tools\
                         gstreamer1.0-x\
                         gstreamer1.0-alsa\
                         gstreamer1.0-gl\
                         gstreamer1.0-gtk3\
                         gstreamer1.0-qt5\
                         gstreamer1.0-pulseaudio\
                         libgstreamer1.0-dev\
                         libgstreamer-plugins-base1.0-dev\
                         gstreamer1.0-tools\
                         libavcodec-dev\
                         libavformat-dev\
                         libavutil-dev\
                         libswscale-dev\
                         libavresample-dev\
                         libgtk-3-dev\
                         git\
                         libbullet-dev \
                         python3-colcon-common-extensions \
                         python3-flake8 \
                         python3-pip \
                         python3-pytest-cov \
                         python3-rosdep \
                         python3-setuptools \
                         python3-vcstool \
                         wget \
                         clang-format-10 -y

    color_echo "Installing build pacakges."

    sudo apt-get install ros-humble-robot-upstart\
                         ros-humble-rosidl-generator-dds-idl\
                         ros-humble-teleop-twist-keyboard\
                         ros-humble-teleop-twist-joy\
                         ros-humble-geodesy\
                         ros-humble-pcl-ros\
                         ros-humble-nmea-msgs\
                         ros-humble-robot-localization\
                         ros-humble-interactive-marker-twist-server\
                         ros-humble-pointcloud-to-laserscan\
                         ros-humble-twist-mux\
                         ros-humble-rmw-cyclonedds-cpp\
                         ros-humble-rosidl-generator-dds-idl\
                         ros-humble-navigation2\
                         ros-humble-moveit\
                         ros-humble-dynamixel-sdk\
                         ros-humble-dynamixel-workbench\
                         ros-humble-joint-state-publisher-gui\
                         ros-humble-ros2-control\
                         ros-humble-ros2-controllers\
                         ros-humble-gripper-controllers\
                         ros-humble-moveit\
                         ros-humble-gazebo-ros2-control\
                         ros-humble-moveit-servo\
                         ros-humble-xacro\
                         ros-humble-navigation2\
                         ros-humble-nav2-* -y
}

RED='\033[0;31m'
DGREEN='\033[0;32m'
GREEN='\033[1;32m'
WHITE='\033[0;37m'
BLUE='\033[1;34m'
CYAN='\033[1;36m'
NC='\033[0m' 
                                                                                          
echo -e "${DGREEN}------------------------------------------------------------------------------------------------------"
echo -e " _______           _______  ______   _______           _______  _______  ______     ______   _______  "
echo -e "(  ___  )|\     /|(  ___  )(  __  \ (  ____ )|\     /|(  ____ )(  ____ \(  __  \   (  __  \ (  ____ \ "
echo -e "| (   ) || )   ( || (   ) || (  \  )| (    )|| )   ( || (    )|| (    \/| (  \  )  | (  \  )| (    \/ "
echo -e "| |   | || |   | || (___) || |   ) || (____)|| |   | || (____)|| (__    | |   ) |  | |   ) || (__     "
echo -e "| |   | || |   | ||  ___  || |   | ||     __)| |   | ||  _____)|  __)   | |   | |  | |   | ||  __)    "
echo -e "| | /\| || |   | || (   ) || |   ) || (\ (   | |   | || (      | (      | |   ) |  | |   ) || (       "
echo -e "| (_\ \ || (___) || )   ( || (__/  )| ) \ \__| (___) || )      | (____/\| (__/  )_ | (__/  )| (____/\ "
echo -e "(____\/_)(_______)|/     \|(______/ |/   \__/(_______)|/       (_______/(______/(_)(______/ (_______/ "
echo -e ""                                                                                                                                   
echo -e "------------------------------------------------------------------------------------------------------"
echo -e "Installing Required Libraries and ROS dependencies! "                                                                                                                                         
echo -e "------------------------------------------------------------------------------------------------------${NC}"

# Binary packages installation
install_binary_packages