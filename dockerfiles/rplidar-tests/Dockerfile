ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base

# rospy, rosbag, sensor_msgs and numpy already come with the base image. ROS 1 Noetic
# talks to the Melodic lidar driver (rplidar-lidar) through the same ROS master.
RUN useradd -m robot
WORKDIR /home/robot
COPY scripts/ /home/robot/scripts/
RUN mkdir -p /home/robot/results && chown -R robot:robot /home/robot
USER robot
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/robot/.bashrc

# HTTP API (scripts/api.py); results/ is meant to be bind-mounted to the host.
EXPOSE 8091
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && exec python3 /home/robot/scripts/api.py"]
