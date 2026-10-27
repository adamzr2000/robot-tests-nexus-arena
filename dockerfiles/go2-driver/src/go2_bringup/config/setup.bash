#!/bin/bash

echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/qre_go2/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp88s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'