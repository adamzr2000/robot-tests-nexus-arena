#!/usr/bin/env python3
import os
import robot_upstart
from ament_index_python.packages import get_package_share_directory

# Job name
job_name = "ros2"

# Uninstall existing Startup job
os.system("sudo service {} stop".format(job_name))
uninstall_job = robot_upstart.Job(name=job_name, rosdistro=os.environ['ROS_DISTRO'])
uninstall_job.uninstall()

# Install/Update Startup job
main_job = robot_upstart.Job(name=job_name, 
                             user='root',
                             ros_domain_id=0,
                             rmw='rmw_cyclonedds_cpp', 
                            #  interface=args.interface, 
                             workspace_setup=os.path.join(get_package_share_directory('go2_bringup'), 'config/setup.bash') 
                             )

main_job.add(package="go2_bringup", filename="launch/system.launch.py")
main_job.install()

# Refresh for activation
os.system("sudo systemctl daemon-reload && sudo systemctl start {}".format(job_name))
