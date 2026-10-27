#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import launch_ros.actions
from launch import LaunchDescription

def generate_launch_description():

    launch_go2_scan = launch_ros.actions.Node(namespace='go2',
                                              package='pointcloud_to_laserscan', 
                                              executable='pointcloud_to_laserscan_node',
                                              remappings=[('cloud_in', '/utlidar/cloud_deskewed'),
                                                          ('scan', 'scan/unfiltered')],
                                              parameters=[{'target_frame': 'lidar_link',
                                                            'transform_tolerance': 0.01,
                                                            'min_height': 0.4,
                                                            'max_height': 1.0,
                                                            'angle_min': -1.57,  
                                                            'angle_max': 1.57,  
                                                            'angle_increment': 0.033, 
                                                            'scan_time': 0.005,
                                                            'range_min': 0.2,
                                                            'range_max': 5.0,
                                                            'use_inf': True,
                                                            'inf_epsilon': 1.0}],
                                              name='pointcloud_to_laserscan')

    ld = LaunchDescription()
    ld.add_action(launch_go2_scan)

    return ld