#!/usr/bin/env python3
"""
:module:        gz_world.launch.py
:description:   Launch file for BFFP Gazebo environment
:owner:         (C) C-Boost B.V. (cboost) - All Rights Reserved
:author:        [Patrick Verwimp](mailto:patrick.verwimp@cboost.nl)
:project:       Cboost Internal

This file is proprietary and confidential.
Unauthorized copying of this file via any medium is strictly prohibited.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

__author__ = "Patrick Verwimp"
__copyright__ = "(C) C-Boost B.V. (cboost) - All Rights Reserved"
__credits__ = ["Patrick Verwimp", "Simon Mingaars"]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Patrick Verwimp"
__email__ = "patrick.verwimp@cboost.nl"
__status__ = "Prototype"

def generate_launch_description():

    resource_path = os.path.join(get_package_share_directory("bffp_gazebo"), "resources")
    world = os.path.join(resource_path, "world.sdf") 
    config_path = os.path.join(get_package_share_directory("bffp_gazebo"), "config")
    gui_config = os.path.join(config_path, "gui.config")
    bridge = os.path.join(config_path, "ros_gz_bridge.yaml")

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments=[('gz_args', [' -r -v 4 ', world, ' --gui-config ', gui_config])])
    
 
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', "robot_description",
            '-allow_renaming', 'false',
            '-x', "0.0",
            '-y', "0.0",
            '-z', "0.6",
            '-R', "0.0",
            '-P', "0.0",
            '-Y', "0.0"
        ],
        output='screen')
    
        # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments="0.8 0 -0.05 3.14159 0 0 world conveyor_belt".split(' '),
    )

    


    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge }
        ],
        # arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )  
    return LaunchDescription(
        [
            static_tf,
            start_gazebo_ros_bridge_cmd,
            start_gazebo_cmd,
            start_gazebo_ros_spawner_cmd,
        ]
    )