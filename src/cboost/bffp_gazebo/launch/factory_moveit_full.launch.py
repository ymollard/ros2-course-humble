#!/usr/bin/env python3
"""
:module:        gz_moveit_full.launch.py
:description:   Template for GZ-sim robot controller launch
:owner:         (C) C-Boost B.V. (cboost) - All Rights Reserved
:author:        [Patrick Verwimp](mailto:patrick.verwimp@cboost.nl)
:project:       Cboost Internal

This file is proprietary and confidential.
Unauthorized copying of this file via any medium is strictly prohibited.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare


from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
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
# Launch Gazebo BFFP setup
    gz_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("bffp_gazebo"),
                'launch',
                'gz_world.launch.py'
            ])
        ])
    )

    hc10dtp_gz_moveit_full_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("motoman_hc10dtp"),
                'launch',
                'gz_moveit_full.launch.py'
            ])
        ]),
        launch_arguments= {"use_rviz": "True"}.items()
    )
#region LaunchDescription
    return LaunchDescription(
        [
            gz_world_launch,
            hc10dtp_gz_moveit_full_launch,
        ]
    )
#endregion