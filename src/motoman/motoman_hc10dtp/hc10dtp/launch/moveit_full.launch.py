#!/usr/bin/env python3
"""
:module:        moveit_full.launch.py
:description:   Launch file for HC10DTP
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer

__author__ = "Patrick Verwimp"
__copyright__ = "(C) C-Boost B.V. (cboost) - All Rights Reserved"
__credits__ = ["Patrick Verwimp", "Simon Mingaars"]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Patrick Verwimp"
__email__ = "patrick.verwimp@cboost.nl"
__status__ = "Prototype"

def generate_launch_description():
#region LaunchConfiguration
#TODO MAKE SURE THIS WORKS
    launch_as_standalone_node = LaunchConfiguration("launch_as_standalone_node")
    use_gz_controller = LaunchConfiguration("use_gz_controller")
    use_rviz = LaunchConfiguration("use_rviz")

    stand_alone_arg = DeclareLaunchArgument(
        "launch_as_standalone_node",
        default_value="False"
    )
    use_gz_controller_arg = DeclareLaunchArgument(
        "use_gz_controller",
        default_value="False"
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False"
    )

#endregion
#region Parameters
    urdf_file = "motoman_hc10dtp.urdf.xacro"
    rviz_config_file = "servo.rviz"
    motion_planning_file = "motion_planning.yaml"
    ros2_controller_file = "ros2_controllers.yaml"
    
    gazebo_pkg_share = get_package_share_directory("hc10dtp_gzsim")
    moveit_pkg_share = get_package_share_directory("hc10dtp_moveit_config")

    # Conditioned vaiables
    use_sim_time = {"use_sim_time": True if use_gz_controller == "True" else False}
    urdf_pkg_share = gazebo_pkg_share if use_gz_controller == "True" else moveit_pkg_share

    robot_description_path = os.path.join(urdf_pkg_share, "config", urdf_file)
    rviz_config_path = os.path.join(moveit_pkg_share, "config", rviz_config_file)
    motion_planning_path = os.path.join(moveit_pkg_share, "config", motion_planning_file)
    ros2_controllers_path = os.path.join(moveit_pkg_share, "config", ros2_controller_file)

    # Create configuration for move_group
    moveit_config = (
        MoveItConfigsBuilder(robot_name="hc10dtp", package_name= "hc10dtp_moveit_config")
        .robot_description(file_path= robot_description_path)
        .moveit_cpp(file_path= motion_planning_path)
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("hc10dtp_moveit_config")
        .yaml("config/servo_config.yaml")
        .to_dict()
    }

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}

#endregion
#region Nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            use_sim_time
        ],
        condition=IfCondition(use_rviz)
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=UnlessCondition(use_gz_controller),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    hc10dtp_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hc10dtp_controller", "-c", "/controller_manager"],
    )
    
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            moveit_config.to_dict(),
            use_sim_time,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
            {'publish_robot_description_semantic': True}            #TODO make sure RobotState is published
        ],
    )
#endregion
#region NodeContainer
    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    moveit_config.to_dict(),
                    use_sim_time,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[use_sim_time, moveit_config.robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "base_link", "frame_id": "world"}],
            ),
        ],
        output="screen",
    )
#endregion
#region LaunchDescription
    return LaunchDescription(
        [
            use_gz_controller_arg,
            use_rviz_arg,
            stand_alone_arg,
            rviz_node,
            ros2_control_node,
            container,
            servo_node,
            move_group_node,
            joint_state_broadcaster_spawner,
            hc10dtp_controller_spawner,
        ]
    )
#endregion