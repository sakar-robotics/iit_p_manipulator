from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rebel_arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}],
        output="both",
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}],
        output="both",
    )
    
    return LaunchDescription([
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
    ])