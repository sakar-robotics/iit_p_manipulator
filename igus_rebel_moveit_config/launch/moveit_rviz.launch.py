import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_param_builder import load_yaml

def opaque_func(context, *args, **kwargs):
    # Load parameters from move_group.launch.py
    hardware_protocol = LaunchConfiguration('hardware_protocol')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_file = PathJoinSubstitution(
        [
            get_package_share_directory("igus_rebel_description"),
            "urdf",
            "igus_rebel_robot2.urdf.xacro",
        ]
    )
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " hardware_protocol:=",
            hardware_protocol,
        ]
    )

    robot_description_semantic_file = PathJoinSubstitution(
        [
            get_package_share_directory("igus_rebel_moveit_config"),
            "config",
            "igus_rebel2.srdf",
        ]
    )
    robot_description_semantic = Command(
        [
            FindExecutable(name="cat"),
            " ",
            robot_description_semantic_file,
        ]
    )

    kinematics_file = PathJoinSubstitution(
        [
            get_package_share_directory("igus_rebel_moveit_config"),
            "config",
            "kinematics.yaml",
        ]
    )
    kinematics_config = load_yaml(Path(kinematics_file.perform(context)))

    joint_limits_file = PathJoinSubstitution(
        [
            get_package_share_directory("igus_rebel_moveit_config"),
            "config",
            "joint_limits.yaml",
        ]
    )
    joint_limits_config = load_yaml(Path(joint_limits_file.perform(context)))

    default_rviz_file = os.path.join(
        get_package_share_directory('igus_rebel_moveit_config'),
        'launch',
        'moveit.rviz'
    )

    rviz_parameters = [
        {'robot_description': robot_description.perform(context)},
        {'robot_description_semantic': robot_description_semantic.perform(context)},
        {'robot_description_kinematics': kinematics_config},
        {'robot_description_planning': joint_limits_config},
        {'use_sim_time': use_sim_time},
    ]

    # Define the RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_file],
        parameters=rviz_parameters,
    )

    return [rviz_node]


def generate_launch_description():
    # Declare arguments
    hardware_protocol_arg = DeclareLaunchArgument(
        "hardware_protocol",
        default_value="rebel",
        choices=["mock_hardware", "gazebo", "rebel"],
        description="Which hardware protocol or mock hardware should be used",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use sim time if true",
    )

    # Build the launch description
    ld = LaunchDescription()
    ld.add_action(hardware_protocol_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(OpaqueFunction(function=opaque_func))

    return ld