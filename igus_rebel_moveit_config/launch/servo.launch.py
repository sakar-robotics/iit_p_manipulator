import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("igus_de")
        .robot_description(file_path="urdf/igus_rebel_robot2.urdf.xacro")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("igus_rebel_moveit_config", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('igus_rebel_moveit_config'),
        'launch',
        'moveit.rviz'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    
    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),           
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    joint_limits_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_moveit_config"),
            "config",
            "joint_limits.yaml",
        ]
    )

    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("igus_rebel_description"),
            "urdf",
            "igus_rebel_robot2.urdf.xacro",
        ]
    )
    hardware_protocol = 'rebel'
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
            FindPackageShare("igus_rebel_moveit_config"),
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
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        parameters=[
            servo_params,
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic.perform(context)},
            kinematics_config,
        ],
        output='screen'
    )

    return LaunchDescription(
        [
            rviz_node,
            servo_node,
            # container,
        ]
    )