from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_gui',
        default_value='False',
        choices=['True', 'False'],  
        description='Use MoveIt RViz'),
    DeclareLaunchArgument(
        'servo_control',
        default_value='none',
        choices=['joystick', 'keyboard', 'none'],
        description='Control method for the robot when using moveit servo'),
    DeclareLaunchArgument(
        "launch_mode",
        default_value="both",
        choices=["move_group", "servo", "both"],
        description="Specify which nodes to launch: move_group, servo, or both.",
    )
]


def generate_launch_description():
    """Launch the robot in Gazebo and RViz."""
    # Launch configuration and arguments
    use_rviz = LaunchConfiguration('use_gui')
    servo_control = LaunchConfiguration('servo_control')
    launch_mode = LaunchConfiguration('launch_mode')

    # Package directories
    igus_rebel_pkg = get_package_share_directory('igus_rebel')
    igus_rebel_moveit_config = get_package_share_directory('igus_rebel_moveit_config')
    

    # File paths
    igus_rebel_file_path = PathJoinSubstitution(
        [igus_rebel_pkg, 'launch', 'rebel.launch.py'])
    move_group_file_path = PathJoinSubstitution(
        [igus_rebel_moveit_config, 'launch', 'move_group.launch.py'])
    # Launch Files
    rebel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(igus_rebel_file_path),
        launch_arguments=[('hardware_protocol', 'rebel')],
    )

    motion_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_file_path),
        launch_arguments=[
            ('use_gui', use_rviz),
            ('launch_mode', launch_mode),
        ],
    )

    joystick_control_servo = Node(
        package='igus_rebel',
        executable='joystick_servo',
        name='joystick_input_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", servo_control, "' == 'joystick'"])
        ),
    )

    keyboard_control_servo = Node(
        package='igus_rebel',
        executable='keyboard_input',
        name='keyboard_input_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", servo_control, "' == 'keyboard'"])
        ),
    )

    ld = LaunchDescription(ARGUMENTS)
    # Launch
    ld.add_action(rebel_launch)
    ld.add_action(motion_planner_launch)
    # Nodes
    ld.add_action(joystick_control_servo)
    ld.add_action(keyboard_control_servo)
    return ld
