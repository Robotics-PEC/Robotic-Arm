from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the XACRO file
    xacro_file = os.path.join(
        get_package_share_directory("ur_description"), "urdf", "ur5.xacro"
    )

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    # Define a Node to launch the robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("universal_robot_5"), "rviz", "default.rviz"
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", arguments=["-d", rviz_config_path]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    return LaunchDescription(
        [robot_state_publisher_node, rviz_node, joint_state_publisher_gui_node]
    )
