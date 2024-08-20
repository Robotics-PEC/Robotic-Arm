from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument(
        "log_level",
        default_value="error",
        choices=["info", "warn", "error"],
        description="log level",
    ),
    DeclareLaunchArgument(
        "model",
        default_value="ur5.xacro",
        choices=[
            "ur3.xacro",
            "ur3e.xacro",
            "ur5.xacro",
            "ur5e.xacro",
            "ur10.xacro",
            "ur10e.xacro",
            "ur16e.xacro",
            "ur20.xacro",
            "ur30.xacro",
        ],
        description="Model to spawn",
    ),
]


def generate_launch_description():
    # Get the path of the ur_description package
    pkg_ur_description = get_package_share_directory("ur_description")

    # Ensure pkg_ur_description is converted to string before using it
    pkg_ur_description = str(pkg_ur_description)

    # Construct the path to the Xacro file
    xacro_file = PathJoinSubstitution(
        [pkg_ur_description, "urdf", LaunchConfiguration("model")]
    )

    robot_desc = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    rviz_config_path = os.path.join(
        get_package_share_directory("ur_description"), "rviz", "default.rviz"
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", arguments=["-d", rviz_config_path]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    ur_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ur_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": robot_desc},
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription(ARGUMENTS + [ur_state_publisher,joint_state_publisher_gui,rviz_node])
