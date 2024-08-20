from os import environ
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument("x", default_value=["-4.552800"], description="x position"),
    DeclareLaunchArgument("y", default_value=["-2.12171"], description="y position"),
    DeclareLaunchArgument("z", default_value=["1"], description="z position"),
    DeclareLaunchArgument(
        "yaw", default_value=["-0.1300000"], description="yaw position"
    ),
    DeclareLaunchArgument("world", default_value="empty", description="GZ World"),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument(
        "spawn_model",
        default_value="true",
        choices=["true", "false"],
        description="Whether to spawn the UR model",
    ),
]


def generate_launch_description():
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("gz_args", [LaunchConfiguration("world"), ".sdf", " -v 4", " -r"])
        ],
    )

    return LaunchDescription(ARGUMENTS + [gz_sim])
