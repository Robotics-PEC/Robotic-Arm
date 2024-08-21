from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config_path = os.path.join(
        get_package_share_directory("ur_description"), "rviz", "default.rviz"
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", arguments=["-d", rviz_config_path]
    )

    return LaunchDescription([rviz_node])
