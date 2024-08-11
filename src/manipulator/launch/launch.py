from launch_ros.actions import Node
from launch import LaunchDescription
import os 
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('ur_description'),
                             'urdf','ur5.xacro')
    
    # rviz_config_path = os.path.join(get_package_share_path('arm2'),
    #                                 'rviz','urdf_config.rviz')

    robot_description = ParameterValue(Command(['xacro ',urdf_path]),value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        # arguments=['-d', rviz_config_path]
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_path('gazebo_ros'),'launch'),"/gazebo.launch.py"])
        
    )

    spawn_node = Node( 
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=['-topic', 'robot_description','-entity', 'arm2'], 
        output='screen'
    )



    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        gazebo_node,
        spawn_node

    ])