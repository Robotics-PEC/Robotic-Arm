from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch.conditions import IfCondition
from launch_ros.actions import Node

# Define launch arguments
ARGUMENTS = [
    DeclareLaunchArgument("world", default_value="basic_world", description="GZ World"),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument(
        "description",
        default_value="true",
        choices=["true", "false"],
        description="Run description",
    ),
    DeclareLaunchArgument(
        "rviz",
        default_value="false",
        choices=["true", "false"],
        description="Run rviz",
    ),
    DeclareLaunchArgument(
        "spawn_model",
        default_value="true",
        choices=["true", "false"],
        description="Spawn Model",
    ),
    DeclareLaunchArgument(
        "model",
        default_value="ur5",
        description="URDF or Xacro model file",
    ),
    DeclareLaunchArgument(
        "model_xacro",
        default_value=[LaunchConfiguration("model"), ".xacro"],
        description="Xacro file for the Model",
    ),
    DeclareLaunchArgument(
        "world_sdf",
        default_value=[LaunchConfiguration("world"), ".sdf"],
        description="World file for simulation",
    ),
]


def generate_launch_description():
    pkg_ur_description = get_package_share_directory("ur_description")

    # Ensure pkg_ur_description is converted to string before using it
    pkg_ur_description = str(pkg_ur_description)

    # Construct the path to the Xacro file
    xacro_file = PathJoinSubstitution(
        [
            pkg_ur_description,
            "urdf",
            LaunchConfiguration("model_xacro"),
        ]
    )

    # Command to process the Xacro file into URDF format
    urdf_file = PathJoinSubstitution(["/tmp", "processed_ur.urdf"])

    # Process the Xacro file into a URDF file
    process_xacro = ExecuteProcess(
        cmd=["xacro", xacro_file, "-o", urdf_file], output="screen"
    )

    # Include the robot description launch file
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ur_description"),
                    "launch",
                    "robot_description.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("description")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": LaunchConfiguration("world_sdf")}.items(),
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ur_description"),
                    "launch",
                    "robot_rviz.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-name",
            LaunchConfiguration("model"),
            "-topic",
            "robot_description",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("spawn_model")),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ur_moveit_bringup"),
                    "launch",
                    "demo.launch.py",
                ]
            )
        )
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Clock (IGN -> ROS2)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Joint states (IGN -> ROS2)
            [
                "/world/",
                LaunchConfiguration("world"),
                "/model/",
                LaunchConfiguration("model"),
                "/joint_state",
                "@sensor_msgs/msg/JointState[gz.msgs.Model",
            ],
        ],
        remappings=[
            (
                [
                    "/world/",
                    LaunchConfiguration("world"),
                    "/model/",
                    LaunchConfiguration("model"),
                    "/joint_state",
                ],
                "joint_states",
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        ARGUMENTS + [process_xacro, gz_sim, moveit, spawn_robot, bridge]
    )
