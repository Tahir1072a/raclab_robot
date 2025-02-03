import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    raclab_robot_description_dir = get_package_share_directory("raclab_robot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
        raclab_robot_description_dir,
        "urdf",
        "raclab_robot_core.urdf.xacro"
    ))

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
            raclab_robot_description_dir,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(raclab_robot_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("raclab_robot_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )

    raclab_robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": raclab_robot_description, 
            "use_sim_time": True }]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
            )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "raclab_robot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ]
    )
    
    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])