# rviz'de robotu görüntülemek için oluşturulmuş bir dosya
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    raclab_robot_description_dir = get_package_share_directory("raclab_robot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            raclab_robot_description_dir,
            "urdf",
            "raclab_robot_core.urdf.xacro"
        )
    )

    raclab_robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": raclab_robot_description}]
    )

    robot_joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(raclab_robot_description_dir, "rviz", "description_display.rviz")],
    )

    return LaunchDescription([
        model_arg,
        robot_joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz2
    ])