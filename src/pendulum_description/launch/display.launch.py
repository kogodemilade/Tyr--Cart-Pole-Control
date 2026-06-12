from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch_ros.actions import SetParameter
import os
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    sim_time_parameter = SetParameter(name='use_sim_time', value=True)
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("pendulum_description"), 
            "urdf", 
            "pendulum.urdf.xacro"
        ),
        description="Absolute path to URDF file"
    )
    
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), 
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name='joint_state_publisher',
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("pendulum_description"), 
            "rviz", 
            "display.rviz"
        )],
    )
    
    return LaunchDescription([
        # sim_time_parameter,
        model_arg, 
        robot_state_publisher,
        joint_state_publisher_gui,  # Remove TimerAction, not needed
        rviz_node,
    ])