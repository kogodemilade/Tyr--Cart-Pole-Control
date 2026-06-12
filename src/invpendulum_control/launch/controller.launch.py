from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
import os
from ament_index_python import get_package_share_directory, get_package_prefix


# def noisy_controller(context, *args, **kwargs):
#     wheel_radius = float(LaunchConfiguration('wheel_radius').perform(context))
#     wheel_sep = float(LaunchConfiguration('wheel_seperation').perform(context))
#     wheel_radius_err = float(LaunchConfiguration('wheel_radius_error').perform(context))
#     wheel_sep_err = float(LaunchConfiguration('wheel_seperation_error').perform(context))
#     noisy_controller_py = Node(
#         package="bumperbot_controller",
#         executable="noisy_controller",
#         parameters=[{
#             "wheel_radius": wheel_radius + wheel_radius_err,
#             "wheel_seperation": wheel_sep + wheel_sep_err
#         }]
#     )
#     return [noisy_controller_py]


def generate_launch_description():
    sim_time_parameter = SetParameter(name='use_sim_time', value=True)

    # use_python_arg = DeclareLaunchArgument(
    #     "use_python",
    #     default_value="True",
    #     description="Choose whether to use the python script or Cpp script."
    # )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.1",
        description="Radius of wheels"
    )

    pendulum_length_arg = DeclareLaunchArgument(
        "pendulum_length",
        default_value="0.56",
        description="Distance between the center of both wheels."
    )


    kp_arg = DeclareLaunchArgument(
        "kp",
        default_value="18.2",
        description="Proportional gain"
    )

    kd_arg = DeclareLaunchArgument(
        "kd",
        default_value="5.0",
        description="Derivative gain"
    )

    ki_arg = DeclareLaunchArgument(
        "ki",
        default_value="1.8",
        description="Integral gain"
    )

    # wheel_radius_err = DeclareLaunchArgument(
    #     "wheel_radius_error",
    #     default_value="0.005",
    #     description="amount of error in error read"
    # )

    # wheel_sep_err = DeclareLaunchArgument(
    #     "wheel_seperation_error",
    #     default_value="0.01",
    #     description="Use custom simple controller "
    # )

    use_pid_controller_arg = DeclareLaunchArgument(
        "use_pid_controller",
        default_value="False",
        description="Use pid controller"
    )

    use_pid_controller = LaunchConfiguration("use_pid_controller")

    
    wheel_radius = LaunchConfiguration("wheel_radius")
    pendulum_length = LaunchConfiguration("pendulum_length")
    kp = LaunchConfiguration("kp")
    kd = LaunchConfiguration("kd")
    ki = LaunchConfiguration("ki")
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]

    )
    velocity_controller = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
                parameters=[{'use_sim_time': True}]
            )
    
    effort_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['effort_controller', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}]
        )

    pid_controller = GroupAction(
    condition = IfCondition(use_pid_controller),
    actions= [Node(
                package="invpendulum_control",
                executable="pid_controller",
                parameters=[{"wheel_radius": wheel_radius, "pendulum_length": pendulum_length, "kp": kp, "kd": kd, "ki": ki}, {'use_sim_time': True}]
            )]
    )

    lqr_controller = GroupAction(
    condition = UnlessCondition(use_pid_controller),
    actions= [Node(
                package="invpendulum_control",
                executable="lqr_controller",
                parameters=[{"wheel_radius": wheel_radius, "pendulum_length": pendulum_length}, {'use_sim_time': True}]
            )]
    )


    # # noisy_controller_spawner = OpaqueFunction(function=noisy_controller)
    # noisy_controller_ = GroupAction(
    #     condition = UnlessCondition(use_simple_controller),
    #     actions=[OpaqueFunction(function=noisy_controller)]
    #     )

    ld = LaunchDescription()
    ld.add_action(sim_time_parameter)
    ld.add_action(use_pid_controller_arg)
    ld.add_action(kp_arg)
    ld.add_action(kd_arg)
    ld.add_action(ki_arg)
    ld.add_action(wheel_radius_arg)
    ld.add_action(pendulum_length_arg)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(effort_controller)
    ld.add_action(pid_controller)
    ld.add_action(lqr_controller)
    return ld

