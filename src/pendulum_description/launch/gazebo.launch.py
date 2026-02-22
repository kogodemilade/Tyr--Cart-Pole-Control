from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, AppendEnvironmentVariable, LogInfo
from ros_gz_bridge.actions import RosGzBridge
import os
from ament_index_python import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue 
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sim_time_parameter = SetParameter(name='use_sim_time', value=True)
    pendulum_description = get_package_share_directory("pendulum_description")
    pendulum_description_prefix = get_package_prefix("pendulum_description")

    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = '0.0'
    model_path = os.path.join(pendulum_description, 'models') + os.pathsep + os.path.join(pendulum_description_prefix, "share")
    
    xacro_file = os.path.join(pendulum_description, "urdf", "pendulum.urdf.xacro")

    set_env_vars_resources = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path) 

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=xacro_file,
        description="Absolute path to URDF file" 
        )
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    world = os.path.join(
        get_package_share_directory('pendulum_description'),
        'worlds',
        'empty_world.world'
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{"robot_description": robot_description}, {'use_sim_time': True}]

    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={'gz_args': ['-s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
)
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
            launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "pendulum", "-topic", "robot_description", '-x', x_pose,
        '-y', y_pose, '-z', z_pose, "-iterations", "10", "--physics-engine", "bullet"],
        parameters=[{'use_sim_time': True}],
        output="screen",
    )
    
    clock_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='clock_bridge',
    output='screen',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    ],
    )
    
    imu_ros_bridge = Node(
    package="ros_gz_bridge", 
    executable="parameter_bridge", 
    output="screen",
    arguments=["/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
    parameters=[{'use_sim_time': True}])
    
    log_model_path = LogInfo(
        condition=None,
        msg=EnvironmentVariable('GZ_SIM_RESOURCE_PATH')
    )

    ld = LaunchDescription()
    ld.add_action(sim_time_parameter)
    ld.add_action(model_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(clock_bridge)
    ld.add_action(imu_ros_bridge)
    ld.add_action(log_model_path)

    

    return ld