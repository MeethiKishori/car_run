import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():

    # Paths to other packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory('f1tenth_description') 
    pkg_sim_env = get_package_share_directory('f1tenth_sim_env')

    # Paths to config files from the existing packages
    robot_description_path = os.path.join(pkg_description, 'urdf', 'racecar.xacro')
    controller_config_path = os.path.join(pkg_description, 'config', 'controllers.yaml')

    # --- CRITICAL FIX: Generate URDF using XACRO with the gazebo:=true argument ---
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', 
        robot_description_path, ' ', 
        'gazebo:=true' 
    ])

    # 1. Start Gazebo
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'gz_args': '-s libgazebo_ros_init.so -s libgazebo_ros_factory.so'}.items(),
    )

    # 2. Publish Robot State
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 3. Spawn the Car
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'f1tenth_racecar'],
        output='screen',
    )

    # 4. ROS 2 Control Node (Delayed Start for Stability)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config_path, 
            {'robot_description': robot_description_content}
        ],
        output='screen'
    )

    # 5. Controller Commands (Delayed and Sequential)
    # Note: We must ensure robot_state_publisher and spawn_entity run first.
    # We assume 15 seconds is enough time for Gazebo and the Manager to start.

    # Sequentially delay subsequent commands
    delayed_manager = TimerAction(
        period=15.0,
        actions=[controller_manager_node]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'configure', 'joint_state_broadcaster'],
        output='screen'
    )
    start_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', 'joint_state_broadcaster', 'start'],
        output='screen'
    )

    load_ackermann_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'configure', 'ackermann_steering_controller'],
        output='screen'
    )
    start_ackermann_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', 'ackermann_steering_controller', 'start'],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        spawn_entity,

        # Sequenced control commands start late
        delayed_manager,
        TimerAction(period=16.0, actions=[load_joint_state_broadcaster]),
        TimerAction(period=17.0, actions=[start_joint_state_broadcaster]),
        TimerAction(period=18.0, actions=[load_ackermann_controller]),
        TimerAction(period=19.0, actions=[start_ackermann_controller]),
    ])