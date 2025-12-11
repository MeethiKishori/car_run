import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. --- Configuration & Arguments ---
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory('f1tenth_description') 
    
    # CRITICAL FIX: Hardcoded URDF file path
    urdf_xml_path = '/tmp/f1tenth_racecar.urdf'
    
    # CRITICAL FIX for FOXY: Read the URDF file content using standard Python
    try:
        with open(urdf_xml_path, 'r') as infp:
            robot_description_content = infp.read()
        print(f"INFO: Successfully read URDF content from {urdf_xml_path}")
    except EnvironmentError:
        print(f"FATAL ERROR: Failed to read URDF file at {urdf_xml_path}. Did you run the 'ros2 run xacro...' command?")
        robot_description_content = ''

    controller_config_path = os.path.join(pkg_description, 'config', 'controllers.yaml')
    
    # 2. --- Launch Gazebo and Robot State Publishers ---
    
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'gz_args': '-s libgazebo_ros_init.so -s libgazebo_ros_factory.so'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # Pass content directly
        parameters=[{'robot_description': robot_description_content}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # Spawning by topic is the reliable method here
        arguments=['-topic', 'robot_description', '-entity', 'f1tenth_racecar'],
        output='screen',
    )
    
    # 3. --- Controller and Drive Node Definitions ---
    
    # Pass the robot_description directly to the Controller Manager node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config_path, 
            {'robot_description': robot_description_content} # Passed file content
        ],
        output='screen'
    )
    
    # Control startup commands (unchanged timing)
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
    
    drive_node = Node(
        package='my_racing_agent',
        executable='drive_node',
        name='wall_follower_node',
        output='screen',
    )
    
    # 4. --- Sequential Timing (TimerAction) - Foxy Compatible ---
    
    # Start the Controller Manager after a generous delay (15s)
    delayed_manager = TimerAction(
        period=15.0,
        actions=[controller_manager_node]
    )
    
    # Sequentially delay subsequent commands
    delayed_load1 = TimerAction(
        period=16.0,
        actions=[load_joint_state_broadcaster]
    )
    
    delayed_start1 = TimerAction(
        period=17.0,
        actions=[start_joint_state_broadcaster]
    )
    
    delayed_load2 = TimerAction(
        period=18.0,
        actions=[load_ackermann_controller]
    )
    
    delayed_start2 = TimerAction(
        period=19.0,
        actions=[start_ackermann_controller]
    )

    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        spawn_entity,
        drive_node, 
        
        # Sequenced control commands start late
        delayed_manager,
        delayed_load1,
        delayed_start1,
        delayed_load2,
        delayed_start2,
    ])