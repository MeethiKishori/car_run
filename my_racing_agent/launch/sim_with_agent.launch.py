import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    # --- 1. SETUP VARIABLES ---
    CAR_MODEL_PKG = 'my_racing_agent' 
    YOUR_AGENT_PKG = 'my_racing_agent'
    
    pkg_share_dir = get_package_share_directory(CAR_MODEL_PKG)
    
    # 1. Path to the main car model XACRO file 
    # MODIFIED: Changed 'macros.xacro' to 'racecar.xacro' (the main car definition)
    car_model_path = os.path.join(pkg_share_dir, 'urdf', 'racecar.xacro')
    
    # 2. Path to the directory containing all your Gazebo models (e.g., walker_racecourse)
    models_path = os.path.join(pkg_share_dir, 'models')
    
    # 3. Path to load the desired world file. We'll use the basic MIT_Tunnel model's SDF 
    # to function as the world environment, as per typical F1TENTH setups.
    gazebo_world_path = os.path.join(pkg_share_dir, 'models', 'MIT_Tunnel', 'model.sdf')

    # --- 2. ENVIRONMENT SETUP ---
    # CRITICAL: Tell Gazebo where to look for your local models (cone, walker_racecourse)
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), models_path],
    )
    
    # --- 3. XACRO TO URDF PROCESSING ---
    # Convert the XACRO file into a readable URDF XML string 
    robot_description_content = Command(['xacro', car_model_path])

    # --- 4. LAUNCH GAZEBO ---
    # Launches gzserver and gzclient using the MIT_Tunnel model as the base environment
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': gazebo_world_path}.items()
    )

    # --- 5. ROBOT STATE PUBLISHER (RSP) ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,'use_sim_time': True}],
    )

    # --- 6. GAZEBO ENTITY SPAWNER (Car) ---
    # Spawns the car model defined by the robot_description topic
    spawn_car_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'f1tenth_car', 
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.1' # Lift car slightly above ground
        ],
        output='screen',
    )
    
    # --- 7. GAZEBO ENTITY SPAWNER (Track Model) ---
    # Spawns the walker_racecourse model by referencing its folder name in the -database argument.
    spawn_track_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-database', 'walker_racecourse', 
            '-entity', 'track_model',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.0'
        ],
        output='screen',
    )
    
    # --- 8. YOUR WALL FOLLOWER NODE ---
    wall_follower_node = Node(
        package=YOUR_AGENT_PKG,
        executable='drive_node',
        name='wall_follower_node',
        output='screen',
        emulate_tty=True,
    )

    # --- 9. RETURN LAUNCH DESCRIPTION ---
    return LaunchDescription([
        set_model_path, # Must be set before Gazebo launches
        gazebo,
        rsp_node,
        spawn_car_node,
        spawn_track_node, # Spawns the track model
        wall_follower_node,
    ])