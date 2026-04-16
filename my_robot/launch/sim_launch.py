import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('my_robot')
    
    # Try to get f1tenth_description, but don't fail if it's not installed
    try:
        pkg_f1tenth = get_package_share_directory('f1tenth_description')
        # Point to models/ subdirectory where model.config files exist
        f1tenth_models_path = os.path.join(pkg_f1tenth, 'models')
        model_path = f1tenth_models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    except:
        model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    # Set Gazebo model path to find meshes
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )
    
    # IMPORTANT: Don't override GAZEBO_RESOURCE_PATH, just append to it
    # Gazebo needs its default resources or it crashes
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.environ.get('GAZEBO_RESOURCE_PATH', '') + ':' + 
              os.path.join(pkg_robot, 'urdf')
    )

    # Paths
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='mitrack',
        description='World to load: mitrack , empty_world'
    )
    
    world_name = LaunchConfiguration('world')
    # Construct world path (this will be evaluated at launch time)
    world_path_base = os.path.join(pkg_robot, 'worlds')
    xacro_path = os.path.join(pkg_robot, 'urdf', 'xacros', 'race.xacro')
    

    # Generate URDF from XACRO
    robot_description_content = xacro.process_file(xacro_path).toxml()
    
    # Replace package:// with file:// for Gazebo mesh loading
    robot_description_content = robot_description_content.replace(
        'package://my_robot/urdf/meshes/',
        'file://' + os.path.join(pkg_robot, 'urdf', 'meshes') + '/'
    )

    # Robot state publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Gazebo launch with dynamic world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': [TextSubstitution(text=world_path_base + '/'), world_name, TextSubstitution(text='.world')],
            'verbose': 'true'
        }.items()
    )

    # Spawn entity after 5 seconds delay (reduced from 20)
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'racecar',
                    '-x', '1.0',
                    '-y', '-7.0',
                    '-z', '1.0'
                ],
                output='screen'
            )
        ]
    )

    wall_follower_node = TimerAction(
    period=6.0,  # after car spawn
    actions=[
        Node(
            package='my_racing_agent',
            executable='drive_node.py',
            #executable='wall_log.py',   #for logging
            name='wall_follower_node',
            output='screen',
            emulate_tty=True,
        )
    ]
)

    return LaunchDescription([
        world_arg,
        gazebo_model_path,
        gazebo_resource_path,
        gazebo,
        rsp_node,
        spawn_entity,
        wall_follower_node
    ])