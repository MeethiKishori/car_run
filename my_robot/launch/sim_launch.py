import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('my_robot')
    
    # Set Gazebo model path to find meshes
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=pkg_robot + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    
    # IMPORTANT: Don't override GAZEBO_RESOURCE_PATH, just append to it
    # Gazebo needs its default resources or it crashes
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.environ.get('GAZEBO_RESOURCE_PATH', '') + ':' + 
              os.path.join(pkg_robot, 'urdf')
    )

    # Paths
    #world_path = os.path.join(pkg_robot, 'worlds', 'empty_world.world')
    world_path = os.path.join(pkg_robot, 'worlds', 'mitrack.world')
    #world_path = os.path.join(pkg_robot, 'worlds', 'silverstone_track.world')
    #world_path = os.path.join(pkg_robot, 'worlds', 'interlagos_track.world')
    #world_path = os.path.join(pkg_robot, 'worlds', 'monza_track.world')
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

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
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
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1'
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
        gazebo_model_path,
        gazebo_resource_path,
        gazebo,
        rsp_node,
        spawn_entity,
        wall_follower_node
    ])