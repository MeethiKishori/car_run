import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('my_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_robot, 'config', 'slam', 'slam_toolbox_params.yaml'),
        description='Full path to the SLAM parameters file'
    )
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_robot, 'config', 'nav2', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )
    
    # Set Gazebo paths
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=pkg_robot + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.environ.get('GAZEBO_RESOURCE_PATH', '') + ':' + 
              os.path.join(pkg_robot, 'urdf')
    )

    # Paths
    world_path = os.path.join(pkg_robot, 'worlds', 'mitrack.world')
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
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # Spawn entity
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

    # ============================================================
    # SLAM TOOLBOX - For mapping
    # ============================================================
    slam_toolbox = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    # ============================================================
    # NAV2 STACK - For navigation
    # ============================================================
    nav2_bringup = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # ============================================================
    # RVIZ2 - For visualization
    # ============================================================
    rviz_config_file = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    
    rviz_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # ============================================================
    # Optional: Wall follower for autonomous exploration
    # ============================================================
    # wall_follower_node = TimerAction(
    #     period=15.0,
    #     actions=[
    #         Node(
    #             package='my_racing_agent',
    #             executable='wall_log.py',
    #             name='wall_follower_node',
    #             output='screen',
    #             emulate_tty=True,
    #         )
    #     ]
    # )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        declare_nav2_params_file_cmd,
        gazebo_model_path,
        gazebo_resource_path,
        gazebo,
        rsp_node,
        spawn_entity,
        slam_toolbox,
        nav2_bringup,
        rviz_node,
        # wall_follower_node,  # Uncomment for autonomous mapping
    ])