import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    pkg_gazebo_ros   = get_package_share_directory('gazebo_ros')
    pkg_robot        = get_package_share_directory('my_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # ── env ───────────────────────────────────────────────────────────────────
    try:
        pkg_f1tenth = get_package_share_directory('f1tenth_description')
        model_path  = os.path.join(pkg_f1tenth, 'models') + ':' + \
                      os.environ.get('GAZEBO_MODEL_PATH', '')
    except Exception:
        model_path = os.environ.get('GAZEBO_MODEL_PATH', '')

    set_model_path    = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)
    set_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        os.environ.get('GAZEBO_RESOURCE_PATH', '') + ':' +
        os.path.join(pkg_robot, 'urdf')
    )

    # ── args ──────────────────────────────────────────────────────────────────
    world_arg       = DeclareLaunchArgument('world', default_value='mitrack')
    world_name      = LaunchConfiguration('world')
    world_path_base = os.path.join(pkg_robot, 'worlds')

    nav2_params_file = os.path.join(pkg_robot, 'config', 'nav2', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_robot, 'config', 'slam', 'slam_toolbox_params.yaml')

    sim = {'use_sim_time': True} 

    # ── robot description ─────────────────────────────────────────────────────
    xacro_path = os.path.join(pkg_robot, 'urdf', 'xacros', 'race.xacro')
    robot_desc = xacro.process_file(xacro_path).toxml().replace(
        'package://my_robot/urdf/meshes/',
        'file://' + os.path.join(pkg_robot, 'urdf', 'meshes') + '/'
    )

    # ── 1. Robot state publisher ──────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, **sim}],
        output='screen'
    )

    bt_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_distance.xml',
    ])

    # ── 2. Gazebo ─────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': [
            TextSubstitution(text=world_path_base + '/'),
            world_name,
            TextSubstitution(text='.world'),
        ]}.items()
    )

    # ── 3. Spawn robot ────────────────────────────────────────────────────────
    spawn = TimerAction(period=4.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'racecar',
                       '-x', '1.0', '-y', '-7.0', '-z', '1.0'],
            output='screen'
        )
    ])

    base_link_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_alias_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'car_1_base_link', 'base_link'],
        parameters=[sim],
        output='screen'
    )

    # ── 4. SLAM toolbox ───────────────────────────────────────────────────────
    slam = TimerAction(period=7.0, actions=[
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, sim],
        )
    ])

    # ── 5. Nav2 nodes ─────────────────────────────────────────────────────────
    controller = TimerAction(period=12.0, actions=[
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file, sim],
            remappings=[
                ('/odom',    '/car_1/odom'),
                ('/cmd_vel', '/car_1/cmd_vel'),
            ]
        )
    ])

    planner = TimerAction(period=12.0, actions=[
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file, sim],
        )
    ])

    recoveries = TimerAction(period=12.0, actions=[
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params_file, sim],
            remappings=[
                ('/cmd_vel', '/car_1/cmd_vel'),
            ]
        )
    ])

    bt_navigator = TimerAction(period=12.0, actions=[
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            respawn=True,
            respawn_delay=2.0,
            output='screen',
            parameters=[
                nav2_params_file,
                sim,
                {
                    'default_bt_xml_filename': bt_xml,
                    'robot_base_frame': 'car_1_base_link',
                    'global_frame': 'map',
                    'odom_topic': '/car_1/odom',
                },
            ],
            remappings=[
                ('/odom', '/car_1/odom'),
            ]
        )
    ])

    waypoint_follower = TimerAction(period=12.0, actions=[
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_file, sim],
        )
    ])

    lifecycle_manager = TimerAction(period=14.0, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                **sim,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                    'waypoint_follower',
                ],
            }]
        )
    ])

    # ── 6. RViz ───────────────────────────────────────────────────────────────
    rviz_cfg = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    rviz = TimerAction(period=16.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg, '-f', 'map'],
            parameters=[sim],
            output='screen',
            remappings=[
                ('/scan', '/car_1/scan'),
            ]
        )
    ])

    return LaunchDescription([
        world_arg,
        set_model_path,
        set_resource_path,
        gazebo,
        rsp,
        base_link_alias_tf,
        spawn,
        slam,
        controller,
        planner,
        recoveries,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        rviz,
    ])