import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot = get_package_share_directory('my_robot')
    urdf_path = os.path.join(pkg_robot, 'urdf', 'racecar.urdf')

    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    return LaunchDescription([rsp_node])
