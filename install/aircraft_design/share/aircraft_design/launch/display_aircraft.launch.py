import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('aircraft_design')
    aircraft_urdf = os.path.join(pkg_dir, 'urdf', 'airplane.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'config.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', name='aircraft_state_pub',
            parameters=[{'robot_description': open(aircraft_urdf).read()}],
           
            remappings=[('/robot_description', '/aircraft/robot_description_v3')]
        ),
        Node(package='tf2_ros', executable='static_transform_publisher', name='world_to_aircraft',
             arguments=['0', '0', '0', '0', '0', '0', 'world', 'aircraft/base_link']),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config])
    ])