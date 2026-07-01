import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('aircraft_design')
    aircraft_urdf = os.path.join(pkg_dir, 'urdf', 'airplane.urdf')
    tug_urdf = os.path.join(pkg_dir, 'urdf', 'tug.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'config.rviz')

    return LaunchDescription([
        # Aircraft Publisher
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='aircraft_state_pub',
             parameters=[{'robot_description': open(aircraft_urdf).read()}],
             remappings=[('/robot_description', '/aircraft/robot_description_v2')]),
        
        # Tug Publisher
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='tug_state_pub',
             parameters=[{'robot_description': open(tug_urdf).read()}],
             remappings=[('/robot_description', '/tug/robot_description_v2')]),

        # Transforms to place them statically
        # 1. Put the Tug in the World
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_tug',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'tug/base_link']
    ),
    # 2. Attach the Aircraft to the Tug
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hitch_to_aircraft',
        #arguments=['4.0', '-0.5', '-0.3', '0', '0', '0', 'tug/hitch_link', 'aircraft/base_link']
        arguments=['0', '0', '0', '0', '0', '0', 'tug/hitch_link', 'aircraft/nose_wheel_gear_link']
    ),

        # RViz
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config])
    ])