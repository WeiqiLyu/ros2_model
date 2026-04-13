import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('aircraft_design')
    
    # Paths to the map, URDFs, and RViz config
    map_yaml_file = os.path.join(pkg_share, 'maps', 'airport_map.yaml')
    urdf_tug = os.path.join(pkg_share, 'urdf', 'tug.urdf')
    urdf_aircraft = os.path.join(pkg_share, 'urdf', 'airplane.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')

    return LaunchDescription([
        # 1. Map Server: Loads the PNG and YAML metadata
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]
        ),

        # The Bridge between the Map and the World
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        # 2. Lifecycle Manager: Forces the map_server to start up automatically
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}]
        ),

        # 3. Tug State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='tug_state_pub',
            namespace='tug',
            parameters=[{'robot_description': open(urdf_tug).read()}]
        ),

        # 4. Aircraft State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='aircraft_state_pub',
            namespace='aircraft',
            parameters=[{'robot_description': open(urdf_aircraft).read()}]
        ),

        # 5. The Planner (in Playback Mode reading the CSV)
        Node(
            package='aircraft_design',
            executable='dynamic_pushback_planner',
            name='dynamic_pushback_planner',
            output='screen'
        ),

        # 6. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])