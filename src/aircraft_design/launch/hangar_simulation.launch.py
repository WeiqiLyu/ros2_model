import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_aircraft_design = get_package_share_directory('aircraft_design')

    # --- 1. LOAD 3D MODELS & RVIZ ---
    # This includes your existing launch file so we don't have to rewrite the URDF logic
    display_aircraft_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_aircraft_design, 'launch', 'display_system.launch.py')
        )
    )

    # --- 2. START THE HANGAR MAP PUBLISHER ---
    map_publisher_node = Node(
        package='aircraft_design',
        executable='hangar_map_publisher',
        name='hangar_map_publisher',
        output='screen'
    )

    # --- 3. START THE CSV TRAJECTORY PLAYER ---
    # We use a TimerAction to delay the player by 3 seconds. 
    # This gives RViz and the Map time to fully load before the plane starts driving!
    trajectory_player_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='aircraft_design',
                executable='csv_trajectory_player',
                name='csv_trajectory_player',
                output='screen'
            )
        ]
    )

    # --- RETURN THE MASTER LAUNCH DESCRIPTION ---
    return LaunchDescription([
        display_aircraft_cmd,
        map_publisher_node,
        trajectory_player_node
    ])