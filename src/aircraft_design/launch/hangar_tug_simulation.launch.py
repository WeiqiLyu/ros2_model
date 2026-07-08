#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('aircraft_design')

    # 1. Load standalone Tug URDF strictly
    xacro_file = os.path.join(pkg_share, 'urdf', 'tug.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. Start Hangar Map Publisher
    map_node = Node(
        package='aircraft_design',
        executable='hangar_map_publisher',
        name='hangar_map_publisher',
        output='screen'
    )

    # 3. Start Tug Trajectory Player (Delayed by 3 seconds for clean TF initialization)
    trajectory_player_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='aircraft_design',
                executable='tug_trajectory_player',
                name='tug_trajectory_player',
                output='screen'
            )
        ]
    )

    # 4. Isolated RViz config: Guaranteed zero interference with aircraft configs!
    rviz_config = os.path.join(pkg_share, 'rviz', 'tug_simulation.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        map_node,
        trajectory_player_node,
        rviz_node
    ])