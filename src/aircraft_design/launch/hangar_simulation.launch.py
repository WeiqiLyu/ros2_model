#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('aircraft_design')

    # 1. LOAD URDF STRICTLY FOR ROBOT STATE PUBLISHER (No GUI Slider!)
    # Using xacro.process_file is cleaner and safer in ROS 2 than manual open()
    xacro_file = os.path.join(pkg_share, 'urdf', 'system.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. START THE MAP PUBLISHER
    map_node = Node(
        package='aircraft_design',
        executable='hangar_map_publisher',
        name='hangar_map_publisher',
        output='screen'
    )

    # 3. START THE CSV PLAYER (Delayed by 3 seconds)
    # This delay allows RViz and Robot State Publisher to build the TF tree first,
    # preventing "Frame [map] does not exist" errors on startup!
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

    # 4. START RVIZ (Using your dedicated pushback.rviz config file)
    rviz_config = os.path.join(pkg_share, 'rviz', 'pushback.rviz')
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