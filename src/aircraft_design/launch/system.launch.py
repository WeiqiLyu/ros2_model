#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_dir = get_package_share_directory('aircraft_design')
    
    # Process the XACRO file instead of a plain URDF
    xacro_file = os.path.join(pkg_dir, 'urdf', 'system.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    
    # Isolated RViz config (Forced to SRC directory so Ctrl+S saves permanently!)
    src_dir = os.path.expanduser('~/flight_ws/src/aircraft_design')
    rviz_config = os.path.join(src_dir, 'rviz', 'config.rviz')

    # Create the file instantly if it doesn't exist yet, avoiding RViz crash
    if not os.path.exists(rviz_config):
        open(rviz_config, 'a').close()

    return LaunchDescription([
        # Robot State Publisher computes all the fixed TF offsets (L_2, L_landing, c)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        
        # Joint State Publisher GUI allows you to test the tow_joint manually!
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz Visualizer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])