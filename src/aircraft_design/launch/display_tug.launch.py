#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('aircraft_design')
    xacro_file = os.path.join(pkg_share, 'urdf', 'tug.urdf.xacro')
    
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Isolated RViz config (Forced to SRC directory so Ctrl+S saves permanently!)
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    src_dir = os.path.join(ws_root, 'src', 'aircraft_design')
    rviz_config = os.path.join(src_dir, 'rviz', 'tug.rviz')
    
    # Create the file instantly if it doesn't exist yet, avoiding RViz crash
    if not os.path.exists(rviz_config):
        open(rviz_config, 'a').close()
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])