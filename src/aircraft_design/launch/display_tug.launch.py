#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('aircraft_design')
    xacro_file = os.path.join(pkg_share, 'urdf', 'tug.urdf.xacro')
    map_file = os.path.join(pkg_share, 'maps', 'halle_straight_map.yaml')

    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

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

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_robot_tf',
        arguments=['-609.9134', '-649.9574', '0', '0.5236', '0', '0', 'map', 'tug_base_link']
    )

    
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    src_dir = os.path.join(ws_root, 'src', 'aircraft_design')
    rviz_config = os.path.join(src_dir, 'rviz', 'tug.rviz')
    
    
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
        map_server_node,
        lifecycle_manager_node,
        rsp_node,
        jsp_gui_node,
        static_tf_node,
        rviz_node
    ])