#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('aircraft_design')
    
    # Paths
    xacro_file = os.path.join(pkg_dir, 'urdf', 'system.urdf.xacro')
    
    # Path to your new map file
    map_file = os.path.join(pkg_dir, 'maps', 'halle_straight_map.yaml')
    
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir))))
    src_dir = os.path.join(ws_root, 'src', 'aircraft_design')
    rviz_config = os.path.join(src_dir, 'rviz', 'config.rviz')

    if not os.path.exists(rviz_config):
        open(rviz_config, 'a').close()

    return LaunchDescription([
        
        # 1. MAP SERVER: Loads the .yaml and .png files
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        ),
        
        # 2. LIFECYCLE MANAGER: Activates the Map Server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        # 3. STATIC TRANSFORM: Places the Tractor onto the Map
        # Arguments: [X, Y, Z, Yaw, Pitch, Roll, Parent, Child]
        # Change the first two '0's to move the robot's X and Y starting position!
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_robot_tf',
            arguments=['-609.9134', '-649.9574', '0', '0.5236', '0', '0', 'map', 'tractor_base_link']
        ),

        # 4. ROBOT STATE PUBLISHER: Computes your URDF math
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        
        # 5. JOINT STATE PUBLISHER: Handles the tow_joint
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # 6. RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])