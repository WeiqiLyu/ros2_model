import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('aircraft_design')
    
    # Locate URDFs and RViz config
    aircraft_urdf = os.path.join(pkg_dir, 'urdf', 'airplane.urdf')
    tug_urdf = os.path.join(pkg_dir, 'urdf', 'tug.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'config.rviz')

    return LaunchDescription([
        # 1. Aircraft Visuals
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='aircraft_state_pub',
             parameters=[{'robot_description': open(aircraft_urdf).read()}],
             remappings=[('/robot_description', '/aircraft/robot_description_v2')]),
             
        # 2. Tug Visuals
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='tug_state_pub',
             parameters=[{'robot_description': open(tug_urdf).read()}],
             remappings=[('/robot_description', '/tug/robot_description_v2')]),

        # 3. The Dynamic Physics Engine (This drives the simulation now)
        #Node(package='aircraft_design', executable='csv_trajectory_player', name='csv_trajectory_player', output='screen'),
        Node(package='aircraft_design', executable='dynamic2_pushback_planner', name='dynamic2_pushback_planner', output='screen'),
        
        # 4. RViz
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config])
    ])