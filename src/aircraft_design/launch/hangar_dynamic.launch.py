import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('aircraft_design')
    
    # Locate URDFs and RViz config
    aircraft_urdf = os.path.join(pkg_dir, 'urdf', 'airplane.urdf')
    tug_urdf = os.path.join(pkg_dir, 'urdf', 'tug.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'config.rviz')

    return LaunchDescription([
        # 1. The Hangar Map
        Node(
            package='aircraft_design', 
            executable='hangar_map_publisher', 
            name='hangar_map_publisher', 
            output='screen'
        ),

        # 2. Aircraft Visuals
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='aircraft_state_pub',
             parameters=[{'robot_description': open(aircraft_urdf).read()}],
             remappings=[('/robot_description', '/aircraft/robot_description_v2')]),
             
        # 3. Tug Visuals
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='tug_state_pub',
             parameters=[{'robot_description': open(tug_urdf).read()}],
             remappings=[('/robot_description', '/tug/robot_description_v2')]),

        # 4. The Dynamic Physics Engine (Delayed by 2 seconds to let map load)
        TimerAction(
            period=2.0,
            actions=[
                Node(package='aircraft_design', executable='hangar_dynamic_planner', name='hangar_dynamic_planner', output='screen')
            ]
        ),
        
        # 5. RViz
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config])
    ])