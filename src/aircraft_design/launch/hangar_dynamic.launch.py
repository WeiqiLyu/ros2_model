import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_dir = get_package_share_directory('aircraft_design')
    
    # 1. Parse the NEW unified Xacro system instead of the old separate URDFs
    xacro_file = os.path.join(pkg_dir, 'urdf', 'system.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    
    rviz_config = os.path.join(pkg_dir, 'rviz', 'config.rviz')

    return LaunchDescription([
        # 1. The Hangar Map Publisher
        Node(
            package='aircraft_design', 
            executable='hangar_map_publisher', 
            name='hangar_map_publisher', 
            output='screen'
        ),

        # 2. Unified Robot State Publisher (This single node replaces both old Aircraft and Tug nodes!)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        
        # 3. Default Joint State Publisher (Publishes a static 0.0 rad for tow_joint so the TF tree connects)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),

        # 4. The Dynamic Physics Engine (Delayed by 2 seconds to let the map load fully)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='aircraft_design', 
                    executable='hangar_dynamic_planner', 
                    name='hangar_dynamic_planner', 
                    output='screen'
                )
            ]
        ),
        
        # 5. RViz2 Visualizer
        Node(
            package='rviz2', 
            executable='rviz2', 
            name='rviz2', 
            arguments=['-d', rviz_config]
        )
    ])