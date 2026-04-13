from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aircraft_design'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Copies all your Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Copies all your URDF models
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        
        # 3. Copies your RViz settings (Crucial for Week 1)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        
        # 4. Copies your Trajectory CSV files (Needed for Week 2)
        (os.path.join('share', package_name, 'data'), glob('data/*')),
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*.json')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Registers your Week 2 Trajectory Node
            
            'trajectory_player = aircraft_design.trajectory_player:main',
            'path_publisher = aircraft_design.path_publisher:main',
            'dynamic_pushback_planner = aircraft_design.dynamic_pushback_planner:main',
            'simulation_inputs = aircraft_design.simulation_inputs:main',
            'dynamic1_pushback_planner = aircraft_design.dynamic1_pushback_planner:main',
            'dynamic2_pushback_planner = aircraft_design.dynamic2_pushback_planner:main',
            'swept_path_mapper = aircraft_design.swept_path_mapper:main',
            'hangar_map_publisher = aircraft_design.hangar_map_publisher:main',
            'csv_trajectory_player = aircraft_design.csv_trajectory_player:main',
            'dynamic_hangar_driver = aircraft_design.dynamic_hangar_driver:main',
            'hangar_dynamic_planner = aircraft_design.hangar_dynamic_planner:main',
        ],
    },
)