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
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        #Now copies BOTH .urdf and .xacro files!
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
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
            'hangar_map_publisher = aircraft_design.hangar_map_publisher:main',
            'csv_trajectory_player = aircraft_design.csv_trajectory_player:main',
            'tug_trajectory_player = aircraft_design.tug_trajectory_player:main',
            'hangar_dynamic_planner = aircraft_design.hangar_dynamic_planner:main',
        ],
    },
)