from setuptools import setup
import os
from glob import glob

package_name = 'bug2_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        
    ],
    install_requires=['setuptools', 'bug2_interfaces'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='rocotics@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ROS_RUN_NAME = PACKAGE_NAME.SCRIPT_NAME:main',
            'bug2_navigation = bug2_navigation.bug2_navigation:main',
            'bug2_wall_follow = bug2_navigation.bug2_wall_follow:main',
            'bug2_go_to_point = bug2_navigation.bug2_go_to_point:main',
            'bug2_controller = bug2_navigation.bug2_controller:main',
            'bug2_controller_competition = bug2_navigation.bug2_controller_competition:main',
            'robot_controller = bug2_navigation.robot_controller:main',
        ],
    },
)
