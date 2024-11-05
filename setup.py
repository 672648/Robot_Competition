from setuptools import setup
import os
from glob import glob

package_name = 'competition_robot'

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
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='jonasskaslien@hotmail.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bug2_controller = bug2_navigation.bug2_controller:main',
            'bug2_wall_follow = bug2_navigation.bug2_wall_follow:main',
            'bug2_go_to_point = bug2_navigation.bug2_go_to_point:main',
            'marker_recognition = competition_robot.marker_pose:main',
            'scoring = scoring.scoring:main',
        ],
    },
)
