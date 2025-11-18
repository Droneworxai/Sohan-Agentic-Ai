from setuptools import setup
import os
from glob import glob

package_name = 'lavender_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Droneworx.ai',
    maintainer_email='contact@droneworx.ai',
    description='Autonomous lavender farm robot with waypoint navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_simulator = lavender_bot.robot_simulator:main',
            'waypoint_follower = lavender_bot.waypoint_follower:main',
        ],
    },
)
