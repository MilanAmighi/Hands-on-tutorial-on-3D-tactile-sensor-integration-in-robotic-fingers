from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_tactile_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'Resources'),
            glob('ros2_tactile_robot/Resources/*')),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Milan Amighi',
    maintainer_email='Milan.Francois.T.Amighi@vub.be',
    description='ROS2 nodes for the 3D tactile sensor robotic finger system.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp_bridge_node    = ros2_tactile_robot.esp_bridge_node:main',
            'joystick_node      = ros2_tactile_robot.joystick_node:main',
            'potentiometer_node = ros2_tactile_robot.potentiometer_node:main',
            'motor_node         = ros2_tactile_robot.motor_node:main',
            'sensor_node        = ros2_tactile_robot.sensor_node:main',
            'lcd_node           = ros2_tactile_robot.lcd_node:main',
            'gui_node           = ros2_tactile_robot.gui_node:main',
            'pid_control_node   = ros2_tactile_robot.pid_control_node:main',
            'pid_gui_node	= ros2_tactile_robot.pid_gui_node:main',
            'tactile_finger_gui_node = ros2_tactile_robot.tactile_finger_gui_node:main',
        ],
    },
)
