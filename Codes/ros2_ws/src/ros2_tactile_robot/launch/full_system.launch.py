"""
Launches all nodes of the tactile robot system.

Launch arguments:
  com_port          Serial port for the ESP32 (default: auto-detect)
  gui               Set to 'true' to also start the GUI node (default: true)
  motor_follows_pot Set to 'false' to disable pot → motor (default: true)
  rviz              Set to 'false' to skip RViz2 (default: true)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_tactile_robot')

    com_port_arg = DeclareLaunchArgument(
        'com_port', default_value='',
        description='Serial COM port for ESP32 (empty = auto-detect)')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch the GUI node')

    motor_follows_pot_arg = DeclareLaunchArgument(
        'motor_follows_pot', default_value='true',
        description='Potentiometer drives motor position automatically')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 with tactile visualisation')

    com_port          = LaunchConfiguration('com_port')
    gui               = LaunchConfiguration('gui')
    motor_follows_pot = LaunchConfiguration('motor_follows_pot')
    rviz              = LaunchConfiguration('rviz')

    # ESP Bridge (hardware interface)
    esp_bridge = Node(
        package='ros2_tactile_robot',
        executable='esp_bridge_node',
        name='esp_bridge_node',
        parameters=[{
            'com_port':        com_port,
            'burst_frequency': 500,
            'publish_rate_hz': 50.0,
        }],
        output='screen',
    )

    # Joystick
    joystick = Node(
        package='ros2_tactile_robot',
        executable='joystick_node',
        name='joystick_node',
        parameters=[{
            'max_linear_speed':  1.0,
            'max_angular_speed': 1.5,
            'deadzone':          0.05,
        }],
        output='screen',
    )

    # Potentiometer
    potentiometer = Node(
        package='ros2_tactile_robot',
        executable='potentiometer_node',
        name='potentiometer_node',
        parameters=[{
            'motor_follows_pot': motor_follows_pot,
            'motor_speed':       1000,
            'motor_acc':         20,
            'debounce_lsb':      5,
        }],
        output='screen',
    )

    # Motor controller
    motor = Node(
        package='ros2_tactile_robot',
        executable='motor_node',
        name='motor_node',
        output='screen',
    )

    # Force / IMU sensor processor
    sensor = Node(
        package='ros2_tactile_robot',
        executable='sensor_node',
        name='sensor_node',
        parameters=[{
            'contact_threshold': 0.5,
        }],
        output='screen',
    )

    # LCD
    lcd = Node(
        package='ros2_tactile_robot',
        executable='lcd_node',
        name='lcd_node',
        parameters=[{
            'refresh_rate_hz': 5.0,
        }],
        output='screen',
    )

    # GUI (optional)
    gui_node = Node(
        package='ros2_tactile_robot',
        executable='gui_node',
        name='gui_node',
        parameters=[{
            'poll_ms':    50,
            'force_scale': 10.0,
        }],
        condition=IfCondition(gui),
        output='screen',
    )

    # RViz2 (optional)
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(pkg_share, 'rviz', 'tactile_viz.rviz')],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        com_port_arg,
        gui_arg,
        motor_follows_pot_arg,
        rviz_arg,
        esp_bridge,
        joystick,
        potentiometer,
        motor,
        sensor,
        lcd,
        gui_node,
        rviz_node,
    ])
