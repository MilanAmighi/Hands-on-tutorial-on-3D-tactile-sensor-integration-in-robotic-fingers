"""
Launches the adaptive force control system with the real-time force plot GUI.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    args = [
        DeclareLaunchArgument('com_port',    default_value='',
                              description='ESP32 serial port (empty = auto-detect)'),
        DeclareLaunchArgument('kp',          default_value='0.0',
                              description='Contact-phase proportional gain'),
        DeclareLaunchArgument('ki',          default_value='0.0',
                              description='Contact-phase integral gain'),
        DeclareLaunchArgument('kd',          default_value='0.0',
                              description='Contact-phase derivative gain'),
        DeclareLaunchArgument('alpha',       default_value='0.80',
                              description='LPF weight on measured force (0=frozen, 1=raw)'),
        DeclareLaunchArgument('deadband_n',  default_value='0.20',
                              description='Error deadband in N'),
        DeclareLaunchArgument('window_s',    default_value='20.0',
                              description='Plot rolling window in seconds'),
    ]

    com_port   = LaunchConfiguration('com_port')
    kp         = LaunchConfiguration('kp')
    ki         = LaunchConfiguration('ki')
    kd         = LaunchConfiguration('kd')
    alpha      = LaunchConfiguration('alpha')
    deadband_n = LaunchConfiguration('deadband_n')
    window_s   = LaunchConfiguration('window_s')

    # Nodes

    esp_bridge = Node(
        package='ros2_tactile_robot',
        executable='esp_bridge_node',
        name='esp_bridge_node',
        parameters=[{
            'com_port':        com_port,
            'burst_frequency': 500,
            'publish_rate_hz': 100.0,
        }],
        output='screen',
    )

    sensor = Node(
        package='ros2_tactile_robot',
        executable='sensor_node',
        name='sensor_node',
        parameters=[{'contact_threshold': 0.3}],
        output='screen',
    )

    motor = Node(
        package='ros2_tactile_robot',
        executable='motor_node',
        name='motor_node',
        output='screen',
    )

    pid = Node(
        package='ros2_tactile_robot',
        executable='pid_control_node',
        name='pid_control_node',
        parameters=[{
            'kp':          kp,
            'ki':          ki,
            'kd':          kd,
            'alpha':       alpha,
            'deadband_n':  deadband_n,
            'motor_acc':   100,
            'enabled':     True,
        }],
        output='screen',
    )

    pid_gui = Node(
        package='ros2_tactile_robot',
        executable='pid_gui_node',
        name='pid_gui_node',
        parameters=[{
            'poll_ms':  50,
            'window_s': window_s,
        }],
        output='screen',
    )

    return LaunchDescription(args + [
        esp_bridge,
        sensor,
        motor,
        pid,
        pid_gui,
    ])
