"""
Converts raw ESP32 joystick data into a normalised Twist command.

Subscriber:
  /esp/joystick       sensor_msgs/Joy

Publishers:
  /joystick/cmd_vel   geometry_msgs/Twist   (linear.x = Y-axis,  angular.z = X-axis)
  /joystick/pressed   std_msgs/Bool         (True while button held)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


_JOY_MAX = 2048.0   # half-range of the raw ADC joystick


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        self.declare_parameter('max_linear_speed',  1.0)   # m/s
        self.declare_parameter('max_angular_speed', 1.5)   # rad/s
        self.declare_parameter('deadzone',          0.05)  # fraction 0–1

        self._max_lin = self.get_parameter('max_linear_speed').value
        self._max_ang = self.get_parameter('max_angular_speed').value
        self._deadzone = self.get_parameter('deadzone').value

        self._pub_twist   = self.create_publisher(Twist, '/joystick/cmd_vel', 10)
        self._pub_pressed = self.create_publisher(Bool,  '/joystick/pressed', 10)

        self.create_subscription(Joy, '/esp/joystick', self._on_joy, 10)
        self.get_logger().info('Joystick node ready.')

    def _on_joy(self, msg: Joy):
        # Normalise axes to [-1, 1]
        raw_x = msg.axes[0] / _JOY_MAX if len(msg.axes) > 0 else 0.0
        raw_y = msg.axes[1] / _JOY_MAX if len(msg.axes) > 1 else 0.0

        # Apply deadzone
        norm_x = raw_x if abs(raw_x) > self._deadzone else 0.0
        norm_y = raw_y if abs(raw_y) > self._deadzone else 0.0

        twist = Twist()
        twist.linear.x  = norm_y * self._max_lin    # push forward/back → linear
        twist.angular.z = -norm_x * self._max_ang   # push left/right  → rotation
        self._pub_twist.publish(twist)

        pressed = Bool()
        pressed.data = bool(msg.buttons[0]) if len(msg.buttons) > 0 else False
        self._pub_pressed.publish(pressed)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
