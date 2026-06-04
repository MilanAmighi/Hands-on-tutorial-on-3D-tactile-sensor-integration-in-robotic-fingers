"""
Subscribes to sensor diagnostics, motor state, and joystick data, then
formats a short status string and "sends" it to the LCD.

In the current hardware the LCD is updated by the ESP32 firmware directly;
this node produces the content that could be forwarded via a serial command
or published for logging.

Subscribers:
  /sensors/diagnostics    std_msgs/String
  /motor/state            std_msgs/Int32
  /joystick/pressed       std_msgs/Bool
  /potentiometer/angle    std_msgs/Float32

Publishers:
  /lcd/line1              std_msgs/String   (top line,    max 20 chars)
  /lcd/line2              std_msgs/String   (bottom line, max 20 chars)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool, Float32


class LcdNode(Node):
    def __init__(self):
        super().__init__('lcd_node')

        self.declare_parameter('refresh_rate_hz', 5.0)
        rate = self.get_parameter('refresh_rate_hz').value

        # State cache
        self._motor_pos:  int   = 0
        self._pot_angle:  float = 0.0
        self._btn:        bool  = False
        self._diag:       str   = ''

        # Publishers
        self._pub_l1 = self.create_publisher(String, '/lcd/line1', 10)
        self._pub_l2 = self.create_publisher(String, '/lcd/line2', 10)

        # Subscriptions
        self.create_subscription(String,  '/sensors/diagnostics', self._on_diag,   10)
        self.create_subscription(Int32,   '/motor/state',         self._on_motor,  10)
        self.create_subscription(Bool,    '/joystick/pressed',    self._on_btn,    10)
        self.create_subscription(Float32, '/potentiometer/angle', self._on_pot,    10)

        self._timer = self.create_timer(1.0 / rate, self._refresh)
        self.get_logger().info(f'LCD node ready — refresh at {rate} Hz.')

    # State updates
    def _on_motor(self, msg: Int32):   self._motor_pos = msg.data
    def _on_btn(self,   msg: Bool):    self._btn = msg.data
    def _on_pot(self,   msg: Float32): self._pot_angle = msg.data
    def _on_diag(self,  msg: String):  self._diag = msg.data

    # Periodic LCD update 
    def _refresh(self):
        btn_str = 'BTN' if self._btn else '   '
        line1 = f'M:{self._motor_pos:4d} P:{self._pot_angle:5.1f}d {btn_str}'[:20]
        # Truncate diagnostics to fit second line
        line2 = self._diag[:20] if self._diag else 'No data'

        msg1 = String(); msg1.data = line1
        msg2 = String(); msg2.data = line2
        self._pub_l1.publish(msg1)
        self._pub_l2.publish(msg2)

        self.get_logger().debug(f'LCD | {line1!r} | {line2!r}')


def main(args=None):
    rclpy.init(args=args)
    node = LcdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
