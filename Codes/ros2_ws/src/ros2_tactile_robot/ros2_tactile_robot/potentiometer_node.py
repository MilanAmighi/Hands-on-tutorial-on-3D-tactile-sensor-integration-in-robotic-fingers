"""
Converts the raw 12-bit potentiometer ADC reading into a physical angle
and optionally generates a motor position target

Subscriber:
  /esp/potentiometer      std_msgs/Int32     (raw ADC 0–4095)

Publishers:
  /potentiometer/angle    std_msgs/Float32   (degrees, 0–360)
  /motor/command          std_msgs/Float32MultiArray  ([position, speed, acc])
                          Only published when motor_follows_pot is True.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Float32MultiArray

_ADC_MAX   = 4095.0
_MOTOR_MIN = 1500
_MOTOR_MAX = 2800


class PotentiometerNode(Node):
    def __init__(self):
        super().__init__('potentiometer_node')

        self.declare_parameter('motor_follows_pot', True)
        self.declare_parameter('motor_speed',       1000)
        self.declare_parameter('motor_acc',         20)
        self.declare_parameter('debounce_lsb',      5)    # min ADC change to re-command motor
        self.declare_parameter('motor_min',         _MOTOR_MIN)
        self.declare_parameter('motor_max',         _MOTOR_MAX)

        self._follows   = self.get_parameter('motor_follows_pot').value
        self._speed     = self.get_parameter('motor_speed').value
        self._acc       = self.get_parameter('motor_acc').value
        self._debounce  = self.get_parameter('debounce_lsb').value
        self._motor_min = float(self.get_parameter('motor_min').value)
        self._motor_max = float(self.get_parameter('motor_max').value)

        self._pub_angle   = self.create_publisher(Float32,          '/potentiometer/angle', 10)
        self._pub_mot_cmd = self.create_publisher(Float32MultiArray, '/motor/command',       10)

        self.create_subscription(Int32, '/esp/potentiometer', self._on_pot, 10)

        self._last_cmd_pot: int | None = None
        self.get_logger().info(
            f'Potentiometer node ready. motor_follows_pot={self._follows}')

    def _on_pot(self, msg: Int32):
        raw = msg.data

        # Publish angle
        angle_msg = Float32()
        angle_msg.data = float(raw) / _ADC_MAX * 360.0
        self._pub_angle.publish(angle_msg)

        # Motor-follows-pot with debounce
        if self._follows:
            if (self._last_cmd_pot is None
                    or abs(raw - self._last_cmd_pot) >= self._debounce):
                motor_pos = self._motor_min + (raw / _ADC_MAX) * (self._motor_max - self._motor_min)
                cmd = Float32MultiArray()
                cmd.data = [motor_pos, float(self._speed), float(self._acc)]
                self._pub_mot_cmd.publish(cmd)
                self._last_cmd_pot = raw


def main(args=None):
    rclpy.init(args=args)
    node = PotentiometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
