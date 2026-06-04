"""
Motor controller: arbitrates commands from multiple sources (potentiometer,
joystick button, direct external commands) with a simple priority scheme and
forwards the winning command to the ESP bridge

Priority (highest → lowest)
  1. /motor/external_command    direct override from user / higher-level planner
  2. /motor/command             from potentiometer_node (motor-follows-pot)

Subscribers:
  /motor/command           std_msgs/Float32MultiArray  ([pos, speed, acc])  - pot / planner
  /motor/external_command  std_msgs/Float32MultiArray  ([pos, speed, acc])  - high-priority override
  /esp/motor_position      std_msgs/Int32

Publishers:
  /motor/drive             std_msgs/Float32MultiArray  (arbitrated command - esp_bridge_node)
  /motor/state             std_msgs/Int32              (current position echo)
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

_OVERRIDE_TIMEOUT_S = 2.0   # external override expires after this many seconds


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self._pub_cmd   = self.create_publisher(Float32MultiArray, '/motor/drive',   10)
        self._pub_state = self.create_publisher(Int32,             '/motor/state',   10)

        # Subscribe to both command sources (external wins)
        self.create_subscription(
            Float32MultiArray, '/motor/command',
            self._on_normal_command, 10)
        self.create_subscription(
            Float32MultiArray, '/motor/external_command',
            self._on_external_command, 10)
        self.create_subscription(
            Int32, '/esp/motor_position', self._on_motor_position, 10)

        self._override_cmd: list | None = None
        self._override_t: float = 0.0

        self.get_logger().info('Motor node ready.')


    # Subscribers
    
    def _on_external_command(self, msg: Float32MultiArray):
        """High-priority override, valid for _OVERRIDE_TIMEOUT_S seconds."""
        if len(msg.data) < 3:
            return
        self._override_cmd = list(msg.data)
        self._override_t   = time.monotonic()
        self._send(msg.data)
        self.get_logger().info(
            f'[OVERRIDE] pos={msg.data[0]:.0f}  spd={msg.data[1]:.0f}  acc={msg.data[2]:.0f}')

    def _on_normal_command(self, msg: Float32MultiArray):
        """Normal command (e.g. from potentiometer). Ignored while override is active."""
        if len(msg.data) < 3:
            return
        if self._override_active():
            return   # override in effect — drop this command
        self._send(msg.data)

    def _on_motor_position(self, msg: Int32):
        state_msg = Int32()
        state_msg.data = msg.data
        self._pub_state.publish(state_msg)

    # Helpers

    def _override_active(self) -> bool:
        return (self._override_cmd is not None
                and (time.monotonic() - self._override_t) < _OVERRIDE_TIMEOUT_S)

    def _send(self, data):
        out = Float32MultiArray()
        out.data = list(data)
        self._pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
