"""
Processes raw force and IMU data coming from the ESP bridge.

Subscribers:
  /esp/force     std_msgs/Float32MultiArray   (24 floats: Fx0,Fy0,Fz0,...,Fx7,Fy7,Fz7)
  /esp/imu       sensor_msgs/Imu
  /esp/status    std_msgs/String              (JSON acq-status dict)

Publishers:
  /sensors/force_magnitudes   std_msgs/Float32MultiArray  (8 floats, one per sensor)
  /sensors/contact_mask       std_msgs/Int32              (bit-mask, bit i = sensor i in contact)
  /sensors/imu_gravity        geometry_msgs/Vector3       (m/s², gravity direction)
  /sensors/diagnostics        std_msgs/String             (human-readable summary)
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        self.declare_parameter('contact_threshold', 0.5)   # N (or raw units)
        self._threshold = self.get_parameter('contact_threshold').value

        # Publishers
        self._pub_magnitudes = self.create_publisher(
            Float32MultiArray, '/sensors/force_magnitudes', 10)
        self._pub_contact    = self.create_publisher(
            Int32,             '/sensors/contact_mask',     10)
        self._pub_gravity    = self.create_publisher(
            Vector3,           '/sensors/imu_gravity',      10)
        self._pub_diag       = self.create_publisher(
            String,            '/sensors/diagnostics',      10)

        # Subscribers
        self.create_subscription(Float32MultiArray, '/esp/force',  self._on_force,  10)
        self.create_subscription(Imu,               '/esp/imu',    self._on_imu,    10)
        self.create_subscription(String,            '/esp/status', self._on_status, 10)

        self._last_diag_parts: dict = {}
        self.get_logger().info(
            f'Sensor node ready. Contact threshold = {self._threshold}')

    # Forces
    def _on_force(self, msg: Float32MultiArray):
        data = msg.data
        magnitudes = []
        contact_mask = 0
        for i in range(8):
            base = i * 3
            if base + 2 < len(data):
                fx, fy, fz = data[base], data[base + 1], data[base + 2]
                mag = math.sqrt(fx * fx + fy * fy + fz * fz)
            else:
                mag = 0.0
            magnitudes.append(mag)
            if mag > self._threshold:
                contact_mask |= (1 << i)

        mag_msg = Float32MultiArray()
        mag_msg.data = magnitudes
        self._pub_magnitudes.publish(mag_msg)

        mask_msg = Int32()
        mask_msg.data = contact_mask
        self._pub_contact.publish(mask_msg)

        self._last_diag_parts['force'] = (
            f"contact={''.join(str((contact_mask >> i) & 1) for i in range(8))} "
            f"mag=[{', '.join(f'{m:.2f}' for m in magnitudes)}]"
        )
        self._publish_diag()

    # IMU data 
    def _on_imu(self, msg: Imu):
        grav = Vector3()
        grav.x = msg.linear_acceleration.x
        grav.y = msg.linear_acceleration.y
        grav.z = msg.linear_acceleration.z
        self._pub_gravity.publish(grav)

        self._last_diag_parts['imu'] = (
            f"grav=({grav.x:.2f},{grav.y:.2f},{grav.z:.2f}) m/s²"
        )
        self._publish_diag()

    # Acq status
    def _on_status(self, msg: String):
        try:
            s = json.loads(msg.data)
            imu_err   = s.get('imu_last_err', 0)
            imu_ok    = s.get('imu_last_read_ok', 1)
            force_ok  = s.get('force_last_read_ok', 1)
            if imu_err != 0 or imu_ok == 0 or force_ok == 0:
                #self.get_logger().warn(
                #    f'Acq error — imu_err={imu_err} imu_ok={imu_ok} force_ok={force_ok}')
                pass
            self._last_diag_parts['status'] = (
                f"imu_ok={imu_ok} force_ok={force_ok} imu_err={imu_err}"
            )
            self._publish_diag()
        except json.JSONDecodeError:
            pass

    def _publish_diag(self):
        parts = '  |  '.join(self._last_diag_parts.values())
        diag_msg = String()
        diag_msg.data = parts
        self._pub_diag.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
