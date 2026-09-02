import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String
from sensor_msgs.msg import Imu, Joy
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


_MESH_OFFSET_X = 0.054
_MESH_OFFSET_Y = -0.014
_MESH_OFFSET_Z = 0.0
_SENSOR_D = 0.0044

_SENSOR_XY = [
    ( _SENSOR_D, -_SENSOR_D),  # tactaxis 0
    (_SENSOR_D, _SENSOR_D),  # tactaxis 1
    (- _SENSOR_D,  -_SENSOR_D),  # tactaxis 2
    (-_SENSOR_D,  _SENSOR_D),  # tactaxis 3
]



class EspBridgeNode(Node):
    def __init__(self):
        super().__init__('esp_bridge_node')

        # Parameters
        self.declare_parameter('com_port',        '')
        self.declare_parameter('burst_frequency', 500)
        self.declare_parameter('publish_rate_hz', 500.0)

        com_port        = self.get_parameter('com_port').value or None
        burst_frequency = self.get_parameter('burst_frequency').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        # Publishers
        self._pub_force   = self.create_publisher(Float32MultiArray, '/esp/force',          10)
        self._pub_imu     = self.create_publisher(Imu,               '/esp/imu',            10)
        self._pub_joy     = self.create_publisher(Joy,               '/esp/joystick',       10)
        self._pub_pot     = self.create_publisher(Int32,             '/esp/potentiometer',  10)
        self._pub_mot_pos = self.create_publisher(Int32,             '/esp/motor_position', 10)
        self._pub_status  = self.create_publisher(String,            '/esp/status',         10)

        self._pub_markers  = self.create_publisher(MarkerArray,   '/tactile/markers',             10)

        # Static TF: world to frame_left_finger and frame_right_finger
        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_finger_frames()

        # Subscriber
        self._sub_motor_cmd = self.create_subscription(
            Float32MultiArray, '/motor/drive', self._on_motor_command, 10)

        # ESP driver
        from ros2_tactile_robot.SerialLibrary.lib_esp32 import espDriver
        self._esp = espDriver(comPort=com_port).__enter__()
        self._esp.startThreadBurstForce(frequency=burst_frequency)
        self.get_logger().info(
            f'ESP32 connected — burst at {burst_frequency} Hz, publishing at {publish_rate_hz} Hz')

        # Publish timer
        period = 1.0 / publish_rate_hz
        self._timer = self.create_timer(period, self._publish_all)

    # Destructor
    def destroy_node(self):
        try:
            self._esp.stopThread()
            self._esp.__exit__(None, None, None)
        except Exception:
            pass
        super().destroy_node()

    # Motor command subscriber
    def _on_motor_command(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warn('motor/command needs [position, speed, acc]')
            return
        position = int(msg.data[0])
        speed    = int(msg.data[1])
        acc      = int(msg.data[2])
        try:
            self._esp.sendCommandMotor(position=position, speed=speed, acc=acc)
        except Exception as exc:
            self.get_logger().warn(f'motor command not sent: {exc}',
                                   throttle_duration_sec=2.0)

    # Main publish tick
    def _publish_all(self):
        esp = self._esp
        stamp = self.get_clock().now().to_msg()

        # Force
        force_vectors = esp.buffer[-1]
        flat = []
        for i in range(8):
            if i < len(force_vectors):
                fx, fy, fz = force_vectors[i]
                flat += [float(fx), float(fy), float(fz)]
            else:
                flat += [0.0, 0.0, 0.0]
        force_msg = Float32MultiArray()
        force_msg.data = flat
        self._pub_force.publish(force_msg)

        # IMU
        if esp.gravity_raw_lsb is not None:
            gx, gy, gz = esp.gravity_raw_lsb
            imu_msg = Imu()
            imu_msg.header.stamp    = stamp
            imu_msg.header.frame_id = 'imu_link'
            # BNO055 gravity in units of 1/100 m/s²  →  m/s²
            imu_msg.linear_acceleration.x = gx * 0.01
            imu_msg.linear_acceleration.y = gy * 0.01
            imu_msg.linear_acceleration.z = gz * 0.01
            self._pub_imu.publish(imu_msg)

        # Joystick
        if esp.joystick_x is not None:
            joy_msg = Joy()
            joy_msg.header.stamp = stamp
            joy_msg.axes    = [float(esp.joystick_x), float(esp.joystick_y)]
            joy_msg.buttons = [int(esp.joystick_button) if esp.joystick_button is not None else 0]
            self._pub_joy.publish(joy_msg)

        # Potentiometer
        if esp.potentiometer_value is not None:
            pot_msg = Int32()
            pot_msg.data = int(esp.potentiometer_value)
            self._pub_pot.publish(pot_msg)

        # Motor position
        if esp.motor_position is not None:
            mot_msg = Int32()
            mot_msg.data = int(esp.motor_position)
            self._pub_mot_pos.publish(mot_msg)

        # Acquisition status
        if esp.acq_status is not None:
            status_msg = String()
            status_msg.data = json.dumps(esp.acq_status)
            self._pub_status.publish(status_msg)

        # RViz finger mesh markers. Note: the /tactile/wrench/* WrenchStamped
        # topics are NOT published here — see gui_node.py, which derives them
        # from /esp/force.
        self._publish_mesh_markers(stamp)

    def _publish_finger_frames(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []

        # Transformation of frame in Rviz
        for base, ty in [
            ('frame_left_finger_origin',  -0.05),
            ('frame_right_finger_origin',  0.05),
        ]:
            t = TransformStamped()
            t.header.stamp    = stamp
            t.header.frame_id = 'world'
            t.child_frame_id  = base
            t.transform.translation.y = ty
            t.transform.rotation.w    = 1.0
            transforms.append(t)

        for child, parent, qx, qy, qz, qw in [
            ('frame_left_finger',  'frame_left_finger_origin',   -0.5,  0.5, 0.5, 0.5),
            ('frame_right_finger', 'frame_right_finger_origin',  0.5, -0.5,  0.5, 0.5),
        ]:
            t = TransformStamped()
            t.header.stamp    = stamp
            t.header.frame_id = parent
            t.child_frame_id  = child
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            transforms.append(t)


        for f_idx, finger_frame in enumerate(['frame_left_finger', 'frame_right_finger']):
            side = 'left' if f_idx == 0 else 'right'

            # Apply rotation to the finger frame 
            t_rot = TransformStamped()
            t_rot.header.stamp    = stamp
            t_rot.header.frame_id = finger_frame
            t_rot.child_frame_id  = f'frame_{side}_finger_center'
            t_rot.transform.rotation.x = 0.0
            t_rot.transform.rotation.y = 0.0
            t_rot.transform.rotation.z = -math.sin(math.pi / 4)
            t_rot.transform.rotation.w =  math.cos(math.pi / 4)
            transforms.append(t_rot)

            # Apply only translation for each tactile sensor
            for s_idx, (sx, sy) in enumerate(_SENSOR_XY):
                t = TransformStamped()
                t.header.stamp    = stamp
                t.header.frame_id = f'frame_{side}_finger_center'  # child of rotated frame
                t.child_frame_id  = f'frame_{side}_tactile_{s_idx}'
                t.transform.translation.x = sx
                t.transform.translation.y = sy
                t.transform.translation.z = 0.004
                t.transform.rotation.w    = 1.0 
                transforms.append(t)


        self._tf_static.sendTransform(transforms)

    def _publish_mesh_markers(self, stamp):
        # RViz finger mesh markers (STL meshes), NOT WrenchStamped messages.
        ma = MarkerArray()
        for idx, (frame_id, mesh_file) in enumerate([
            ('frame_left_finger',  'package://ros2_tactile_robot/meshes/Tactaxis_sensor_left_finger.stl'),
            ('frame_right_finger', 'package://ros2_tactile_robot/meshes/Tactaxis_sensor_right_finger.stl'),
        ]):
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = frame_id
            m.ns              = 'tactile_fingers'
            m.id              = idx
            m.type            = Marker.MESH_RESOURCE
            m.action          = Marker.ADD
            m.mesh_resource   = mesh_file
            m.scale.x = m.scale.y = m.scale.z = 0.001  # mm → m
            m.color.r = 0.55; m.color.g = 0.70; m.color.b = 0.90; m.color.a = 0.85
            m.pose.position.x = _MESH_OFFSET_X
            m.pose.position.y = _MESH_OFFSET_Y
            m.pose.position.z = _MESH_OFFSET_Z
            m.pose.orientation.w = 1.0
            ma.markers.append(m)

        self._pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = EspBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
