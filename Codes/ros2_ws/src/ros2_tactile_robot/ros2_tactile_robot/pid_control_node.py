"""
Adaptive force controller with firmware-matched control structure.

Potentiometer sets target normal force (0–5 N).

Reference gain values for the three test objects :
  CUBE   → Kp=11  Kd=25  alpha=0.70  deadband=0.20 N
  SPONGE → Kp=20  Kd=28  alpha=0.75  deadband=0.15 N
  CUP    → Kp=28  Kd=35  alpha=0.80  deadband=0.10 N

Subscribers:
  /esp/force            std_msgs/Float32MultiArray  (24 floats: Fx0,Fy0,Fz0,...)
  /esp/imu              sensor_msgs/Imu             (gravity vector in m/s²)
  /esp/potentiometer    std_msgs/Int32
  /esp/motor_position   std_msgs/Int32
  /pid/gains            std_msgs/Float32MultiArray  ([kp, ki, kd])
  /pid/tare             std_msgs/Bool               (any message → zero fn and fg offsets)

Publishers:
  /motor/command        std_msgs/Float32MultiArray  ([position, speed, acc])
  /pid/state            std_msgs/Float32MultiArray
        [target_force, filtered_force, error, integral, derivative, pid_output, fg_fn_ratio]
  /pid/slip             std_msgs/Float32MultiArray
        [fg_fn_ratio, fg, fn]  (filtered, offset-corrected)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray, Int32

_FORCE_MAX   = 5.0     # N — full potentiometer deflection
_POT_MAX     = 4095
_MOTOR_MIN   = 1500
_MOTOR_MAX   = 2800
_MOTOR_SPEED = 300     

_MIN_TARGET  = 0.1     # N — below this the controller is inhibited (gripper idle)

_N_WARMUP    = 50      # samples discarded so the filter + IMU stream settle first
_N_CALIB     = 20      # samples averaged for the startup auto-zero baseline


class PidControlNode(Node):
    def __init__(self):
        super().__init__('pid_control_node')

        self.declare_parameter('kp',             0.0)
        self.declare_parameter('ki',             0.0)
        self.declare_parameter('kd',             0.0)
        self.declare_parameter('alpha',          0.75)   # LPF weight for fn/fg (0=frozen, 1=raw)
        self.declare_parameter('alpha_ratio',    0.30)   # LPF weight for the Fg/Fn ratio
        self.declare_parameter('deadband_n',     0.15)   # N — error deadband
        self.declare_parameter('motor_acc',      200)
        self.declare_parameter('enabled',        True)

        self._kp             = self.get_parameter('kp').value
        self._ki             = self.get_parameter('ki').value
        self._kd             = self.get_parameter('kd').value
        self._alpha          = float(self.get_parameter('alpha').value)
        self._alpha_ratio    = float(self.get_parameter('alpha_ratio').value)
        self._deadband       = float(self.get_parameter('deadband_n').value)
        self._motor_acc      = self.get_parameter('motor_acc').value
        self._enabled        = self.get_parameter('enabled').value

        if self._kp == 0.0 and self._ki == 0.0 and self._kd == 0.0:
            self.get_logger().warn(
                'kp, ki and kd are all 0.0 (the launch-file default) — the '
                'controller will compute zero correction and the motor will not '
                'move. Pass kp:=/ki:=/kd:= on the command line, e.g. '
                "'ros2 launch ros2_tactile_robot pid_force_control.launch.py "
                "kp:=11 kd:=25' — see the reference gain table in Codes/README.md."
            )

        self._filtered_force  = 0.0
        self._filtered_fg     = 0.0
        self._filtered_ratio  = 0.0
        self._prev_dif_force  = 0.0
        self._integral_error  = 0.0
        self._offset_fn       = 0.0
        self._offset_fg       = 0.0
        self._last_fn_raw     = 0.0
        self._last_fg_raw     = 0.0

        self._offset_ready    = False
        self._calib_count     = 0
        self._calib_fn: list[float] = []
        self._calib_fg: list[float] = []

        self._pot_value: int   | None = None
        self._motor_pos: int   | None = None
        self._gravity:   tuple | None = None

        self._pub_motor = self.create_publisher(Float32MultiArray, '/motor/command', 10)
        self._pub_state = self.create_publisher(Float32MultiArray, '/pid/state',     10)
        self._pub_slip  = self.create_publisher(Float32MultiArray, '/pid/slip',      10)

        self.create_subscription(Float32MultiArray, '/esp/force',
                                 self._on_force,     10)
        self.create_subscription(Imu,               '/esp/imu',
                                 self._on_imu,       10)
        self.create_subscription(Int32,             '/esp/potentiometer',
                                 self._on_pot,       10)
        self.create_subscription(Int32,             '/esp/motor_position',
                                 self._on_motor_pos, 10)
        self.create_subscription(Float32MultiArray, '/pid/gains',
                                 self._on_gains,     10)
        self.create_subscription(Bool,              '/pid/tare',
                                 self._on_tare,      10)



    def _on_tare(self, _msg: Bool):
        self._offset_fn      = self._last_fn_raw
        self._offset_fg      = self._last_fg_raw
        self._offset_ready   = True
        self._filtered_force = 0.0
        self._filtered_fg    = 0.0
        self._filtered_ratio = 0.0
        self._integral_error = 0.0
        self._prev_dif_force = 0.0
        self.get_logger().info(
            f'PID tare applied: fn offset = {self._offset_fn:.3f} N  '
            f'fg offset = {self._offset_fg:.3f} N')

    def _on_gains(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._kp, self._ki, self._kd = (
                float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))
            self._integral_error = 0.0
            self.get_logger().info(
                f'Gains updated: Kp={self._kp}  Ki={self._ki}  Kd={self._kd}')

    def _on_pot(self, msg: Int32):
        self._pot_value = msg.data

    def _on_motor_pos(self, msg: Int32):
        self._motor_pos = msg.data

    def _on_imu(self, msg: Imu):
        self._gravity = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )

    def _on_force(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 24:
            return

        s = [[float(data[i*3]), float(data[i*3+1]), float(data[i*3+2])]
             for i in range(8)]
        lF, rF = s[:4], s[4:]

        fx_L = sum(v[0] for v in lF)
        fy_L = sum(v[1] for v in lF)
        fz_L = sum(v[2] for v in lF)
        fx_R = sum(v[0] for v in rF)
        fy_R = sum(v[1] for v in rF)
        fz_R = sum(v[2] for v in rF)

        fx_tot = fx_L + fx_R
        fy_tot = fy_L - fy_R
        fz_tot = fz_L - fz_R

        self._last_fn_raw = -(fz_L + fz_R)

        # Gravity-aligned shear force fg
        if self._gravity is not None:
            gx, gy, gz = self._gravity
            g_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
            # Firmware axis mapping: Fx·gx + Fy·gz + Fz·gy (sensor y/z swapped vs IMU)
            fg_raw = ((fx_tot*gx + fy_tot*gz + fz_tot*gy) / g_mag
                      if g_mag > 1e-6 else 0.0)
        else:
            g_mag  = 0.0
            fg_raw = math.sqrt(fx_tot**2 + fy_tot**2)
        self._last_fg_raw = fg_raw

        # Auto-zero baseline offsets when the gripper is not in contact
        if not self._offset_ready:
            self._calib_count += 1
            if self._calib_count <= _N_WARMUP or g_mag <= 1e-6:
                return
            self._calib_fn.append(self._last_fn_raw)
            self._calib_fg.append(self._last_fg_raw)
            if len(self._calib_fn) < _N_CALIB:
                return
            self._offset_fn    = sum(self._calib_fn) / len(self._calib_fn)
            self._offset_fg    = sum(self._calib_fg) / len(self._calib_fg)
            self._offset_ready = True
            self.get_logger().info(
                f'Auto-zero: fn offset = {self._offset_fn:.3f} N  '
                f'fg offset = {self._offset_fg:.3f} N '
                f'(averaged {len(self._calib_fn)} samples)')

        fn = self._last_fn_raw - self._offset_fn
        fg = fg_raw - self._offset_fg

        # Smooth fn and fg
        self._filtered_force = (self._alpha * fn
                                + (1.0 - self._alpha) * self._filtered_force)
        self._filtered_fg    = (self._alpha * fg
                                + (1.0 - self._alpha) * self._filtered_fg)

        ratio_inst = (self._filtered_fg / self._filtered_force
                      if abs(self._filtered_force) > 1e-6 else 0.0)
        self._filtered_ratio = (self._alpha_ratio * ratio_inst
                                + (1.0 - self._alpha_ratio) * self._filtered_ratio)
        ratio = self._filtered_ratio

        slip_msg = Float32MultiArray()
        slip_msg.data = [float(ratio), float(self._filtered_fg),
                         float(self._filtered_force)]
        self._pub_slip.publish(slip_msg)

        if not self._enabled or self._pot_value is None or self._motor_pos is None:
            return

        target_force = (self._pot_value / _POT_MAX) * _FORCE_MAX

        if target_force < _MIN_TARGET:
            self._integral_error = 0.0
            self._prev_dif_force = 0.0
            # Open the gripper so no residual force is held at 0 N target
            if self._motor_pos > _MOTOR_MIN:
                cmd = Float32MultiArray()
                cmd.data = [float(_MOTOR_MIN), float(_MOTOR_SPEED), float(self._motor_acc)]
                self._pub_motor.publish(cmd)
            return

        dif_force = target_force - self._filtered_force
        d_force   = dif_force - self._prev_dif_force
        self._prev_dif_force = dif_force

        if self._ki != 0.0:
            self._integral_error += dif_force

        correction = int(dif_force * self._kp
                         + self._integral_error * self._ki
                         + d_force * self._kd)

        send_cmd = False
        new_pos  = self._motor_pos

        if dif_force < -self._deadband:
            candidate = self._motor_pos + correction
            if candidate >= _MOTOR_MIN:
                new_pos  = candidate
                send_cmd = True
        elif dif_force > self._deadband:
            candidate = self._motor_pos + correction
            if candidate <= _MOTOR_MAX:
                new_pos  = candidate
                send_cmd = True

        if send_cmd:
            new_pos = max(_MOTOR_MIN, min(_MOTOR_MAX, new_pos))
            cmd = Float32MultiArray()
            cmd.data = [float(new_pos), float(_MOTOR_SPEED), float(self._motor_acc)]
            self._pub_motor.publish(cmd)

        state_msg = Float32MultiArray()
        state_msg.data = [
            float(target_force),
            float(self._filtered_force),
            float(dif_force),
            float(self._integral_error),
            float(d_force),
            float(correction),
            float(ratio),
        ]
        self._pub_state.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PidControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
