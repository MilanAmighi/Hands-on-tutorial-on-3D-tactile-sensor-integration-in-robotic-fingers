"""
Live GUI node for visualizing the tactile sensor data, joystick, potentiometer, motor position, and IMU gravity vector.

Subscriber:
  /esp/force          std_msgs/Float32MultiArray  (8×3 forces floats)
  /esp/imu            sensor_msgs/Imu             (linear_acceleration = gravity)
  /esp/joystick       sensor_msgs/Joy
  /esp/potentiometer  std_msgs/Int32
  /esp/motor_position std_msgs/Int32

Publishers:
  /motor/command                std_msgs/Float32MultiArray  ([position, speed, acc])
  /tactile/wrench/left_finger   geometry_msgs/WrenchStamped (tared left-finger wrench)
  /tactile/wrench/right_finger  geometry_msgs/WrenchStamped (tared right-finger wrench)
  /tactile/wrench/total         geometry_msgs/WrenchStamped (tared total wrench)
  /tactile/wrench/left_0..3     geometry_msgs/WrenchStamped (tared per-taxel wrench, left finger)
  /tactile/wrench/right_0..3    geometry_msgs/WrenchStamped (tared per-taxel wrench, right finger)

Note: these are the only nodes that publish /tactile/wrench/*. esp_bridge_node
publishes /tactile/markers (RViz mesh markers) but does NOT publish wrenches.
"""


import csv
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import Imu, Joy
from geometry_msgs.msg import WrenchStamped

_N_SENSORS          = 8
_MOTOR_DEBOUNCE_LSB = 5
_SIN45              = math.sin(math.radians(45))

# Path to save recorded CSV files 
def _find_record_dir() -> str:
    p = os.path.abspath(__file__)
    while True:
        p = os.path.dirname(p)
        if os.path.basename(p) == 'Codes' or p == os.path.dirname(p):
            break
    return os.path.join(p, 'record_data')

_RECORD_DIR = _find_record_dir()
os.makedirs(_RECORD_DIR, exist_ok=True)

# Proxy object to hold the latest data from the ESP and provide a method to send motor commands
class _EspProxy:
    def __init__(self, motor_pub):
        self._motor_pub      = motor_pub
        self.sample_counter  = 0
        # buffer shape matches espDriver after startThreadBurstForce: (N, 8, 3)
        self.buffer          = np.zeros((1, _N_SENSORS, 3), dtype=np.float32)
        self.gravity_raw_lsb = None
        self.joystick_x      = None
        self.joystick_y      = None
        self.joystick_button = None
        self.potentiometer_value = None
        self.motor_position  = None

    def sendCommandMotor(self, position=0, speed=1000, acc=0):
        msg = Float32MultiArray()
        msg.data = [float(position), float(speed), float(acc)]
        self._motor_pub.publish(msg)

# Main GUI class
class BurstModeGui:
    """Live visualization GUI for the burst-force stream.

    Panels
    ------
    - Force bars  : one group of 3 bars (Fx/Fy/Fz) per sensor (S0–S7).
    - Joystick    : dot on a pad circle; red = released, green = pressed.
    - Potentiometer: rotating knob, 0–4095 mapped to 0–360°.
    - Motor       : rotating needle on a motor diagram, 0–4095 → 0–360°.
    - IMU         : isometric 3-D arrow for the BNO055 gravity vector.
    """

    # Colours (Catppuccin Mocha palette)
    _BG        = "#1e1e2e"
    _SURFACE   = "#181825"
    _SURFACE1  = "#313244"
    _OVERLAY   = "#45475a"
    _TEXT      = "#cdd6f4"
    _SUBTEXT   = "#6c7086"
    _RED       = "#e63946"
    _TEAL      = "#2a9d8f"
    _BLUE      = "#89b4fa"
    _PINK      = "#f38ba8"
    _YELLOW    = "#f9e2af"
    _GREEN     = "#a6e3a1"
    _FX_COL    = "#e63946"
    _FY_COL    = "#2a9d8f"
    _FZ_COL    = "#457b9d"

    _N_SENSORS    = 8
    _GRAVITY_NORM = 981   # LSB for 1 g  (BNO055 gravity in units of 1/100 m/s²)
    _JOY_HALF     = 2048  # joystick half-range

    def __init__(
        self,
        esp,
        poll_ms: int = 50,
        shear_scale: float = 2.0,
        normal_scale: float = 5.0,
        no_motor_pot: bool = False,
        motor_debounce: int = _MOTOR_DEBOUNCE_LSB,
        wrench_callback=None,
    ) -> None:
        import tkinter as tk
        self._tk = tk

        self._esp = esp
        self._poll_ms = max(10, poll_ms)
        # Full-scale deflection per axis, in each direction: shear on Fx/Fy,
        # normal on Fz. Indexed by the axis order used everywhere below.
        self._axis_scale = (max(0.1, shear_scale),
                            max(0.1, shear_scale),
                            max(0.1, normal_scale))
        self._no_motor_pot = no_motor_pot
        self._motor_debounce = motor_debounce
        self._last_motor_cmd_pot: int | None = None
        self._wrench_callback = wrench_callback

        self._running = False
        self._last_rate_t = time.time()
        self._last_sample_count = esp.sample_counter

        self._root = tk.Tk()
        self._root.title("Burst Mode  —  Live Visualizer")
        self._root.resizable(False, False)
        self._root.configure(bg=self._BG)

        self._build_header()
        self._build_force_panel()
        self._build_peripheral_row()
        self._build_status_bar()

        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._lock = threading.Lock()
        self._recording: deque = deque()
        self._is_recording = False
        self._rec_t0: float | None = None
        self._offset = np.zeros((_N_SENSORS, 3), dtype=float)

        # Two-stage LPF for Fn/Fg/ratio (mirrors pid_control_node)
        self._lpf_alpha       = 0.75
        self._lpf_alpha_ratio = 0.30
        self._filtered_fn     = 0.0
        self._filtered_fg     = 0.0
        self._filtered_ratio  = 0.0
        self._lpf_seeded      = False

        # Rolling-window peak-to-peak amplitude of the Fg/Fn ratio 
        self._ratio_window_size          = 20
        self._ratio_window: deque[float] = deque(maxlen=self._ratio_window_size)
        self._ratio_amp                  = 0.0
        self._amp_alpha                  = 1.0

        self._build_record_bar()


    # Layout builders
    def _build_header(self) -> None:
        tk = self._tk
        row = tk.Frame(self._root, bg=self._BG)
        row.pack(fill="x", padx=12, pady=(10, 4))

        self._sample_var = tk.StringVar(value="Sample: 0")
        self._rate_var   = tk.StringVar(value="Rate: — Hz")

        tk.Label(row, textvariable=self._sample_var, bg=self._BG,
                 fg=self._TEXT, font=("Segoe UI", 10)).pack(side="left")
        tk.Label(row, textvariable=self._rate_var, bg=self._BG,
                 fg=self._TEXT, font=("Segoe UI", 10)).pack(side="right")

    def _build_force_panel(self) -> None:
        tk = self._tk

        # Canvas dimensions are derived from the number of sensors and desired bar sizes/margins
        self._fc_col_w  = 136
        self._fc_bar_h  = 26
        self._fc_margin = 5
        self._fc_w = self._N_SENSORS * self._fc_col_w
        self._fc_h = 3 * (self._fc_bar_h + self._fc_margin) + 32  # 32 for sensor label

        outer = tk.LabelFrame(
            self._root, text="Force Sensors", padx=6, pady=4,
            bg=self._BG, fg=self._TEXT, font=("Segoe UI", 9, "bold"), bd=1,
        )
        outer.pack(padx=12, pady=(0, 6), fill="x")

        c = tk.Canvas(outer, width=self._fc_w, height=self._fc_h,
                      bg=self._SURFACE, highlightthickness=0)
        c.pack()
        self._fc = c

        self._bar_ids: list[list[int]] = []
        self._val_ids: list[list[int]] = []

        self._fc_bar_x_off = 20
        self._fc_bar_max_w = self._fc_col_w - 24

        for s in range(self._N_SENSORS):
            x0     = s * self._fc_col_w
            col_cx = x0 + self._fc_col_w // 2

            c.create_text(col_cx, 8, text=f"S{s}",
                          fill=self._TEXT, font=("Segoe UI", 8, "bold"), anchor="n")

            sensor_bar_ids: list[int] = []
            sensor_val_ids: list[int] = []

            for i, (axis, color) in enumerate(
                [("Fx", self._FX_COL), ("Fy", self._FY_COL), ("Fz", self._FZ_COL)]
            ):
                bar_y  = 24 + i * (self._fc_bar_h + self._fc_margin)
                mid_y  = bar_y + self._fc_bar_h // 2
                bar_x0 = x0 + self._fc_bar_x_off

                c.create_rectangle(bar_x0, bar_y,
                                   x0 + self._fc_col_w - 4,
                                   bar_y + self._fc_bar_h,
                                   fill=self._SURFACE1, outline="", width=0)
                bar_xc = bar_x0 + self._fc_bar_max_w // 2
                c.create_line(bar_xc, bar_y, bar_xc, bar_y + self._fc_bar_h,
                              fill=self._SUBTEXT, width=1)
                c.create_text(x0 + 4, mid_y, text=axis, fill=color,
                              font=("Segoe UI", 7, "bold"), anchor="w")

                bid = c.create_rectangle(bar_xc, bar_y + 3, bar_xc,
                                         bar_y + self._fc_bar_h - 3,
                                         fill=color, outline="", width=0)
                vid = c.create_text(x0 + self._fc_col_w - 6, mid_y, text="0.00",
                                    fill=self._TEXT, font=("Segoe UI", 7), anchor="e")

                sensor_bar_ids.append(bid)
                sensor_val_ids.append(vid)

            self._bar_ids.append(sensor_bar_ids)
            self._val_ids.append(sensor_val_ids)

    def _build_peripheral_row(self) -> None:
        tk = self._tk

        row = tk.Frame(self._root, bg=self._BG)
        row.pack(padx=12, pady=(0, 6), fill="x")

        self._build_joystick(row)
        self._build_pot(row)
        self._build_motor(row)
        self._build_imu(row)

    def _panel_canvas(self, parent, title: str, size: int):
        tk = self._tk
        frame = tk.LabelFrame(
            parent, text=title, padx=4, pady=4,
            bg=self._BG, fg=self._TEXT, font=("Segoe UI", 9, "bold"), bd=1,
        )
        frame.pack(side="left", padx=(0, 10))
        c = tk.Canvas(frame, width=size, height=size, bg=self._SURFACE,
                      highlightthickness=0)
        c.pack()
        return frame, c

    def _build_joystick(self, parent) -> None:
        tk = self._tk
        SZ = 220
        frame, c = self._panel_canvas(parent, "Joystick", SZ)

        cx = cy = SZ / 2
        r  = SZ * 0.40

        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      outline=self._OVERLAY, width=2, fill=self._SURFACE1)
        c.create_line(cx - r + 4, cy, cx + r - 4, cy,
                      fill=self._OVERLAY, dash=(4, 4), width=1)
        c.create_line(cx, cy - r + 4, cx, cy + r - 4,
                      fill=self._OVERLAY, dash=(4, 4), width=1)
        c.create_text(cx + r - 4, cy - 10, text="+X", fill=self._SUBTEXT,
                      font=("Segoe UI", 7), anchor="e")
        c.create_text(cx + 10, cy - r + 4, text="+Y", fill=self._SUBTEXT,
                      font=("Segoe UI", 7), anchor="w")

        DOT_R = 9
        self._joy_dot = c.create_oval(cx - DOT_R, cy - DOT_R, cx + DOT_R, cy + DOT_R,
                                      fill=self._RED, outline=self._TEXT, width=1)
        self._joy_c   = (cx, cy, r)
        self._joy_canvas = c

        self._joy_lbl = tk.Label(frame, text="(0, 0)  btn: released",
                                 bg=self._BG, fg=self._SUBTEXT,
                                 font=("Segoe UI", 8))
        self._joy_lbl.pack(pady=(2, 0))

    def _build_pot(self, parent) -> None:
        tk = self._tk
        SZ = 220
        frame, c = self._panel_canvas(parent, "Potentiometer", SZ)

        cx = cy = SZ / 2
        r  = SZ * 0.36

        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      fill=self._SURFACE1, outline=self._OVERLAY, width=6)
        ir = r * 0.30
        c.create_oval(cx - ir, cy - ir, cx + ir, cy + ir,
                      fill=self._BG, outline=self._OVERLAY, width=2)

        for deg in range(0, 360, 30):
            a = math.radians(deg - 90)
            x1 = cx + (r - 2) * math.cos(a)
            y1 = cy + (r - 2) * math.sin(a)
            x2 = cx + (r + 6) * math.cos(a)
            y2 = cy + (r + 6) * math.sin(a)
            c.create_line(x1, y1, x2, y2, fill=self._OVERLAY, width=1)

        tip_x = cx
        tip_y = cy - r * 0.82
        self._pot_needle = c.create_line(cx, cy, tip_x, tip_y,
                                         fill=self._PINK, width=3,
                                         capstyle="round")
        self._pot_dot = c.create_oval(tip_x - 5, tip_y - 5,
                                       tip_x + 5, tip_y + 5,
                                       fill=self._PINK, outline="")
        self._pot_c = (cx, cy, r)
        self._pot_canvas = c

        self._pot_lbl = tk.Label(frame, text="0 / 4095  (0.0°)",
                                 bg=self._BG, fg=self._SUBTEXT,
                                 font=("Segoe UI", 8))
        self._pot_lbl.pack(pady=(2, 0))

    def _build_motor(self, parent) -> None:
        tk = self._tk
        SZ = 220
        frame, c = self._panel_canvas(parent, "Motor", SZ)

        cx = cy = SZ / 2
        r  = SZ * 0.36

        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      fill=self._SURFACE1, outline=self._BLUE, width=4)

        slot_r = r - 4
        for deg in range(0, 360, 45):
            a = math.radians(deg)
            x = cx + slot_r * math.cos(a)
            y = cy + slot_r * math.sin(a)
            c.create_oval(x - 5, y - 5, x + 5, y + 5,
                          fill=self._OVERLAY, outline="")

        ir = r * 0.42
        c.create_oval(cx - ir, cy - ir, cx + ir, cy + ir,
                      fill=self._BG, outline=self._BLUE, width=2)

        c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                      fill=self._BLUE, outline="")

        tip_x = cx
        tip_y = cy - ir * 0.88
        self._motor_needle = c.create_line(cx, cy, tip_x, tip_y,
                                            fill=self._BLUE, width=3,
                                            arrow=self._tk.LAST,
                                            arrowshape=(10, 12, 4))
        self._motor_c = (cx, cy, ir)
        self._motor_canvas = c

        self._motor_lbl = tk.Label(frame, text="0 / 4095  (0.0°)",
                                    bg=self._BG, fg=self._SUBTEXT,
                                    font=("Segoe UI", 8))
        self._motor_lbl.pack(pady=(2, 0))

    def _build_imu(self, parent) -> None:
        tk = self._tk
        SZ = 220
        frame, c = self._panel_canvas(parent, "IMU  Gravity Vector", SZ)

        cx = cy = SZ / 2
        arm = SZ * 0.34

        for (x3, y3, z3), color, label in [
            (( arm, 0, 0), self._FX_COL, "X"),
            ((0,  arm, 0), self._FY_COL, "Y"),
            ((0, 0,  arm), self._FZ_COL, "Z"),
        ]:
            ex, ey = self._iso(cx, cy, x3, y3, z3)
            c.create_line(cx, cy, ex, ey, fill=color, width=1, dash=(3, 4))
            c.create_text(ex, ey - 8, text=label, fill=color,
                          font=("Segoe UI", 8, "bold"))

        self._imu_arrow = c.create_line(cx, cy, cx, cy + arm * 0.85,
                                         fill=self._YELLOW, width=3,
                                         arrow=self._tk.LAST, arrowshape=(10, 12, 4))
        self._imu_dot = c.create_oval(cx - 5, cy + arm * 0.85 - 5,
                                       cx + 5, cy + arm * 0.85 + 5,
                                       fill=self._YELLOW, outline="")
        self._imu_c   = (cx, cy, arm)
        self._imu_canvas = c

        self._imu_lbl = tk.Label(frame, text="gx=0  gy=0  gz=0 LSB",
                                  bg=self._BG, fg=self._SUBTEXT,
                                  font=("Segoe UI", 8))
        self._imu_lbl.pack(pady=(2, 0))

    def _build_status_bar(self) -> None:
        tk = self._tk
        self._status_var = tk.StringVar(value="Status: waiting for data…")
        tk.Label(self._root, textvariable=self._status_var,
                 bg=self._BG, fg=self._SUBTEXT,
                 font=("Segoe UI", 8)).pack(pady=(0, 4))

    def _build_record_bar(self) -> None:
        tk = self._tk
        bar = tk.Frame(self._root, bg=self._BG)
        bar.pack(fill='x', padx=12, pady=(0, 8))

        tk.Label(bar, text='  File:', bg=self._BG, fg=self._SUBTEXT,
                 font=('Segoe UI', 9)).pack(side='left')
        self._csv_var = tk.StringVar(value='gui_data.csv')
        tk.Entry(bar, textvariable=self._csv_var, width=22,
                 bg=self._SURFACE1, fg=self._TEXT,
                 insertbackground=self._TEXT,
                 font=('Segoe UI', 9), relief='flat').pack(side='left', padx=4)


        self._rec_btn_var = tk.StringVar(value='Record')
        self._rec_btn = tk.Button(
            bar, textvariable=self._rec_btn_var,
            command=self._toggle_record,
            bg=self._RED, fg='white',
            font=('Segoe UI', 9, 'bold'), relief='flat', padx=8)
        self._rec_btn.pack(side='left')

        tk.Button(bar, text='Save CSV',
                  command=lambda: self._save_csv(self._root),
                  bg=self._SURFACE1, fg=self._TEXT,
                  font=('Segoe UI', 9), relief='flat', padx=8).pack(side='left')

        tk.Button(bar, text='Remove_offsets',
                  command=self.tare,
                  bg=self._TEAL, fg='white',
                  font=('Segoe UI', 9, 'bold'), relief='flat', padx=8).pack(side='left', padx=(6, 0))


    def tare(self):
        with self._lock:
            self._offset = np.array(self._esp.buffer[-1], dtype=float)

    # Isometric projection helper
    @staticmethod
    def _iso(cx: float, cy: float,
             x3: float, y3: float, z3: float) -> tuple[float, float]:
        cos30 = 0.8660254
        sin30 = 0.5
        sx = (x3 - z3) * cos30
        sy = -y3 + (x3 + z3) * sin30
        return cx + sx, cy + sy


    # Per-frame draw helpers
    def _draw_force(self, force_vectors) -> None:
        c       = self._fc
        col_w   = self._fc_col_w
        bar_h   = self._fc_bar_h
        margin  = self._fc_margin
        bar_max = self._fc_bar_max_w

        for s in range(self._N_SENSORS):
            x_col = s * col_w
            bar_x0 = x_col + self._fc_bar_x_off
            for i in range(3):
                bar_y = 24 + i * (bar_h + margin)

                # Signed value: the sign carries the direction of the shear
                # (Fx, Fy) and tells pushing from pulling on Fz.
                value = float(force_vectors[s][i]) if s < len(force_vectors) else 0.0
                norm  = max(-1.0, min(1.0, value / self._axis_scale[i]))
                bar_xc = bar_x0 + bar_max // 2
                px     = int(norm * (bar_max // 2))

                # Grow right of the zero line when positive, left when negative.
                x_lo, x_hi = (bar_xc, bar_xc + px) if px >= 0 else (bar_xc + px, bar_xc)
                c.coords(self._bar_ids[s][i],
                         x_lo, bar_y + 3, x_hi, bar_y + bar_h - 3)
                c.itemconfig(self._val_ids[s][i], text=f"{value:+.2f}")

    def _draw_joystick(self, joy_x, joy_y, joy_btn) -> None:
        c          = self._joy_canvas
        cx, cy, r  = self._joy_c
        DOT_R      = 9

        nx = max(-1.0, min(1.0, joy_x / self._JOY_HALF)) if joy_x is not None else 0.0
        ny = max(-1.0, min(1.0, joy_y / self._JOY_HALF)) if joy_y is not None else 0.0

        dx = cx + nx * r * 0.88
        dy = cy - ny * r * 0.88

        c.coords(self._joy_dot, dx - DOT_R, dy - DOT_R, dx + DOT_R, dy + DOT_R)
        c.itemconfig(self._joy_dot, fill=self._GREEN if joy_btn else self._RED)

        btn_txt = "PRESSED" if joy_btn else "released"
        jx = joy_x or 0
        jy = joy_y or 0
        self._joy_lbl.config(text=f"({jx:+5d}, {jy:+5d})  btn: {btn_txt}")

    def _draw_pot(self, pot_value) -> None:
        if pot_value is None:
            return
        c         = self._pot_canvas
        cx, cy, r = self._pot_c

        angle_deg = (pot_value / 4095.0) * 360.0 - 90.0
        angle_rad = math.radians(angle_deg)
        tip_r     = r * 0.82

        tip_x = cx + tip_r * math.cos(angle_rad)
        tip_y = cy + tip_r * math.sin(angle_rad)

        c.coords(self._pot_needle, cx, cy, tip_x, tip_y)
        c.coords(self._pot_dot, tip_x - 5, tip_y - 5, tip_x + 5, tip_y + 5)
        self._pot_lbl.config(
            text=f"{pot_value} / 4095  ({pot_value / 4095.0 * 360.0:.1f}°)"
        )

    def _draw_motor(self, motor_pos) -> None:
        if motor_pos is None:
            return
        c         = self._motor_canvas
        cx, cy, r = self._motor_c

        angle_deg = (motor_pos / 4095.0) * 360.0 - 90.0
        angle_rad = math.radians(angle_deg)
        tip_r     = r * 0.88

        tip_x = cx + tip_r * math.cos(angle_rad)
        tip_y = cy + tip_r * math.sin(angle_rad)

        c.coords(self._motor_needle, cx, cy, tip_x, tip_y)
        self._motor_lbl.config(
            text=f"{motor_pos} / 4095  ({motor_pos / 4095.0 * 360.0:.1f}°)"
        )

    def _draw_imu(self, gravity_raw_lsb) -> None:
        if gravity_raw_lsb is None:
            return
        gx, gy, gz = gravity_raw_lsb
        c           = self._imu_canvas
        cx, cy, arm = self._imu_c

        scale = arm / self._GRAVITY_NORM
        tx, ty = self._iso(cx, cy, gx * scale, gy * scale, gz * scale)

        c.coords(self._imu_arrow, cx, cy, tx, ty)
        c.coords(self._imu_dot, tx - 5, ty - 5, tx + 5, ty + 5)
        self._imu_lbl.config(text=f"gx={gx:+d}  gy={gy:+d}  gz={gz:+d} LSB")


    # Main tick
    def _tick(self) -> None:
        if not self._running:
            return
        try:
            esp = self._esp

            now = time.time()
            dt  = now - self._last_rate_t
            if dt >= 0.5:
                dc = esp.sample_counter - self._last_sample_count
                self._rate_var.set(f"Rate: {dc / dt:.1f} Hz")
                self._last_rate_t = now
                self._last_sample_count = esp.sample_counter

            self._sample_var.set(f"Sample: #{esp.sample_counter:,}")

            forces_tared = np.array(esp.buffer[-1], dtype=float) - self._offset
            self._draw_force(forces_tared)
            self._draw_joystick(esp.joystick_x, esp.joystick_y, esp.joystick_button)
            self._draw_pot(esp.potentiometer_value)
            self._draw_motor(esp.motor_position)
            self._draw_imu(esp.gravity_raw_lsb)

            if not self._no_motor_pot and esp.potentiometer_value is not None:
                pot = esp.potentiometer_value
                if (self._last_motor_cmd_pot is None
                        or abs(pot - self._last_motor_cmd_pot) >= self._motor_debounce):
                    esp.sendCommandMotor(position=pot, speed=1000, acc=20)
                    self._last_motor_cmd_pot = pot

            self._status_var.set("Status: streaming")

            # Compute wrenches from tared forces
            n  = self._N_SENSORS
            lF = forces_tared[:4]
            fx_L = sum(float(lF[i][0]) for i in range(4))
            fy_L = sum(float(lF[i][1]) for i in range(4))
            fz_L = sum(float(lF[i][2]) for i in range(4))
            tx_L = (+float(lF[0][2])*0.0035 + float(lF[1][2])*0.0035
                    - float(lF[2][2])*0.0035 - float(lF[3][2])*0.0035)
            ty_L = (-float(lF[0][2])*0.0035 + float(lF[1][2])*0.0035
                    - float(lF[2][2])*0.0035 + float(lF[3][2])*0.0035)
            tz_L = (-float(lF[0][0])*0.0035*_SIN45 + float(lF[0][1])*0.0035*_SIN45
                  - float(lF[1][0])*0.0035*_SIN45 - float(lF[1][1])*0.0035*_SIN45
                  + float(lF[2][0])*0.0035*_SIN45 + float(lF[2][1])*0.0035*_SIN45
                  + float(lF[3][0])*0.0035*_SIN45 - float(lF[3][1])*0.0035*_SIN45)
            rF = forces_tared[4:]
            fx_R = sum(float(rF[i][0]) for i in range(4))
            fy_R = sum(float(rF[i][1]) for i in range(4))
            fz_R = sum(float(rF[i][2]) for i in range(4))
            tx_R = (+float(rF[0][2])*0.0035 + float(rF[1][2])*0.0035
                    - float(rF[2][2])*0.0035 - float(rF[3][2])*0.0035)
            ty_R = (-float(rF[0][2])*0.0035 + float(rF[1][2])*0.0035
                    - float(rF[2][2])*0.0035 + float(rF[3][2])*0.0035)
            tz_R = (-float(rF[0][0])*0.0035*_SIN45 + float(rF[0][1])*0.0035*_SIN45
                   - float(rF[1][0])*0.0035*_SIN45 - float(rF[1][1])*0.0035*_SIN45
                  + float(rF[2][0])*0.0035*_SIN45 + float(rF[2][1])*0.0035*_SIN45
                  + float(rF[3][0])*0.0035*_SIN45 - float(rF[3][1])*0.0035*_SIN45)
            fx_tot = fx_L + fx_R
            fy_tot = fy_L - fy_R
            fz_tot = fz_L - fz_R
            tx_tot = tx_L + tx_R
            ty_tot = ty_L - ty_R
            tz_tot = tz_L - tz_R

            if esp.gravity_raw_lsb is not None:
                gx = esp.gravity_raw_lsb[0] / 100.0
                gy = esp.gravity_raw_lsb[1] / 100.0
                gz = esp.gravity_raw_lsb[2] / 100.0
            else:
                gx = gy = gz = 0.0
            g_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
            fg    = ((fx_tot*gx + fy_tot*gz + fz_tot*gy) / g_mag
                     if g_mag > 1e-6 else 0.0)
            fn_tot = -(fz_L + fz_R)

            # Smooth fn and fg 
            a, ar = self._lpf_alpha, self._lpf_alpha_ratio
            if not self._lpf_seeded:
                self._filtered_fn = fn_tot
                self._filtered_fg = fg
                self._lpf_seeded  = True
            else:
                self._filtered_fn = a * fn_tot + (1.0 - a) * self._filtered_fn
                self._filtered_fg = a * fg     + (1.0 - a) * self._filtered_fg
            ratio_inst = (self._filtered_fg / self._filtered_fn
                          if abs(self._filtered_fn) > 1e-6 else 0.0)
            self._filtered_ratio = (ar * ratio_inst
                                    + (1.0 - ar) * self._filtered_ratio)
            fn_tot      = self._filtered_fn
            fg          = self._filtered_fg
            ratio_fg_fn = self._filtered_ratio

            # Rolling-window peak-to-peak amplitude of the Fg/Fn ratio 
            self._ratio_window.append(ratio_fg_fn)
            if len(self._ratio_window) == self._ratio_window_size:
                amp_inst = max(self._ratio_window) - min(self._ratio_window)
                aa = self._amp_alpha
                self._ratio_amp = aa * amp_inst + (1.0 - aa) * self._ratio_amp

            if self._wrench_callback is not None:
                self._wrench_callback(
                    fx_L, fy_L, fz_L, tx_L, ty_L, tz_L,
                    fx_R, fy_R, fz_R, tx_R, ty_R, tz_R,
                    fx_tot, fy_tot, fz_tot, tx_tot, ty_tot, tz_tot,
                    forces_tared,
                )

            if self._is_recording:
                t = time.time()
                with self._lock:
                    if self._rec_t0 is None:
                        self._rec_t0 = t
                    sensor_fn  = [float(forces_tared[s][2]) for s in range(n)]
                    sensor_fsx = [float(forces_tared[s][0]) for s in range(n)]
                    sensor_fsy = [float(forces_tared[s][1]) for s in range(n)]

                    row = [round(t - self._rec_t0, 4)]
                    for s in range(n):
                        row += [round(sensor_fn[s], 4),
                                round(sensor_fsx[s], 4),
                                round(sensor_fsy[s], 4)]
                    row += [round(fx_L, 4), round(fy_L, 4), round(fz_L, 4),
                            round(tx_L, 4), round(ty_L, 4), round(tz_L, 4)]
                    row += [round(fx_R, 4), round(fy_R, 4), round(fz_R, 4),
                            round(tx_R, 4), round(ty_R, 4), round(tz_R, 4)]
                    row += [round(fx_tot, 4), round(fy_tot, 4), round(fz_tot, 4),
                            round(tx_tot, 4), round(ty_tot, 4), round(tz_tot, 4)]
                    row += [
                        esp.motor_position if esp.motor_position is not None else '',
                        round(fg, 4),
                        round(fn_tot, 4),
                        round(ratio_fg_fn, 4),
                        '',
                        round(self._ratio_amp, 4),
                    ]
                    self._recording.append(row)

        except Exception as exc:
            self._status_var.set(f"Status: error — {exc}")

        self._root.after(self._poll_ms, self._tick)

    def _on_close(self) -> None:
        self._running = False
        self._root.destroy()

    def _toggle_record(self):
        with self._lock:
            self._is_recording = not self._is_recording
            recording = self._is_recording
            if recording:
                self._recording.clear()
                self._rec_t0 = None

        if recording:
            self._rec_btn_var.set('Stop Record')
            self._rec_btn.config(bg='#f9e2af', fg='#1e1e2e')
        else:
            self._rec_btn_var.set('Start Record')
            self._rec_btn.config(bg=self._RED, fg='white')

    def _build_csv_header(self) -> list[str]:
        n = self._N_SENSORS
        cols = ['time_s']
        for i in range(n):
            cols += [f'Sensor{i+1}_Fnormal [N]',
                     f'Sensor{i+1}_Fshear_X [N]',
                     f'Sensor{i+1}_Fshear_Y [N]']
        for side in ('Left finger', 'Right finger'):
            cols += [f'{side} - force in x [N]',
                     f'{side} - force in y [N]',
                     f'{side} - force in z [N]',
                     f'{side} - torque in x [N.m]',
                     f'{side} - torque in y [N.m]',
                     f'{side} - torque in z [N.m]']
        cols += ['Total tactaxis - force in x [N]',
                 'Total tactaxis - force in y [N]',
                 'Total tactaxis - force in z [N]',
                 'Total tactaxis - torque in x [N.m]',
                 'Total tactaxis - torque in y [N.m]',
                 'Total tactaxis - torque in z [N.m]',
                 'Position gripper',
                 'Gravity force Fg [N]',
                 'Normal force Fn [N]',
                 'Ratio Fg/Fn [-]',
                 'Target force [N]',
                 'Fg/Fn ratio peak-to-peak amplitude [-]']
        return cols

    def _save_csv(self, root):
        import tkinter.messagebox as mb
        with self._lock:
            rows = list(self._recording)

        if not rows:
            mb.showwarning('No data', 'No recorded data yet.', parent=root)
            return

        basename = self._csv_var.get().strip() or 'gui_data.csv'
        if not basename.endswith('.csv'):
            basename += '.csv'
        filepath = os.path.join(_RECORD_DIR, basename)

        try:
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self._build_csv_header())
                writer.writerows(rows)
            abs_path = os.path.abspath(filepath)
            mb.showinfo('Saved',
                        f'✅  {len(rows)} rows saved to:\n{abs_path}',
                        parent=root)
        except OSError as e:
            mb.showerror('Save failed', str(e), parent=root)

    def run(self) -> None:
        self._running = True
        self._tick()
        self._root.mainloop()



# ROS2 node

class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')

        self.declare_parameter('poll_ms',      50)
        self.declare_parameter('shear_scale',  2.0)
        self.declare_parameter('normal_scale', 5.0)
        self.declare_parameter('no_motor_pot', True)

        poll_ms      = self.get_parameter('poll_ms').value
        shear_scale  = self.get_parameter('shear_scale').value
        normal_scale = self.get_parameter('normal_scale').value
        no_motor_pot = self.get_parameter('no_motor_pot').value

        self._motor_pub    = self.create_publisher(Float32MultiArray, '/motor/command',               10)
        self._pub_wrench_L = self.create_publisher(WrenchStamped,      '/tactile/wrench/left_finger',  10)
        self._pub_wrench_R = self.create_publisher(WrenchStamped,      '/tactile/wrench/right_finger', 10)
        self._pub_wrench_T = self.create_publisher(WrenchStamped,      '/tactile/wrench/total',        10)
        self._pub_sensor_wrench = (
            [self.create_publisher(WrenchStamped, f'/tactile/wrench/left_{i}',  10) for i in range(4)] +
            [self.create_publisher(WrenchStamped, f'/tactile/wrench/right_{i}', 10) for i in range(4)]
        )
        self._proxy        = _EspProxy(self._motor_pub)

        self.create_subscription(Float32MultiArray, '/esp/force',
                                 self._on_force,     10)
        self.create_subscription(Imu,               '/esp/imu',
                                 self._on_imu,       10)
        self.create_subscription(Joy,               '/esp/joystick',
                                 self._on_joy,       10)
        self.create_subscription(Int32,             '/esp/potentiometer',
                                 self._on_pot,       10)
        self.create_subscription(Int32,             '/esp/motor_position',
                                 self._on_motor_pos, 10)

        self._poll_ms      = poll_ms
        self._shear_scale  = shear_scale
        self._normal_scale = normal_scale
        self._no_motor_pot = no_motor_pot

        self.get_logger().info('GUI node ready, opening window.')

    def _on_force(self, msg: Float32MultiArray):
        d = msg.data
        arr = np.zeros((_N_SENSORS, 3), dtype=np.float32)
        for i in range(_N_SENSORS):
            b = i * 3
            if b + 2 < len(d):
                arr[i] = [d[b], d[b + 1], d[b + 2]]
        self._proxy.buffer = arr.reshape((1, _N_SENSORS, 3))
        self._proxy.sample_counter += 1

    def _on_imu(self, msg: Imu):
        # esp_bridge_node multiplied raw LSB by 0.01 to get m/s²; undo that here.
        gx = int(msg.linear_acceleration.x * 100)
        gy = int(msg.linear_acceleration.y * 100)
        gz = int(msg.linear_acceleration.z * 100)
        self._proxy.gravity_raw_lsb = (gx, gy, gz)

    def _on_joy(self, msg: Joy):
        self._proxy.joystick_x      = int(msg.axes[0])    if len(msg.axes)    > 0 else None
        self._proxy.joystick_y      = int(msg.axes[1])    if len(msg.axes)    > 1 else None
        self._proxy.joystick_button = bool(msg.buttons[0]) if len(msg.buttons) > 0 else None

    def _on_pot(self, msg: Int32):
        self._proxy.potentiometer_value = msg.data

    def _on_motor_pos(self, msg: Int32):
        self._proxy.motor_position = msg.data

    def run_gui(self):
        def _wrench_cb(fx_L, fy_L, fz_L, tx_L, ty_L, tz_L,
                       fx_R, fy_R, fz_R, tx_R, ty_R, tz_R,
                       fx_T, fy_T, fz_T, tx_T, ty_T, tz_T,
                       forces_tared):
            stamp = self.get_clock().now().to_msg()
            def _make(frame_id, fx, fy, fz, tx=0.0, ty=0.0, tz=0.0):
                w = WrenchStamped()
                w.header.stamp    = stamp
                w.header.frame_id = frame_id
                w.wrench.force.x  = fx;  w.wrench.force.y  = fy;  w.wrench.force.z  = fz
                w.wrench.torque.x = tx;  w.wrench.torque.y = ty;  w.wrench.torque.z = tz
                return w
            self._pub_wrench_L.publish(_make('frame_left_finger',  fx_L, fy_L, fz_L, tx_L, ty_L, tz_L))
            self._pub_wrench_R.publish(_make('frame_right_finger', fx_R, fy_R, fz_R, tx_R, ty_R, tz_R))
            self._pub_wrench_T.publish(_make('world',              fx_T, fy_T, fz_T, tx_T, ty_T, tz_T))
            for f_idx, side in enumerate(('left', 'right')):
                for s_idx in range(4):
                    sensor = forces_tared[f_idx * 4 + s_idx]
                    self._pub_sensor_wrench[f_idx * 4 + s_idx].publish(
                        _make(f'frame_{side}_tactile_{s_idx}',
                              float(sensor[0]), float(sensor[1]), float(sensor[2]))
                    )

        gui = BurstModeGui(
            self._proxy,
            poll_ms=self._poll_ms,
            shear_scale=self._shear_scale,
            normal_scale=self._normal_scale,
            no_motor_pot=self._no_motor_pot,
            motor_debounce=_MOTOR_DEBOUNCE_LSB,
            wrench_callback=_wrench_cb,
        )
        gui.run()

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().warn(f'GUI unavailable ({e}), running headless.')
        try:
            spin_thread.join()
        except KeyboardInterrupt:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
