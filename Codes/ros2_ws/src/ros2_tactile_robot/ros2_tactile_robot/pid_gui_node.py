"""
Live GUI with real-time Fx / Fy / Fz plots, PID state panel, and detailed
per-sensor CSV recording.

Subscribers:
  /esp/force          std_msgs/Float32MultiArray   (n_sensors × 3 floats)
  /esp/imu            sensor_msgs/Imu              (gravity in linear_acceleration)
  /esp/motor_position std_msgs/Int32               (gripper position 0–4095)
  /pid/state          std_msgs/Float32MultiArray   [target, current, error, integral, deriv, output]

Publishers:
  /pid/gains          std_msgs/Float32MultiArray   ([kp, ki, kd])
  /pid/tare           std_msgs/Bool                (trigger zero offset on pid_control_node)
"""

import csv
import math
import os
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int32
from sensor_msgs.msg import Imu

# Palette 
_BG      = '#1e1e2e'
_SURFACE = '#313244'
_TEXT    = '#cdd6f4'
_SUBTEXT = "#c4c4c7"
_FX_COL  = '#e63946'
_FY_COL  = '#4895ef'
_FZ_PLOT = '#f9e2af'
_GREEN   = '#a6e3a1'
_YELLOW  = '#f9e2af'
_PINK    = '#f38ba8'
_BLUE    = '#89b4fa'

_WINDOW_S    = 20.0
_MAX_PTS     = 10000
_RECORD_DIR  = 'record_data'
_N_WARMUP    = 50   # samples discarded so the filter + IMU stream settle first
_N_CALIB     = 20   # samples averaged for the startup auto-zero baseline
try:
    from ament_index_python.packages import get_package_share_directory
    _RESOURCES_DIR = os.path.join(
        get_package_share_directory('ros2_tactile_robot'), 'Resources')
except Exception:
    # Fallback for running the script directly outside a ROS 2 install
    _RESOURCES_DIR = os.path.join(os.path.dirname(__file__), 'Resources')


class ForcePlotGuiNode(Node):
    def __init__(self):
        super().__init__('force_plot_gui_node')

        self.declare_parameter('poll_ms',       50)
        self.declare_parameter('window_s',      _WINDOW_S)
        self.declare_parameter('sensor_numero', 8)

        self._poll_ms   = self.get_parameter('poll_ms').value
        self._window_s  = self.get_parameter('window_s').value
        self._n_sensors = self.get_parameter('sensor_numero').value

        os.makedirs(_RECORD_DIR, exist_ok=True)

        # Ring buffers
        self._lock   = threading.Lock()
        self._t_buf  = deque(maxlen=_MAX_PTS)
        self._fn_buf = deque(maxlen=_MAX_PTS)
        self._fg_buf = deque(maxlen=_MAX_PTS)
        self._t0: float | None = None

        # Startup auto-zero
        self._filter_initialized = False   # set True after first-sample filter init
        self._calib_count = 0              # samples seen during warm-up + calibration
        self._calib_fn: list[float] = []
        self._calib_fg: list[float] = []
        self._offset_ready = False

        # Zero offset
        self._zeroed      = False
        self._last_raw_fn = 0.0
        self._last_raw_fg = 0.0

        # Butterworth low-pass filter (cutoff frequency: 10 Hz, order 2, sampling frequency: 500 Hz)
        self._filter_enabled = True
        self._filter_fs      = 500.0
        self._filter_cutoff  = 10.0
        self._filter_order   = 2
        self._sos            = None
        self._zi_sensors     = None
        self._build_filter()

        # Latest peripheral state
        self._gravity_ms2   = (0.0, 0.0, 0.0)
        self._gripper_pos   = 0

        # PID state  [target, current, error, integral, derivative, pid_output]
        self._pid_state: list[float] = [0.0] * 6
        self._pid_target = 0.0

        # Latest /pid/slip values  [ratio, fg, fn]
        self._slip_fn    = 0.0
        self._slip_fg    = 0.0
        self._slip_ratio = 0.0
        self._offset_fn  = 0.0
        self._offset_fg  = 0.0

        # Peak-to-peak amplitude of the Fg/Fn ratio over a rolling window
        # micro-slip / vibration indicator
        self._ratio_window_size               = 20
        self._ratio_window: deque[float]      = deque(maxlen=self._ratio_window_size)
        self._ratio_amp                       = 0.0
        self._amp_alpha                       = 1.0


        # Rate tracking
        self._sample_count = 0
        self._rate_count   = 0
        self._rate_t       = time.time()
        self._rate_hz      = 0.0

        # Recording
        self._recording: list[tuple] = []
        self._is_recording = False
        self._rec_t0: float | None = None

        # Publishers / Subscriptions
        self._gains_pub = self.create_publisher(Float32MultiArray, '/pid/gains', 10)
        self._tare_pub  = self.create_publisher(Bool,              '/pid/tare',  10)

        self.create_subscription(Float32MultiArray, '/esp/force',
                                 self._on_force, 10)
        self.create_subscription(Imu, '/esp/imu',
                                 self._on_imu, 10)
        self.create_subscription(Int32, '/esp/motor_position',
                                 self._on_motor_pos, 10)
        self.create_subscription(Int32, '/esp/potentiometer',
                                 self._on_pot, 10)
        self.create_subscription(Float32MultiArray, '/pid/state',
                                 self._on_pid_state, 10)
        self.create_subscription(Float32MultiArray, '/pid/slip',
                                 self._on_slip, 10)

        self.get_logger().info('Force-plot GUI node ready.')

    # Butterworth filter 

    def _build_filter(self):
        from scipy.signal import butter, sosfilt_zi
        nyq    = 0.5 * self._filter_fs
        cutoff = min(self._filter_cutoff, nyq * 0.99)
        sos    = butter(self._filter_order, cutoff / nyq, btype='low', output='sos')
        zi_tmpl = sosfilt_zi(sos)
        with self._lock:
            self._sos = sos
            self._zi_sensors = [
                [zi_tmpl.copy() for _ in range(3)]
                for _ in range(self._n_sensors)
            ]
            self._filter_initialized = False   # next message will re-warm the filter

    def _filter_sample(self, value: float, zi):
        from scipy.signal import sosfilt
        y, zi_new = sosfilt(self._sos, [value], zi=zi)
        return float(y[0]), zi_new

    # ROS2 callbacks

    def _on_force(self, msg: Float32MultiArray):
        data = msg.data
        n    = self._n_sensors
        now  = time.perf_counter()

        with self._lock:
            if self._filter_enabled and self._sos is not None and not self._filter_initialized:
                from scipy.signal import sosfilt_zi
                zi_tmpl = sosfilt_zi(self._sos)
                for i in range(n):
                    b   = i * 3
                    fx0 = float(data[b])     if b     < len(data) else 0.0
                    fy0 = float(data[b + 1]) if b + 1 < len(data) else 0.0
                    fz0 = float(data[b + 2]) if b + 2 < len(data) else 0.0
                    self._zi_sensors[i][0] = zi_tmpl.copy() * fx0
                    self._zi_sensors[i][1] = zi_tmpl.copy() * fy0
                    self._zi_sensors[i][2] = zi_tmpl.copy() * fz0
                self._filter_initialized = True

            # Per-sensor forces 
            sensor_fx = []
            sensor_fy = []
            sensor_fz = []
            for i in range(n):
                b  = i * 3
                fx = float(data[b])     if b     < len(data) else 0.0
                fy = float(data[b + 1]) if b + 1 < len(data) else 0.0
                fz = float(data[b + 2]) if b + 2 < len(data) else 0.0
                if self._filter_enabled and self._sos is not None:
                    fx, self._zi_sensors[i][0] = self._filter_sample(fx, self._zi_sensors[i][0])
                    fy, self._zi_sensors[i][1] = self._filter_sample(fy, self._zi_sensors[i][1])
                    fz, self._zi_sensors[i][2] = self._filter_sample(fz, self._zi_sensors[i][2])
                sensor_fx.append(fx)
                sensor_fy.append(fy)
                sensor_fz.append(fz)

            # Aggregate left / right
            half     = n // 2
            fx_L     = sum(sensor_fx[:half])
            fy_L     = sum(sensor_fy[:half])
            fz_L     = sum(sensor_fz[:half])
            fx_R     = sum(sensor_fx[half:])
            fy_R     = sum(sensor_fy[half:])
            fz_R     = sum(sensor_fz[half:])
            fx_tot   = fx_L + fx_R
            fy_tot   = fy_L - fy_R
            fz_tot   = fz_L - fz_R

            # Compute fn and fg locally for auto-zero calibration
            fn_raw = -(fz_L + fz_R)
            gx_c, gy_c, gz_c = self._gravity_ms2
            g_mag_c = math.sqrt(gx_c*gx_c + gy_c*gy_c + gz_c*gz_c)
            fg_raw  = ((fx_tot*gx_c + fy_tot*gz_c + fz_tot*gy_c) / g_mag_c
                       if g_mag_c > 1e-6 else 0.0)

            # Track latest filtered totals
            self._last_raw_fn = fn_raw
            self._last_raw_fg = fg_raw

            # Startup auto-zero 
            if not self._offset_ready:
                self._calib_count += 1
                if self._calib_count <= _N_WARMUP or g_mag_c <= 1e-6:
                    return  # discard while the filter settles / waiting for IMU
                self._calib_fn.append(fn_raw)
                self._calib_fg.append(fg_raw)
                if len(self._calib_fn) < _N_CALIB:
                    return  # hold the plot until the calibration window is filled
                self._offset_fn    = sum(self._calib_fn) / len(self._calib_fn)
                self._offset_fg    = sum(self._calib_fg) / len(self._calib_fg)
                self._offset_ready = True
                self._zeroed       = True
                self.get_logger().info(
                    f'Auto-zero: Fn={self._offset_fn:.3f}  Fg={self._offset_fg:.3f} N '
                    f'(averaged {len(self._calib_fn)} samples)')
                # no return — fall through so this sample plots as 0 N

            # Per-finger torques 
            _s45 = math.sin(math.radians(45))
            lf   = list(zip(sensor_fx[:half], sensor_fy[:half], sensor_fz[:half]))
            rf   = list(zip(sensor_fx[half:], sensor_fy[half:], sensor_fz[half:]))
            tx_L = (+lf[0][2]*0.0035 + lf[1][2]*0.0035
                    - lf[2][2]*0.0035 - lf[3][2]*0.0035)
            ty_L = (-lf[0][2]*0.0035 + lf[1][2]*0.0035
                    - lf[2][2]*0.0035 + lf[3][2]*0.0035)
            tz_L = (-lf[0][0]*0.0035*_s45 + lf[0][1]*0.0035*_s45
                  - lf[1][0]*0.0035*_s45 - lf[1][1]*0.0035*_s45
                  + lf[2][0]*0.0035*_s45 + lf[2][1]*0.0035*_s45
                  + lf[3][0]*0.0035*_s45 - lf[3][1]*0.0035*_s45)
            tx_R = (+rf[0][2]*0.0035 + rf[1][2]*0.0035
                    - rf[2][2]*0.0035 - rf[3][2]*0.0035)
            ty_R = (-rf[0][2]*0.0035 + rf[1][2]*0.0035
                    - rf[2][2]*0.0035 + rf[3][2]*0.0035)
            tz_R = (-rf[0][0]*0.0035*_s45 + rf[0][1]*0.0035*_s45
                  - rf[1][0]*0.0035*_s45 - rf[1][1]*0.0035*_s45
                  + rf[2][0]*0.0035*_s45 + rf[2][1]*0.0035*_s45
                  + rf[3][0]*0.0035*_s45 - rf[3][1]*0.0035*_s45)
            tx_tot = tx_L + tx_R
            ty_tot = ty_L - ty_R
            tz_tot = tz_L - tz_R

            gripper = self._gripper_pos

            fn_plot = fn_raw - self._offset_fn
            fg_plot = fg_raw - self._offset_fg

            if self._t0 is None:
                self._t0 = now
            t_plot = now - self._t0
            self._t_buf.append(t_plot)
            self._fn_buf.append(fn_plot)
            self._fg_buf.append(fg_plot)
            self._sample_count += 1
            self._rate_count   += 1

            # Recording
            if self._is_recording:
                if self._rec_t0 is None:
                    self._rec_t0 = now
                t_rec = now - self._rec_t0
                row = [t_rec]
                for i in range(n):
                    row += [sensor_fz[i], sensor_fx[i], sensor_fy[i]]
                row += [fx_L, fy_L, fz_L, tx_L, ty_L, tz_L]
                row += [fx_R, fy_R, fz_R, tx_R, ty_R, tz_R]
                row += [fx_tot, fy_tot, fz_tot, tx_tot, ty_tot, tz_tot]
                row += [gripper,
                        self._slip_fg, self._slip_fn,
                        self._slip_ratio,
                        self._pid_target,
                        self._ratio_amp]
                self._recording.append(tuple(row))

        now_wall = time.time()
        dt = now_wall - self._rate_t
        if dt >= 0.5:
            with self._lock:
                self._rate_hz    = self._rate_count / dt
                self._rate_count = 0
            self._rate_t = now_wall

    def _on_imu(self, msg: Imu):
        with self._lock:
            self._gravity_ms2 = (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )

    def _on_motor_pos(self, msg: Int32):
        with self._lock:
            self._gripper_pos = msg.data

    def _on_pot(self, msg: Int32):
        with self._lock:
            self._pid_target = (msg.data / 4095.0) * 5.0

    def _on_pid_state(self, msg: Float32MultiArray):
        if msg.data:
            padded = list(msg.data) + [0.0] * 6
            with self._lock:
                self._pid_state = padded[:6]

    def _on_slip(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            with self._lock:
                self._slip_ratio = float(msg.data[0])
                self._slip_fg    = float(msg.data[1])
                self._slip_fn    = float(msg.data[2])
                # Rolling-window peak-to-peak amplitude of the Fg/Fn ratio as a slip / vibration indicator
                self._ratio_window.append(self._slip_ratio)
                if len(self._ratio_window) == self._ratio_window_size:
                    amp_inst = max(self._ratio_window) - min(self._ratio_window)
                    a = self._amp_alpha
                    self._ratio_amp = a * amp_inst + (1.0 - a) * self._ratio_amp

    # Zero offset 

    def _zero_offset(self):
        with self._lock:
            self._offset_fn = self._last_raw_fn
            self._offset_fg = self._last_raw_fg
            self._zeroed    = True
            self._t_buf.clear()
            self._fn_buf.clear()
            self._fg_buf.clear()
            self._t0 = None
        tare_msg = Bool()
        tare_msg.data = True
        self._tare_pub.publish(tare_msg)
        self.get_logger().info(
            f'Tare applied: Fn={self._offset_fn:.4f}  Fg={self._offset_fg:.4f} N  '
            f'→ published /pid/tare')

    # Publish PID gains 
    def _apply_gains(self, kp_var, ki_var, kd_var):
        try:
            kp = float(kp_var.get())
            ki = float(ki_var.get())
            kd = float(kd_var.get())
        except ValueError:
            return
        msg = Float32MultiArray()
        msg.data = [kp, ki, kd]
        self._gains_pub.publish(msg)
        self.get_logger().info(f'PID gains → Kp={kp}  Ki={ki}  Kd={kd}')

    # GUI entry point (main thread)

    def run_gui(self):
        import tkinter as tk
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.animation import FuncAnimation

        root = tk.Tk()
        root.title('Force Monitor — Real-Time')
        root.configure(bg=_BG)

        # Logo bar (top) — left / centre / right 
        logo_frame = tk.Frame(root, bg=_BG)
        logo_frame.pack(fill='x', padx=6, pady=(6, 2))
        # Three equal columns so logos sit at left edge, centre, and right edge
        for c in range(3):
            logo_frame.columnconfigure(c, weight=1)
        self._logo_refs = []  # prevent GC of PhotoImage objects
        logo_specs = [
            ('VUB_LOGO.png',           80, 0, 'w'),
            ('IMEC_LOGO.png',          120, 1, ''),
            ('Melexis_logo_white.png', 80, 2, 'e'),
        ]
        for fname, target_h, col, sticky in logo_specs:
            path = os.path.join(_RESOURCES_DIR, fname)
            if not os.path.isfile(path):
                continue
            try:
                from PIL import Image, ImageTk
                img = Image.open(path).convert('RGBA')
                ratio = target_h / img.height
                img = img.resize((max(1, int(img.width * ratio)), target_h),
                                 Image.LANCZOS)
                photo = ImageTk.PhotoImage(img)
            except ImportError:
                photo = tk.PhotoImage(file=path)
                ss = max(1, photo.height() // target_h)
                if ss > 1:
                    photo = photo.subsample(ss, ss)
            except Exception as e:
                self.get_logger().warn(f'Logo load failed ({fname}): {e}')
                continue
            self._logo_refs.append(photo)
            tk.Label(logo_frame, image=photo, bg=_BG).grid(
                row=0, column=col, sticky=sticky, padx=8, pady=4)

        # Plot
        fig, ax = plt.subplots(figsize=(11, 4.5))
        fig.patch.set_facecolor(_BG)
        ax.set_facecolor(_SURFACE)
        ax.set_xlabel('Time [s]', color=_TEXT)
        ax.set_ylabel('Force [N]', color=_TEXT)
        ax.set_title('Normal Force Fn and Gravity Force Fg', color=_TEXT)
        ax.tick_params(colors=_TEXT)
        for spine in ax.spines.values():
            spine.set_edgecolor(_SUBTEXT)
        ax.axhline(0, color=_SUBTEXT, linewidth=0.6, linestyle='--')
        ax.grid(True, color='#45475a', linewidth=0.4)

        line_fn,     = ax.plot([], [], color=_FX_COL,  linewidth=1.2, label='Fn')
        line_fg,     = ax.plot([], [], color=_FY_COL,  linewidth=1.2, label='Fg')
        line_target, = ax.plot([], [], color=_GREEN,   linewidth=2.0,
                               linestyle='--', label='Target', alpha=0.9)
        ax.legend(facecolor=_SURFACE, edgecolor=_SUBTEXT,
                  labelcolor=_TEXT, loc='upper left', fontsize=8)

        canvas = FigureCanvasTkAgg(fig, master=root)
        canvas.get_tk_widget().pack(fill='both', expand=True, padx=6, pady=6)

        # State bar 
        pid_frame = tk.Frame(root, bg=_SURFACE)
        pid_frame.pack(fill='x', padx=6, pady=2)

        def _lbl(parent, text, fg=_SUBTEXT, bg=_SURFACE):
            return tk.Label(parent, text=text, bg=bg, fg=fg, font=('Courier', 11))

        self._pid_vars = {}
        for name, fg in (('Target', _GREEN), ('Error', _PINK)):
            _lbl(pid_frame, f'{name}:', bg=_SURFACE).pack(side='left', padx=(6, 2))
            v = tk.StringVar(value='—')
            self._pid_vars[name] = v
            tk.Label(pid_frame, textvariable=v, bg=_SURFACE, fg=fg,
                     font=('Courier', 11, 'bold'), width=8).pack(side='left', padx=(0, 4))

        # PID gains bar
        gains_frame = tk.Frame(root, bg=_SURFACE)
        gains_frame.pack(fill='x', padx=6, pady=(0, 2))

        _lbl(gains_frame, 'PID Gains ▸').pack(side='left', padx=(4, 8))
        kp_var = tk.StringVar(value='0.0')
        ki_var = tk.StringVar(value='0.0')
        kd_var = tk.StringVar(value='0.0')

        def _gain_field(parent, label, var):
            _lbl(parent, f'{label}:', fg=_TEXT).pack(side='left', padx=(0, 2))
            tk.Entry(parent, textvariable=var, width=8,
                     bg=_BG, fg=_YELLOW, insertbackground=_TEXT,
                     font=('Courier', 11), relief='flat').pack(side='left', padx=(0, 10))

        _gain_field(gains_frame, 'Kp', kp_var)
        _gain_field(gains_frame, 'Ki', ki_var)
        _gain_field(gains_frame, 'Kd', kd_var)
        tk.Button(gains_frame, text='Apply',
                  command=lambda: self._apply_gains(kp_var, ki_var, kd_var),
                  bg=_GREEN, fg='#1e1e2e', activebackground='#74c69d',
                  font=('Courier', 11, 'bold'), relief='flat', padx=10
                  ).pack(side='left')

        # Fn / Fg / Fg/Fn live readings 
        gauge_outer = tk.Frame(root, bg=_SURFACE)
        gauge_outer.pack(fill='x', padx=6, pady=(2, 0))

        gauge_info = tk.Frame(gauge_outer, bg=_SURFACE)
        gauge_info.pack(fill='x', padx=4, pady=(3, 3))

        _lbl(gauge_info, 'Fn:', bg=_SURFACE).pack(side='left', padx=(2, 2))
        fn_var = tk.StringVar(value='— N')
        tk.Label(gauge_info, textvariable=fn_var, bg=_SURFACE, fg=_FX_COL,
                 font=('Courier', 11, 'bold'), width=8).pack(side='left', padx=(0, 8))

        _lbl(gauge_info, 'Fg:', bg=_SURFACE).pack(side='left', padx=(2, 2))
        fg_var = tk.StringVar(value='— N')
        tk.Label(gauge_info, textvariable=fg_var, bg=_SURFACE, fg=_FY_COL,
                 font=('Courier', 11, 'bold'), width=8).pack(side='left', padx=(0, 8))

        _lbl(gauge_info, 'Fg/Fn:', bg=_SURFACE).pack(side='left', padx=(2, 2))
        ratio_var = tk.StringVar(value='—')
        tk.Label(gauge_info, textvariable=ratio_var, bg=_SURFACE, fg=_YELLOW,
                 font=('Courier', 11, 'bold'), width=7).pack(side='left', padx=(0, 12))

        _lbl(gauge_info, f'Amp:',
             bg=_SURFACE).pack(side='left', padx=(2, 2))
        amp_var = tk.StringVar(value='—')
        tk.Label(gauge_info, textvariable=amp_var, bg=_SURFACE, fg=_GREEN,
                 font=('Courier', 11, 'bold'), width=7).pack(side='left', padx=(0, 12))

        # Control bar
        ctrl_frame = tk.Frame(root, bg=_BG)
        ctrl_frame.pack(fill='x', padx=6, pady=(2, 2))

        _lbl(ctrl_frame, 'CSV:', fg=_TEXT, bg=_BG).pack(side='left', padx=(0, 4))
        self._csv_var = tk.StringVar(value='force_pid_data.csv')
        tk.Entry(ctrl_frame, textvariable=self._csv_var, width=18,
                 bg=_SURFACE, fg=_TEXT, insertbackground=_TEXT,
                 font=('Courier', 11), relief='flat').pack(side='left', padx=(0, 6))

        self._rec_btn_var = tk.StringVar(value='Record')
        self._rec_btn = tk.Button(
            ctrl_frame, textvariable=self._rec_btn_var,
            command=self._toggle_record,
            bg='#e63946', fg='white', activebackground='#c1121f',
            font=('Courier', 11, 'bold'), relief='flat', padx=8)
        self._rec_btn.pack(side='left', padx=(0, 4))

        tk.Button(ctrl_frame, text='Save CSV',
                  command=lambda: self._save_csv(root),
                  bg='#4895ef', fg='white', activebackground='#023e8a',
                  font=('Courier', 11, 'bold'), relief='flat', padx=8
                  ).pack(side='left', padx=(0, 4))

        tk.Button(ctrl_frame, text='Remove offset', command=self._zero_offset,
                  bg=_PINK, fg='#1e1e2e', activebackground='#c77daa',
                  font=('Courier', 11, 'bold'), relief='flat', padx=8
                  ).pack(side='left', padx=(0, 12))

        self._rate_var = tk.StringVar(value='Rate: — Hz')
        tk.Label(ctrl_frame, textvariable=self._rate_var,
                 bg=_BG, fg=_GREEN, font=('Courier', 11)).pack(side='right', padx=8)
        self._sample_var = tk.StringVar(value='Samples: 0')
        tk.Label(ctrl_frame, textvariable=self._sample_var,
                 bg=_BG, fg=_SUBTEXT, font=('Courier', 11)).pack(side='right', padx=4)

        # Animation
        def animate(_):
            with self._lock:
                t_data  = list(self._t_buf)
                fn_data = list(self._fn_buf)
                fg_data = list(self._fg_buf)
                target  = self._pid_target
                rate    = self._rate_hz
                sc      = self._sample_count
                fn_gauge = self._slip_fn
                fg_gauge = self._slip_fg
                ratio    = self._slip_ratio
                amp      = self._ratio_amp

            if not t_data:
                return

            t_now = t_data[-1]
            t_min = max(0.0, t_now - self._window_s)
            t_max = t_now + 0.5

            line_fn.set_data(t_data, fn_data)
            line_fg.set_data(t_data, fg_data)
            line_target.set_data([t_min, t_max], [target, target])

            fn_now = fn_data[-1]

            ax.set_xlim(t_min, t_max)
            all_vals = fn_data + fg_data + [target]
            mn, mx   = min(all_vals), max(all_vals)
            margin   = max(0.5, (mx - mn) * 0.1)
            ax.set_ylim(mn - margin, mx + margin)

            self._pid_vars['Target'].set(f'{target:+.3f}')
            self._pid_vars['Error'].set(f'{target - fn_now:+.3f}')

            self._rate_var.set(f'Rate: {rate:.1f} Hz')
            self._sample_var.set(f'Samples: {sc:,}')

            fn_var.set(f'{fn_gauge:+.3f} N')
            fg_var.set(f'{fg_gauge:+.3f} N')
            ratio_var.set(f'{ratio:.3f}')
            amp_var.set(f'{amp:.3f}')

        self._anim = FuncAnimation(
            fig, animate, interval=self._poll_ms, blit=False, cache_frame_data=False)

        root.protocol('WM_DELETE_WINDOW', root.destroy)
        root.mainloop()

    # Record / Save 

    def _toggle_record(self):
        with self._lock:
            self._is_recording = not self._is_recording
            recording = self._is_recording
            if recording:
                self._recording.clear()
                self._rec_t0 = None

        if recording:
            self._rec_btn_var.set('Stop Rec')
            self._rec_btn.config(bg='#f9e2af', fg='#1e1e2e')
        else:
            self._rec_btn_var.set('Record')
            self._rec_btn.config(bg='#e63946', fg='white')

    def _build_csv_header(self) -> list[str]:
        n = self._n_sensors
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

        basename = self._csv_var.get().strip() or 'force_data.csv'
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
                        f'  {len(rows)} rows saved to:\n{abs_path}',
                        parent=root)
            self.get_logger().info(f'CSV saved: {abs_path}  ({len(rows)} rows)')
        except OSError as e:
            mb.showerror('Save failed', str(e), parent=root)


def main(args=None):
    rclpy.init(args=args)
    node = ForcePlotGuiNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
