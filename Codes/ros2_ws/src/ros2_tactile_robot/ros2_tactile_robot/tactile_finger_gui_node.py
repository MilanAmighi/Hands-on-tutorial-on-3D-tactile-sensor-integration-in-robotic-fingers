"""
Live visualisation of the two-finger tactile gripper.

Press "Remove offsets" to subtract the current sensor readings as a baseline offset.
"""

import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32


# Sensor layout
_N_SENSORS = 8          # total taxels across both fingers
_HALF      = _N_SENSORS // 2   # taxels per finger (4)
_AVERAGING = 5          # number of frames averaged to smooth sensor noise


# Display geometry 
R_CIRCLE = 1.60         # radius of the dashed reference circle drawn around each finger

# Torque ring parameters
_RMEAN           = R_CIRCLE * 0.32  # nominal radius of the torque ring at zero Mz
_COEFWIDTH       = 500.0            # visual ring width before log-scaling
_COEFMULTANGLE   = 45.0             # degrees of arc swept per log-unit of torque
_COEFGROWTHWEDGE = 0.10             # radial growth of the ring per log-unit of torque
_W_MIN           = 0.05             # minimum visual width so a direction arrow always appears

# Dot (taxel) parameters
_DOT_BASE  = R_CIRCLE * 0.13   # dot radius at zero normal force
_DOT_FZ    = R_CIRCLE * 0.024  # additional radius per unit of Fz
_DOT_MIN   = R_CIRCLE * 0.025  # minimum dot radius so dots never disappear completely
_DISP_COEF = R_CIRCLE * 0.20   # shear-to-displacement gain (data-units per N)
_FZ_SCALE  = 8.0               # Fz value that saturates the colour map to full red


# Taxel rest positions in the plot
_G = R_CIRCLE * 0.60
_BASE_LOCAL_LEFT = np.array([
    [-_G, -_G],   # taxel 1
    [ _G, -_G],   # taxel 2
    [-_G,  _G],   # taxel 3
    [ _G,  _G],   # taxel 4
], dtype=float)
_BASE_LOCAL_RIGHT = np.array([
    [ _G,  _G],   # taxel 1
    [-_G,  _G],   # taxel 2
    [ _G, -_G],   # taxel 3
    [-_G, -_G],   # taxel 4
], dtype=float)
_BASE_LOCAL = [_BASE_LOCAL_LEFT, _BASE_LOCAL_RIGHT]

# Physical moment arms used to compute Mz from individual taxel forces
_POS_SENS_LEFT = np.array([
    [ 5,  5, 2],   # taxel 1
    [-5,  5, 2],   # taxel 2
    [ 5, -5, 2],   # taxel 3
    [-5, -5, 2],   # taxel 4
], dtype=float)
_POS_SENS_RIGHT = np.array([
    [ 5,  5, 2],   # taxel 1
    [-5,  5, 2],   # taxel 2
    [ 5, -5, 2],   # taxel 3
    [-5, -5, 2],   # taxel 4
], dtype=float)
_POS_SENS = [_POS_SENS_LEFT, _POS_SENS_RIGHT]


# Colour theme
_BG      = '#00354b'
_SURFACE = '#004060'
_TEXT    = '#ffffff'
_SUBTEXT = '#a8c8d8'
_ACCENT  = '#EEA320'


_s45 = np.sin(np.deg2rad(45))  # sin(45°)


def _compute_mz(fv: np.ndarray, fi: int) -> float:
    """Compute contact torque Mz using physical moment arms at 45-degree taxel layout."""
    return float(
        - fv[0][0] * 0.0035 * _s45 + fv[0][1] * 0.0035 * _s45
        - fv[1][0] * 0.0035 * _s45 - fv[1][1] * 0.0035 * _s45
        + fv[2][0] * 0.0035 * _s45 + fv[2][1] * 0.0035 * _s45
        + fv[3][0] * 0.0035 * _s45 - fv[3][1] * 0.0035 * _s45
    )


class TactileFingerGuiNode(Node):

    def __init__(self):
        super().__init__('tactile_finger_gui_node')

        self._lock    = threading.Lock()
        self._buf     = deque(maxlen=_AVERAGING)   # rolling buffer of raw force frames
        self._offset  = np.zeros((_N_SENSORS, 3), dtype=float)  # tare baseline
        self._gripper = 0

        self.create_subscription(Float32MultiArray, '/esp/force',
                                 self._on_force, 10)
        self.create_subscription(Int32, '/esp/motor_position',
                                 self._on_motor_pos, 10)
        self.get_logger().info('Tactile finger GUI node ready.')

    def _on_force(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=float).reshape(-1, 3)
        if data.shape[0] < _N_SENSORS:
            # Pad with zeros if fewer than 8 sensors are reported
            data = np.vstack([data, np.zeros((_N_SENSORS - data.shape[0], 3))])
        with self._lock:
            self._buf.append(data[:_N_SENSORS])

    def _on_motor_pos(self, msg: Int32):
        with self._lock:
            self._gripper = msg.data

    def _get_display_data(self):
        """Return (averaged_forces, gripper_position), thread-safe."""
        with self._lock:
            if not self._buf:
                return np.zeros((_N_SENSORS, 3)), 0
            mean = np.mean(list(self._buf), axis=0) - self._offset
            return mean, self._gripper

    def tare(self):
        """Capture the current mean reading as a zero offset"""
        with self._lock:
            if self._buf:
                self._offset = np.mean(list(self._buf), axis=0).copy()
        self.get_logger().info('Zero offset applied.')

    def run_gui(self):
        import tkinter as tk
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.animation import FuncAnimation
        from matplotlib.patches import Circle, Wedge

        # orange (low force/torque) and red (high)
        cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
            '', ['#EEA320', '#DB4140'])

        root = tk.Tk()
        root.title('Tactile Finger Monitor')
        root.configure(bg=_BG)

        # Top bar: tare button 
        top = tk.Frame(root, bg=_BG)
        top.pack(fill='x', padx=8, pady=(8, 2))

        tk.Button(top, text='Remove offsets', command=self.tare,
                  bg=_ACCENT, fg='#1e1e2e', font=('Lato', 11, 'bold'),
                  relief='flat', padx=14).pack(side='left', padx=6)

        _PAD    = R_CIRCLE * 1.15   # axis half-extent with a small margin
        _TITLES = ['Left Finger', 'Right Finger']

        fig, axes = plt.subplots(1, 2, figsize=(11, 5))
        fig.patch.set_facecolor(_BG)
        fig.subplots_adjust(left=0.02, right=0.98, top=0.92, bottom=0.05, wspace=0.12)

        for ax, title in zip(axes, _TITLES):
            ax.set_aspect('equal')
            ax.set_facecolor(_SURFACE)
            ax.set_title(title, color=_TEXT, fontsize=13, fontweight='bold', pad=8)
            ax.spines[['right', 'top', 'bottom', 'left']].set_visible(False)
            ax.tick_params(bottom=False, left=False,
                           labelbottom=False, labelleft=False)
            ax.set_xlim(-_PAD, _PAD)
            ax.set_ylim(-_PAD, _PAD)

        canvas = FigureCanvasTkAgg(fig, master=root)
        canvas.get_tk_widget().pack(fill='both', expand=True, padx=6, pady=4)

        # Bottom status bar: numeric Fx, Fy, Fz, Mz for each finger
        bar = tk.Frame(root, bg=_SURFACE, pady=4)
        bar.pack(fill='x', padx=6, pady=(0, 6))

        self._fvars: dict[str, tk.StringVar] = {}
        for side in ('Left', 'Right'):
            tk.Label(bar, text=f'  {side}:', bg=_SURFACE, fg=_TEXT,
                     font=('Courier', 9, 'bold')).pack(side='left', padx=(10, 4))
            for comp in ('Fx', 'Fy', 'Fz', 'Mz'):
                key = f'{side}_{comp}'
                tk.Label(bar, text=f'{comp}:', bg=_SURFACE, fg=_SUBTEXT,
                         font=('Courier', 9)).pack(side='left', padx=(4, 1))
                v = tk.StringVar(value=' 0.00')
                self._fvars[key] = v
                tk.Label(bar, textvariable=v, bg=_SURFACE, fg=_ACCENT,
                         font=('Courier', 9, 'bold'), width=7
                         ).pack(side='left', padx=(0, 8))

        def animate(_):
            forces, gripper = self._get_display_data()

            for fi, ax in enumerate(axes):
                ax.cla()
                ax.set_aspect('equal')
                ax.set_facecolor(_SURFACE)
                ax.set_title(_TITLES[fi], color=_TEXT, fontsize=13,
                             fontweight='bold', pad=8)
                ax.spines[['right', 'top', 'bottom', 'left']].set_visible(False)
                ax.tick_params(bottom=False, left=False,
                               labelbottom=False, labelleft=False)
                ax.set_xlim(-_PAD, _PAD)
                ax.set_ylim(-_PAD, _PAD)

                ax.add_patch(Circle((0, 0), R_CIRCLE, fill=False,
                                    edgecolor='#336680', linewidth=0.8,
                                    linestyle='--'))

                fv = forces[fi * _HALF:(fi + 1) * _HALF]
                mz = _compute_mz(fv, fi)

                # Torque ring + arrowhead
                if mz != 0:
                    width = mz * _COEFWIDTH
                    w_vis = np.sign(width) * max(abs(width), _W_MIN)
                    angle = 90 + np.sign(w_vis) * _COEFMULTANGLE * np.log(abs(w_vis) + 1)
                    rad   = angle * np.pi / 180.0
                    cos_a, sin_a = np.cos(rad), np.sin(rad)
                    w_s = np.sign(w_vis) * np.log(abs(w_vis) + 1)

                    ring_r = _RMEAN + abs(w_s * _COEFGROWTHWEDGE) / 2
                    ring_w = max(0.06, abs(w_s * _COEFGROWTHWEDGE))
                    tc     = cmap(min(1.0, abs(mz) * 0.1))
                    t1, t2 = (angle, 90) if w_vis < 0 else (90, angle)
                    ax.add_artist(Wedge(center=(0, 0), r=ring_r,
                                       theta1=t1, theta2=t2,
                                       width=ring_w, color=tc))
                    arr_hw = max(ring_w * 0.55, R_CIRCLE * 0.065)   # base half-width
                    arr_h  = max(ring_w * 1.8,  R_CIRCLE * 0.14)    # tip protrusion
                    mid_r  = ring_r - ring_w / 2
                    cx_tip, cy_tip = mid_r * cos_a, mid_r * sin_a
                    tang = np.sign(w_vis) * np.array([-sin_a,  cos_a])
                    rad_d = np.array([cos_a, sin_a])
                    A = np.array([cx_tip, cy_tip]) + arr_hw * rad_d
                    B = np.array([cx_tip, cy_tip]) - arr_hw * rad_d
                    C = np.array([cx_tip, cy_tip]) + arr_h  * tang
                    ax.add_artist(plt.Polygon([A, B, C], closed=True,
                                              color=tc, zorder=6))
                #ax.text(0, 0, f'Mz\n{mz:+.2f}',
                #        ha='center', va='center', color=_TEXT,
                #        fontsize=7.5, fontweight='bold')

                # Taxel dots 
                for k in range(_HALF):
                    fx, fy, fz = fv[k]
                    bx, by = _BASE_LOCAL[fi][k]

                    # Right finger 
                    sign_fy = 1.0 if fi == 0 else -1.0
                    sign_fx = -1.0 if fi == 0 else 1.0
                    dx = bx + sign_fx * fx * _DISP_COEF
                    dy = by - sign_fy * fy * _DISP_COEF

                    r = max(_DOT_MIN, _DOT_BASE + _DOT_FZ * fz)
                    c = cmap(min(1.0, max(0.0, -fz / _FZ_SCALE)))
                    ax.add_patch(Circle((dx, dy), r, color=c, zorder=5))
                    ax.text(bx * 1.05, by * 1.05, str(k),
                            ha='center', va='center',
                            color='#80a0b0', fontsize=6)

                # Update the numeric status bar at the bottom of the window
                side = 'Left' if fi == 0 else 'Right'
                ftot = np.sum(fv, axis=0)
                self._fvars[f'{side}_Fx'].set(f'{ftot[0]:+.2f}')
                self._fvars[f'{side}_Fy'].set(f'{ftot[1]:+.2f}')
                self._fvars[f'{side}_Fz'].set(f'{ftot[2]:+.2f}')
                self._fvars[f'{side}_Mz'].set(f'{mz:+.4f}')

            canvas.draw()
        self._anim = FuncAnimation(
            fig, animate, interval=50, blit=False, cache_frame_data=False)

        root.protocol('WM_DELETE_WINDOW', root.destroy)
        root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TactileFingerGuiNode()
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
