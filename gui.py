"""
ESP32 Stepper Controller GUI
Two tabs:
  - Controller: Connect to ESP32, motion control, PID, parameters
  - Plot:       Load session CSV data, pick X/Y columns, plot (with live mode)

Usage: python gui.py
"""
import glob
import json
import math
import os
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np
import pandas as pd

from autotune import PIDAutotuner
from controller import StepperController
from log_serial import SessionLogger
from waveforms import (generate_sine, generate_triangle, generate_rounded_triangle,
                       get_period, compute_triangle_params,
                       compute_rounded_triangle_params)

LOG_DIR = "logs"
SETTINGS_FILE = "settings.json"

DEFAULT_SETTINGS = {
    "port": "",
    "velocity_mm_s": 4.0,
    "amplitude_mm": 0.5,
    "frequency_hz": 0.1,
    "duty_cycle": 0.9,
    "function": "Sine",
    "plot_x": "time_s",
    "plot_y": "actual_mm",
    "window_geometry": "1100x850",
    "pid_kp": 1.0,
    "pid_ki": 0.0,
    "pid_kd": 0.0,
    "live_plot_window_s": 30,
}


def load_settings():
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE) as f:
                saved = json.load(f)
            merged = dict(DEFAULT_SETTINGS)
            merged.update(saved)
            return merged
        except (json.JSONDecodeError, IOError):
            pass
    return dict(DEFAULT_SETTINGS)


def save_settings(settings):
    with open(SETTINGS_FILE, 'w') as f:
        json.dump(settings, f, indent=2)


class PlotTab(ttk.Frame):
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.df = None
        self._live = False
        self._derivative_mode = False
        self._build_controls()
        self._build_plot()

    def _build_controls(self):
        ctrl = ttk.Frame(self)
        ctrl.pack(side=tk.TOP, fill=tk.X, padx=8, pady=4)

        ttk.Label(ctrl, text="Session:").grid(row=0, column=0, sticky=tk.W, padx=(0, 4))
        self.session_var = tk.StringVar()
        self.session_combo = ttk.Combobox(ctrl, textvariable=self.session_var, width=30)
        self.session_combo.grid(row=0, column=1, sticky=tk.W, padx=(0, 4))
        self.session_combo.bind("<<ComboboxSelected>>", self._on_session_selected)

        ttk.Button(ctrl, text="Refresh", command=self._refresh_sessions).grid(
            row=0, column=2, padx=(0, 8)
        )

        ttk.Label(ctrl, text="X axis:").grid(row=0, column=3, sticky=tk.W, padx=(8, 4))
        self.x_var = tk.StringVar(value=self.app.settings.get("plot_x", "time_s"))
        self.x_combo = ttk.Combobox(ctrl, textvariable=self.x_var, width=14, state="readonly")
        self.x_combo.grid(row=0, column=4, padx=(0, 8))

        ttk.Label(ctrl, text="Y axis:").grid(row=0, column=5, sticky=tk.W, padx=(0, 4))
        self.y_var = tk.StringVar(value=self.app.settings.get("plot_y", "actual_mm"))
        self.y_combo = ttk.Combobox(ctrl, textvariable=self.y_var, width=14, state="readonly")
        self.y_combo.grid(row=0, column=6, padx=(0, 8))

        ttk.Button(ctrl, text="Plot", command=self._plot).grid(row=0, column=7, padx=(0, 4))
        ttk.Button(ctrl, text="dy/dx", command=self._plot_derivative).grid(row=0, column=8, padx=(0, 4))

        self.live_btn = ttk.Button(ctrl, text="Live: OFF", command=self._toggle_live)
        self.live_btn.grid(row=0, column=9, padx=(4, 4))

        ttk.Label(ctrl, text="Window (s):").grid(row=0, column=10, sticky=tk.W, padx=(8, 4))
        self.window_var = tk.StringVar(
            value=str(self.app.settings.get("live_plot_window_s", 30))
        )
        ttk.Entry(ctrl, textvariable=self.window_var, width=6).grid(row=0, column=11, padx=(0, 4))

        self.info_var = tk.StringVar(value="")
        ttk.Label(ctrl, textvariable=self.info_var, foreground="gray").grid(
            row=1, column=0, columnspan=12, sticky=tk.W, pady=(2, 0)
        )

        self._refresh_sessions()

    def _build_plot(self):
        self.fig = Figure(figsize=(9, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("actual_mm")
        self.ax.grid(True, alpha=0.3)
        self.fig.tight_layout()

        canvas_frame = ttk.Frame(self)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.toolbar = NavigationToolbar2Tk(self.canvas, canvas_frame)
        self.toolbar.update()

    def _refresh_sessions(self):
        if not os.path.isdir(LOG_DIR):
            self.session_combo["values"] = []
            return
        meta_files = sorted(
            glob.glob(os.path.join(LOG_DIR, "*", "*_meta.json")), reverse=True
        )
        sessions = []
        for mf in meta_files:
            name = os.path.basename(mf).replace("_meta.json", "")
            sessions.append(name)
        self.session_combo["values"] = sessions
        if sessions and not self.session_var.get():
            self.session_var.set(sessions[0])
            self._on_session_selected(None)

    def _on_session_selected(self, _event):
        session = self.session_var.get()
        if not session:
            return
        self.df = self._load_session(session)
        if self.df is None:
            return
        cols = list(self.df.columns)
        self.x_combo["values"] = cols
        self.y_combo["values"] = cols
        if "time_s" in cols and self.x_var.get() not in cols:
            self.x_var.set("time_s")
        if "actual_mm" in cols and self.y_var.get() not in cols:
            self.y_var.set("actual_mm")

        meta_path = os.path.join(LOG_DIR, session, f"{session}_meta.json")
        if os.path.exists(meta_path):
            with open(meta_path) as f:
                meta = json.load(f)
            self.info_var.set(
                f"Started: {meta.get('iso_time', '?')}  |  "
                f"Unix: {meta.get('unix_timestamp', '?')}  |  "
                f"{len(self.df)} samples"
            )
        else:
            self.info_var.set(f"{len(self.df)} samples (no metadata file)")

    def _load_session(self, session_id):
        session_dir = os.path.join(LOG_DIR, session_id)
        pattern = os.path.join(session_dir, f"{session_id}_[0-9][0-9][0-9][0-9].csv")
        files = sorted(glob.glob(pattern))
        if not files:
            return None
        frames = []
        for f in files:
            try:
                df = pd.read_csv(f)
                if not df.empty:
                    frames.append(df)
            except (pd.errors.EmptyDataError, pd.errors.ParserError):
                continue
        if not frames:
            return None
        result = pd.concat(frames, ignore_index=True)
        # Convert time_ms to time_s for plotting
        if "time_ms" in result.columns:
            result["time_s"] = result["time_ms"] / 1000.0
            result.drop(columns=["time_ms"], inplace=True)
            # Reorder so time_s is first column
            cols = ["time_s"] + [c for c in result.columns if c != "time_s"]
            result = result[cols]
        return result

    def _apply_time_window(self, df):
        """Filter dataframe to the most recent N seconds based on the Window field."""
        try:
            window_s = float(self.window_var.get())
        except ValueError:
            return df
        if "time_s" in df.columns and window_s > 0:
            max_time = df["time_s"].max()
            df = df[df["time_s"] >= max_time - window_s]
        return df

    def _plot(self):
        if self.df is None or self.df.empty:
            messagebox.showinfo("No data", "Load a session first.")
            return
        x_col = self.x_var.get()
        y_col = self.y_var.get()
        if x_col not in self.df.columns or y_col not in self.df.columns:
            messagebox.showwarning("Bad column", "Column not found in data.")
            return
        self._derivative_mode = False
        windowed = self._apply_time_window(self.df)
        self._draw_data(windowed, x_col, y_col)

    def _plot_derivative(self):
        if self.df is None or self.df.empty:
            messagebox.showinfo("No data", "Load a session first.")
            return
        x_col = self.x_var.get()
        y_col = self.y_var.get()
        if x_col not in self.df.columns or y_col not in self.df.columns:
            messagebox.showwarning("Bad column", "Column not found in data.")
            return
        self._derivative_mode = True
        windowed = self._apply_time_window(self.df)
        self._draw_derivative(windowed, x_col, y_col)

    @staticmethod
    def _col_label(col):
        """Return a human-readable axis label for a column name."""
        labels = {
            "time_s": "Time (s)",
            "target_mm": "Target (mm)",
            "actual_mm": "Actual (mm)",
            "step_pos": "Step position",
            "pid_correction": "PID correction (mm)",
        }
        return labels.get(col, col)

    def _draw_derivative(self, df, x_col, y_col):
        """Compute and draw dy/dx, dropping the last point to avoid edge artifacts."""
        x_data = df[x_col].values
        y_data = df[y_col].values
        dy = np.gradient(y_data, x_data)
        # Drop last point to avoid discontinuity at the boundary
        x_data = x_data[:-1]
        dy = dy[:-1]

        # Build label with units (e.g. "Velocity (mm/s)" when differentiating mm by s)
        if x_col == "time_s" and y_col in ("target_mm", "actual_mm"):
            deriv_label = "velocity_mm_s"
            ylabel = "Velocity (mm/s)"
        else:
            deriv_label = f"d({y_col})/d({x_col})"
            ylabel = deriv_label
        deriv_df = pd.DataFrame({x_col: x_data, deriv_label: dy})
        self._draw_data(deriv_df, x_col, deriv_label, ylabel=ylabel)

    def _draw_data(self, df, x_col, y_col, ylabel=None):
        self.ax.clear()

        x_data = df[x_col].values
        y_data = df[y_col].values

        # Downsample for performance if more than 2000 points
        if len(x_data) > 2000:
            step = len(x_data) // 2000
            x_data = x_data[::step]
            y_data = y_data[::step]

        self.ax.plot(x_data, y_data, linewidth=0.5)
        self.ax.set_xlabel(self._col_label(x_col))
        self.ax.set_ylabel(ylabel or self._col_label(y_col))
        self.ax.grid(True, alpha=0.3)
        self.fig.tight_layout()
        self.canvas.draw()

    def _toggle_live(self):
        self._live = not self._live
        self.live_btn.config(text=f"Live: {'ON' if self._live else 'OFF'}")
        if self._live:
            self._live_update()

    def start_live(self, session_id):
        """Called externally to activate live plotting for a session."""
        self._refresh_sessions()
        self.session_var.set(session_id)
        if not self._live:
            self._live = True
            self.live_btn.config(text="Live: ON")
            self._live_update()

    def _live_update(self):
        if not self._live:
            return
        ctrl_tab = self.app.ctrl_tab
        if ctrl_tab.logger.is_active and ctrl_tab.logger.session_id:
            session_id = ctrl_tab.logger.session_id
            df = self._load_session(session_id)
            if df is not None and not df.empty:
                x_col = self.x_var.get()
                y_col = self.y_var.get()
                if x_col in df.columns and y_col in df.columns:
                    df = self._apply_time_window(df)
                    try:
                        window_s = float(self.window_var.get())
                    except ValueError:
                        window_s = 30
                    if self._derivative_mode:
                        self._draw_derivative(df, x_col, y_col)
                        self.info_var.set(
                            f"Live dy/dx: {len(df)} points (last {window_s}s)"
                        )
                    else:
                        self._draw_data(df, x_col, y_col)
                        self.info_var.set(f"Live: {len(df)} points (last {window_s}s)")
        self.after(1500, self._live_update)


class ControllerTab(ttk.Frame):
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.ctrl = None
        self.logger = SessionLogger(base_dir=LOG_DIR)
        self._is_running = False
        self._motor_enabled = False
        self._pid_active = False
        # Track "current" (confirmed) values from ESP32
        self._cur_vel = self.app.settings.get("velocity_mm_s", 4.0)
        self._cur_accel = 0.0  # auto-computed, not user-set
        self._cur_amp = self.app.settings.get("amplitude_mm", 0.5)
        self._cur_freq = self.app.settings.get("frequency_hz", 0.1)
        self._cur_duty = self.app.settings.get("duty_cycle", 0.9)
        self._cur_kp = self.app.settings.get("pid_kp", 1.0)
        self._cur_ki = self.app.settings.get("pid_ki", 0.0)
        self._cur_kd = self.app.settings.get("pid_kd", 0.0)
        self._cur_pos = 0.0  # encoder position in mm
        self._waveform_view = "position"  # "position", "velocity", "acceleration"
        self._autotuner = None
        self._build()

    def _build(self):
        # Scrollable frame
        outer = ttk.Frame(self)
        outer.pack(fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(outer, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        self._scroll_frame = ttk.Frame(canvas)

        self._scroll_frame.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=self._scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Mousewheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        main_frame = self._scroll_frame

        # --- Connection frame ---
        conn_frame = ttk.LabelFrame(main_frame, text="Connection")
        conn_frame.pack(fill=tk.X, padx=12, pady=(8, 4))

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=(8, 4), pady=6)
        self.port_var = tk.StringVar(value=self.app.settings.get("port", ""))
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=20)
        self.port_combo.grid(row=0, column=1, padx=(0, 4), pady=6)
        ttk.Button(conn_frame, text="Scan", command=self._scan_ports).grid(
            row=0, column=2, padx=(0, 4), pady=6
        )
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._on_connect)
        self.connect_btn.grid(row=0, column=3, padx=(0, 8), pady=6)

        self.conn_status_var = tk.StringVar(value="Disconnected")
        ttk.Label(conn_frame, textvariable=self.conn_status_var, foreground="gray").grid(
            row=0, column=4, padx=(8, 8), pady=6
        )

        self._scan_ports()

        # --- Motor Enable frame ---
        enable_frame = ttk.LabelFrame(main_frame, text="Motor Power")
        enable_frame.pack(fill=tk.X, padx=12, pady=4)

        enable_btn_frame = ttk.Frame(enable_frame)
        enable_btn_frame.pack(pady=6)

        self.enable_btn = ttk.Button(
            enable_btn_frame, text="Enable Motor", command=self._on_enable, state=tk.DISABLED
        )
        self.enable_btn.pack(side=tk.LEFT, padx=8)
        self.disable_btn = ttk.Button(
            enable_btn_frame, text="Disable Motor", command=self._on_disable, state=tk.DISABLED
        )
        self.disable_btn.pack(side=tk.LEFT, padx=8)

        # --- Motion Parameters frame ---
        params_frame = ttk.LabelFrame(main_frame, text="Motion Parameters")
        params_frame.pack(fill=tk.X, padx=12, pady=4)

        ttk.Label(params_frame, text="Velocity (mm/s):").grid(row=0, column=0, padx=(8, 4), pady=4, sticky=tk.W)
        self.vel_var = tk.StringVar(value=str(self.app.settings.get("velocity_mm_s", 4.0)))
        ttk.Entry(params_frame, textvariable=self.vel_var, width=10).grid(row=0, column=1, padx=(0, 12), pady=4)

        ttk.Label(params_frame, text="Acceleration (mm/s\u00b2):").grid(row=0, column=2, padx=(8, 4), pady=4, sticky=tk.W)
        self.accel_var = tk.StringVar(value=str(self.app.settings.get("acceleration_mm_s2", 10.0)))
        ttk.Entry(params_frame, textvariable=self.accel_var, width=10).grid(row=0, column=3, padx=(0, 12), pady=4)

        self.apply_params_btn = ttk.Button(
            params_frame, text="Apply", command=self._on_apply_params, state=tk.DISABLED
        )
        self.apply_params_btn.grid(row=0, column=4, padx=(4, 8), pady=4)

        self.params_current_var = tk.StringVar(value="")
        ttk.Label(params_frame, textvariable=self.params_current_var, foreground="gray").grid(
            row=1, column=0, columnspan=5, sticky=tk.W, padx=(8, 8), pady=(0, 4)
        )
        self._update_params_current_label()

        # --- Waveform frame ---
        wave_frame = ttk.LabelFrame(main_frame, text="Waveform")
        wave_frame.pack(fill=tk.X, padx=12, pady=4)

        ttk.Label(wave_frame, text="Function:").grid(row=0, column=0, padx=(8, 4), pady=4, sticky=tk.W)
        self.func_var = tk.StringVar(value=self.app.settings.get("function", "Sine"))
        self.func_combo = ttk.Combobox(wave_frame, textvariable=self.func_var, width=16, state="readonly")
        self.func_combo["values"] = ["Sine", "Triangle", "Rounded Triangle"]
        self.func_combo.grid(row=0, column=1, padx=(0, 12), pady=4)
        self.func_combo.bind("<<ComboboxSelected>>", self._on_func_changed)

        ttk.Label(wave_frame, text="Amplitude (mm):").grid(row=0, column=2, padx=(8, 4), pady=4, sticky=tk.W)
        self.amp_var = tk.StringVar(value=str(self.app.settings.get("amplitude_mm", 0.5)))
        ttk.Entry(wave_frame, textvariable=self.amp_var, width=10).grid(
            row=0, column=3, padx=(0, 12), pady=4
        )

        # Frequency (only for Sine)
        self.freq_label = ttk.Label(wave_frame, text="Frequency (Hz):")
        self.freq_label.grid(row=1, column=0, padx=(8, 4), pady=4, sticky=tk.W)
        self.freq_var = tk.StringVar(value=str(self.app.settings.get("frequency_hz", 0.1)))
        self.freq_entry = ttk.Entry(wave_frame, textvariable=self.freq_var, width=10)
        self.freq_entry.grid(row=1, column=1, padx=(0, 12), pady=4)

        # Velocity (for Triangle and Rounded Triangle)
        self.wave_vel_label = ttk.Label(wave_frame, text="Velocity (mm/s):")
        self.wave_vel_label.grid(row=1, column=0, padx=(8, 4), pady=4, sticky=tk.W)
        self.wave_vel_var = tk.StringVar(value=str(self.app.settings.get("velocity_mm_s", 4.0)))
        self.wave_vel_entry = ttk.Entry(wave_frame, textvariable=self.wave_vel_var, width=10)
        self.wave_vel_entry.grid(row=1, column=1, padx=(0, 12), pady=4)

        # Duty cycle (for Triangle and Rounded Triangle)
        self.duty_label = ttk.Label(wave_frame, text="Duty cycle:")
        self.duty_label.grid(row=1, column=2, padx=(8, 4), pady=4, sticky=tk.W)
        self.duty_var = tk.StringVar(value=str(self.app.settings.get("duty_cycle", 0.9)))
        self.duty_entry = ttk.Entry(wave_frame, textvariable=self.duty_var, width=10)
        self.duty_entry.grid(row=1, column=3, padx=(0, 12), pady=4)

        self.apply_wave_btn = ttk.Button(
            wave_frame, text="Apply", command=self._on_apply_waveform, state=tk.DISABLED
        )
        self.apply_wave_btn.grid(row=0, column=4, padx=(4, 8), pady=4)

        self.wave_current_var = tk.StringVar(value="")
        ttk.Label(wave_frame, textvariable=self.wave_current_var, foreground="gray").grid(
            row=2, column=0, columnspan=5, sticky=tk.W, padx=(8, 8), pady=(0, 2)
        )

        self.computed_var = tk.StringVar(value="")
        ttk.Label(wave_frame, textvariable=self.computed_var, foreground="gray").grid(
            row=3, column=0, columnspan=5, sticky=tk.W, padx=(8, 8), pady=(0, 4)
        )

        # Waveform preview plot
        preview_frame = ttk.Frame(wave_frame)
        preview_frame.grid(row=4, column=0, columnspan=5, sticky=tk.EW, padx=8, pady=(0, 4))

        preview_btn_frame = ttk.Frame(preview_frame)
        preview_btn_frame.pack(side=tk.TOP, fill=tk.X)
        self.preview_pos_btn = ttk.Button(preview_btn_frame, text="Position",
                                          command=lambda: self._set_preview_view("position"))
        self.preview_pos_btn.pack(side=tk.LEFT, padx=2)
        self.preview_vel_btn = ttk.Button(preview_btn_frame, text="Velocity",
                                          command=lambda: self._set_preview_view("velocity"))
        self.preview_vel_btn.pack(side=tk.LEFT, padx=2)
        self.preview_acc_btn = ttk.Button(preview_btn_frame, text="Acceleration",
                                          command=lambda: self._set_preview_view("acceleration"))
        self.preview_acc_btn.pack(side=tk.LEFT, padx=2)

        self.preview_fig = Figure(figsize=(7, 2), dpi=100)
        self.preview_ax = self.preview_fig.add_subplot(111)
        self.preview_ax.grid(True, alpha=0.3)
        self.preview_fig.tight_layout()
        self.preview_canvas = FigureCanvasTkAgg(self.preview_fig, master=preview_frame)
        self.preview_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.X)

        self._on_func_changed()
        self._update_wave_current_label()
        self._update_computed_params()
        self._update_preview()

        # --- Motion Control frame ---
        ctrl_frame = ttk.LabelFrame(main_frame, text="Motion Control")
        ctrl_frame.pack(fill=tk.X, padx=12, pady=4)

        motion_row = ttk.Frame(ctrl_frame)
        motion_row.pack(pady=(8, 4))

        self.start_btn = ttk.Button(
            motion_row, text="Start", command=self._on_start, state=tk.DISABLED
        )
        self.start_btn.pack(side=tk.LEFT, padx=8)
        self.stop_btn = ttk.Button(
            motion_row, text="Stop", command=self._on_stop, state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=8)

        ttk.Separator(motion_row, orient=tk.VERTICAL).pack(side=tk.LEFT, padx=12, fill=tk.Y)

        self.home_neg_btn = ttk.Button(
            motion_row, text="Home (-)", command=self._on_home_neg, state=tk.DISABLED
        )
        self.home_neg_btn.pack(side=tk.LEFT, padx=8)
        self.home_pos_btn = ttk.Button(
            motion_row, text="Home (+)", command=self._on_home_pos, state=tk.DISABLED
        )
        self.home_pos_btn.pack(side=tk.LEFT, padx=8)

        # Jog row
        jog_row = ttk.Frame(ctrl_frame)
        jog_row.pack(pady=(4, 4))

        self.jog_neg_btn = ttk.Button(
            jog_row, text="\u25c4 -10mm", command=lambda: self._on_jog(-10), state=tk.DISABLED
        )
        self.jog_neg_btn.pack(side=tk.LEFT, padx=8)
        self.jog_pos_btn = ttk.Button(
            jog_row, text="\u25ba +10mm", command=lambda: self._on_jog(10), state=tk.DISABLED
        )
        self.jog_pos_btn.pack(side=tk.LEFT, padx=8)

        ttk.Separator(jog_row, orient=tk.VERTICAL).pack(side=tk.LEFT, padx=12, fill=tk.Y)

        self.move_var = tk.StringVar(value="")
        ttk.Entry(jog_row, textvariable=self.move_var, width=8).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Label(jog_row, text="mm").pack(side=tk.LEFT, padx=(0, 4))
        self.move_btn = ttk.Button(
            jog_row, text="Move", command=self._on_manual_move, state=tk.DISABLED
        )
        self.move_btn.pack(side=tk.LEFT, padx=8)

        # Position display
        self.position_var = tk.StringVar(value="Position: -- mm (encoder)")
        ttk.Label(ctrl_frame, textvariable=self.position_var, foreground="gray").pack(
            padx=8, pady=(0, 8), anchor=tk.W
        )

        # --- PID Control frame ---
        pid_frame = ttk.LabelFrame(main_frame, text="PID Control")
        pid_frame.pack(fill=tk.X, padx=12, pady=4)

        pid_row1 = ttk.Frame(pid_frame)
        pid_row1.pack(pady=(6, 2))

        ttk.Label(pid_row1, text="Kp:").pack(side=tk.LEFT, padx=(8, 2))
        self.kp_var = tk.StringVar(value=str(self.app.settings.get("pid_kp", 1.0)))
        ttk.Entry(pid_row1, textvariable=self.kp_var, width=8).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(pid_row1, text="Ki:").pack(side=tk.LEFT, padx=(0, 2))
        self.ki_var = tk.StringVar(value=str(self.app.settings.get("pid_ki", 0.0)))
        ttk.Entry(pid_row1, textvariable=self.ki_var, width=8).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(pid_row1, text="Kd:").pack(side=tk.LEFT, padx=(0, 2))
        self.kd_var = tk.StringVar(value=str(self.app.settings.get("pid_kd", 0.0)))
        ttk.Entry(pid_row1, textvariable=self.kd_var, width=8).pack(side=tk.LEFT, padx=(0, 8))

        self.send_pid_btn = ttk.Button(
            pid_row1, text="Send PID", command=self._on_send_pid, state=tk.DISABLED
        )
        self.send_pid_btn.pack(side=tk.LEFT, padx=8)

        pid_row2 = ttk.Frame(pid_frame)
        pid_row2.pack(pady=(2, 2))

        self.pid_toggle_btn = ttk.Button(
            pid_row2, text="Activate PID", command=self._on_toggle_pid, state=tk.DISABLED
        )
        self.pid_toggle_btn.pack(side=tk.LEFT, padx=8)

        self.autotune_btn = ttk.Button(
            pid_row2, text="Autotune", command=self._on_autotune, state=tk.DISABLED
        )
        self.autotune_btn.pack(side=tk.LEFT, padx=8)

        self.pid_status_var = tk.StringVar(value="PID: Off")
        ttk.Label(pid_row2, textvariable=self.pid_status_var, foreground="gray").pack(
            side=tk.LEFT, padx=(12, 8)
        )

        self.pid_current_var = tk.StringVar(value="")
        ttk.Label(pid_frame, textvariable=self.pid_current_var, foreground="gray").pack(
            padx=8, pady=(0, 6), anchor=tk.W
        )
        self._update_pid_current_label()

        # --- Status frame ---
        status_frame = ttk.LabelFrame(main_frame, text="Status")
        status_frame.pack(fill=tk.X, padx=12, pady=(4, 8))

        self.motor_status_var = tk.StringVar(value="Disconnected")
        ttk.Label(status_frame, text="Motor:").grid(row=0, column=0, padx=(8, 4), pady=4)
        ttk.Label(status_frame, textvariable=self.motor_status_var).grid(
            row=0, column=1, sticky=tk.W, pady=4
        )

        self.log_status_var = tk.StringVar(value="Not logging")
        ttk.Label(status_frame, text="Logging:").grid(row=1, column=0, padx=(8, 4), pady=4)
        ttk.Label(status_frame, textvariable=self.log_status_var).grid(
            row=1, column=1, sticky=tk.W, pady=4
        )

        self.sample_count_var = tk.StringVar(value="0 samples")
        ttk.Label(status_frame, text="Samples:").grid(row=2, column=0, padx=(8, 4), pady=4)
        ttk.Label(status_frame, textvariable=self.sample_count_var).grid(
            row=2, column=1, sticky=tk.W, pady=4
        )

    # --- "Current value" label updaters ---

    def _update_params_current_label(self):
        self.params_current_var.set(
            f"Current: vel = {self._cur_vel:.2f} mm/s  |  "
            f"accel = {self._cur_accel:.2f} mm/s\u00b2"
        )

    def _update_wave_current_label(self):
        func = self.func_var.get()
        parts = [f"Current: {func}"]
        parts.append(f"amp = {self._cur_amp:.3f} mm")
        if func == "Sine":
            parts.append(f"freq = {self._cur_freq:.4f} Hz")
        if func in ("Triangle", "Rounded Triangle"):
            parts.append(f"vel = {self._cur_vel:.3f} mm/s")
            parts.append(f"duty = {self._cur_duty:.2f}")
        self.wave_current_var.set("  |  ".join(parts))

    def _update_pid_current_label(self):
        self.pid_current_var.set(
            f"Current: Kp = {self._cur_kp:.4f}  |  "
            f"Ki = {self._cur_ki:.4f}  |  "
            f"Kd = {self._cur_kd:.4f}"
        )

    def _update_computed_params(self, _event=None):
        func = self.func_var.get()
        try:
            amp = float(self.amp_var.get())
        except ValueError:
            self.computed_var.set("")
            return

        if func == "Sine":
            try:
                f = float(self.freq_var.get())
                v_max = 2 * math.pi * f * amp
                a_max = (2 * math.pi * f) ** 2 * amp
                self.computed_var.set(
                    f"Computed: v_max = {v_max:.3f} mm/s  |  a_max = {a_max:.3f} mm/s\u00b2"
                )
            except ValueError:
                self.computed_var.set("")
        elif func == "Triangle":
            try:
                v = float(self.wave_vel_var.get())
                F = float(self.duty_var.get())
                _, a_cap, period = compute_triangle_params(amp, v, F)
                freq = 1.0 / period if period > 0 else 0
                self.computed_var.set(
                    f"Computed: period = {period:.4f} s  |  freq = {freq:.4f} Hz  |  "
                    f"accel = {a_cap:.2f} mm/s\u00b2"
                )
            except (ValueError, ZeroDivisionError):
                self.computed_var.set("")
        elif func == "Rounded Triangle":
            try:
                v = float(self.wave_vel_var.get())
                F = float(self.duty_var.get())
                _, a_peak, jerk, period = compute_rounded_triangle_params(amp, v, F)
                freq = 1.0 / period if period > 0 else 0
                self.computed_var.set(
                    f"Computed: period = {period:.4f} s  |  freq = {freq:.4f} Hz  |  "
                    f"a_peak = {a_peak:.2f} mm/s\u00b2  |  jerk = {jerk:.1f} mm/s\u00b3"
                )
            except (ValueError, ZeroDivisionError):
                self.computed_var.set("")

    def _on_func_changed(self, _event=None):
        func = self.func_var.get()
        is_sine = func == "Sine"
        is_triangle = func in ("Triangle", "Rounded Triangle")

        # Show/hide frequency field (Sine only)
        if is_sine:
            self.freq_label.grid()
            self.freq_entry.grid()
        else:
            self.freq_label.grid_remove()
            self.freq_entry.grid_remove()

        # Show/hide velocity and duty cycle (Triangle/Rounded Triangle only)
        if is_triangle:
            self.wave_vel_label.grid()
            self.wave_vel_entry.grid()
            self.duty_label.grid()
            self.duty_entry.grid()
        else:
            self.wave_vel_label.grid_remove()
            self.wave_vel_entry.grid_remove()
            self.duty_label.grid_remove()
            self.duty_entry.grid_remove()

        self._update_computed_params()
        self._update_preview()

    def _set_preview_view(self, view):
        self._waveform_view = view
        self._update_preview()

    def _update_preview(self):
        func = self.func_var.get()
        try:
            amp = float(self.amp_var.get())
        except ValueError:
            return

        try:
            if func == "Sine":
                freq = float(self.freq_var.get())
                t, x, v, a = generate_sine(amp, freq, n_periods=3)
            elif func == "Triangle":
                vel = float(self.wave_vel_var.get())
                F = float(self.duty_var.get())
                t, x, v, a = generate_triangle(amp, vel, F, n_periods=3)
            elif func == "Rounded Triangle":
                vel = float(self.wave_vel_var.get())
                F = float(self.duty_var.get())
                t, x, v, a = generate_rounded_triangle(amp, vel, F, n_periods=3)
            else:
                return
        except (ValueError, ZeroDivisionError):
            return

        self.preview_ax.clear()
        if self._waveform_view == "position":
            self.preview_ax.plot(t, x, linewidth=1)
            self.preview_ax.set_ylabel("Position (mm)")
        elif self._waveform_view == "velocity":
            self.preview_ax.plot(t, v, linewidth=1, color="tab:orange")
            self.preview_ax.set_ylabel("Velocity (mm/s)")
        elif self._waveform_view == "acceleration":
            self.preview_ax.plot(t, a, linewidth=1, color="tab:red")
            self.preview_ax.set_ylabel("Accel (mm/s\u00b2)")
        self.preview_ax.set_xlabel("Time (s)")
        self.preview_ax.grid(True, alpha=0.3)
        self.preview_fig.tight_layout()
        self.preview_canvas.draw()

    def _scan_ports(self):
        import serial.tools.list_ports
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        saved_port = self.app.settings.get("port", "")
        if saved_port in ports:
            self.port_var.set(saved_port)
        elif ports:
            self.port_var.set(ports[0])

    def _set_buttons_disconnected(self):
        for btn in [self.start_btn, self.stop_btn, self.home_neg_btn, self.home_pos_btn,
                     self.jog_neg_btn, self.jog_pos_btn, self.move_btn,
                     self.enable_btn, self.disable_btn,
                     self.apply_params_btn, self.apply_wave_btn,
                     self.send_pid_btn, self.pid_toggle_btn, self.autotune_btn]:
            btn.config(state=tk.DISABLED)

    def _set_buttons_idle(self):
        self.start_btn.config(state=tk.NORMAL if self._motor_enabled else tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self.home_neg_btn.config(state=tk.NORMAL if self._motor_enabled else tk.DISABLED)
        self.home_pos_btn.config(state=tk.NORMAL if self._motor_enabled else tk.DISABLED)
        self.jog_neg_btn.config(state=tk.NORMAL if self._motor_enabled else tk.DISABLED)
        self.jog_pos_btn.config(state=tk.NORMAL if self._motor_enabled else tk.DISABLED)
        self.move_btn.config(state=tk.NORMAL if self._motor_enabled else tk.DISABLED)
        self.enable_btn.config(state=tk.DISABLED if self._motor_enabled else tk.NORMAL)
        self.disable_btn.config(state=tk.NORMAL if self._motor_enabled else tk.DISABLED)
        self.apply_params_btn.config(state=tk.NORMAL)
        self.apply_wave_btn.config(state=tk.NORMAL)
        self.send_pid_btn.config(state=tk.NORMAL)
        self.pid_toggle_btn.config(state=tk.NORMAL)
        self.autotune_btn.config(state=tk.NORMAL)

    def _set_buttons_busy(self):
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.home_neg_btn.config(state=tk.DISABLED)
        self.home_pos_btn.config(state=tk.DISABLED)
        self.jog_neg_btn.config(state=tk.DISABLED)
        self.jog_pos_btn.config(state=tk.DISABLED)
        self.move_btn.config(state=tk.DISABLED)
        self.enable_btn.config(state=tk.DISABLED)
        self.disable_btn.config(state=tk.DISABLED)
        self.apply_params_btn.config(state=tk.DISABLED)
        self.apply_wave_btn.config(state=tk.DISABLED)
        self.autotune_btn.config(state=tk.DISABLED)

    def _on_connect(self):
        if self.ctrl and self.ctrl.ser and self.ctrl.ser.is_open:
            if self._is_running:
                self._on_stop()
            if self._motor_enabled:
                self._end_logging_session()
            self.ctrl.disconnect()
            self.ctrl = None
            self.connect_btn.config(text="Connect")
            self.conn_status_var.set("Disconnected")
            self.motor_status_var.set("Disconnected")
            self._motor_enabled = False
            self._set_buttons_disconnected()
            return

        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a serial port first.")
            return

        try:
            self.ctrl = StepperController(port)
            self.ctrl.set_line_callback(self._on_serial_line)
            self.ctrl.connect()
            self.connect_btn.config(text="Disconnect")
            self.conn_status_var.set(f"Connected ({port})")
            self.motor_status_var.set("Idle (disabled)")
            self._motor_enabled = False
            self._set_buttons_idle()
            # Start position polling
            self._poll_position()
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))

    def _on_enable(self):
        if not self.ctrl:
            return
        self.ctrl.enable()
        self._motor_enabled = True
        self.motor_status_var.set("Idle (enabled)")
        self._set_buttons_idle()
        # Start logging session on enable
        session_id = self.logger.begin()
        self.log_status_var.set(f"Session: {session_id}")
        # Auto-start live plot
        self.app.plot_tab.start_live(session_id)

    def _on_disable(self):
        if not self.ctrl:
            return
        if self._is_running:
            self._on_stop()
        self.ctrl.disable()
        self._motor_enabled = False
        self.motor_status_var.set("Idle (disabled)")
        self._set_buttons_idle()
        # End logging session on disable
        self._end_logging_session()

    def _end_logging_session(self):
        if self.logger.is_active:
            result = self.logger.end()
            self.log_status_var.set(
                f"Done: {result['session_id']} "
                f"({result['total_samples']} samples, "
                f"{result['num_files']} files)"
            )
            self.app.plot_tab._refresh_sessions()

    def _on_apply_params(self):
        if not self.ctrl:
            return
        try:
            vel = float(self.vel_var.get())
            accel = float(self.accel_var.get())
            self.ctrl.set_velocity(vel)
            self.ctrl.set_acceleration(accel)
            self._cur_vel = vel
            self._cur_accel = accel
            self._update_params_current_label()
            self._update_computed_params()
        except ValueError:
            messagebox.showwarning("Invalid input", "Enter numeric values for velocity and acceleration.")

    def _on_apply_waveform(self):
        if not self.ctrl:
            return
        func = self.func_var.get()
        try:
            amp = float(self.amp_var.get())
            self.ctrl.set_amplitude(amp)
            self._cur_amp = amp

            if func == "Sine":
                freq = float(self.freq_var.get())
                self.ctrl.set_frequency(freq)
                self.ctrl.set_waveform("SINE")
                self._cur_freq = freq
                # Auto-update velocity and acceleration from sine params
                v_max = 2 * math.pi * freq * amp
                a_max = (2 * math.pi * freq) ** 2 * amp
                self.vel_var.set(f"{v_max:.4f}")
                self.accel_var.set(f"{a_max:.4f}")
                self.ctrl.set_velocity(v_max)
                self.ctrl.set_acceleration(a_max)
                self._cur_vel = v_max
                self._cur_accel = a_max
                self._update_params_current_label()
            elif func == "Triangle":
                vel = float(self.wave_vel_var.get())
                F = float(self.duty_var.get())
                _, a_cap, period = compute_triangle_params(amp, vel, F)
                self.ctrl.set_velocity(vel)
                self.ctrl.set_acceleration(a_cap)
                self.ctrl.set_duty_cycle(F)
                self.ctrl.set_waveform("TRAP")
                self._cur_vel = vel
                self._cur_accel = a_cap
                self._cur_duty = F
                # Update Motion Parameters fields
                self.vel_var.set(f"{vel:.4f}")
                self.accel_var.set(f"{a_cap:.4f}")
                self._update_params_current_label()
            elif func == "Rounded Triangle":
                vel = float(self.wave_vel_var.get())
                F = float(self.duty_var.get())
                _, a_peak, jerk, period = compute_rounded_triangle_params(amp, vel, F)
                self.ctrl.set_velocity(vel)
                self.ctrl.set_acceleration(a_peak)
                self.ctrl.set_jerk(jerk)
                self.ctrl.set_duty_cycle(F)
                self.ctrl.set_waveform("SCURVE")
                self._cur_vel = vel
                self._cur_accel = a_peak
                self._cur_duty = F
                # Update Motion Parameters fields
                self.vel_var.set(f"{vel:.4f}")
                self.accel_var.set(f"{a_peak:.4f}")
                self._update_params_current_label()

            self._update_wave_current_label()
            self._update_computed_params()
            self._update_preview()
        except ValueError:
            messagebox.showwarning("Invalid input", "Enter valid numeric values.")

    def _on_start(self):
        if not self.ctrl:
            return
        self.ctrl.start()
        self._is_running = True
        self.motor_status_var.set("Running")
        self._set_buttons_busy()
        self._update_sample_count()

    def _on_stop(self):
        if not self.ctrl:
            return
        self.ctrl.stop()
        self._is_running = False
        self.motor_status_var.set("Idle" + (" (enabled)" if self._motor_enabled else " (disabled)"))
        self._set_buttons_idle()

    def _on_home_neg(self):
        if not self.ctrl:
            return
        self.ctrl.home_neg()
        self.motor_status_var.set("Homing (-)")
        self._set_buttons_busy()

    def _on_home_pos(self):
        if not self.ctrl:
            return
        self.ctrl.home_pos()
        self.motor_status_var.set("Homing (+)")
        self._set_buttons_busy()

    def _on_jog(self, mm):
        if not self.ctrl:
            return
        self.ctrl.move(mm)
        self.motor_status_var.set(f"Moving {mm:+.1f} mm")
        self._set_buttons_busy()

    def _on_manual_move(self):
        if not self.ctrl:
            return
        try:
            mm = float(self.move_var.get())
        except ValueError:
            messagebox.showwarning("Invalid input", "Enter a numeric distance in mm.")
            return
        self.ctrl.move(mm)
        self.motor_status_var.set(f"Moving {mm:+.4f} mm")
        self._set_buttons_busy()

    def _on_send_pid(self):
        if not self.ctrl:
            return
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            self.ctrl.set_pid(kp, ki, kd)
            self._cur_kp = kp
            self._cur_ki = ki
            self._cur_kd = kd
            self._update_pid_current_label()
        except ValueError:
            messagebox.showwarning("Invalid input", "Enter numeric values for PID gains.")

    def _on_toggle_pid(self):
        if not self.ctrl:
            return
        if self._pid_active:
            self.ctrl.pid_off()
            self._pid_active = False
            self.pid_toggle_btn.config(text="Activate PID")
            self.pid_status_var.set("PID: Off")
        else:
            self.ctrl.pid_on()
            self._pid_active = True
            self.pid_toggle_btn.config(text="Deactivate PID")
            self.pid_status_var.set("PID: Active")

    def _on_autotune(self):
        if not self.ctrl:
            return
        # Gather waveform parameters
        func = self.func_var.get()
        try:
            amp = float(self.amp_var.get())
        except ValueError:
            amp = 0.5
        wparams = {"type": func, "amplitude": amp}
        if func == "Sine":
            try:
                wparams["frequency"] = float(self.freq_var.get())
            except ValueError:
                wparams["frequency"] = 0.1
        else:
            try:
                wparams["velocity"] = float(self.wave_vel_var.get())
            except ValueError:
                wparams["velocity"] = 4.0
            try:
                wparams["duty_cycle"] = float(self.duty_var.get())
            except ValueError:
                wparams["duty_cycle"] = 0.9

        initial = (self._cur_kp, self._cur_ki, self._cur_kd)

        self._autotuner = PIDAutotuner(
            ctrl=self.ctrl,
            waveform_params=wparams,
            initial_gains=initial,
            n_cycles=5,
            settle_cycles=2,
            callback=self._autotune_callback,
        )
        self._autotuner.start()
        self.motor_status_var.set("Autotuning...")
        self.pid_status_var.set("PID: Autotuning (starting)...")
        self.autotune_btn.config(text="Cancel Autotune", command=self._on_cancel_autotune)
        self._set_buttons_busy()
        self.autotune_btn.config(state=tk.NORMAL)

    def _on_cancel_autotune(self):
        if self._autotuner:
            self._autotuner.cancel()
        self.pid_status_var.set("PID: Cancelling...")

    def _autotune_callback(self, event, data):
        """Called from autotune thread — dispatch to main thread."""
        self.after(0, self._handle_autotune_event, event, data)

    def _handle_autotune_event(self, event, data):
        if event == "phase":
            self.pid_status_var.set(
                f"Autotune phase {data['phase']}: {data['message']}"
            )
        elif event == "eval":
            self.pid_status_var.set(
                f"Autotune iter {data['iteration']}: "
                f"cost={data['cost']:.6f}  "
                f"Kp={data['kp']:.3f} Ki={data['ki']:.3f} Kd={data['kd']:.3f}"
            )
        elif event == "done":
            self.kp_var.set(f"{data['kp']:.4f}")
            self.ki_var.set(f"{data['ki']:.4f}")
            self.kd_var.set(f"{data['kd']:.4f}")
            self._cur_kp = data["kp"]
            self._cur_ki = data["ki"]
            self._cur_kd = data["kd"]
            self._update_pid_current_label()
            self.pid_status_var.set(
                f"PID: Tuned in {data['iterations']} iters "
                f"(cost={data['cost']:.6f})"
            )
            self.motor_status_var.set(
                "Idle" + (" (enabled)" if self._motor_enabled else "")
            )
            self._finish_autotune()
        elif event == "cancelled":
            self.pid_status_var.set("PID: Autotune cancelled")
            self.motor_status_var.set(
                "Idle" + (" (enabled)" if self._motor_enabled else "")
            )
            self._finish_autotune()
        elif event == "error":
            self.pid_status_var.set(f"PID: Autotune error: {data['message']}")
            self.motor_status_var.set(
                "Idle" + (" (enabled)" if self._motor_enabled else "")
            )
            self._finish_autotune()

    def _finish_autotune(self):
        self._autotuner = None
        self.autotune_btn.config(text="Autotune", command=self._on_autotune)
        self._set_buttons_idle()

    def _poll_position(self):
        """Periodically request position from ESP32."""
        if self.ctrl and self.ctrl.ser and self.ctrl.ser.is_open:
            self.ctrl.pos()
            self.after(500, self._poll_position)

    def _on_serial_line(self, line):
        """Called from reader thread for every serial line."""
        print(f"[serial] {line}")

        # Handle state transition messages (dispatch to main thread)
        if line.startswith("OK:HOMED_") or line.startswith("OK:BACKOFF_DONE"):
            self.after(0, self._on_became_idle, line)
        elif line.startswith("ALARM:LIMIT_"):
            self.after(0, self._on_limit_alarm, line)
        elif line == "OK:STOPPED":
            self.after(0, self._on_stopped)
        elif line == "OK:MOVE_DONE":
            self.after(0, self._on_move_done)
        elif line == "OK:PID_ON":
            self.after(0, self._on_pid_on_confirmed)
        elif line == "OK:PID_OFF":
            self.after(0, self._on_pid_off_confirmed)
        elif line.startswith("POS:"):
            self.after(0, self._on_pos_response, line)
        elif line.startswith("OK:VEL:"):
            self.after(0, self._on_vel_confirmed, line)
        elif line.startswith("OK:ACCEL:"):
            self.after(0, self._on_accel_confirmed, line)

        # Log CSV data lines (start with digit = telemetry)
        if line and line[0].isdigit():
            if self.logger.is_active:
                self.logger.write_line(line)
            # Feed autotune buffer
            if self._autotuner and self._autotuner.is_running:
                parts = line.split(',')
                if len(parts) >= 3:
                    try:
                        self._autotuner.buffer.append(
                            float(parts[1]), float(parts[2])
                        )
                    except ValueError:
                        pass

    def _on_became_idle(self, line):
        if line.startswith("OK:HOMED_"):
            self.motor_status_var.set(f"Homed ({line})")
        else:
            self.motor_status_var.set("Idle" + (" (enabled)" if self._motor_enabled else " (disabled)"))
        self._set_buttons_idle()

    def _on_limit_alarm(self, line):
        self.motor_status_var.set(f"LIMIT HIT ({line})")
        self._is_running = False

    def _on_stopped(self):
        self._is_running = False
        self.motor_status_var.set("Idle" + (" (enabled)" if self._motor_enabled else " (disabled)"))
        self._set_buttons_idle()

    def _on_move_done(self):
        self.motor_status_var.set("Idle" + (" (enabled)" if self._motor_enabled else ""))
        self._set_buttons_idle()

    def _on_pid_on_confirmed(self):
        self._pid_active = True
        self.pid_toggle_btn.config(text="Deactivate PID")
        self.pid_status_var.set("PID: Active")

    def _on_pid_off_confirmed(self):
        self._pid_active = False
        self.pid_toggle_btn.config(text="Activate PID")
        self.pid_status_var.set("PID: Off")

    def _on_pos_response(self, line):
        try:
            parts = line.split(":")[1].split(",")
            enc_mm = float(parts[1])
            self._cur_pos = enc_mm
            self.position_var.set(f"Position: {enc_mm:.4f} mm (encoder)")
        except (IndexError, ValueError):
            pass

    def _on_vel_confirmed(self, line):
        try:
            val = float(line.split(":")[2])
            self._cur_vel = val
            self._update_params_current_label()
        except (IndexError, ValueError):
            pass

    def _on_accel_confirmed(self, line):
        try:
            val = float(line.split(":")[2])
            self._cur_accel = val
            self._update_params_current_label()
        except (IndexError, ValueError):
            pass

    def _update_sample_count(self):
        if self._is_running:
            self.sample_count_var.set(f"{self.logger.total_samples} samples")
            self.after(500, self._update_sample_count)

    def get_settings(self):
        """Collect current settings from GUI fields."""
        return {
            "port": self.port_var.get(),
            "velocity_mm_s": self._safe_float(self.wave_vel_var.get(), 4.0),
            "amplitude_mm": self._safe_float(self.amp_var.get(), 0.5),
            "frequency_hz": self._safe_float(self.freq_var.get(), 0.1),
            "duty_cycle": self._safe_float(self.duty_var.get(), 0.9),
            "function": self.func_var.get(),
            "pid_kp": self._safe_float(self.kp_var.get(), 1.0),
            "pid_ki": self._safe_float(self.ki_var.get(), 0.0),
            "pid_kd": self._safe_float(self.kd_var.get(), 0.0),
        }

    @staticmethod
    def _safe_float(val, default):
        try:
            return float(val)
        except ValueError:
            return default

    def destroy(self):
        if self._is_running:
            self._on_stop()
        if self._motor_enabled:
            self._end_logging_session()
        if self.ctrl:
            self.ctrl.disconnect()
        super().destroy()


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.settings = load_settings()

        self.title("ESP32 Stepper Controller")
        self.geometry(self.settings.get("window_geometry", "1100x850"))

        notebook = ttk.Notebook(self)
        notebook.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Controller tab first, Plot tab second
        self.ctrl_tab = ControllerTab(notebook, app=self)
        self.plot_tab = PlotTab(notebook, app=self)

        notebook.add(self.ctrl_tab, text="  Controller  ")
        notebook.add(self.plot_tab, text="  Plot  ")

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        settings = dict(self.settings)
        settings.update(self.ctrl_tab.get_settings())
        settings["plot_x"] = self.plot_tab.x_var.get()
        settings["plot_y"] = self.plot_tab.y_var.get()
        settings["window_geometry"] = self.geometry()
        try:
            settings["live_plot_window_s"] = float(self.plot_tab.window_var.get())
        except ValueError:
            pass

        save_settings(settings)

        self.ctrl_tab.destroy()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()
