"""
Python-side PID autotune for the ESP32 stepper controller.

Hybrid approach:
  Phase 1 (analytical): Run the waveform once with current gains, measure
    phase lag (cross-correlation), gain error (amplitude ratio), and DC offset.
    Compute improved Kp/Ki/Kd directly from these error characteristics.
  Phase 2 (refinement): Run a small number of Nelder-Mead iterations starting
    from the analytical estimate to fine-tune.

Perfect tracking: actual == target → RMS error = 0.
Phase lag, amplitude mismatch, or any deviation → RMS error > 0.
"""
import threading
import time
import numpy as np
from scipy.optimize import minimize
from waveforms import get_period


class TelemetryBuffer:
    """Thread-safe buffer for (target_mm, actual_mm) pairs."""

    def __init__(self):
        self._lock = threading.Lock()
        self._target = []
        self._actual = []

    def append(self, target_mm, actual_mm):
        with self._lock:
            self._target.append(target_mm)
            self._actual.append(actual_mm)

    def drain(self):
        """Return (target_array, actual_array) and clear buffer."""
        with self._lock:
            t = np.array(self._target, dtype=np.float64)
            a = np.array(self._actual, dtype=np.float64)
            self._target.clear()
            self._actual.clear()
        return t, a

    def clear(self):
        with self._lock:
            self._target.clear()
            self._actual.clear()


class _CancelledError(Exception):
    pass


class PIDAutotuner:
    """Hybrid PID optimizer: analytical estimate + Nelder-Mead refinement.

    Parameters
    ----------
    ctrl : StepperController
        Serial controller instance (already connected).
    waveform_params : dict
        Keys: type ("Sine"/"Triangle"/"Rounded Triangle"),
              amplitude (mm), frequency (Hz, Sine only),
              velocity (mm/s, Triangle/Rounded Triangle only),
              duty_cycle (fraction).
    initial_gains : tuple
        Starting (Kp, Ki, Kd).
    n_cycles : int
        Number of waveform cycles to measure per evaluation.
    settle_cycles : int
        Number of waveform cycles to discard for settling.
    callback : callable or None
        callback(event, data) where event is one of:
          "phase"  — data = dict(phase, message)
          "eval"   — data = dict(iteration, cost, kp, ki, kd)
          "done"   — data = dict(kp, ki, kd, cost, iterations)
          "cancelled" — data = dict(kp, ki, kd)  (original gains)
          "error"  — data = dict(message)
    """

    KP_BOUNDS = (0.0, 50.0)
    KI_BOUNDS = (0.0, 100.0)
    KD_BOUNDS = (0.0, 5.0)

    def __init__(self, ctrl, waveform_params, initial_gains=(1.0, 0.0, 0.0),
                 n_cycles=5, settle_cycles=2, callback=None):
        self.ctrl = ctrl
        self.waveform_params = waveform_params
        self.initial_gains = initial_gains
        self.n_cycles = n_cycles
        self.settle_cycles = settle_cycles
        self._callback = callback

        self.buffer = TelemetryBuffer()

        wp = waveform_params
        self.period = get_period(
            wp["type"], wp["amplitude"],
            frequency=wp.get("frequency"),
            velocity=wp.get("velocity"),
            duty_cycle=wp.get("duty_cycle", 0.9),
        )

        self._cancel = False
        self._running = False
        self._thread = None
        self._iteration = 0

    @property
    def is_running(self):
        return self._running

    def start(self):
        """Launch the optimization in a background thread."""
        if self._running:
            return
        self._cancel = False
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def cancel(self):
        """Request cancellation. Takes effect between evaluations."""
        self._cancel = True

    def _emit(self, event, data):
        if self._callback:
            self._callback(event, data)

    def _check_cancel(self):
        if self._cancel:
            raise _CancelledError()

    def _run_waveform(self, kp, ki, kd):
        """Run waveform with given gains, return (target, actual) arrays."""
        self.ctrl.set_pid(kp, ki, kd)
        time.sleep(0.05)

        self.buffer.clear()
        self.ctrl.start()

        # Settling phase
        settle_time = self.settle_cycles * self.period
        time.sleep(settle_time)
        self._check_cancel()

        # Measurement phase
        self.buffer.clear()
        measure_time = self.n_cycles * self.period
        time.sleep(measure_time)

        self.ctrl.stop()
        time.sleep(0.1)
        self._check_cancel()

        return self.buffer.drain()

    def _run(self):
        """Thread target: hybrid optimization."""
        original_kp, original_ki, original_kd = self.initial_gains
        try:
            self.ctrl.pid_on()
            time.sleep(0.1)
            self._iteration = 0

            # --- Phase 1: Analytical estimate ---
            self._emit("phase", {"phase": 1,
                                 "message": "Measuring tracking errors..."})

            target, actual = self._run_waveform(
                original_kp, original_ki, original_kd)

            if len(target) < 50:
                raise RuntimeError("Not enough telemetry data collected. "
                                   "Check serial connection.")

            # Analyze tracking errors
            analysis = self._analyze_tracking(target, actual)
            phase_lag_s = analysis["phase_lag_s"]
            gain_ratio = analysis["gain_ratio"]
            dc_offset = analysis["dc_offset"]
            initial_rms = analysis["rms_error"]

            self._emit("eval", {
                "iteration": 0,
                "cost": initial_rms,
                "kp": original_kp, "ki": original_ki, "kd": original_kd,
                "analysis": {
                    "phase_lag_ms": phase_lag_s * 1000,
                    "gain_ratio": gain_ratio,
                    "dc_offset_mm": dc_offset,
                },
            })

            # Compute improved gains from analysis
            new_kp, new_ki, new_kd = self._compute_gains_from_analysis(
                original_kp, original_ki, original_kd, analysis)

            self._emit("phase", {
                "phase": 1,
                "message": (f"Analytical: Kp={new_kp:.2f} Ki={new_ki:.2f} "
                            f"Kd={new_kd:.4f} "
                            f"(lag={phase_lag_s*1000:.1f}ms, "
                            f"gain={gain_ratio:.3f}, "
                            f"offset={dc_offset:.4f}mm)"),
            })

            self._check_cancel()

            # Verify analytical gains
            target2, actual2 = self._run_waveform(new_kp, new_ki, new_kd)
            analytical_rms = (self._compute_rms_error(target2, actual2)
                              if len(target2) >= 50 else 1e6)

            self._iteration = 1
            self._emit("eval", {
                "iteration": 1,
                "cost": analytical_rms,
                "kp": new_kp, "ki": new_ki, "kd": new_kd,
            })

            # If analytical is worse, fall back to original gains for refinement
            if analytical_rms > initial_rms * 1.5:
                new_kp, new_ki, new_kd = original_kp, original_ki, original_kd
                self._emit("phase", {
                    "phase": 1,
                    "message": "Analytical estimate worse; reverting to "
                               "original gains for refinement.",
                })

            self._check_cancel()

            # --- Phase 2: Nelder-Mead refinement ---
            self._emit("phase", {"phase": 2,
                                 "message": "Refining with Nelder-Mead..."})

            x0 = np.array([new_kp, new_ki, new_kd])
            result = minimize(
                self._objective,
                x0,
                method="Nelder-Mead",
                options={
                    "maxiter": 15,
                    "xatol": 0.01,
                    "fatol": 1e-6,
                    "adaptive": True,
                },
            )

            best_kp, best_ki, best_kd = result.x
            self.ctrl.set_pid(best_kp, best_ki, best_kd)
            time.sleep(0.05)

            self._emit("done", {
                "kp": best_kp, "ki": best_ki, "kd": best_kd,
                "cost": result.fun, "iterations": self._iteration,
            })

        except _CancelledError:
            self.ctrl.stop()
            time.sleep(0.1)
            self.ctrl.set_pid(original_kp, original_ki, original_kd)
            self._emit("cancelled", {
                "kp": original_kp, "ki": original_ki, "kd": original_kd,
            })

        except Exception as e:
            self.ctrl.stop()
            self._emit("error", {"message": str(e)})

        finally:
            self._running = False

    def _analyze_tracking(self, target, actual):
        """Analyze how actual tracks target. Returns dict with error metrics."""
        # Telemetry rate (samples per second) — ESP32 sends at 1000 Hz
        fs = 1000.0

        # DC offset: mean difference
        dc_offset = float(np.mean(actual - target))

        # Remove DC offset for phase/gain analysis
        target_ac = target - np.mean(target)
        actual_ac = actual - np.mean(actual)

        # Gain ratio: RMS(actual) / RMS(target)
        rms_target = np.sqrt(np.mean(target_ac ** 2))
        rms_actual = np.sqrt(np.mean(actual_ac ** 2))
        gain_ratio = rms_actual / rms_target if rms_target > 1e-9 else 1.0

        # Phase lag via cross-correlation
        # Only search within ±half period to avoid wrapping artifacts
        max_lag_samples = int(self.period * fs / 2)
        max_lag_samples = min(max_lag_samples, len(target) // 2)

        cross_corr = np.correlate(
            actual_ac[:len(target_ac)],
            target_ac,
            mode='full'
        )
        mid = len(target_ac) - 1
        search_region = cross_corr[mid - max_lag_samples:mid + max_lag_samples + 1]
        peak_idx = np.argmax(search_region)
        lag_samples = peak_idx - max_lag_samples  # negative = actual lags target
        phase_lag_s = lag_samples / fs

        rms_error = float(np.sqrt(np.mean((actual - target) ** 2)))

        return {
            "phase_lag_s": phase_lag_s,
            "gain_ratio": gain_ratio,
            "dc_offset": dc_offset,
            "rms_error": rms_error,
        }

    def _compute_gains_from_analysis(self, kp, ki, kd, analysis):
        """Compute improved PID gains from tracking error analysis.

        Strategy:
        - Phase lag > 0 (actual lags target): increase Kd (derivative anticipation)
        - Gain ratio < 1 (actual undershoots): increase Kp
        - Gain ratio > 1 (actual overshoots): decrease Kp
        - DC offset != 0: increase Ki to eliminate steady-state error
        """
        phase_lag_s = analysis["phase_lag_s"]
        gain_ratio = analysis["gain_ratio"]
        dc_offset = analysis["dc_offset"]
        amplitude = self.waveform_params["amplitude"]

        # --- Kp adjustment based on gain error ---
        # If gain_ratio < 1, system is too sluggish → increase Kp
        # If gain_ratio > 1, system overshoots → decrease Kp
        gain_error = 1.0 - gain_ratio  # positive = undershoot
        new_kp = kp * (1.0 + 0.5 * gain_error)

        # --- Ki adjustment based on DC offset ---
        # Larger offset relative to amplitude → need more Ki
        if amplitude > 1e-9:
            offset_frac = abs(dc_offset) / amplitude
            if offset_frac > 0.01:  # more than 1% offset
                new_ki = max(ki, 0.5) * (1.0 + 2.0 * offset_frac)
            else:
                new_ki = ki
        else:
            new_ki = ki

        # --- Kd adjustment based on phase lag ---
        # Phase lag (positive lag_s means actual leads, negative means lags)
        # Negative phase_lag_s = actual is behind target → need more Kd
        if phase_lag_s < -0.001:  # more than 1ms lag
            lag_fraction = abs(phase_lag_s) / self.period
            new_kd = max(kd, 0.01) * (1.0 + 5.0 * lag_fraction)
        elif phase_lag_s > 0.001:  # actual leads (too much derivative)
            new_kd = kd * 0.7
        else:
            new_kd = kd

        # Clamp to bounds
        new_kp = float(np.clip(new_kp, self.KP_BOUNDS[0] + 0.01, self.KP_BOUNDS[1]))
        new_ki = float(np.clip(new_ki, self.KI_BOUNDS[0], self.KI_BOUNDS[1]))
        new_kd = float(np.clip(new_kd, self.KD_BOUNDS[0], self.KD_BOUNDS[1]))

        return new_kp, new_ki, new_kd

    def _objective(self, params):
        """Evaluate one set of PID gains. Returns the RMS tracking error."""
        self._check_cancel()

        kp, ki, kd = params

        if (kp < self.KP_BOUNDS[0] or kp > self.KP_BOUNDS[1] or
                ki < self.KI_BOUNDS[0] or ki > self.KI_BOUNDS[1] or
                kd < self.KD_BOUNDS[0] or kd > self.KD_BOUNDS[1]):
            return 1e6

        self._iteration += 1

        target, actual = self._run_waveform(kp, ki, kd)

        if len(target) < 10:
            cost = 1e6
        else:
            cost = self._compute_rms_error(target, actual)

        self._emit("eval", {
            "iteration": self._iteration,
            "cost": cost,
            "kp": kp, "ki": ki, "kd": kd,
        })

        return cost

    @staticmethod
    def _compute_rms_error(target, actual):
        """RMS tracking error. Perfect tracking = 0."""
        return float(np.sqrt(np.mean((actual - target) ** 2)))
