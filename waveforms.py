"""
Waveform generators for the ESP32 stepper controller.
Computes position, velocity, and acceleration arrays for preview and reference.

Supported waveforms:
  - Sine: continuous sinusoidal oscillation (params: amplitude, frequency)
  - Triangle: C1-continuous triangle position wave with parabolic corners
      (params: amplitude, velocity, duty_cycle)
  - Rounded Triangle: C2-continuous triangle position wave with jerk-limited corners
      (params: amplitude, velocity, duty_cycle)

Triangle gives a trapezoidal velocity profile with constant acceleration in
the corner caps. Rounded Triangle extends this to C2 continuity using a
2-phase symmetric jerk microprofile in the corners.
"""
import math
import numpy as np


def generate_sine(amplitude, frequency, n_periods=3, dt=0.001):
    """Generate sinusoidal oscillation waveform."""
    T = 1.0 / frequency
    t = np.arange(0, n_periods * T, dt)
    w = 2 * math.pi * frequency
    x = amplitude * np.sin(w * t)
    v = amplitude * w * np.cos(w * t)
    a = -amplitude * w * w * np.sin(w * t)
    return t, x, v, a


def generate_triangle(amplitude, velocity, duty_cycle=0.9, n_periods=3, dt=0.001):
    """Generate C1-continuous triangle position wave with parabolic corners.

    5-segment piecewise construction per period:
      1. Valley cap (parabola, accel = +s/dt_cap)
      2. Rising linear (velocity = +s)
      3. Peak cap (parabola, accel = -s/dt_cap)
      4. Falling linear (velocity = -s)
      5. Valley cap near T0 (wraps with segment 1 of next period)

    Parameters:
        amplitude: peak position A (oscillates -A to +A)
        velocity: constant speed on flat segments (s, mm/s)
        duty_cycle: fraction of period at constant velocity (F, 0 < F < 1)
    """
    A = amplitude
    s = velocity
    F = duty_cycle

    if A < 1e-12 or s <= 0 or F <= 0 or F >= 1:
        t = np.arange(0, 1, dt)
        return t, np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)

    T0 = 8.0 * A / (s * (1.0 + F))
    dc = (1.0 - F) * T0 / 4.0  # cap half-width
    tc = T0 / 2.0
    a_cap = s / dc

    t = np.arange(0, n_periods * T0, dt)
    tm = t % T0

    x = np.zeros_like(t)
    v = np.zeros_like(t)
    a = np.zeros_like(t)

    # Segment 1: valley cap [0, dc)
    m = tm < dc
    tau = tm[m]
    x[m] = -A + (s / (2.0 * dc)) * tau**2
    v[m] = (s / dc) * tau
    a[m] = a_cap

    # Segment 2: rising linear [dc, tc - dc)
    m = (tm >= dc) & (tm < tc - dc)
    tau = tm[m] - dc
    x[m] = -A + 0.5 * s * dc + s * tau
    v[m] = s
    # a[m] = 0 (already zero)

    # Segment 3: peak cap [tc - dc, tc + dc)
    m = (tm >= tc - dc) & (tm < tc + dc)
    tau = tm[m] - tc
    x[m] = A - (s / (2.0 * dc)) * tau**2
    v[m] = -(s / dc) * tau
    a[m] = -a_cap

    # Segment 4: falling linear [tc + dc, T0 - dc)
    m = (tm >= tc + dc) & (tm < T0 - dc)
    tau = tm[m] - (tc + dc)
    x[m] = A - 0.5 * s * dc - s * tau
    v[m] = -s
    # a[m] = 0 (already zero)

    # Segment 5: valley cap near T0 [T0 - dc, T0)
    m = tm >= T0 - dc
    tau = tm[m] - T0
    x[m] = -A + (s / (2.0 * dc)) * tau**2
    v[m] = (s / dc) * tau
    a[m] = a_cap

    return t, x, v, a


def generate_rounded_triangle(amplitude, velocity, duty_cycle=0.9, n_periods=3, dt=0.001):
    """Generate C2-continuous triangle position wave with jerk-limited corners.

    Like generate_triangle but corners use a 2-phase symmetric jerk profile
    instead of constant acceleration, giving C2 (continuous acceleration).

    6 segments per period starting from valley center:
      1. Valley exit  (j = -J, accel ramps down from peak to 0, vel 0 -> +s)
      2. Rising flat  (j = 0, vel = +s)
      3. Peak entry   (j = -J, accel ramps from 0 to -peak, vel +s -> 0)
      4. Peak exit    (j = +J, accel ramps from -peak to 0, vel 0 -> -s)
      5. Falling flat (j = 0, vel = -s)
      6. Valley entry (j = +J, accel ramps from 0 to +peak, vel -s -> 0)

    Parameters:
        amplitude: peak position A (oscillates -A to +A)
        velocity: constant speed on flat segments (s, mm/s)
        duty_cycle: fraction of period at constant velocity (F, 0 < F < 1)
    """
    A = amplitude
    s = velocity
    F = duty_cycle

    if A < 1e-12 or s <= 0 or F <= 0 or F >= 1:
        t = np.arange(0, 1, dt)
        return t, np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)

    T0 = 12.0 * A / (s * (2.0 + F))
    Tj = (1.0 - F) * T0 / 4.0
    tc = T0 / 2.0
    J = 2.0 * s / (Tj * Tj)
    a_peak = J * Tj  # peak acceleration at corner centers

    T_flat = tc - 2.0 * Tj  # duration of each flat segment
    if T_flat < 0:
        T_flat = 0

    # 6 segments per period: (duration, jerk)
    seg_defs = [
        (Tj,     -J),    # 1. Valley exit
        (T_flat,  0.0),  # 2. Rise
        (Tj,     -J),    # 3. Peak entry
        (Tj,     +J),    # 4. Peak exit
        (T_flat,  0.0),  # 5. Fall
        (Tj,     +J),    # 6. Valley entry
    ]

    # Forward-integrate to get initial conditions per segment
    segments = []
    px, pv, pa = -A, 0.0, a_peak
    for dur, j in seg_defs:
        segments.append((dur, j, px, pv, pa))
        if dur > 1e-15:
            px += pv * dur + 0.5 * pa * dur**2 + (1.0 / 6.0) * j * dur**3
            pv += pa * dur + 0.5 * j * dur**2
            pa += j * dur

    # Cumulative start times
    seg_starts = []
    cum = 0.0
    for dur, j, x0, v0, a0 in segments:
        seg_starts.append(cum)
        cum += dur
    seg_starts.append(cum)  # sentinel for last segment boundary

    t = np.arange(0, n_periods * T0, dt)
    tm = t % T0

    x_arr = np.zeros_like(t)
    v_arr = np.zeros_like(t)
    a_arr = np.zeros_like(t)

    for si, (dur, j, x0, v0, a0) in enumerate(segments):
        if dur < 1e-15:
            continue
        if si < len(segments) - 1:
            mask = (tm >= seg_starts[si]) & (tm < seg_starts[si + 1])
        else:
            mask = tm >= seg_starts[si]

        lt = tm[mask] - seg_starts[si]
        x_arr[mask] = x0 + v0 * lt + 0.5 * a0 * lt**2 + (1.0 / 6.0) * j * lt**3
        v_arr[mask] = v0 + a0 * lt + 0.5 * j * lt**2
        a_arr[mask] = a0 + j * lt

    return t, x_arr, v_arr, a_arr


def compute_triangle_params(amplitude, velocity, duty_cycle=0.9):
    """Compute derived parameters for Triangle (C1).

    Returns (velocity, accel_cap, period).
    """
    A = amplitude
    s = velocity
    F = duty_cycle
    if A < 1e-12 or s <= 0 or F <= 0 or F >= 1:
        return s, 0.0, 1.0
    T0 = 8.0 * A / (s * (1.0 + F))
    dc = (1.0 - F) * T0 / 4.0
    a_cap = s / dc
    return s, a_cap, T0


def compute_rounded_triangle_params(amplitude, velocity, duty_cycle=0.9):
    """Compute derived parameters for Rounded Triangle (C2).

    Returns (velocity, accel_peak, jerk, period).
    """
    A = amplitude
    s = velocity
    F = duty_cycle
    if A < 1e-12 or s <= 0 or F <= 0 or F >= 1:
        return s, 0.0, 0.0, 1.0
    T0 = 12.0 * A / (s * (2.0 + F))
    Tj = (1.0 - F) * T0 / 4.0
    J = 2.0 * s / (Tj * Tj)
    a_peak = J * Tj
    return s, a_peak, J, T0


def get_period(waveform_type, amplitude, frequency=None, velocity=None, duty_cycle=0.9):
    """Get the period of the waveform in seconds."""
    if waveform_type == "Sine":
        return 1.0 / frequency if frequency and frequency > 0 else 1.0
    elif waveform_type == "Triangle":
        if velocity and velocity > 0:
            _, _, T0 = compute_triangle_params(amplitude, velocity, duty_cycle)
            return T0
    elif waveform_type == "Rounded Triangle":
        if velocity and velocity > 0:
            _, _, _, T0 = compute_rounded_triangle_params(amplitude, velocity, duty_cycle)
            return T0
    return 1.0
