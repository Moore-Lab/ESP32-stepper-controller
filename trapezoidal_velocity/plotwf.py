import numpy as np
import matplotlib.pyplot as plt

# --- Parameters for a rest-to-rest point-to-point move ---
S = 1.0        # total move distance [m]
V_max = 0.6    # velocity limit [m/s]
A_max = 1.0    # acceleration limit [m/s^2]
J_max = 4.0    # jerk limit [m/s^3]

# Time step for numerical integration (small for smooth curves)
dt = 5e-4

# --- Helper: integrate profile defined by piecewise jerk segments ---
def integrate_segments(segments, dt=1e-3):
    t_list = [0.0]
    a_list = [0.0]
    v_list = [0.0]
    x_list = [0.0]

    for (dur, j) in segments:
        n = int(np.round(dur / dt))
        for k in range(n):
            a_new = a_list[-1] + j * dt
            v_new = v_list[-1] + a_list[-1] * dt
            x_new = x_list[-1] + v_list[-1] * dt
            t_new = t_list[-1] + dt
            a_list.append(a_new)
            v_list.append(v_new)
            x_list.append(x_new)
            t_list.append(t_new)
        leftover = dur - n * dt
        if leftover > 1e-12:
            a_new = a_list[-1] + j * leftover
            v_new = v_list[-1] + a_list[-1] * leftover
            x_new = x_list[-1] + v_list[-1] * leftover
            t_new = t_list[-1] + leftover
            a_list.append(a_new)
            v_list.append(v_new)
            x_list.append(x_new)
            t_list.append(t_new)

    return np.array(t_list), np.array(a_list), np.array(v_list), np.array(x_list)

# --- Jerk-limited S-curve segment durations ---
T_j = A_max / J_max
T_a = max(0.0, (V_max - (A_max**2)/J_max) / A_max)

# Distance covered during the acceleration half (three segments)
x_acc = (1.5 * (A_max**2 / J_max) * T_a) + (A_max**3 / (J_max**2)) + (0.5 * A_max * T_a**2)

# Plateau time (may be negative; handle below)
T_v = (S - 2 * x_acc) / V_max

if T_v < -1e-9:
    T_v = 0.0
    k2 = 0.5 * A_max
    k1 = 1.5 * (A_max**2 / J_max)
    k0 = (A_max**3 / (J_max**2)) - (S / 2.0)
    disc = k1**2 - 4*k2*k0
    if disc >= 0:
        r1 = (-k1 + np.sqrt(disc)) / (2*k2)
        r2 = (-k1 - np.sqrt(disc)) / (2*k2)
        Ta_candidates = [r for r in [r1, r2] if r >= 0]
        if len(Ta_candidates) > 0:
            T_a = min(Ta_candidates)
        else:
            T_a = 0.0
    else:
        T_a = 0.0

    if T_a <= 1e-12:
        # Jerk-only triangular S-curve: S = 2 * J * Tj'^3
        T_j = (S / (2.0 * J_max)) ** (1/3)
        T_a = 0.0

x_acc = (1.5 * (A_max**2 / J_max) * T_a) + (A_max**3 / (J_max**2)) + (0.5 * A_max * T_a**2)
T_v = max(0.0, (S - 2 * x_acc) / V_max)

segments_jerk = [
    (T_j, +J_max),
    (T_a, 0.0),
    (T_j, -J_max),
    (T_v, 0.0),
    (T_j, -J_max),
    (T_a, 0.0),
    (T_j, +J_max),
]

TJ, aJ, vJ, xJ = integrate_segments(segments_jerk, dt=dt)

# --- Trapezoidal (non-jerk-limited) profile ---
T_acc_trap = V_max / A_max
x_acc_trap = 0.5 * A_max * T_acc_trap**2
T_cv_trap = (S - 2 * x_acc_trap) / V_max
if T_cv_trap < 0:
    T_cv_trap = 0.0
    T_acc_trap = np.sqrt(S / A_max)

# Integrate trapezoid with piecewise constant acceleration
def integrate_trapezoid(Ta, Tcv, A, dt=1e-3):
    times, accs, vels, poss = [], [], [], []
    t = 0.0; a = 0.0; v = 0.0; x = 0.0

    def step_with_acc(a_cmd, duration):
        nonlocal t, a, v, x
        n = int(np.round(duration / dt))
        for _ in range(n):
            a = a_cmd
            v = v + a * dt
            x = x + v * dt
            t = t + dt
            times.append(t); accs.append(a); vels.append(v); poss.append(x)
        leftover = duration - n * dt
        if leftover > 1e-12:
            a = a_cmd
            v = v + a * leftover
            x = x + v * leftover
            t = t + leftover
            times.append(t); accs.append(a); vels.append(v); poss.append(x)

    step_with_acc(+A, Ta)
    step_with_acc(0.0, Tcv)
    step_with_acc(-A, Ta)
    return np.array(times), np.array(accs), np.array(vels), np.array(poss)

TT, aT, vT, xT = integrate_trapezoid(T_acc_trap, T_cv_trap, A_max, dt=dt)

# --- Plot and SAVE three figures ---
plt.figure(figsize=(8, 4.5))
plt.plot(TJ, xJ, label='Position – Jerk-limited S-curve', linewidth=2)
plt.plot(TT, xT, label='Position – Trapezoidal (non-jerk-limited)', linewidth=2, linestyle='--')
plt.xlabel('Time [s]'); plt.ylabel('Position [m]')
plt.title('Position vs Time: Jerk-limited S-curve vs Trapezoidal')
plt.grid(True, alpha=0.3); plt.legend(); plt.tight_layout()
plt.show()

plt.figure(figsize=(8, 4.5))
plt.plot(TJ, vJ, label='Velocity – Jerk-limited S-curve', linewidth=2)
plt.plot(TT, vT, label='Velocity – Trapezoidal (non-jerk-limited)', linewidth=2, linestyle='--')
plt.xlabel('Time [s]'); plt.ylabel('Velocity [m/s]')
plt.title('Velocity vs Time: Jerk-limited S-curve vs Trapezoidal')
plt.grid(True, alpha=0.3); plt.legend(); plt.tight_layout()
plt.show()

plt.figure(figsize=(8, 4.5))
plt.plot(TJ, aJ, label='Acceleration – Jerk-limited S-curve', linewidth=2)
plt.plot(TT, aT, label='Acceleration – Trapezoidal (non-jerk-limited)', linewidth=2, linestyle='--')
plt.xlabel('Time [s]'); plt.ylabel('Acceleration [m/s²]')
plt.title('Acceleration vs Time: Jerk-limited S-curve vs Trapezoidal')
plt.grid(True, alpha=0.3); plt.legend(); plt.tight_layout()
plt.show()

print('--- Jerk-limited profile ---')
print(f'T_j = {T_j:.4f} s, T_a = {T_a:.4f} s, T_v = {T_v:.4f} s')
print(f'Total time (approx) = {TJ[-1]:.4f} s, Final pos = {xJ[-1]:.4f} m, Final vel = {vJ[-1]:.4f} m/s')
print('--- Trapezoidal profile ---')
print(f'T_acc = {T_acc_trap:.4f} s, T_cv = {T_cv_trap:.4f} s')
print(f'Total time (approx) = {TT[-1]:.4f} s, Final pos = {xT[-1]:.4f} m, Final vel = {vT[-1]:.4f} m/s')