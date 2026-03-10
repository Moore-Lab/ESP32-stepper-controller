# Motion Profiles for Microcontrollers
## From Velocity Trapezoid → Position, and Jerk‑Limited S‑Curve (Derivations & Final Piece‑wise Forms)

**Goal:** Provide clean, implementation‑ready derivations you can port to a microcontroller firmware:
1) Start with a **velocity trapezoid** (piecewise‑linear velocity).  
2) Integrate to obtain the **position** (piecewise quadratic/linear/quadratic).  
3) Motivate **jerk‑limited** motion (S‑curve) and derive its **7‑segment** form with final piece‑wise expressions suitable for discrete‑time generation.

---

## 0) Notation & Assumptions

- $x(t)$: position, $v(t)=\dot x(t)$: velocity, $a(t)=\dot v(t)$: acceleration, $j(t)=\dot a(t)$: jerk.  
- Motion is **rest‑to‑rest** (start/end velocity $=0$).  
- Constraints: maximum velocity $V_{\max}$, maximum acceleration $A_{\max}$, optional maximum jerk $J_{\max}$.  
- Times:
  - Trapezoid: rise (accel) time $T_r$, constant‑velocity time $T_c$, fall (decel) time $T_r$ (symmetric case).
  - S‑curve: jerk ramp time $T_j = A_{\max}/J_{\max}$, acceleration hold time $T_a$, constant‑velocity time $T_v$.

---

## 1) Velocity Trapezoid → Position (Derivation)

Consider a **single positive half‑move** (accelerate → constant velocity → decelerate), then mirror for the negative half if you need a full symmetric cycle. For a rest‑to‑rest point‑to‑point move of length $S$, the standard **trapezoidal velocity** is:

### 1.1 Velocity profile $v(t)$
Let $a = V_{\max}/T_r$ (constant acceleration magnitude during the ramp).

$$
v(t)=
\begin{cases}
a\,t, & 0 \le t < T_r \\[4pt]
V_{\max}, & T_r \le t < T_r + T_c \\[4pt]
V_{\max} - a\,(t - T_r - T_c), & T_r + T_c \le t < 2T_r + T_c
\end{cases}
$$

### 1.2 Integrate to get position $x(t)$

We integrate piece‑by‑piece, with $x(0)=0$.

**(i) Acceleration segment $0 \le t < T_r$:**
$$
x(t)=\int_0^t a\,\tau\,d\tau \;=\; \tfrac12 a t^2
$$
At $t=T_r$: $x(T_r)=\tfrac12 V_{\max} T_r$.

**(ii) Constant‑velocity segment $T_r \le t < T_r + T_c$:**
$$
x(t) = x(T_r) + \int_{T_r}^t V_{\max}\,d\tau
= \tfrac12 V_{\max} T_r + V_{\max}(t-T_r)
$$

**(iii) Deceleration segment $T_r + T_c \le t < 2T_r + T_c$:**
$$
v(t) = V_{\max} - a\,(t - T_r - T_c)
$$
$$
x(t)=x(T_r+T_c)+\int_{0}^{t-(T_r+T_c)}\!\!\!\Big[V_{\max}-a\,\tau\Big]\,d\tau
= x(T_r+T_c) + V_{\max}\,\Delta t - \tfrac12 a\,\Delta t^2
$$
with $\Delta t=t-(T_r+T_c)$ and $x(T_r+T_c)=\tfrac12 V_{\max} T_r + V_{\max}T_c$.

### 1.3 Final piece‑wise position (Trapezoid)

$$
\boxed{
x(t)=
\begin{cases}
\dfrac12 a t^2, & 0 \le t < T_r \\[6pt]
\dfrac12 V_{\max} T_r + V_{\max}(t-T_r), & T_r \le t < T_r + T_c \\[6pt]
\dfrac12 V_{\max} T_r + V_{\max}T_c + V_{\max}(t-T_r-T_c) - \dfrac12 a(t-T_r-T_c)^2, & T_r+T_c \le t < 2T_r+T_c
\end{cases}
}
$$
with $a=V_{\max}/T_r$.

> **Distance & time closure**: For a rest‑to‑rest move of length $S$,
> $$
> S = V_{\max}T_c + V_{\max}T_r \quad\text{(since }x(2T_r+T_c)=S\text{)}
> $$
> and $T_r = V_{\max}/A_{\max}$ if you saturate acceleration at $A_{\max}$. If $S$ is too short to reach $V_{\max}$, you get a **triangular velocity** (set $T_c=0$ and solve $S=A_{\max}T_r^2$).

---

## 2) Why Jerk‑Limited (S‑Curve) Profiles?

In the trapezoid, acceleration **steps** at the junctions:
- $a: 0 \to +A_{\max}$, then $+A_{\max}\to 0$, then $0\to -A_{\max}$, etc.
- These steps imply **infinite jerk** $j=\dot a$ at those instants.

**Practical consequences** (especially on real mechanisms and tight controllers):
- Excites resonances and increases vibration/noise.
- Causes overshoot/settling issues, tracking error, and wear.
- Feels uncomfortable in human‑facing systems (elevators, AGVs).

**Jerk‑limited S‑curves** ramp acceleration **linearly** (finite jerk), so:
- $a(t)$ becomes **piecewise linear**.
- $v(t)$ becomes **twice continuously differentiable** (rounded shoulders).
- $x(t)$ becomes **very smooth** (S‑shaped curvature).

---

## 3) Jerk‑Limited S‑Curve (7‑Segment) Derivation

We create a symmetric 7‑segment profile (rest‑to‑rest), with max jerk $J_{\max}$, max accel $A_{\max}$, and optionally reaching $V_{\max}$.

### 3.1 Segment structure and times

Let
$$
T_j = \frac{A_{\max}}{J_{\max}}
$$
be the time to ramp acceleration from 0 to $A_{\max}$ at jerk $J_{\max}$.

The 7 segments:

1. $j=+J_{\max}$ for $T_j$ (accel ramps: $0 \to +A_{\max}$)
2. $j=0$ for $T_a$ (accel hold: $+A_{\max}$)
3. $j=-J_{\max}$ for $T_j$ (accel ramps: $+A_{\max}\to 0$)
4. $j=0$ for $T_v$ (constant velocity plateau)
5. $j=-J_{\max}$ for $T_j$ (accel ramps: $0 \to -A_{\max}$)
6. $j=0$ for $T_a$ (accel hold: $-A_{\max}$)
7. $j=+J_{\max}$ for $T_j$ (accel ramps: $-A_{\max}\to 0$)

### 3.2 Velocity and distance in the acceleration half (segments 1–3)

- **Velocity gain** in the acceleration half:
  $$
  \Delta v_{\text{acc}} \;=\; \frac{A_{\max}^2}{J_{\max}} \;+\; A_{\max}T_a
  $$
  If you enforce a velocity cap $V_{\max}$, set $\Delta v_{\text{acc}}=V_{\max}$ to determine $T_a$:
  $$
  T_a \;=\; \max\!\left(0,\; \frac{V_{\max} - A_{\max}^2/J_{\max}}{A_{\max}}\right).
  $$

- **Distance** in the acceleration half:
  $$
  x_{\text{acc}}
  \;=\;
  \underbrace{\frac{A_{\max}^3}{J_{\max}^2}}_{\text{two jerk ramps}}
  \;+\;
  \underbrace{\frac{3}{2}\frac{A_{\max}^2}{J_{\max}}\,T_a}_{\text{mixed}}
  \;+\;
  \underbrace{\frac{1}{2}A_{\max}T_a^2}_{\text{accel hold}}
  $$

- For a long enough move (with plateau):
  $$
  T_v = \frac{S - 2x_{\text{acc}}}{V_{\max}} \quad (\text{must be } \ge 0).
  $$

- **Short‑move regimes**:
  - If $T_v<0$: set $T_v=0$ and solve $x_{\text{acc}}(T_a)=S/2$ for $T_a$.
  - If that yields $T_a<0$: use **jerk‑only triangular S‑curve** (no acceleration nor velocity holds).
    In that case, with equal jerk up/down times $T_j'$,
    $$
    S \;=\; 2\,J_{\max}\,T_j'^3,\qquad
    V_{\text{peak}} = J_{\max} T_j'^2.
    $$

### 3.3 Piece‑wise formulas (ready to integrate)

Below, write each segment with its own local time origin and concatenate using the previous segment’s terminal $(x,v,a)$ as initial conditions. This is exactly how you’ll implement it on a microcontroller (update laws are simple polynomials).

Let $(x_0, v_0, a_0)$ be the state at the start of a segment and $\tau\in[0,\Delta T]$ the local segment time.

- **Jerk segment** with constant jerk $j$:
  $$
  a(\tau) = a_0 + j\,\tau,\quad
  v(\tau) = v_0 + a_0\,\tau + \tfrac12 j\,\tau^2,\quad
  x(\tau) = x_0 + v_0\,\tau + \tfrac12 a_0\,\tau^2 + \tfrac16 j\,\tau^3.
  $$

- **Acceleration hold** (constant $a=a_h$, i.e., $j=0$):
  $$
  a(\tau)=a_h,\quad
  v(\tau)=v_0 + a_h\,\tau,\quad
  x(\tau)=x_0 + v_0\,\tau + \tfrac12 a_h\,\tau^2.
  $$

- **Velocity hold** (constant $v=v_h$, i.e., $a=0, j=0$):
  $$
  a(\tau)=0,\quad
  v(\tau)=v_h,\quad
  x(\tau)=x_0 + v_h\,\tau.
  $$

**Assemble the 7 segments** (symbolically):

1) $j=+J_{\max}$, $\Delta T=T_j$, init $(x_0,v_0,a_0)=(0,0,0)$.  
2) $j=0$, $a=+A_{\max}$, $\Delta T=T_a$, init = end of seg. 1.  
3) $j=-J_{\max}$, $\Delta T=T_j$, init = end of seg. 2 (returns $a$ to 0).  
4) $j=0$, $a=0$, $\Delta T=T_v$, init = end of seg. 3 (constant $v$).  
5) $j=-J_{\max}$, $\Delta T=T_j$, init = end of seg. 4 (accel goes to $-A_{\max}$).  
6) $j=0$, $a=-A_{\max}$, $\Delta T=T_a$, init = end of seg. 5.  
7) $j=+J_{\max}$, $\Delta T=T_j$, init = end of seg. 6 (accel returns to 0, $v\to 0$, stop at $x=S$).

Because each segment is a **cubic** (or lower) polynomial in local time $\tau$, you can compute $x, v, a$ with a few multiply‑accumulate operations per tick (perfect for microcontrollers).

---

## 4) Final “Firmware‑ready” Summary

### 4.1 Trapezoidal profile (inputs: $S, V_{\max}, A_{\max}$)

- Compute:
  - If $S \ge V_{\max}^2/A_{\max}$: 3‑segment trapezoid with
    $$
    T_r=\frac{V_{\max}}{A_{\max}},\qquad
    T_c=\frac{S - V_{\max}T_r}{V_{\max}}.
    $$
  - Else: **triangular** (no $T_c$):
    $$
    T_r=\sqrt{\frac{S}{A_{\max}}},\qquad T_c=0,\qquad V_{\text{peak}}=\sqrt{A_{\max}S}.
    $$

- **Position piece‑wise** (for the accelerate → cruise → decelerate phases):
$$
x(t)=
\begin{cases}
\dfrac12 a t^2, & 0 \le t < T_r \\[6pt]
\dfrac12 V_{\max} T_r + V_{\max}(t-T_r), & T_r \le t < T_r + T_c \\[6pt]
\dfrac12 V_{\max} T_r + V_{\max}T_c + V_{\max}(t-T_r-T_c) - \dfrac12 a(t-T_r-T_c)^2, & T_r+T_c \le t < 2T_r+T_c
\end{cases}
$$
with $a=V_{\max}/T_r$.

**Discrete‑time implementation tip:** Use the acceleration setpoint $a_k\in\{+A_{\max},0,-A_{\max}\}$ per phase, then
$$
v_{k+1}=v_k + a_k\,\Delta t,\qquad
x_{k+1}=x_k + v_k\,\Delta t.
$$

---

### 4.2 Jerk‑limited S‑curve (inputs: $S, V_{\max}, A_{\max}, J_{\max}$)

- Compute basic times:
  $$
  T_j = \frac{A_{\max}}{J_{\max}},\qquad
  T_a = \max\!\left(0,\; \frac{V_{\max} - A_{\max}^2/J_{\max}}{A_{\max}}\right).
  $$

- Distance covered in acceleration half:
  $$
  x_{\text{acc}} = \frac{A_{\max}^3}{J_{\max}^2}
   + \frac{3}{2}\frac{A_{\max}^2}{J_{\max}}\,T_a
   + \frac{1}{2}A_{\max}T_a^2.
  $$

- Plateau time (if long enough move):
  $$
  T_v = \frac{S - 2x_{\text{acc}}}{V_{\max}} \quad (\ge 0).
  $$

- **Short‑move handling**:
  - If $T_v<0$: set $T_v=0$ and solve $x_{\text{acc}}(T_a)=S/2$ for $T_a$.
  - If that gives $T_a<0$: use jerk‑only triangular S‑curve with $S=2J_{\max}T_j'^3$.

- **Piece‑wise (per segment) update laws** for discrete time (local time $\tau$ within each segment):

  **Jerk segments**:
  $$
  \boxed{
  \begin{aligned}
  a &\leftarrow a + j\,\Delta t \\
  v &\leftarrow v + a\,\Delta t + \tfrac12 j\,(\Delta t)^2 \\
  x &\leftarrow x + v\,\Delta t + \tfrac12 a\,(\Delta t)^2 + \tfrac16 j\,(\Delta t)^3
  \end{aligned}}
  \quad \text{(with constant } j=\pm J_{\max})
  $$

  **Accel holds** $(j=0,\ a=\pm A_{\max})$:
  $$
  \boxed{
  \begin{aligned}
  v &\leftarrow v + a\,\Delta t \\
  x &\leftarrow x + v\,\Delta t + \tfrac12 a\,(\Delta t)^2
  \end{aligned}}
  $$

  **Velocity holds** $(a=0)$:
  $$
  \boxed{x \leftarrow x + v\,\Delta t}
  $$

- **Segment schedule** (durations $\{T_j, T_a, T_j, T_v, T_j, T_a, T_j\}$) with jerks:
  $$
  \{+J_{\max},\, 0,\, -J_{\max},\, 0,\, -J_{\max},\, 0,\, +J_{\max}\}
  $$
  Initialize $(x,v,a)=(0,0,0)$ and march through the segments. The end state is $(x=S, v=0, a=0)$ by construction.

---

## 5) Practical Firmware Notes

1. **Fixed‑point vs float:** If using fixed‑point, pre‑scale $A_{\max}, J_{\max}, \Delta t$ to keep headroom for the cubic term $\tfrac16 j\,(\Delta t)^3$.  
2. **Time tracking:** Use an integer tick counter inside each segment; when the local duration elapses, switch to the next segment and re‑seed $(x_0,v_0,a_0)$.  
3. **Clamping:** Small numerical drift can accumulate; clamp $a$ and $v$ to their targets at segment boundaries.  
4. **Feedforward + feedback:** These profiles are typically **feedforward setpoints**; pair with a PID/PI or state‑space loop for disturbance rejection.  
5. **Short‑move logic:** Implement the two short‑move fallbacks (no $T_v$, and jerk‑only triangle) to guarantee feasibility for any $S$.

---

## 6) Quick Checklist (What to compute before running)

- **Trapezoid:** Decide if you’re triangular or trapezoidal, then compute $(T_r, T_c)$.  
- **S‑curve:** Compute $T_j$, then $T_a$ from $V_{\max}$, compute $x_{\text{acc}}$ and $T_v$. If $T_v<0$, solve reduced cases.

---

## 7) What to expect in the signals

- **Trapezoid**  
  - $a(t)$: step‑like (0 → $\pm A_{\max}$ → 0 → $\mp A_{\max}$ → 0)  
  - $v(t)$: straight lines and flats, sharp corners  
  - $x(t)$: parabola → line → parabola

- **Jerk‑limited S‑curve**  
  - $a(t)$: **triangular/hat‑shaped**, piecewise linear  
  - $v(t)$: **rounded shoulders**, $C^2$ continuous  
  - $x(t)$: **smooth S‑shape**, $C^3$ continuous

---

## 8) Minimal Example Parameters (for test & tuning)

- $S=1.0\ \text{m}$  
- $V_{\max}=0.6\ \text{m/s}$, $A_{\max}=1.0\ \text{m/s}^2$, $J_{\max}=4.0\ \text{m/s}^3$  
- From these: $T_j=A_{\max}/J_{\max}=0.25\ \text{s}$.  
- If long enough: $T_a=\max\!\bigl(0,\,(V_{\max}-A_{\max}^2/J_{\max})/A_{\max}\bigr)$, then $x_{\text{acc}}$ and $T_v$.

Use these to verify your firmware’s segment switch‑over and endpoint accuracy.

---

### End of document