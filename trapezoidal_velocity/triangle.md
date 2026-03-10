# C1 and C2 Corner Smoothing for High-Duty-Cycle Constant-Velocity Waveforms
### (Triangle-Position -> Trapezoid-Velocity Systems)

This file specifies two motion-profile constructions for embedded or real-time generation when you want **~90% constant-velocity duty cycle** with smooth transitions:

1) **C1 Parabolic-Corner Profile — Baseline**
   - Guarantees **continuous position (C0)** and **continuous velocity (C1)**.
   - **Acceleration finite** but piecewise constant (jerk discontinuous at cap boundaries).
   - Achieves **about 90% truly constant velocity** per period with minimal compute load.

2) **C2 Jerk-Limited Corner Microprofile — Improvement**
   - Extends the C1 caps to achieve **continuous acceleration (C2)** via a 2-phase jerk microprofile.
   - Preserves the **same 90% constant-velocity duty cycle**.
   - Avoids acceleration steps; jerk is finite inside the caps.

Target use cases: scanning stages, dithering actuators, beam steering, and any periodic motion that benefits from long constant-velocity segments and controlled corner dynamics.

---

## 0) Symbols and Setup

- $A$: position amplitude (peak)
- $T_0$: period
- $F$: desired constant-velocity duty fraction (for example, $F=0.90$)
- $t_c$: $T_0/2$ (time of the peak)
- $s$: speed magnitude on flat segments (slope of position)
- $\Delta t$: half-width of each smoothing cap (per corner)
- $T_j$: jerk ramp duration per ramp (used in C2)
- $T_a$: constant-accel hold duration inside corner (used in C2; often 0)
- $J$: jerk magnitude $|j|$ (used in C2)

We construct a triangle-like position wave ($\pm A$, period $T_0$) with smoothed corners so that:

- **C1**: position and velocity are continuous; acceleration is finite but piecewise constant.
- **C2**: position, velocity, and acceleration are continuous; jerk is finite.

The **velocity is perfectly constant** ($\pm s$) for about **$F \cdot 100\%$** of the cycle (for example, 90%).

---

## 1) C1 Parabolic-Corner Smoothing (Baseline)

### 1.1 Design goals
- Maximize constant-velocity time (about $F \cdot T_0$).
- Enforce **C0 (position)** and **C1 (velocity)** continuity everywhere.
- Keep implementation simple (few multiplies per tick; easy on MCUs).

### 1.2 Pick the cap width to get the flat fraction

Each period has **four parabolic caps** (two peaks + two valleys), each with half-width $\Delta t$. The total non-flat time per period is $4\Delta t$, leaving flat time $T_0 - 4\Delta t$.

Choose $\Delta t$ from the desired flat fraction $F$:

$$
\Delta t = \frac{(1 - F)T_0}{4}
$$

For $F=0.90$, $\Delta t = 0.025\,T_0$.

### 1.3 Adjust the slope to preserve amplitude $A$

The linear spans are shorter because of the caps, so increase the linear slope from $4A/T_0$ to:

$$
 s = \frac{4A}{T_0 - 2\Delta t}
$$

This ensures the peak still reaches $+A$ and the valley still reaches $-A$ after inserting the caps.

### 1.4 One-period piecewise position (C1)

Let $t_c = T_0/2$. Define five segments on $t \in [0, T_0)$:

1) **Valley cap near 0**, $t \in [0, \Delta t)$

$$
 x(t) = -A + \frac{s}{2\Delta t} t^2
$$

2) **Rising linear**, $t \in [\Delta t, t_c - \Delta t)$

$$
 x(t) = x_{\text{val\_end}} + s(t - \Delta t)
$$

$$
 x_{\text{val\_end}} = -A + \frac{1}{2}s\Delta t
$$

3) **Peak cap near $t_c$**, $t \in [t_c - \Delta t, t_c + \Delta t)$

$$
 x(t) = A - \frac{s}{2\Delta t}(t - t_c)^2
$$

4) **Falling linear**, $t \in [t_c + \Delta t, T_0 - \Delta t)$

$$
 x(t) = x_{\text{peak\_end}} - s\bigl(t - (t_c + \Delta t)\bigr)
$$

$$
 x_{\text{peak\_end}} = A - \frac{1}{2}s\Delta t
$$

5) **Valley cap near $T_0$**, $t \in [T_0 - \Delta t, T_0)$

$$
 x(t) = -A + \frac{s}{2\Delta t}(t - T_0)^2
$$

This construction guarantees **C0 and C1** continuity at all joins. The **velocity** is exactly $\pm s$ on the linear spans (the flats), and during caps it changes linearly with time (constant acceleration inside each cap). The **acceleration** is $\pm s/\Delta t$ inside caps and zero on flats; jerk is impulsive at the transitions into or out of caps.

**What you get**
- Position: rounded triangle (C1).
- Velocity: trapezoid with flat tops and bottoms and linear ramps only inside caps.
- Acceleration: square-like bursts (constant in caps, zero on flats).
- Flat fraction: about $F$.

---

## 2) C2 Jerk-Limited Corner Microprofile (Improvement)

### 2.1 Motivation
If your mechanism is sensitive to acceleration steps, upgrade to a **C2 corner**:

- Acceleration is **continuous** (no steps), with finite jerk.
- Keep the **same flat fraction** $F$ (for example, 90%) by fixing the corner time and adjusting the slope $s$ and jerk $J$ accordingly.

### 2.2 Corner microprofile (jerk-only, $T_a = 0$ typical for high flat ratio)

At each corner (velocity reversal from $-s$ to $+s$ or vice versa), use a symmetric **triangular-jerk** profile:

- Phase 1: jerk $= +J$, duration $T_j$ (accel ramps $0 \to +A_{\text{acc}}$)
- Phase 2: jerk $= -J$, duration $T_j$ (accel ramps $+A_{\text{acc}} \to 0$)

This yields a smooth S-curve in velocity and continuous acceleration within the corner. The net velocity change per corner is:

$$
\Delta v_{\text{corner}} = J T_j^2
$$

To execute a full reversal from $-s$ to $+s$:

$$
\Delta v_{\text{corner}} = 2s \quad \Rightarrow \quad J T_j^2 = 2s \quad \Rightarrow \quad J = \frac{2s}{T_j^2}
$$

### 2.3 Keep at least 90% of the cycle flat

Per half-period (valley -> peak), budget the time:

$$
T_{\text{half}} = \frac{T_0}{2}
$$

$$
T_{\text{lin,half}} = F\frac{T_0}{2}
$$

$$
T_{\text{corner,half}} = T_{\text{half}} - T_{\text{lin,half}} = (1 - F)\frac{T_0}{2}
$$

With jerk-only corners ($T_a = 0$), two ramps back-to-back give:

$$
T_j = \frac{T_{\text{corner,half}}}{2} = \frac{(1 - F)T_0}{4}
$$

This enforces the corner duration while preserving the flat length. Solve for $s$ (for example, by a simple 1D bisection) so that the position change from valley to peak equals $2A$ given the fixed times. Finally, compute the required jerk:

$$
J = \frac{2s}{T_j^2}
$$

If your actuator has a jerk limit $J_{\max}$, you can invert the relation to choose $T_j$ and or relax $F$ slightly to meet $J_{\max}$.

### 2.4 Properties (C2)

- **C0/C1/C2** continuity across the entire period.
- **Velocity plateau** unchanged: flat fraction still about $F$.
- **Acceleration** is continuous, peaking within corners and returning smoothly to zero on flats.
- **Jerk** is finite and constant in each jerk phase ($\pm J$).
- Implementation: slightly more math per tick (integrate jerk -> accel -> vel -> pos), but still lightweight for MCUs.

---

## 3) Quick Comparison

| Feature | C1 Parabolic Caps (Baseline) | C2 Jerk-Limited Caps (Improvement) |
| --- | --- | --- |
| Continuity | C0 + C1 | C0 + C1 + C2 |
| Velocity plateau length | >= 90% (exact by design) | >= 90% (exact by design) |
| Acceleration | Piecewise constant (in caps) | Continuous (ramps via jerk) |
| Jerk | Impulses at cap edges | Finite, bounded ($\pm J$) |
| Complexity (MCU) | Very low (few multiplies) | Moderate (jerk -> accel -> vel -> pos) |
| Good for | Long flats; simple implementation | Smooth mechanics; resonance-aware |

---

## 4) Parameter Checklist (How to choose numbers)

### If you only need C1

Given $A$, $T_0$, and desired flat fraction $F$ (for example, 0.90):

$$
\Delta t = \frac{(1 - F)T_0}{4}
$$

$$
 s = \frac{4A}{T_0 - 2\Delta t}
$$

Caps: parabolas with accel magnitude $s/\Delta t$.

Flats: velocity $= \pm s$ (constant).

### If you want C2 (and keep $F$)

Given $A$, $T_0$, $F$, choose:

$$
T_j = \frac{(1 - F)T_0}{4}
$$

Solve slope $s$ so valley -> peak area $= 2A$ with:

- Corner time per half: $2T_j$ (jerk-only; $T_a = 0$)
- Flat time per half: $T_{\text{lin,half}} = F T_0/2$

Then compute jerk:

$$
J = \frac{2s}{T_j^2}
$$

If $J$ exceeds your actuator limit, increase $T_j$ slightly or reduce $F$ a bit.

---

## 5) Implementation Notes (MCUs)

- **Segment scheduler:** keep a local timer per segment (cap or flat). When it expires, advance to the next segment and re-seed state $(x_0, v_0, a_0)$ for the local polynomial or jerk integration.
- **C1 per-tick updates:**
  - Flats: $v = \pm s$, $a = 0$, $x += v\,dt$
  - Caps (parabola): $a = \pm (s/\Delta t)$, $v += a\,dt$, $x += v\,dt$
- **C2 per-tick updates (jerk-only caps):**
  - Phase 1: $a += (+J)\,dt$, $v += a\,dt$, $x += v\,dt$
  - Phase 2: $a += (-J)\,dt$, $v += a\,dt$, $x += v\,dt$
  - Flats: $a = 0$, $v = \pm s$, $x += v\,dt$
- **Numerical hygiene:** clamp tiny residuals at segment boundaries ($a \approx 0$, $v \approx \pm s$) to avoid drift across cycles.
- **Fixed-point:** pre-scale $s$, $\Delta t$, $T_j$, $J$ so cubic terms remain in range (C2 caps include $j\,dt^3/6$ in exact updates; simple cascaded Euler is fine at high sample rates).

---

## 6) Minimal Example Targets

Given:

- $A = 1.0$
- $T_0 = 2.0\ \text{s}$
- $F = 0.90$ (flats take 90% of the cycle)

C1:

$$
\Delta t = \frac{(1 - 0.90)T_0}{4} = 0.05\ \text{s}
$$

$$
 s = \frac{4A}{T_0 - 2\Delta t} = \frac{4.0}{2.0 - 0.10} \approx 2.1053
$$

$$
 a_{\text{cap}} = \frac{s}{\Delta t} \approx 42.105\ \text{(units/s}^2\text{)}
$$

C2:

$$
T_j = \frac{(1 - 0.90)T_0}{4} = 0.05\ \text{s}
$$

Corner per half: $2T_j = 0.10\ \text{s}$. Solve $s$ so valley -> peak area equals $2A$ (simple 1D bisection). Then:

$$
J = \frac{2s}{T_j^2}
$$

Check against your actuator jerk limit; adjust $T_j$ or $F$ if needed.

---

## 7) Summary

- **C1** is your **baseline**: use it when your top priority is maximizing constant-velocity time (about 90%) with the **simplest** implementation and **C0/C1** continuity.
- **C2** is your **upgrade path**: use it when your plant is sensitive to acceleration steps and you want **C2 continuity** while **preserving the same flat fraction**. Tune $T_j$ (and thus $J$) to meet actuator limits; keep $F$ at 0.90 if feasible.

---

End of document.
