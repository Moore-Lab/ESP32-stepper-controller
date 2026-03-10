# Project Goals & Motion Logic

## Primary Objective
Develop a firmware capable of driving the linear stage using arbitrary mathematical functions where position $x$ is a function of time $t$: $x = f(t)$.

## Target Motion Profiles
1. **Sine Wave Oscillation:** - Range: $\pm 0.5\text{mm}$ (1mm total travel).
   - Speed: $1\text{mm/s}$ to $10\text{mm/s}$.
2. **Custom Functions:** The ability to swap the sine function for polynomial or trapezoidal profiles later.

## Accuracy Requirements
- **Dual-Loop Verification:** Use the linear encoder to verify the physical position of the stage against the "Step" command sent to the CL57T.
- **Backlash Compensation:** The software must use the linear encoder as the "Source of Truth" to correct for any mechanical play during direction changes.

## Safety Constraints
- **Alarm Monitoring:** If the CL57T "Fault" pin triggers, immediately kill all PWM signals.
- **Soft Limits:** Software must prevent travel beyond the 0-300mm physical range of the scale.