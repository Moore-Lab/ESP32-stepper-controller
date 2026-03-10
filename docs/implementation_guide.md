# Software Architecture & Implementation Guide

## 1. Development Environment
- **Platform:** PlatformIO within VS Code.
- **Framework:** Arduino / ESP-IDF.

## 2. Library Recommendations
- **Encoder Reading:** `ESP32Encoder` (Uses hardware PCNT to count pulses from the JCS900 without CPU overhead).
- **Pulse Generation:** `FastAccelStepper` (Provides high-resolution, jitter-free Step pulses using ESP32 hardware timers).
- **Math:** Standard `<cmath>` for sine and trigonometric functions.

## 3. Software Logic Design
### Core 0: Motion Control Loop (The "Real-Time" Core)
- High-frequency interrupt or task (e.g., 1kHz).
- Calculate $Target\_Position = A \sin(\omega t)$.
- Send `moveTo()` command to the stepper driver.
- Read Linear Encoder and compare $Actual\_Position$ to $Target\_Position$.
- Adjust "Step" output if a deviation ($> 10\mu\text{m}$) is detected.

### Core 1: Communication & Telemetry
- Stream $t$, $Target\_X$, and $Actual\_X$ to Serial/USB for plotting.
- Monitor for User Input (Start/Stop/Change Frequency).

## 4. Key Calculation Formulas
- **Steps to move $D$ mm:** $Steps = D \times (\frac{Steps\_Per\_Rev}{Lead})$.
- **Linear Encoder mm:** $Distance = \frac{Encoder\_Counts}{200}$.
- **Target Frequency:** For a $10\text{mm/s}$ peak velocity in a sine wave $x(t) = A \sin(2\pi f t)$, the max velocity is $v_{max} = A \cdot 2\pi f$. Ensure $f$ is set accordingly.