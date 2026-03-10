# Hardware Specifications: Linear Stage System

## 1. Controller: ESP32-S3
- **Logic Level:** 3.3V (Note: Driver requires 5V; use level shifters for Step/Dir).
- **Key Feature:** Hardware Pulse Counter (PCNT) for linear encoder tracking.
- **Connection:** Native USB for high-speed serial data logging.

## 2. Drive System
- **Motor:** Nema 23 Stepper with Integrated Rotary Encoder.
- **Driver:** STEPPERONLINE CL57T (Closed-loop).
  - **Interface:** Pulse/Direction (Step/Dir).
  - **Feedback:** Internal rotary loop (handled by driver, invisible to ESP32).
  - **Alarm Output:** Configured to GPIO to stop motion on stall.
- **Mechanical:** 1605 Ball Screw.
  - **Diameter:** 16mm.
  - **Lead (Pitch):** 5mm per revolution.
  - **Calculated Steps/mm:** (Steps per Rev / 5). *Example: At 1600 microsteps, 320 pulses = 1mm.*

## 3. Feedback: Linear Scale Encoder
- **Model:** JCS900 series glass scale.
- **Output:** TTL Quadrature (A/B phase).
- **Resolution:** 5μm ($0.005\text{mm}$).
- **Pulses per mm:** 200 counts per mm.
- **Voltage:** 5V (Requires level shifting down to 3.3V for ESP32 inputs).