# Hardware Wiring & Integration Guide

This document details the electrical connections between the ESP32-S3, the CL57T Closed-Loop Driver, and the JCS900 Linear Scale.

## 1. Power Distribution
| Component | Voltage | Source |
| :--- | :--- | :--- |
| **CL57T Driver** | 24V - 48V DC | Industrial Switching Power Supply |
| **Nema 23 Motor** | Powered by Driver | Connected via A+/A- B+/B- terminals |
| **ESP32-S3** | 5V DC | USB-C or 5V rail from Power Supply |
| **Linear Scale** | 5V DC | From ESP32 5V pin or external 5V |

---

## 2. Motor to Driver (The "Inner Loop")
The CL57T handles the integrated rotary encoder directly. 
1. **Motor Phases:** Connect A+, A-, B+, B- from the motor to the driver terminals.
2. **Integrated Encoder:** Plug the 6-pin encoder cable from the motor into the **Encoder Port** on the CL57T.
   - *Note: This feedback loop is handled internally by the driver. No connection to the ESP32 is required for this loop.*

---

## 3. ESP32-S3 to CL57T (Control Signals)
The CL57T uses opto-isolated inputs. While it can sometimes trigger at 3.3V, **5V logic is highly recommended** for high-speed sine waves to prevent missed pulses.

| CL57T Pin | ESP32-S3 Pin | Note |
| :--- | :--- | :--- |
| **PUL+ (+5V)** | 5V Rail | Connect to 5V (Common Anode configuration) |
| **PUL- (Step)** | GPIO 12 | Via Level Shifter (to pull to GND) |
| **DIR+ (+5V)** | 5V Rail | Connect to 5V |
| **DIR- (Dir)** | GPIO 13 | Via Level Shifter |
| **ENA+** | (Optional) | Leave disconnected to keep motor always enabled |
| **ALM+** | GPIO 14 | Fault signal (Use 3.3V pull-up on ESP32 side) |
| **ALM-** | GND | Common Ground |

---

## 4. Linear Scale (JCS900) to ESP32-S3
The JCS900 typically uses a DB9 connector and outputs **5V TTL signals**. These **MUST** be stepped down to **3.3V** before entering the ESP32 to avoid damaging the chip.

**DB9 Pinout (Standard):**
- **Pin 7:** +5V (Power for the scale)
- **Pin 8:** 0V (GND)
- **Pin 2:** Channel A (Signal) -> **Level Shifter** -> ESP32 GPIO 10
- **Pin 1:** Channel B (Signal) -> **Level Shifter** -> ESP32 GPIO 11

> **Critical:** Use a high-speed logic level shifter (like a TXS0108E) rather than a simple voltage divider to ensure the 5um pulses aren't "rounded off" at higher speeds.

---

## 5. Wiring Diagram Summary
1. **Common Ground:** Ensure the GND of the 24V supply, the 5V supply, and the ESP32 are all tied together.
2. **Shielding:** The linear scale and motor cables are sensitive to EMI. Keep the ESP32 and logic wiring away from the 24V motor lines. Ground the cable shields to a single point (Star Ground).
3. **Logic Level Shifter:**
   - **HV Side (5V):** Connected to Scale A/B and Driver PUL/DIR.
   - **LV Side (3.3V):** Connected to ESP32 GPIOs.

---

## 6. Checklist Before Power-On
- [ ] Verify 24V polarity on CL57T (+ and -).
- [ ] Ensure Linear Scale is receiving exactly 5V.
- [ ] Check that Level Shifters are correctly separating 5V and 3.3V rails.
- [ ] Confirm Step/Dir Dip Switches on CL57T match the `STEPS_PER_REV` in code (e.g., 1600).