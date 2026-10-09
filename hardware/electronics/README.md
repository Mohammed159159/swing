# Electronics, Wiring & Power Distribution (`hardware/electronics/`)

This directory houses circuit schematics, wiring harness pinouts, motor driver connections, and the project Bill of Materials (BOM).

---

## 🎯 Electrical Architecture

```text
               ┌───────────────────────────┐
               │    DC Power Supply        │ (12V / 24V, 5A regulated)
               └──────┬─────────────┬──────┘
                      │             │
                      ▼             ▼
  ┌───────────────────────┐     ┌───────────────────────┐
  │   Motor Driver        │     │  Microcontroller      │
  │   (Cytron / VNH5019)  │◄────┤  (STM32 / ESP32 / Due)│
  └───────────┬───────────┘ PWM └───────────┬───────────┘
              │ Dir                         ▲
              ▼                             │
  ┌───────────────────────┐                 │ QEI A/B
  │   DC Actuator Motor   │                 │ Interrupts
  │   + Optical Encoder 0 ├─────────────────┤
  └───────────────────────┘                 │
                                            │ QEI A/B
  ┌───────────────────────┐                 │
  │   Pendulum Pivot      │                 │
  │   Optical Encoder 1   ├─────────────────┘
  └───────────────────────┘
```

---

## 📁 Key Documents & Artifacts

1. **Schematics & Pinouts:**
   - Detailed wiring schematic linking microcontroller GPIOs to motor driver inputs (`PWM`, `DIR`, `EN`, `CURRENT_SENSE`).
   - Pin mapping for hardware timer quadrature decoder channels (`TIM_CHANNEL_1`, `TIM_CHANNEL_2`).
2. **Emergency Stop Circuit:**
   - Physical hardware emergency cutoff switch breaking actuator power directly.
3. **Bill of Materials (`BOM.md` / `BOM.xlsx`):**
   - Detailed part list with component model numbers, supplier links, unit costs, and technical ratings.
