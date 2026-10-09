# Embedded Firmware Architecture (`hardware/firmware/`)

This directory houses the real-time C/C++ embedded firmware running on the target microcontroller (STM32 / Arduino Due / TI C2000 / ESP32).

---

## 📁 Subdirectory Breakdown

```text
hardware/firmware/
├── src/        # C/C++ source implementations (main loop, ISRs, QEI, PWM drivers)
├── include/    # Header files, pin definitions, and controller gain parameters
└── protocol/   # Serial binary telemetry packet specification and serializers
```

---

## 🎯 Deterministic Real-Time Loop Requirements

1. **Hardware Quadrature Encoder Interface (QEI):**
   - Configured via timer hardware registers to count optical encoder pulses in $4\times$ quadrature mode without software interrupt overhead.
2. **Periodic Control Loop Timer:**
   - Deterministic SysTick or hardware timer interrupt firing at $500\text{ Hz}$ ($T_s = 2.0\text{ ms}$).
3. **Velocity Estimation:**
   - First-order backward difference with low-pass filtering:
     $$\dot{\theta}[k] = \frac{\theta[k] - \theta[k-1]}{T_s}, \quad \dot{\theta}_{\text{filt}}[k] = \alpha \dot{\theta}_{\text{filt}}[k-1] + (1-\alpha) \dot{\theta}[k]$$
   - Or a second-order Levant robust exact differentiator.
4. **PWM Generation:**
   - Hardware PWM timer driving H-bridge at $\ge 20\text{ kHz}$ to eliminate audible acoustic whine and reduce current ripple.
5. **High-Speed Serial Telemetry:**
   - UART streaming at $\ge 115200\text{ baud}$ (recommended $460800\text{ baud}$) transmitting binary packets to MATLAB.
