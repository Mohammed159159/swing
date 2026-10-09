# Firmware Source Code (`hardware/firmware/src/`)

This directory contains the core C/C++ source code implementing real-time control, sensing, and communication tasks.

---

## 📁 Source Modules Breakdown

- `main.cpp`: System initialization, SysTick timer loop configuration, and background task dispatcher.
- `encoder.cpp`: Hardware quadrature encoder initialization and register read routines for both arm and pendulum encoders.
- `motor.cpp`: High-frequency PWM generator, direction GPIO pin controller, and emergency clamp logic.
- `controller.cpp`: Discrete-time controller implementations (PID with derivative filter, LQR state feedback vector multiplication, and Åström-Furuta swing-up pump).
- `supervisor.cpp`: Embedded supervisory finite state machine implementing `SAFE_STOP`, `SWING_UP`, and `STABILIZE` state transitions with hysteresis.
- `telemetry.cpp`: Binary serial packet packing and transmission over UART.
