# Classical Control Systems (`control/controllers/classical/`)

This directory houses classical control syntheses for the Furuta Rotary Inverted Pendulum.

---

## 📁 Subsystem Structure

```text
control/controllers/classical/
├── pid/                # Dual-loop PID balancing controller with derivative filtering
└── pole_placement/     # Full-State Feedback Pole Placement (Ackermann formula)
```

---

## 🎯 Technical Scope

- **PID Control:** Establishes the baseline classical benchmark. Stabilizes pendulum angle $\theta_1$ around vertical zero while applying secondary regulation to prevent horizontal arm drift $\theta_0$.
- **Full-State Feedback Pole Placement:** Uses the linearized 4th-order state-space model ($A, B$). Places closed-loop poles at desired locations using Ackermann's formula to guarantee target damping ratio $\zeta$ and natural frequency $\omega_n$.
- **Primary Owner:** Pair 2 (Control Syntheses).
