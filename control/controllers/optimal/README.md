# Optimal Control Systems (`control/controllers/optimal/`)

This directory houses optimal control architectures for the Furuta Rotary Inverted Pendulum.

---

## 📁 Subsystem Structure

```text
control/controllers/optimal/
└── lqr/            # Linear Quadratic Regulator (LQR) tuned via Bryson's rule
```

---

## 🎯 Technical Scope

- **Linear Quadratic Regulator (LQR):** Provides the core state-feedback balancing controller mandated by the course specification.
- Formulates a rigorous quadratic trade-off between state regulation error and actuator energy consumption ($u^2$).
- Solves the Continuous-Time Algebraic Riccati Equation (ARE) to guarantee infinite gain margin and at least $60^\circ$ phase margin.
- **Primary Owner:** Pair 2 (Control Syntheses).
