# Controllers Directory (`control/controllers/`)

This directory organizes all controller synthesis scripts and Simulink blocks across multiple control paradigms for stabilizing and swinging up the Furuta Rotary Inverted Pendulum.

---

## 📁 Subsystem Structure

```text
control/controllers/
├── open-loop/      # Unforced decay simulations & open-loop step response models
├── classical/      # Classical feedback control architectures (PID, Pole Placement)
├── optimal/        # Optimal feedback control architectures (LQR)
└── rl/             # Shadow Track Deep Reinforcement Learning agents & TinyML headers
```

---

## 📋 Control Paradigms & Requirements

1. **Course Mandate:**
   - As per course requirements, **PID**, **Pole Placement**, and **LQR** must all be implemented, simulated, and benchmarked.
   - Omission of LQR or Pole Placement incurs severe point deductions.
2. **State Conventions:**
   - State vector: $\mathbf{x} = [\theta_0, \theta_1, \dot{\theta}_0, \dot{\theta}_1]^T$
   - Target upright equilibrium: $\mathbf{x}_e = [0, 0, 0, 0]^T$
   - Control input: $u = V_m$ (armature voltage $[\text{V}]$) with saturation at $\pm V_{\max}$.
