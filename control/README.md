# Control Systems Synthesis & Supervisory Logic (`control/`)

This directory contains all control law implementations, state feedback syntheses, supervisory hybrid logic, and quantitative performance validation routines for the Furuta Rotary Inverted Pendulum.

---

## 📁 Subdirectory Breakdown

```text
control/
├── controllers/    # Control algorithms across open-loop, classical, optimal, and RL paradigms
│   ├── open-loop/
│   ├── classical/  # PID and Full-State Feedback Pole Placement
│   ├── optimal/    # Linear Quadratic Regulator (LQR)
│   └── rl/         # Shadow Track Gymnasium environment & TinyML bare-metal export
├── stateflow/      # Hybrid supervisory Stateflow chart (SAFE_STOP, SWING_UP, STABILIZE)
└── evaluation/     # Automated stability proofs & performance metric calculators (IAE, ISE, ITAE)
```

---

## 🎯 Dual-Track Engineering Strategy Alignment

- **Primary Track (Model-Based Control):**
  - Linearization around the upright unstable equilibrium.
  - Three balancing controllers (PID, Pole Placement, LQR) + Åström-Furuta energy swing-up law.
  - Discrete-continuous coordination managed by Stateflow with $10^\circ$ hysteresis guards.
- **Shadow Track (Deep RL Benchmark):**
  - MuJoCo/Gymnasium SAC & PPO training.
  - Sim-to-real transfer with TinyML distillation to C++ microcontroller code.
- **Primary Owners:** Pair 2 (Control Syntheses) with cross-review by Pair 1 (Modeling) and Pair 4 (Benchmarking).
