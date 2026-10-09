# Controller Evaluation & Benchmarking (`control/evaluation/`)

This directory houses automated analysis, verification, and benchmarking scripts to quantitatively evaluate all Furuta pendulum control strategies.

---

## 📁 Subsystem Structure

```text
control/evaluation/
├── stability/      # Analytical proofs: eigenvalues, controllability, observability, Bode margins
└── performance/    # Quantitative benchmark calculators (IAE, ISE, ITAE, settling time, u_RMS)
```

---

## 🎯 Verification Mandate

In strict accordance with the course requirements:
1. Every candidate controller (PID vs. Pole Placement vs. LQR vs. RL) must be evaluated against identical benchmark scenarios.
2. Stability and margin analyses must be generated programmatically and exported directly for inclusion in the final technical report.
3. Automated test harnesses must evaluate repeatability across at least 3 standardized disturbance runs.
