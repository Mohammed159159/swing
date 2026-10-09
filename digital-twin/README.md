# Multibody Digital Twin Representations (`digital-twin/`)

This directory houses the digital twin plant models across various simulation environments, satisfying the **10% course evaluation requirement for a Synchronized Digital Twin**.

---

## 📁 Directory Structure

```text
digital-twin/
├── simscape/       # Simscape Multibody physical network model (.slx)
├── mujuco/         # MJCF XML model and mesh assets for the RL Shadow Track
└── urdf/           # Unified Robot Description Format & 3D STL meshes
```

---

## 🎯 Engineering Objectives

1. **High-Fidelity Dynamics:** Mirror the exact geometric dimensions ($L_0, L_1, l_1$), masses ($m_0, m_1$), moments of inertia ($J_0, J_1$), and pivot damping ($b_0, b_1$) identified from the physical setup.
2. **Cross-Validation (Phase 1):** Validate the multibody model against analytical nonlinear differential equations (`ode45`) with $\text{NRMSE} < 3\%$.
3. **Hardware-in-the-Loop Synchronization (Phase 3):** Render the digital twin in real time alongside the MATLAB App Designer dashboard, driven directly by the live serial stream of hardware joint angles ($\theta_0, \theta_1$).
4. **Primary Owners:** Pair 1 (Dynamics & Modeling) and Pair 4 (Benchmarking & Dashboard).
