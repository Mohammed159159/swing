# Configuration & Parameter Management (`config/`)

This directory serves as the **Single Source of Truth (SSOT)** for all physical constants, mathematical state-space matrices, and controller gains in the Furuta Rotary Inverted Pendulum project.

---

## 📌 Architectural Rule

> **CRITICAL EXECUTION MANDATE (from HANDBOOK.md §5):**  
> To prevent parameter drift across Simscape models, Simulink controllers, the App Designer GUI, firmware, and the technical report:
> - **Never** hardcode physical parameters or feedback gains inside Simulink blocks or scripts.
> - Every simulation, test script, and model must call:
>   ```matlab
>   run('config/load_parameters.m');
>   run('config/compute_gains.m');
>   ```

---

## 📁 Directory Structure & Expected Files

| File | Description | Ownership |
|---|---|---|
| `load_parameters.m` | Base physical parameters ($m_0, m_1, L_0, l_1, J_0, J_1, R_m, k_t, k_b, b_0, b_1$) | Pair 1 (Dynamics & Modeling) |
| `compute_matrices.m` | Symbolic Jacobian linearization about the inverted apex; generates $A, B, C, D$ | Pair 1 (Dynamics & Modeling) |
| `compute_gains.m` | Calculates balancing gains ($K_{\text{LQR}}, K_{\text{PP}}, K_{\text{PID}}$) & Åström swing-up gain $k_E$ | Pair 2 (Control Syntheses) |

---

## 🔬 Core Parameter Definitions

- $m_0, m_1$: Arm mass and pendulum link mass $[\text{kg}]$
- $L_0, L_1$: Arm length and pendulum link length $[\text{m}]$
- $l_1$: Distance from pivot to pendulum center of mass $[\text{m}] \approx L_1 / 2$
- $J_0, J_1$: Moments of inertia of arm and pendulum $[\text{kg}\cdot\text{m}^2]$
- $R_m, L_a$: Motor terminal resistance $[\Omega]$ and armature inductance $[\text{H}]$
- $k_t, k_b$: Motor torque constant $[\text{N}\cdot\text{m/A}]$ and back-EMF constant $[\text{V}\cdot\text{s/rad}]$
- $b_0, b_1$: Viscous friction coefficients at arm and pendulum pivots $[\text{N}\cdot\text{m}\cdot\text{s/rad}]$
- $V_{\max}$: Actuator saturation voltage $[\text{V}]$ (typically $12\text{ V}$ or $24\text{ V}$)
