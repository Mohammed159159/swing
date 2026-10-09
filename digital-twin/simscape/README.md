# Simscape Multibody Plant Model (`digital-twin/simscape/`)

This directory contains the primary MATLAB/Simscape Multibody physical network model of the Furuta Rotary Inverted Pendulum.

---

## 🎯 Implementation Overview

1. **Kinematic & Dynamic Architecture:**
   - **Base & Rotary Joint 1:** Revolute joint around the vertical Z-axis representing the horizontal arm rotation ($\theta_0$). Actuation via torque or electromechanical DC motor sub-block.
   - **Horizontal Arm Solid:** Rigid body parameterized by length $L_0$, mass $m_0$, and inertia tensor $J_0$. Visualized using imported CAD STL mesh.
   - **Pendulum Joint 2:** Unactuated revolute joint located at arm tip ($L_0$), oriented perpendicular to the arm, allowing free rotation in the vertical plane ($\theta_1$).
   - **Pendulum Solid:** Rigid rod parameterized by length $L_1$, center of mass $l_1$, mass $m_1$, and inertia tensor $J_1$.
   - **Viscous Dissipation:** Internal damping blocks in Revolute joints populated with experimentally measured $b_0$ and $b_1$.

2. **Integration with Single Source of Truth:**
   - Block parameters reference workspace variables loaded from `config/load_parameters.m`.
   - Never enter raw numerical constants into block dialogs.

3. **Deliverables:**
   - `furuta_simscape.slx`: Complete multibody model with sensor scopes, CAD visuals, and electromechanical motor drive.
   - Live HIL visualization harness receiving real-time serial angles to drive Simscape Mechanics Explorer or 3D Animation block.
