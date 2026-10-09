# Simulation Datasets (`data/simulation/`)

This directory stores time-domain simulation datasets produced from numerical integration of the nonlinear equations of motion and Simscape Multibody physical network simulations.

---

## 🎯 Target Datasets & Milestones

1. **Open-Loop Dynamic Responses (Phase 1 Gate — Week 6):**
   - Free unforced pendulum decay oscillations from small initial angles $\theta_1(0) \approx 10^\circ$ (pivot damping $b_1$ identification).
   - Step voltage responses ($V_m = 1\text{ V}, 2\text{ V}$) for both analytical ODE (`ode45`) and Simscape Multibody models.
   - Cross-validation traces demonstrating Normalized Root Mean Square Error $\text{NRMSE} < 3\%$.

2. **Closed-Loop Controller Comparison (Phase 2 Gate — Week 9):**
   - Step reference tracking on horizontal arm angle $\theta_{0,\text{ref}}$.
   - Disturbance rejection simulation: impulsive torque perturbations applied to pendulum apex.
   - Comparative trajectories for PID, Pole Placement, and LQR balancing controllers.
   - Swing-up trajectories under Åström-Furuta energy control.

---

## 📊 Recorded Channels

Each simulation dataset should include the following variables:
- `time`: Simulation timestamp vector $[s]$
- `theta0`: Arm angle $[\text{rad}]$
- `theta1`: Pendulum angle $[\text{rad}]$ ($0 = \text{apex}, \pm\pi = \text{downward}$)
- `theta0_dot`: Arm angular velocity $[\text{rad/s}]$
- `theta1_dot`: Pendulum angular velocity $[\text{rad/s}]$
- `u` / `Vm`: Control input voltage $[\text{V}]$
- `state_id`: Discrete state identifier from Stateflow (`0: SAFE_STOP`, `1: SWING_UP`, `2: STABILIZE`)
