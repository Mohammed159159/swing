# Open-Loop Models & Unforced Decay (`control/controllers/open-loop/`)

This directory contains open-loop simulation models and scripts for testing system dynamics, step response behavior, and natural oscillation damping.

---

## 🎯 Engineering Objectives

1. **Free Unforced Decay Simulation:**
   - Initialize pendulum at hanging rest or small deflection $\theta_1(0) \approx 10^\circ$ with $u(t) = 0$.
   - Simulate mechanical energy dissipation due to joint viscous friction ($b_0, b_1$).
   - Verify logarithmic decrement calculations for experimental damping identification.

2. **Open-Loop Step Voltage Response:**
   - Apply open-loop voltage pulses ($V_m = 1\text{ V}, 2\text{ V}$) to the base motor.
   - Observe forward kinematic acceleration and non-minimum phase behavior of the pendulum.
   - Cross-check analytical nonlinear ODEs (`ode45`) against Simscape Multibody plant models (targeting $\text{NRMSE} < 3\%$).
