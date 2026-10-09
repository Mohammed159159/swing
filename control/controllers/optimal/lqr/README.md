# Linear Quadratic Regulator (LQR) (`control/controllers/optimal/lqr/`)

This directory contains design scripts and Simulink models for the Linear Quadratic Regulator (LQR).

---

## 🎯 Formulation & Synthesis

Given the continuous-time linear plant $\dot{\mathbf{x}} = A \mathbf{x} + B u$:

### 1. Quadratic Performance Index
$$J = \int_0^\infty \left( \mathbf{x}^T Q \mathbf{x} + u^T R u \right) dt$$

### 2. Weighting Matrix Selection via Bryson's Rule
Diagonal matrices normalizing each state and input by its maximum acceptable physical deviation:
$$Q = \operatorname{diag}\left( \frac{1}{\theta_{0,\max}^2}, \; \frac{1}{\theta_{1,\max}^2}, \; \frac{1}{\dot{\theta}_{0,\max}^2}, \; \frac{1}{\dot{\theta}_{1,\max}^2} \right), \quad R = \frac{1}{V_{m,\max}^2}$$

Baseline trial limits (from `HANDBOOK.md` §3.3):
- $\theta_{0,\max} = 0.5\text{ rad} \approx 28.6^\circ$
- $\theta_{1,\max} = 0.08\text{ rad} \approx 4.6^\circ$
- $\dot{\theta}_{0,\max} = 5.0\text{ rad/s}$
- $\dot{\theta}_{1,\max} = 8.0\text{ rad/s}$
- $V_{m,\max} = 12.0\text{ V}$

### 3. Algebraic Riccati Equation (ARE) Solution
$$A^T P + P A - P B R^{-1} B^T P + Q = 0 \implies K_{\text{LQR}} = R^{-1} B^T P$$

### 4. Control Law
$$u(t) = -K_{\text{LQR}} \mathbf{x}(t)$$

---

## 📁 Expected Files

| File | Description |
|---|---|
| `lqr_design.m` | MATLAB script configuring $Q, R$, solving ARE via `lqr()`, and verifying closed-loop eigenvalues |
| `lqr_controller.slx` | Closed-loop Simulink model with saturation limiters |
