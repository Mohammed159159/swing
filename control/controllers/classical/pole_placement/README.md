# Full-State Feedback Pole Placement (`control/controllers/classical/pole_placement/`)

This directory contains design scripts and Simulink models for state feedback pole placement using Ackermann's formula.

---

## 🎯 Theory & Synthesis Procedure

Given the linearized 4th-order state space system around the upright apex:
$$\dot{\mathbf{x}} = A \mathbf{x} + B u, \quad \mathbf{x} = [\theta_0, \theta_1, \dot{\theta}_0, \dot{\theta}_1]^T$$

1. **Controllability Check:**
   $$\mathcal{C} = \begin{bmatrix} B & AB & A^2 B & A^3 B \end{bmatrix}, \quad \operatorname{rank}(\mathcal{C}) = 4$$

2. **Desired Closed-Loop Pole Locations:**
   - **Dominant 2nd-Order Poles:** Designed for target damping ratio $\zeta = 0.707$ and natural frequency $\omega_n = 8\text{--}12\text{ rad/s}$ ($T_s \le 0.8\text{ s}$):
     $$p_{1,2} = -\zeta \omega_n \pm j \omega_n \sqrt{1 - \zeta^2}$$
   - **Non-Dominant Fast Poles:** Placed $3\times\text{--}5\times$ deeper in the left half-plane to ensure dominant dynamics govern the response:
     $$p_{3,4} = -4 \zeta \omega_n \pm j \omega_d$$

3. **Gain Calculation via Ackermann's Formula:**
   $$\alpha_d(s) = (s - p_1)(s - p_2)(s - p_3)(s - p_4) = s^4 + \alpha_3 s^3 + \alpha_2 s^2 + \alpha_1 s + \alpha_0$$
   $$\alpha_d(A) = A^4 + \alpha_3 A^3 + \alpha_2 A^2 + \alpha_1 A + \alpha_0 I$$
   $$K_{\text{PP}} = \begin{bmatrix} 0 & 0 & 0 & 1 \end{bmatrix} \mathcal{C}^{-1} \alpha_d(A)$$

4. **Control Law:**
   $$u(t) = -K_{\text{PP}} \mathbf{x}(t)$$

---

## 📁 Expected Files

| File | Description |
|---|---|
| `pole_placement.m` | MATLAB script checking controllability and computing $K_{\text{PP}}$ via `place()` / `acker()` |
| `pole_placement.slx` | Simulink closed-loop model with state feedback vector and saturation limits |
