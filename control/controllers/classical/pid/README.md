# Classical PID Balancing Controller (`control/controllers/classical/pid/`)

This directory contains the design scripts and Simulink models for the classical PID balancing controller.

---

## 🎯 Architecture & Formulation

Because the Furuta pendulum is an underactuated single-input dual-output system ($m=1, n=2$), a simple single-loop PID controller on $\theta_1$ causes the horizontal arm $\theta_0$ to drift uncontrollably. Hence, a **dual-loop / parallel PID** architecture is implemented:

$$u_{\text{PID}}(t) = K_p e_{\theta_1}(t) + K_i \int_0^t e_{\theta_1}(\tau) d\tau + K_d \frac{d e_{\theta_1}(t)}{dt} - K_{p0} \theta_0(t) - K_{d0} \dot{\theta}_0(t)$$

### Key Implementation Requirements:
1. **First-Order Derivative Low-Pass Filter:**
   $$D(s) = K_d \frac{N s}{s + N}$$
   with filter coefficient $N \approx 50\text{--}100$ to suppress high-frequency encoder quantization noise.
2. **Anti-Windup Clamping:**
   - Integrator clamping when the actuator output saturates at $\pm V_{\max}$ to prevent reset windup.
3. **Arm Centering:**
   - Proportional-derivative feedback on arm angle ($-K_{p0} \theta_0 - K_{d0} \dot{\theta}_0$) to keep the arm centered within the available mechanical workspace.

---

## 📁 Expected Files

| File | Description |
|---|---|
| `pid_design.m` | Script computing gains ($K_p, K_i, K_d, K_{p0}, K_{d0}$, filter coefficient $N$) |
| `pid_controller.slx` | Simulink implementation with saturation and derivative filtering |
