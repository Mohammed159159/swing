# TinyML Embedded Policy Deployment (`control/controllers/rl/tinyml/`)

This directory houses the distilled, bare-metal C++ neural network policy header files for embedded microcontroller deployment.

---

## 🎯 Architecture & Ferrolho Methodology

1. **Lightweight Multi-Layer Perceptron (MLP):**
   - **Input Layer:** 6 inputs ($[\cos\theta_0, \sin\theta_0, \cos\theta_1, \sin\theta_1, \dot{\theta}_0, \dot{\theta}_1]$)
   - **Hidden Layer 1:** 16 neurons with ReLU activation
   - **Hidden Layer 2:** 8 neurons with ReLU activation
   - **Output Layer:** 1 neuron with Tanh activation scaled to $[-V_{\max}, V_{\max}]$
   - **Total Parameters:** $(6\times 16 + 16) + (16\times 8 + 8) + (8\times 1 + 1) = 112 + 136 + 9 = 257$ weights/biases (or $\approx 689$ parameters depending on architecture variant).

2. **Embedded Characteristics:**
   - Pure C++ header (`policy.h` or `tinyml_policy.hpp`).
   - Zero dynamic memory allocation (`malloc`/`new`).
   - Fixed-point or single-precision `float` matrix-vector multiplications.
   - Deterministic execution time: $< 50\ \mu\text{s}$ per control cycle on ARM Cortex-M4 @ 168 MHz.

3. **Comparison Target:**
   - Deployed on microcontroller firmware to directly benchmark against classical PID and optimal LQR under identical impulse disturbances.
