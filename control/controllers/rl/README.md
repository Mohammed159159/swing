# Shadow Track: Deep Reinforcement Learning (`control/controllers/rl/`)

This directory houses the **Shadow Track** experimental pipeline implementing model-free continuous Deep Reinforcement Learning (RL) and TinyML bare-metal microcontroller deployment.

---

## 📁 Subsystem Structure

```text
control/controllers/rl/
├── env/                # Custom Gymnasium environment wrapper around MuJoCo physics
├── tinyml/             # Distilled lightweight C++ Multi-Layer Perceptron header (policy.h)
├── train_sac.py        # Stable-Baselines3 training script (Soft Actor-Critic)
└── evaluate_policy.py  # Evaluation & benchmarking against classical/optimal controllers
```

---

## 🎯 Technical Strategy & Methodologies

1. **State Observation Embedding:**
   To avoid modular discontinuities ($-\pi \leftrightarrow +\pi$), raw angles are mapped into continuous trigonometric coordinates:
   $$s_t = \begin{bmatrix} \cos\theta_0 & \sin\theta_0 & \cos\theta_1 & \sin\theta_1 & \dot{\theta}_0 & \dot{\theta}_1 \end{bmatrix}^T \in \mathbb{R}^6$$

2. **Reward Formulation:**
   $$r_t = - c_1 \theta_1^2 - c_2 \theta_0^2 - c_3 \dot{\theta}_1^2 - c_4 \dot{\theta}_0^2 - c_5 u_t^2 - c_6 (u_t - u_{t-1})^2$$
   - The action smoothness penalty $-c_6 (u_t - u_{t-1})^2$ eliminates motor chattering and protects hardware H-bridge MOSFETs.

3. **Sim-to-Real Domain Randomization:**
   - Link mass: $\pm 15\%$
   - Joint viscous friction: $\pm 30\%$
   - Actuator torque constant: $\pm 10\%$
   - Latency buffer: $10\text{--}30\text{ ms}$ FIFO queue mimicking microcontroller serial lag.

4. **TinyML Microcontroller Deployment:**
   - Policy network is distilled into a compact MLP ($6 \to 16 \to 8 \to 1 \implies \sim 689$ parameters).
   - Exported to a pure C++ header (`policy.h`) with zero dynamic memory allocation, executing in $<50\ \mu\text{s}$ per inference step.
