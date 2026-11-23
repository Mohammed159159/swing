# MuJoCo Physics Model (`digital-twin/mujuco/`)

This directory houses the MuJoCo XML description files (MJCF) and associated 3D geometric mesh files used in the **Shadow Track Deep Reinforcement Learning (RL)** benchmark.

---

## 🎯 Implementation Overview

1. **Model Format:**
   - `furuta.xml`: XML file conforming to MuJoCo MJCF schema.
   - Actuation defined via `<motor>` tag on the base joint representing armature torque/voltage limits.
   - Passive hinge joint on the pendulum tip with calibrated damping coefficients.
   - Visual and collision geoms defined with realistic dimensions and friction parameters.

2. **Meshes:**
   - Low-polygon STL or OBJ files representing the base housing, horizontal arm, and pendulum rod for lightweight rendering during high-throughput vectorized training.

3. **Physics Parameters Alignment:**
   - Masses, inertias, and friction parameters must match values established in `config/load_parameters.m`.
   - Domain randomization ranges (link mass $\pm 15\%$, pivot friction $\pm 30\%$, motor constant $\pm 10\%$) are applied dynamically in the Gymnasium training environment (`control/controllers/rl/env/`).
