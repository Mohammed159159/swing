# Furuta Pendulum (Rotary Inverted Pendulum)

> **MCT411: Hybrid Control Systems | MCT413s: Control Systems Design (2) | MCT414: Modern Control**  
> **Faculty of Engineering, Ain Shams University — Mechatronics Engineering Department**  
> **Academic Year:** Fall 2026

---

## 📌 Master Project Guideline & Quick Links

This repository contains the complete engineering stack, mathematical models, control laws, firmware, hardware CAD assets, and documentation for the **Furuta Rotary Inverted Pendulum** benchmark project.

* 📖 **[Master Project Engineering Guide & Roadmap (HANDBOOK.md)](HANDBOOK.md)**: The definitive, comprehensive operational handbook detailing the Work Breakdown Structure (WBS), Pair Engineering matrix, mathematical derivations, controller syntheses, Stateflow hybrid FSM, 13-week schedule, and oral defense guide.
* 📚 **[Comprehensive Resources Directory (docs/resources.md)](./docs/resources.md)**: Audited catalogue of 158 unique technical resources across modeling, parameter identification, classical/optimal control, RL, hardware builds, and past Ain Shams University submissions.
* 🔬 **[Basic Technical Review (docs/Furuta Pendulum Project Basic Resources Review.md)](./docs/Furuta%20Pendulum%20Project%20Basic%20Resources%20Review.md)**: Theoretical modeling equations, electromechanical DC motor coupling, parameter identification methods, and classical/optimal control derivations.
* 🚀 **[Advanced Computational & RL Review (docs/Furuta Pendulum Advanced Resource Review.md)](./docs/Furuta%20Pendulum%20Advanced%20Resource%20Review.md)**: MuJoCo/Isaac Sim environments, NMPC, trajectory optimization, and TinyML embedded deployment.
* 📋 **[Official Course Project Description (docs/Project Description.pdf)](./docs/Project%20Description.pdf)**: Official faculty specification, deliverable phases, and grading rubric.

---

## 🎯 Dual-Track Engineering Strategy

1. **Primary Model-Based Track (Course Mandate — 100% Academic Requirements):**
   - **Modeling & ID:** Euler-Lagrange equations, experimental parameter identification ($b_1, R_m, k_t, J_0, J_1$), and Simscape Multibody physical network model.
   - **Stability & Controllability:** Linearization about the inverted upright apex, open-loop eigenvalue evaluation, controllability and observability rank proofs.
   - **Control Syntheses:** Classical PID with derivative filter, Ackermann Full-State Feedback Pole Placement, and optimal Linear Quadratic Regulator (LQR) tuned via Bryson's rule.
   - **Energy-Based Swing-Up:** Åström-Furuta Lyapunov energy pump driving the pendulum into the linear capture basin.
   - **Hybrid Supervisory Logic:** Stateflow chart implementing discrete-event state switching (`SAFE_STOP`, `SWING_UP`, `STABILIZE`) with $10^\circ$ hysteresis guards.
   - **Real-Time HIL & Dashboard:** MATLAB App Designer dashboard communicating with microcontroller firmware via high-speed binary serial telemetry, coupled with a synchronized 3D Simscape digital twin.

2. **Shadow Innovation & Benchmark Track (Deep Reinforcement Learning):**
   - MuJoCo physics wrapped in a standardized Gymnasium environment.
   - Continuous trigonometric state embedding: $s_t = [\cos\theta_0, \sin\theta_0, \cos\theta_1, \sin\theta_1, \dot{\theta}_0, \dot{\theta}_1]$.
   - Continuous control training with Soft Actor-Critic (SAC) and PPO featuring action smoothness penalties.
   - Sim-to-real transfer via domain randomization and transport latency buffering.
   - Policy distillation into a lightweight bare-metal C++ Multi-Layer Perceptron (TinyML) deployed on microcontroller hardware for empirical benchmark comparison against LQR and PID.

---

## 👥 Team Structure: Pair Engineering Model

With a team of 6 to 8 students, the workload is distributed across four functional pairs:

| Functional Pair | Core Subsystems | Primary Deliverables |
|---|---|---|
| **Pair 1: Dynamics & Modeling** | Theoretical Mechanics, Stability Analysis, Parameter ID | Euler-Lagrange derivation, Parameter ID table, Linearization, Controllability/Observability |
| **Pair 2: Control Syntheses** | Classical, Optimal, Hybrid & RL Control | PID, Pole Placement, LQR Riccati solver, Stateflow FSM, Gym RL training & TinyML distillation |
| **Pair 3: Hardware & Firmware** | Mechanical Fabrication, Embedded Systems, HIL Firmware | CAD design, 3D printing, motor driver wiring, QEI timer firmware, serial telemetry contract |
| **Pair 4: Dashboard & Telemetry** | Digital Twin, App Designer, Benchmarking, Documentation | Simscape 3D twin, MATLAB App Designer GUI, disturbance benchmark scripts, report & demo video |

---

## 📁 Repository Directory Structure

```text
Furuta-Pendulum/
├── config/                     # SINGLE SOURCE OF TRUTH: physical parameters & gains
│   ├── load_parameters.m      # Physical parameters (m0, m1, L0, l1, J0, J1, Rm, kt, b0, b1)
│   ├── compute_matrices.m     # Symbolic Jacobian linearization; outputs A, B, C, D
│   └── compute_gains.m        # Computes K_lqr, K_pp, K_pid, and energy gain k_E
├── data/                       # Experimental telemetry & disturbance logs (CSV, MAT)
│   ├── simulation/             # Open-loop step responses, ODE validation logs
│   └── hardware/               # Real-time hardware telemetry runs
├── digital-twin/               # Simulation plant representations
│   ├── simscape/               # Simscape Multibody physical network model (.slx)
│   ├── mujuco/                 # MJCF XML model and mesh assets for RL
│   └── urdf/                   # Unified Robot Description Format & STL links
├── control/                    # Control law syntheses and supervisory logic
│   ├── controllers/            # PID, Pole Placement, LQR, and RL algorithms
│   ├── stateflow/              # Hybrid supervisory Stateflow chart (SWING_UP, STABILIZE, SAFE_STOP)
│   └── evaluation/             # Stability proofs and quantitative performance scripts (IAE, ITAE, u_RMS)
├── gui/                        # MATLAB App Designer dashboard and real-time scopes
├── hardware/                   # Physical mechatronics assets
│   ├── cad/                    # SolidWorks / Fusion360 source, STEP, 3D print STL
│   ├── electronics/            # Schematics, pinout diagrams, motor driver wiring, BOM
│   └── firmware/               # Microcontroller C/C++ code (QEI interrupts, PWM, serial packet protocol)
├── docs/                       # Academic documentation, report drafts, meeting logs, and reference PDFs
├── FurutaPendulum.prj          # Root MATLAB Project file
├── startup.m                   # Root initialization script
├── .gitignore
├── README.md
└── PROJECT_MASTER_GUIDELINE.md # Master project guide and execution roadmap
```

---

## 🗓️ Submission Milestones

* **Phase 1 Gate (Week 6):** Validated Simscape model and open-loop dynamic analysis cross-checked against analytical ODEs ($\text{NRMSE} < 3\%$).
* **Phase 2 Gate (Week 9):** Full stability analysis and simulation comparison across all 3 controllers (PID, Pole Placement, LQR) with closed-loop eigenvalue justification.
* **Phase 3 Final Gate (Week 13):** Working hardware prototype balancing under manual disturbances + Real-time HIL dashboard + Synchronized 3D digital twin + Comprehensive technical report + 5-minute demo video + Oral defense.

---

## 🚀 Quick Start (MATLAB / Simulink)

1. Clone the repository:
   ```bash
   git clone https://github.com/[your-repo]/furuta-pendulum.git
   cd furuta-pendulum
   ```
2. Open MATLAB and navigate to the project root.
3. Run `startup.m` or open `FurutaPendulum.prj` to load paths, parameters, and controller gains:
   ```matlab
   startup
   ```
4. To launch the App Designer dashboard:
   ```matlab
   appdesigner gui/app/FurutaDashboard.mlapp
   ```

---
*Maintained by the MCT411 / MCT413s Furuta Pendulum Engineering Team — Ain Shams University.*
