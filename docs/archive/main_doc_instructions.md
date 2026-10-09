I am having this project. I am very excited by it. I even want to go the extra mile and implement RL. I was wondering about the best way to approach it.

I first conducted a review of as many resources as possible that could help us in the project. Now, I am wondering about my next steps...

I am thinking of the following:
- Preparing a GitHub repo to house the project
- Adding all of these as docs in the repo
- Creating the team formation sheet and adding it there
- Creating an outline of the report to be submitted
- Making sure the contribution section is added to the report and being updated as tasks get done
- Creating a plan document to include the following:
  - Detailed project flow and WBS as per the project description
  - Comprehensive suggestion of relevant resources from my reviews to guide each component of the project
  - Detailed timeline of what would be done weekly based on the WBS of the description and the general timeline provided
  - Role assignment (should be balanced to allow contribution to the core parts of the project by everyone and handling of secondary tasks by specialized individuals)
- Using WhatsApp to highlight each week's deliverables, scheduling meetings, sharing ideas, and scheduling co-working sessions

What do you think? Is there anything better I could do?

Your proposed plan reflects mature engineering leadership. Setting up version control, planning the Work Breakdown Structure (WBS), establishing team accountability early, and organizing resources before diving into execution are exactly the right foundational moves.

However, with a team of 6 to 8 members, a complex mechatronics workflow, strict submission milestones (Phase 1 at Week 6, Phase 2 at Week 9, and Phase 3 at Week 13), and the ambition to implement Deep Reinforcement Learning (RL), several critical adjustments will prevent bottlenecks and ensure top-tier execution.

---

### 1. Critical Refinements to Your Proposed Plan

#### A. Avoid the "Hardware Trap": Parallelize Bring-Up from Week 1

* **The Common Pitfall:** Many teams follow a strictly linear path—deriving equations in Weeks 1–4, simulating in Weeks 5–8, and only building hardware around Weeks 9–10. In underactuated systems like the Furuta pendulum, real-world friction, bearing stiction, mechanical backlash, encoder quantization noise, and motor driver saturation frequently break simulations.
* **The Better Approach:** Parallelize mechanical design, procurement, and sensor bring-up immediately. While the theoretical group derives the Euler-Lagrange equations, the hardware team should 3D-print or machine arm prototypes, procure the motor and high-resolution encoders ($\ge 360$ PPR, ideally $\ge 2048$ quadrature counts/rev), and write basic timer/QEI interrupt firmware on the chosen microcontroller (e.g., STM32, Arduino, or TI C2000).

#### B. The Right Strategy for Reinforcement Learning: The "Shadow Track"

Your enthusiasm for RL is great, but it must be structured strategically so it enhances your project rather than putting course grades at risk.

* **The Course Mandate:** The project evaluation explicitly requires a hybrid Stateflow supervisory state machine, classical/optimal controllers (PID, Pole Placement, LQR), and an energy-based swing-up controller. Omitting these in favor of an end-to-end black-box RL policy will result in grade penalties.
* **The Winning Approach:** Treat the academic requirements (the Model-Based Stack: Euler-Lagrange $\to$ Linearization $\to$ LQR/PID $\to$ Stateflow $\to$ App Designer) as your **Primary Track**. Treat Reinforcement Learning as a **Parallel Innovation / Benchmark Track**:
1. **Simulation Environment:** Build a standardized Gymnasium environment wrapped around **MuJoCo** or use the MATLAB Reinforcement Learning Toolbox workflow.
2. **Observation Space:** Avoid feeding raw angles $\theta \in [-\pi, \pi]$ directly due to boundary wrap-around discontinuities; project them into continuous trigonometric coordinates $s_t = [\cos\theta_0, \sin\theta_0, \cos\theta_1, \sin\theta_1, \dot{\theta}_0, \dot{\theta}_1]$.
3. **Algorithm & Reward:** Train using **Soft Actor-Critic (SAC)** for sample efficiency during swing-up, or **PPO** with an action continuity penalty $-c_6 (u_t - u_{t-1})^2$ to suppress high-frequency motor chattering and actuator heating.
4. **Sim-to-Real & TinyML Deployment:** Apply Domain Randomization across link masses, damping, and torque constants, along with a FIFO transport delay buffer ($10\text{--}30\text{ ms}$) to reflect microcontroller/serial lag. Distill the policy into a lightweight Multi-Layer Perceptron (e.g., 2 hidden layers with 16 and 8 units, $\sim 689$ parameters, following Henrique Ferrolho's open-source methodology) that can run standalone in bare-metal C++ on an 8-bit or 32-bit MCU in microseconds.
5. **Report Payoff:** In your final report, present RL as a comprehensive comparative benchmark against LQR and PID across rise time, settling time, IAE/ISE/ITAE error indices, and RMS control effort. This satisfies the benchmarking rubric.



#### C. Beyond Flat WhatsApp Chats: Structured Communication

* **The Risk:** In a 6–8 person team, WhatsApp quickly becomes cluttered. Critical design decisions, pinout changes, CAD iterations, and tuning formulas get lost in message streams.
* **The Solution:** Use a multi-tiered communication setup:
* **WhatsApp:** Urgent alerts, quick check-ins, and meeting logistics.
* **Discord / Slack (or GitHub Discussions):** Organized channels (`#hardware-cad`, `#math-modeling`, `#simscape-dashboard`, `#firmware-hil`, `#rl-benchmarks`).
* **Google Drive / Shared Storage:** Large datasets, raw telemetry bags, and video footage for the required 5-minute final video deliverable.



#### D. Repository Structure & Version Control Best Practices

To keep your GitHub repository clean and accessible to both MATLAB and software developers:

* **Directory Layout:**
```text
├── cad/                # STEP, STL files, 3D printing parameters, BOM
├── docs/               # Technical specs, meeting minutes, resource index
├── firmware/           # MCU code (QEI encoder interrupts, motor PWM, serial comms)
├── modeling/           # Analytical Euler-Lagrange derivations (Live Scripts, SymPy)
├── simulink/           # Simscape digital twin, Stateflow chart, App Designer GUI
├── rl_benchmark/       # MuJoCo/Gymnasium env, SB3 training scripts, TinyML export
├── report/             # LaTeX / Markdown report drafts and figure assets
├── .gitignore          # MATLAB/Simulink specific (ignoring slprj/, *.asv, *.autosave)
└── CONTRIBUTIONS.md    # Living log of tasks, owners, and completion dates

```


* **Use Git LFS:** Track large binary files like CAD files (`.step`, `.stl`), Simulink caches, and RL model checkpoints (`.zip`, `.onnx`) using Git Large File Storage (LFS) to keep repository cloning fast.

#### E. Team Accountability: "Pair Engineering"

With 6 to 8 students, individual accountability is essential, especially since the final evaluation includes an oral defense where any member may be questioned.

* Structure your team into functional pairs:
* **Core Modeling & Stability:** 2 members (Euler-Lagrange, linearization, pole placement, controllability/observability).
* **Optimal, Hybrid & RL Control:** 2 members (LQR tuning, Stateflow supervisor, Åström-Furuta energy swing-up, RL agent).
* **Mechanical & Embedded Firmware:** 2 members (CAD fabrication, wiring, encoder timers, motor driver, safety circuits).
* **Dashboard, Telemetry & Evaluation:** 1–2 members (MATLAB App Designer, Simscape 3D digital twin mirror, IAE/ITAE logging, video production).


* Have teammates cross-review deliverables before merging to ensure everyone understands the full system during the oral defense.

---

### 2. Suggested Milestone Alignment (Weeks 1 to 13)

Align your weekly WBS directly with the official three-phase evaluation schedule:

| Phase & Timeline | Primary Milestones & Work Breakdown | Official Deliverables & Targets |
| --- | --- | --- |
| **Weeks 1–2**<br>

<br>*(Initiation & Setup)* | • Complete team formation sheet, nominate lead, set up GitHub and Discord.<br>

<br>• Inventory available hardware (motor, encoders, power supply, MCU).<br>

<br>• Finalize CAD design of rotary arm and pendulum link; begin 3D printing. | Baseline repo initialized; team formation submitted. |
| **Weeks 3–5**<br>

<br>*(Modeling & ID)* | • Analytical derivation of nonlinear equations via Euler-Lagrange.<br>

<br>• Hardware parameter identification (logarithmic decrement for $b_1$, locked-rotor $R_m$, bifilar inertia).<br>

<br>• Assemble Simscape Multibody digital twin with CAD parameters. | Identified hardware parameter table; validated open-loop Simscape plant. |
| **Week 6**<br>

<br>**(Phase 1 Gate)** | • Cross-validate open-loop dynamic response of analytical ODEs vs. Simscape model. | **Phase 1 Submission:** Validated Simscape model and open-loop dynamic analysis. |
| **Weeks 7–8**<br>

<br>*(Controller Synthesis)* | • Linearize about upright apex; verify controllability and observability matrices.<br>

<br>• Synthesize and simulate PID, Pole Placement, and LQR balancing controllers.<br>

<br>• Implement Åström-Furuta energy-based swing-up controller in simulation.<br>

<br>• *(Shadow Track)*: Build Gymnasium environment and begin baseline SAC/PPO policy training. | Simulation validation of all 3 balancing controllers + swing-up law. |
| **Week 9**<br>

<br>**(Phase 2 Gate)** | • Finalize eigenvalue justification, gain/phase margins, and transient response metrics. | **Phase 2 Submission:** Stability analysis report and simulation comparison. |
| **Weeks 10–11**<br>

<br>*(Hybrid & HIL Bring-Up)* | • Integrate Stateflow supervisory logic (SWING_UP, STABILIZE, SAFE_STOP with hysteresis guards).<br>

<br>• Establish bidirectional serial/UDP telemetry between MCU and App Designer dashboard.<br>

<br>• Synchronize real-time 3D Simscape digital twin with live hardware joint angles.<br>

<br>• Deploy LQR balancing and swing-up logic onto physical hardware. | Closed-loop hardware balancing under manual disturbance. |
| **Week 12**<br>

<br>*(Benchmarking & RL)* | • Run standardized disturbance tests; log IAE, ISE, ITAE, and $u_{\text{RMS}}$ across all controllers.<br>

<br>• *(Shadow Track)*: Export distilled TinyML RL policy to MCU for live comparison.<br>

<br>• Draft the technical report and populate the individual contribution section. | Benchmark comparison tables and response plots. |
| **Week 13**<br>

<br>**(Phase 3 Final Gate)** | • Rehearse oral defense with the entire team.<br>

<br>• Record, edit, and voiceover the 5-minute project demonstration video.<br>

<br>• Finalize the technical report with all citations and experimental validation curves. | **Phase 3 Submission:** Working prototype + HIL Dashboard/Digital Twin + Report + Demo Video. |

---

### 3. Immediate Action Checklist (Next 48 Hours)

1. **Initialize the GitHub Repo:** Set up the recommended directory tree and add a comprehensive `.gitignore` for MATLAB/Simulink and Python.
2. **Add Resource Documents:** Commit your compiled resource directory (`resources.md`) and technical reviews to `/docs/` for team reference.
3. **Draft the Team Formation Sheet:** Collect full student names, IDs, sections, and assigned sub-roles, and submit it per course instructions.
4. **Draft the Report Shell:** Create a report template (in LaTeX or Word) containing the exact sections required by the grading scheme (Contribution Log, Mathematical Modeling, Parameter ID, Stability Analysis, Controller Design & Tuning, Stateflow Architecture, Digital Twin & Dashboard, Simulation vs. Experimental Benchmarking, and Conclusion).
5. **Host the Kickoff Meeting:** Walk the team through the 13-week timeline, review the WBS, and assign Week 1 deliverables (CAD mechanical design vs. Euler-Lagrange dynamic modeling).





I like this layout better:

- docs
  - report/ (contributions logged directly into contributions section)
  - logs/ (can include links to drive resources such as videos) 
- hardware
  - cad/
  - electronics/
  - firmware/
- digital-twin/
  - simscape/
  - mujuco/
  - urdf/
- gui/
- control/
  - stateflow/
  - controllers/
    - open-loop/
    - classical/
      - pid/
      - pp/
    - optimal/
      - lqr/
    - rl/
  - evaluation/
    - stability/
    - performance/
- .gitignore
- README.md


Note: 
- theoretical aspects of the project, results, comparisons, parameters, etc. will be detailed in the report and re-stated in md files at the relevant location/s

What do you think?




This layout is significantly better than a flat structure: it establishes a domain-driven separation of concerns, cleanly decouples physical hardware from software models, and directly mirrors the course deliverables (Stateflow hybrid logic, 3+1 controllers, digital twin, and GUI).

---

### Architectural Strengths

* **Clean Domain Isolation:** Separating `hardware/`, `digital-twin/`, and `control/` prevents cross-contamination between mechanical/electronic CAD assets and software simulation models.


* **Coexistence of Dual Simulation Paradigms:** Grouping `simscape/`, `mujuco/`, and `urdf/` under `digital-twin/` elegantly accommodates both the course-mandated Simscape digital twin and your parallel MuJoCo/Gymnasium RL pipeline.


* **Modular Control Hierarchy:** Splitting `controllers/` into `open-loop/`, `classical/`, `optimal/`, and `rl/` provides a plug-and-play structure where Stateflow can switch between algorithms while evaluating them against a shared set of benchmark scripts in `evaluation/`.



---

### High-Impact Refinements to Consider

1. **Add a Shared Parameter & Workspace Hub (`config/` or Root `parameters.m`):**
* *The Problem:* Your physical parameters ($m_1, L_0, J_0, J_1, R_m, k_t, b_1$) and derived matrices ($A, B, K_{\text{LQR}}, K_{\text{PP}}$) must be identical across `simscape/`, `control/`, `gui/`, and the report. If they are defined locally inside separate folders, values will inevitably drift.


* *Fix:* Introduce a central `config/` directory (or a top-level `parameters.m` script) that populates the base MATLAB workspace with identified physical constants and controller gain matrices before any Simulink model or GUI launches.




2. **Use a MATLAB Project (`.prj`) or `startup.m`:**
* Because Simulink models in `control/` need to reference the Simscape plant in `digital-twin/simscape/`, load gains from `control/controllers/`, and stream data to `gui/`, relative path breaks are frequent when different team members clone the repo.
* Adding a MATLAB Project file (`FurutaPendulum.prj`) or a root `startup.m` automatically manages path dependencies, shortcut scripts, and git source control integration inside MATLAB.


3. **Separate Empirical Telemetry from Administrative Logs:**
* `docs/logs/` is ideal for meeting minutes, weekly WBS milestones, and external video links.
* Create a dedicated `data/` directory (e.g., `data/raw/` and `data/processed/`) to hold hardware CSV logs, serial MCAP traces, and `.mat` disturbance test results so large time-series data doesn't clutter documentation.




4. **Define the Hardware Communication Protocol Contract:**
* Inside `hardware/firmware/`, add a `protocol/` subfolder (or Markdown spec) explicitly defining the serial byte packet structure (e.g., header, $\theta_0$, $\theta_1$, current, mode, CRC). This allows the firmware pair and the App Designer GUI pair to work completely in parallel without integration mismatches.




5. **Mitigate Documentation Drift:**
* Writing full theoretical derivations in the report and re-stating them across multiple Markdown files risks desynchronization when numbers change.
* Keep folder-level Markdown files strictly operational (e.g., *“How to run this model”*, *“Tuning parameters”*, *“Input/Output definitions”*), while keeping the formal mathematical proofs and parameter identification tables centralized in `docs/report/`.





---

### Recommended Layout

```text
├── config/                     # Single source of truth for physical parameters & gains
│   ├── load_parameters.m      # m0, m1, L0, l1, J0, J1, Rm, kt, b0, b1
│   └── compute_matrices.m     # Symbolic Jacobian derivation, A, B, Q, R, K matrices
├── data/                       # Experimental time-series & disturbance test logs
│   ├── simulation/             # Step response & open-loop validation data
│   └── hardware/               # Serial telemetry logs (CSV, MAT, MCAP)
├── digital-twin/
│   ├── mujuco/                 # XML (MJCF) model & meshes for RL
│   ├── simscape/               # Simscape Multibody model & 3D visualization
│   └── urdf/                   # Unified robot description & STL mesh assets
├── control/
│   ├── controllers/
│   │   ├── classical/          # PID & Ackermann Pole Placement models/scripts
│   │   ├── open-loop/          # Open-loop impulse, step, & unforced decay models
│   │   ├── optimal/            # LQR Riccati solver, Bryson's tuning, & saturation
│   │   └── rl/                 # Gymnasium env, SB3/CleanRL scripts, TinyML C++ header
│   ├── evaluation/
│   │   ├── performance/        # IAE, ISE, ITAE, settling time & control effort scripts
│   │   └── stability/          # Eigenvalues, controllability, observability, Bode plots
│   └── stateflow/              # Hybrid supervisory FSM (SWING_UP, STABILIZE, SAFE_STOP)
├── gui/                        # MATLAB App Designer dashboard & real-time telemetry
├── hardware/
│   ├── cad/                    # SolidWorks / STEP / STL files for 3D printing
│   ├── electronics/            # Schematics, pinouts, motor driver wiring, BOM
│   └── firmware/               # Microcontroller code (QEI timers, PWM, serial protocol)
├── docs/
│   ├── logs/                   # Meeting minutes, weekly WBS milestones, drive links
│   └── report/                 # LaTeX / Word report draft & inline contribution log
├── FurutaPendulum.prj          # Root MATLAB Project (or startup.m)
├── .gitignore
└── README.md

```

Which embedded platform (e.g., STM32, Arduino, or TI C2000) are you planning to use for the hardware target, so we can tailor the `firmware/` and serial protocol setup to match its hardware timers?
