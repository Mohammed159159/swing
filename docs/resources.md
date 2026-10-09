# Organized Furuta Pendulum Resources Directory & Comprehensive Technical Index
> **Project:** Furuta Pendulum (Rotary Inverted Pendulum) — Modelling, Hybrid & Control, and Real-Time Dashboard  
> **Course Context:** MCT411: Hybrid Control Systems | MCT413s: Control Systems Design (2) | MCT414: Modern Control  
> **Faculty:** Faculty of Engineering, Ain Shams University — Mechatronics Engineering Department

### Categorical Breakdown

| # | Category Name | Unique Resources | Primary Scope |
|---|---|:---:|---|
| 1 | **Mathematical Modeling, Lagrangian Dynamics & Kinematics** | `10` | Foundational analytical mechanics, Euler-Lagrange equations of motion, conservat... |
| 2 | **System Identification, Friction Modeling & Parameter Estimation** | `4` | Experimental parameter extraction, physical inertia characterization, Coulomb an... |
| 3 | **Energy-Based Swing-Up Control & Hybrid Switching Strategies** | `12` | Energy-shaping control laws, homoclinic orbit targeting, Total Energy Control Sy... |
| 4 | **Linear, State-Space & Classical Stabilization (LQR, Pole Placement, PID)** | `13` | Linearization about the unstable upright equilibrium, Full State Feedback, Acker... |
| 5 | **Nonlinear Model Predictive Control (NMPC), Trajectory Optimization & Advanced Methods** | `19` | Real-time embedded NMPC using the acados solver suite, direct multiple shooting,... |
| 6 | **Deep Reinforcement Learning & Continuous Control** | `20` | Model-free and model-based deep reinforcement learning (PPO, SAC, Monte Carlo me... |
| 7 | **Sim-to-Real Transfer, Domain Randomization & Physics Simulators** | `13` | High-fidelity physics engines (MuJoCo, Drake, NVIDIA Isaac Sim / Isaac Lab, MATL... |
| 8 | **Embedded Hardware Implementation, Microcontrollers & Robotics Middleware** | `10` | Embedded electronics, microcontrollers (STM32 ARM Cortex-M, Arduino Nano, ESP32 ... |
| 9 | **Real-Time Dashboards, Telemetry, Data Logging & Visualization** | `17` | Live operator interfaces, MATLAB App Designer serial streaming, Simulink Real-Ti... |
| 10 | **Open-Source Physical Builds, Laboratory Workbooks & Curated Vaults** | `30` | Replicable open-source hardware builds (Armandpl/furuta, ferrolho £20 Arduino bu... |
| 11 | **Course Specification Benchmark References & Ain Shams University Legacy Projects** | `10` | Official reference videos from MCT411/413s Course Description (§9.1–§9.4): DIY builds, STM32 LQR demos, sim-to-real transfer, and previous ASU student project submissions |
| | **Total Unique Resources** | **`158`** | |

---

## Categorized Resource Catalogue

### 1. Mathematical Modeling, Lagrangian Dynamics & Kinematics (10 Resources)

*Foundational analytical mechanics, Euler-Lagrange equations of motion, conservative nonlinear formulations, conformable mechanics, coordinate transformations, and kinematic coupling for the Furuta rotary inverted pendulum.*

#### `C1.01` [Modelling the Furuta Pendulum (M. Gäfvert)](https://www.semanticscholar.org/paper/Modelling-the-Furuta-Pendulum-G%C3%A4fvert/f976a3a26d55c9f4d0d380080482765f2a25758b)

- **Description:** Technical report from Lund Institute of Technology presenting detailed Euler-Lagrange derivations of the Furuta rotary inverted pendulum dynamics, coordinate transformations, and inertia tensors.
- **Cross-Platform Mirror Note:** Direct institutional PDF available at [Orig #93].

#### `C1.02` [Furuta's Pendulum: A Conservative Nonlinear Model for Theory and Practise](https://www.researchgate.net/publication/42387157_Furuta's_Pendulum_A_Conservative_Nonlinear_Model_for_Theory_and_Practise)

- **Description:** ResearchGate publication detailing conservative nonlinear equations of motion for the rotary inverted pendulum, analyzing Hamiltonian energy conservation and zero-friction idealized mechanics.

#### `C1.03` [Conformable Lagrangian Mechanics of Actuated Pendulum](https://www.mdpi.com/2227-7390/13/10/1634)

- **Description:** MDPI Mathematics paper investigating fractional and conformable calculus extensions to Lagrangian mechanics applied to actuated pendulum systems.

#### `C1.04` [A Multibody-Based Benchmarking Framework for the Control of the Furuta Pendulum (Univ. of Vigo)](https://www.investigo.biblioteca.uvigo.es/xmlui/handle/11093/10496?locale-attribute=en)

- **Description:** Institutional thesis repository entry from Universidade de Vigo establishing a multibody dynamics benchmarking framework for simulating and validating Furuta pendulum controllers.
- **Cross-Platform Mirror Note:** Published journal version available on MDPI at [Orig #46].

#### `C1.05` [Modeling, Simulation, and Control of Inverted Pendulum (Brno University of Technology)](https://dspace.vut.cz/bitstreams/1266d004-314c-4bc7-aa52-ce27c244ed01/download)

- **Description:** DSpace academic thesis from Brno University of Technology covering mathematical formulation, system identification, and simulation of inverted pendulum systems.

#### `C1.06` [A Multibody-Based Benchmarking Framework for the Control of the Furuta Pendulum (MDPI)](https://www.mdpi.com/2076-0825/14/8/377)

- **Description:** Published open-access article in MDPI Machines presenting a standardized multibody simulation and benchmarking suite for rotary inverted pendulum control algorithms.
- **Cross-Platform Mirror Note:** Institutional repository version at [Orig #25].

#### `C1.07` [Revitalizing the Furuta Pendulum (M. R. Hamers, TU Eindhoven)](https://pure.tue.nl/ws/portalfiles/portal/4307900/626064.pdf)

- **Description:** Technical report from Eindhoven University of Technology detailing mechanical redesign, Euler-Lagrange equations of motion, and nonlinear control of a physical Furuta pendulum apparatus.

#### `C1.08` [On the Dynamics of the Furuta Pendulum](https://www.researchgate.net/publication/228421122_On_the_Dynamics_of_the_Furuta_Pendulum)

- **Description:** Comprehensive dynamic analysis exploring non-Euclidean state-space geometry (S¹ × S¹), Coriolis coupling terms, and open-loop stability characteristics of the rotary inverted pendulum.

#### `C1.09` [Modelling the Furuta Pendulum (M. Gäfvert - Lund University Report PDF)](https://lucris.lub.lu.se/ws/files/4453844/8727127.pdf)

- **Description:** Original full-text PDF of Magnus Gäfvert's foundational 1998 report from Lund Institute of Technology (CODEN: LUTFD2/TFRT--7574--SE).
- **Cross-Platform Mirror Note:** Semantic Scholar record at [Orig #2].

#### `C1.10` [Dynamic Modeling and Simulation of an Underactuated System (arXiv / J. Phys.)](https://arxiv.org/pdf/1706.04996)

- **Description:** Conference paper providing dynamic modeling, bond graph / Lagrangian equations, and numerical simulation validation for two-degree-of-freedom underactuated mechanical systems.

---

### 2. System Identification, Friction Modeling & Parameter Estimation (4 Resources)

*Experimental parameter extraction, physical inertia characterization, Coulomb and viscous joint friction modeling, reaction torque sizing, and parasitic oscillation analysis on physical platforms like the Quanser QUBE-Servo.*

#### `C2.01` [Rotary Inverted Pendulum: Rotating Shaft Arm Disconnection Troubleshooting](https://de.mathworks.com/matlabcentral/answers/2184949-rotary-inverted-pendulum-rotating-shaft-is-not-rotating-the-arm-attached-to-it/?s_tid=ans_lp_feed_leaf)

- **Description:** MATLAB Central technical discussion diagnosing mechanical and joint modeling failures where the rotating base shaft fails to transmit torque to the attached pendulum link in Simscape.

#### `C2.02` [Inertia Wheel Pendulum: Reaction Torque Calculator (FIRGELLI)](https://www.firgelliauto.com/blogs/mechanisms/inertia-wheel-pendulum)

- **Description:** Engineering guide and practical reaction torque calculator detailing angular acceleration, mass moments of inertia, and torque sizing for reaction-driven inverted pendulums.

#### `C2.03` [System Identifications of a 2DOF Pendulum Controlled by QUBE-Servo and Unwanted Oscillation Factors (PAN)](https://www.journals.pan.pl/Content/115031/PDF/AME_2020_131699.pdf)

- **Description:** Published journal paper in Archive of Mechanical Engineering (Polish Academy of Sciences) performing parameter identification, dry/viscous friction modeling, and parasitic oscillation analysis on the Quanser QUBE-Servo.
- **Cross-Platform Mirror Note:** ResearchGate version at [Orig #131].

#### `C2.04` [System Identifications of a 2DOF Pendulum Controlled by QUBE-Servo (ResearchGate)](https://www.researchgate.net/publication/375432201_System_identifications_of_a_2DOF_pendulum_controlled_by_QUBE-servo_and_its_unwanted_oscillation_factors)

- **Description:** ResearchGate mirror of the Polish Academy of Sciences study on physical parameter estimation and high-frequency resonance mitigation in QUBE-Servo rotary pendulums.
- **Cross-Platform Mirror Note:** Journal PDF at [Orig #130].

---

### 3. Energy-Based Swing-Up Control & Hybrid Switching Strategies (12 Resources)

*Energy-shaping control laws, homoclinic orbit targeting, Total Energy Control System (TECS) approaches, smooth chatter-free swing-up formulations, and discrete-event hybrid switching logic (Stateflow) governing transitions to upright balancing.*

#### `C3.01` [Energy Based Control of Pendulum (Iwashiro & Furuta - Semantic Scholar)](https://www.semanticscholar.org/paper/Energy-based-control-of-pendulum-Iwashiro-Furuta/bd3f868d1538ab724d305e70f0d9766071c415fe)

- **Description:** Foundational paper by M. Iwashiro and K. Furuta establishing the energy-shaping control strategy to inject kinetic energy into the pendulum link until the homoclinic orbit is reached.
- **Cross-Platform Mirror Note:** ResearchGate mirror at [Orig #7].

#### `C3.02` [Swing-up Control of an Inverted Pendulum (Aalborg University Project Report)](https://projekter.aau.dk/projekter/files/243991610/Swing_up_Control_of_an_Inverted_Pendulum.pdf)

- **Description:** Comprehensive engineering project report from Aalborg University covering energy-based swing-up controller design, state-space balancing, and hybrid switching threshold conditions.

#### `C3.03` [Energy Based Control of Pendulum (Iwashiro & Furuta - ResearchGate)](https://www.researchgate.net/publication/3668724_Energy_based_control_of_pendulum)

- **Description:** ResearchGate publication repository entry for Iwashiro and Furuta's seminal 1996 energy control formulation.
- **Cross-Platform Mirror Note:** Semantic Scholar record at [Orig #5].

#### `C3.04` [The Swing Up Control for the Pendubot Based on Energy Control Approach (IFAC 2002)](https://skoge.folk.ntnu.no/prost/proceedings/ifac2002/data/content/01524/1524.pdf)

- **Description:** IFAC 2002 conference paper extending the energy control approach to two-link underactuated robotic pendulums, analyzing energy pumping and phase-plane manifolds.

#### `C3.05` [A New Family of Smooth Strategies for Swinging up a Pendulum (Åström, Aracil, Gordillo - IFAC 2005)](https://skoge.folk.ntnu.no/prost/proceedings/ifac2005/Fullpapers/02912.pdf)

- **Description:** Foundational IFAC 2005 paper by Karl Johan Åström, Javier Aracil, and Francisco Gordillo introducing smooth (chatter-free) energy feedback strategies for swinging up pendulums without discontinuous sign functions.

#### `C3.06` [Swinging Up the Furuta Pendulum and its Stabilization Via Model Predictive Control](https://www.semanticscholar.org/paper/Swinging-up-the-Furuta-Pendulum-and-its-Via-Model-Seman-Rohal%E2%80%99-Ilkiv/66a9eca51c34ef0e3d561b432399883636d99146/figure/2)

- **Description:** Semantic Scholar paper and schematic excerpt by Seman and Rohal’-Ilkiv illustrating the two-stage hybrid control architecture: energy-based swing-up followed by linear MPC stabilization.

#### `C3.07` [Part 7: Control of Rotary Pendulum Using Julia: Swing Up Control (YouTube)](https://www.youtube.com/watch?v=RhF2NMCYoiw)

- **Description:** Hands-on tutorial implementing energy-based swing-up control and numerical integration for a rotary inverted pendulum using Julia and DifferentialEquations.jl.

#### `C3.08` [Swinging Up a Pendulum by Energy Control (Åström & Furuta, Automatica 2000 PDF)](https://web.ece.ucsb.edu/~hespanha/ece229/references/AstromFurutaAUTOM00.pdf)

- **Description:** Seminal landmark paper by Karl Johan Åström and Katsuhisa Furuta published in Automatica (Vol. 36, 2000), deriving analytical energy feedback laws and global stability proofs for pendulum swing-up.
- **Cross-Platform Mirror Note:** Semantic Scholar record at [Orig #54].

#### `C3.09` [A Swinging Up Controller for the Furuta Pendulum Based on Total Energy Control System (DML-CZ)](https://dml.cz/handle/10338.dmlcz/147844)

- **Description:** Czech Digital Mathematics Library handle for the Total Energy Control System (TECS) approach to swinging up the Furuta pendulum while simultaneously restricting base arm travel.
- **Cross-Platform Mirror Note:** Direct journal PDF on Kybernetika at [Orig #95].

#### `C3.10` [Swinging Up a Pendulum by Energy Control (Åström & Furuta - Semantic Scholar)](https://www.semanticscholar.org/paper/Swinging-up-a-pendulum-by-energy-control-%C3%85str%C3%B6m-Furuta/8a259e547088912d0323b0d4fa7d0dafde50e4ee)

- **Description:** Semantic Scholar indexed publication record for Åström and Furuta's Automatica 2000 classic.
- **Cross-Platform Mirror Note:** Direct PDF reprint at [Orig #51].

#### `C3.11` [Swing-up and Stabilisation of a Furuta Pendulum Using Energy Control (Academia.edu)](https://www.academia.edu/31024303/Swing_up_and_Stabilisation_of_a_Furuta_Pendulum_using_Energy_Control)

- **Description:** Academic paper investigating hybrid switching between Lyapunov/energy swing-up control and pole-placement/LQR stabilization within defined angle bounds.

#### `C3.12` [A Swinging Up Controller for the Furuta Pendulum Based on the Total Energy Control System Approach (Kybernetika)](https://www.kybernetika.cz/content/2019/2/402/paper.pdf)

- **Description:** Direct PDF publication in Kybernetika (Vol. 55, 2019) presenting the TECS swing-up strategy, integrating kinetic and potential energy control with horizontal base angle containment.
- **Cross-Platform Mirror Note:** DML-CZ digital library handle at [Orig #52].

---

### 4. Linear, State-Space & Classical Stabilization (LQR, Pole Placement, PID) (13 Resources)

*Linearization about the unstable upright equilibrium, Full State Feedback, Ackermann pole placement, Linear Quadratic Regulator (LQR) tuning, Kalman filtering, classical PID controllers, and disturbance rejection.*

#### `C4.01` [Design of a Two-Stage Controller for Balancing and Stabilizing the Furuta Pendulum (S. D. Pintor-Ahumada et al.)](https://pdfs.semanticscholar.org/0040/d257f0af17ea9759b8ec10b486a0372cdb37.pdf)

- **Description:** Research article published in Statistics, Optimization and Information Computing (2026) designing an integrated two-stage controller (energy swing-up + state-feedback stabilization) with experimental validation.

#### `C4.02` [Rotary Inverted Pendulum PID Control Based on STM32 (YouTube)](https://www.youtube.com/watch?v=R4VLmF_X5qM)

- **Description:** Hardware demonstration showing discrete-time PID balancing control of a rotary inverted pendulum running on an STM32 ARM Cortex-M microcontroller.

#### `C4.03` [Inverted Pendulum - MATLAB Central File Exchange #3790](https://www.mathworks.com/matlabcentral/fileexchange/3790-inverted-pendulum?s_tid=FX_rc2_behav)

- **Description:** Classic MATLAB/Simulink simulation model demonstrating state-space modeling, LQR regulator design, and linear balance stabilization.

#### `C4.04` [Inverted Pendulum Control - MATLAB Central File Exchange #40801](https://www.mathworks.com/matlabcentral/fileexchange/40801-inverted-pendulum-control)

- **Description:** Simulink library providing comparison models for classical PID versus state-space Pole Placement and LQR controllers on inverted pendulums.

#### `C4.05` [Comparative Analysis of LQR and PID Control for an Inverted Pendulum System (Extrica)](https://www.extrica.com/article/26543/pdf)

- **Description:** Journal paper providing quantitative performance comparisons (settling time, overshoot, disturbance rejection) between Linear Quadratic Regulators (LQR) and PID controllers.

#### `C4.06` [Agentic AI for Real-Time Adaptive PID Control of a Servo Motor (MDPI Robotics)](https://www.mdpi.com/2076-0825/14/9/459)

- **Description:** Published journal paper in MDPI Robotics (2026) showcasing an agentic AI framework for real-time auto-tuning and adaptive scheduling of PID parameters on dynamic servo mechanisms.
- **Cross-Platform Mirror Note:** Preprint version at [Orig #91].

#### `C4.07` [LQR Controller for Rotary Inverted Pendulum Tuning Problems (MATLAB Answers)](https://www.mathworks.com/matlabcentral/answers/2180502-lqr-controller-for-rotary-inverted-pendulum-tuning-problems)

- **Description:** MATLAB Central technical community thread addressing state weighting matrix Q and control penalty R selection in LQR synthesis for rotary inverted pendulums.
- **Original Label in Raw List:** *"LQR controller for Rotary inverted pendulum tuning problems - MATLAB Answers - MATLAB Central"*

#### `C4.08` [Full State Feedback of Furuta Pendulum - MATLAB Central File Exchange #25585](https://www.mathworks.com/matlabcentral/fileexchange/25585-full-state-feedback-of-furuta-pendulum?s_tid=prof_contriblnk)

- **Description:** Simulink implementation of full state feedback control for the Furuta pendulum using linearized state matrices and Ackermann pole placement / LQR gain derivation.

#### `C4.09` [Furuta (Rotary) Pendulum - MATLAB Central File Exchange #34521](https://www.mathworks.com/matlabcentral/fileexchange/34521-furuta-rotary-pendulum)

- **Description:** Simulink simulation and parameter definition files for dynamic modeling, nonlinear simulation, and upright linear state-space balancing of the Furuta pendulum.

#### `C4.10` [Furuta Pendulum - MATLAB Central File Exchange #23490](https://www.mathworks.com/matlabcentral/fileexchange/23490-furuta-pendulum?s_tid=FX_rc1_behav)

- **Description:** MATLAB simulation package including derivation scripts, nonlinear ODE integration, and linear quadratic regulator stabilization for the rotary inverted pendulum.

#### `C4.11` [QUBE-Servo 2 PID Controller Design Script (MathWorks File Exchange #99364)](https://www.mathworks.com/matlabcentral/fileexchange/99364-reinforcement-learning-inverted-pendulum-with-qube-servo2/files/PID/design_PID_control_ja.mlx)

- **Description:** MATLAB Live Script (`design_PID_control_ja.mlx`) providing step-by-step PID control loop synthesis and tuning for the physical Quanser QUBE-Servo 2 inverted pendulum.
- **Original Index:** `#70` *(Duplicate entries removed: `#133`)*
- **Cross-Platform Mirror Note:** Companion PPO RL script in same submission at [Orig #71].

#### `C4.12` [Agentic AI For Real-Time Adaptive PID Control of Servo Motor (Preprints.org 202508.0355)](https://www.preprints.org/manuscript/202508.0355?bm-verify=AAQAAAAO_____5Z0B4S8vilOWXyumcOFWaxkkqzhzgI-tiUoW7FX3AV2BSdMBqJ3J2eETCidQd3yU4-dghnk3vntYClc1wuEj4VOvSzFW6C65i9airRIr3qJF-Dr7hWzhmsEx6uknKr-YdpdTjSf9CWNOuq1ZKAI2aJUQyhG7lZ58YV4V6CcD3ihR32iYwIh9AfQ9lAxV_hAqMG_jPmj-7vW7VMGqrvFaSUsn1Uj9pX46w1sbiBfjv8Bl9xsIm-IUljPJtgD-C0ba6mMC5IfC4R1rlw)

- **Description:** Open preprint manuscript detailing adaptive real-time PID gain scheduling using agentic AI for precision motor and pendulum control.
- **Cross-Platform Mirror Note:** Published MDPI version at [Orig #44].

#### `C4.13` [Rotary Inverted Pendulum Simulink Model (MATLAB Answers #495573)](https://www.mathworks.com/matlabcentral/answers/495573-rotary-inverted-pendulum-simulink-model)

- **Description:** MATLAB Answers discussion providing architectural guidance and debugging tips for building nonlinear Simulink block diagrams of the Furuta pendulum from differential equations.

---

### 5. Nonlinear Model Predictive Control (NMPC), Trajectory Optimization & Advanced Methods (19 Resources)

*Real-time embedded NMPC using the acados solver suite, direct multiple shooting, Sequential Quadratic Programming (SQP) with Anderson acceleration, Sliding Mode Control (SMC), Lyapunov stability analysis, and sum-of-squares invariant region verification.*

#### `C5.01` [Inverted Pendulum With Sliding Mode Control (SMC CompE470)](https://www.scribd.com/document/214655978/Inverted-Pendulum-With-Sliding-Mode-Control-Feedback-Control-CompE470)

- **Description:** Academic report and design guide on Sliding Mode Control (SMC) for inverted pendulums, analyzing chattering suppression, sliding surface design, and Lyapunov stability.

#### `C5.02` [Underactuated Trajectory-Tracking Control for Long-Exposure Photography (F. Muratore)](https://www.ias.informatik.tu-darmstadt.de/uploads/Team/FabioMuratore/IP_UnderactuatedTrajTrackingControlForLongExposurePhotography_I_report_final.pdf)

- **Description:** Advanced research report from TU Darmstadt (Fabio Muratore) applying optimal trajectory tracking and nonlinear control to underactuated inverted pendulum systems for camera stabilization.

#### `C5.03` [Underactuated Robotics Algorithms Guide (MIT 6.832 Notes)](https://www.scribd.com/document/916728578/Underactuated-Robotics-1)

- **Description:** Comprehensive algorithmic guide covering trajectory optimization (direct collocation, shooting), dynamic programming, and convex optimization for underactuated mechanical systems.

#### `C5.04` [Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation (R. Tedrake)](https://www.scribd.com/document/779700974/Underactuated-Robotics)

- **Description:** Course text and reference by Prof. Russ Tedrake (MIT) covering non-holonomic mechanics, partial feedback linearization, energy shaping, and policy search on pendulums and legged systems.

#### `C5.05` [Underactuated Robotics Ch. 9: Lyapunov Analysis (MIT Online Textbook)](https://underactuated.mit.edu/lyapunov.html)

- **Description:** Interactive textbook chapter by Russ Tedrake focusing on Lyapunov function synthesis, sum-of-squares (SOS) optimization, and estimating regions of attraction for nonlinear inverted pendulums.

#### `C5.06` [Regions of Attraction for Limit Cycles and Non-Linear Inverted Systems (ResearchGate)](https://www.researchgate.net/figure/Regions-of-Attraction-for-the-rimless-wheel-limit-cycle-Light-shaded-region-is-the-true_fig5_47385946)

- **Description:** Scientific diagram and analysis illustrating computation of true invariant regions of attraction versus polynomial SOS estimates for underactuated limit cycle systems.

#### `C5.07` [Robotic Manipulation Course Notes (MIT / Scribd)](https://www.scribd.com/document/505953909/manipulation)

- **Description:** Course notes covering kinematic chains, force-torque control, actuator saturation limits, and optimization-based control of robotic manipulators.

#### `C5.08` [Methodology for Executing Optimal Nonlinear Trajectories on Resource-Constrained Hardware (LSU Thesis)](https://repository.lsu.edu/cgi/viewcontent.cgi?article=1092&context=lsu_control)

- **Description:** Master's thesis from Louisiana State University establishing real-time execution pipelines for nonlinear optimal trajectories on embedded microcontrollers.

#### `C5.09` [Related Projects — acados Documentation](https://docs.acados.org/list_of_projects/index.html)

- **Description:** Official documentation page listing research and industrial robotics projects powered by the high-performance acados NMPC and SQP solver suite.

#### `C5.10` [Towards Event-Triggered NMPC for Efficient Communications (arXiv 2409.18589)](https://arxiv.org/pdf/2409.18589)

- **Description:** Research preprint evaluating event-triggered Nonlinear Model Predictive Control architectures to minimize computational overhead and communication bandwidth in real-time control loops.

#### `C5.11` [Direct Multiple Shooting Formulation Architecture (ResearchGate)](https://www.researchgate.net/figure/A-schematic-of-a-direct-multiple-shooting-formulation-Different-intermediate-states-are_fig2_363735005)

- **Description:** Technical diagram illustrating state discretization, continuity constraints, and boundary conditions in direct multiple shooting trajectory optimization for nonlinear systems.

#### `C5.12` [Tracking Control of Unforced and Forced Equilibrium of Pendubot: Nonlinear MHE and MPC (MDPI)](https://www.mdpi.com/2076-0825/12/9/343)

- **Description:** MDPI Robotics paper implementing Moving Horizon Estimation (MHE) and Nonlinear Model Predictive Control (NMPC) for state estimation and stabilization of an underactuated double pendulum.

#### `C5.13` [acados Official GitHub Releases & Source Code](https://github.com/acados/acados/releases)

- **Description:** GitHub repository releases for acados, the state-of-the-art fast solver package for nonlinear optimal control and real-time NMPC based on Sequential Quadratic Programming.

#### `C5.14` [Real-World Examples & Benchmarks — acados Documentation](https://docs.acados.org/real_world_examples/index.html)

- **Description:** Tutorials and benchmarking examples demonstrating physical deployment of acados NMPC controllers on real-time embedded hardware.

#### `C5.15` [Swinging Up a Custom-Made Furuta Pendulum with NMPC Using Acados (YouTube)](https://www.youtube.com/shorts/oJYyD5beMqM)

- **Description:** Real-world demonstration video showcasing swing-up and balance stabilization of a physical Furuta pendulum driven by real-time acados NMPC running at 100+ Hz.

#### `C5.16` [Systems Control and Optimization Laboratory Publications (syscop, Univ. Freiburg)](https://www.syscop.de/publications)

- **Description:** Academic publication catalogue of the syscop laboratory (Prof. Moritz Diehl), originators of acados, qpOASES, and fast multiple shooting SQP algorithms for real-time robotics.

#### `C5.17` [Hannes Homburger Academic Profile (HTWG Konstanz)](https://www.htwg-konstanz.de/forschung-und-transfer/institute-und-labore/isd/regelungstechnik/team/hannes-homburger)

- **Description:** Faculty research page of Hannes Homburger, developer of embedded NMPC swing-up implementations for rotary inverted pendulums using acados and SQP.

#### `C5.18` [Anderson Acceleration for Linearly Converging SQP-Type Methods (Technical Review)](https://www.themoonlight.io/zh/review/anderson-acceleration-for-linearly-converging-sqp-type-methods)

- **Description:** In-depth mathematical commentary analyzing the convergence speedup and computational efficiency of applying Anderson acceleration to sequential quadratic programming solvers.
- **Cross-Platform Mirror Note:** ArXiv paper at [Orig #205].

#### `C5.19` [Anderson Acceleration for Linearly Converging SQP-Type Methods (arXiv 2604.14803)](https://arxiv.org/pdf/2604.14803)

- **Description:** Mathematical optimization preprint developing theoretical guarantees and numerical performance for Anderson-accelerated SQP methods in real-time optimal control.

---

### 6. Deep Reinforcement Learning & Continuous Control (20 Resources)

*Model-free and model-based deep reinforcement learning (PPO, SAC, Monte Carlo methods, ODE-based continuous-time models), CleanRL canonical implementations, Stable-Baselines3, skrl, and comparative benchmarks against classical LQR/PID controllers.*

#### `C6.01` [Efficient Exploration in Continuous-Time Model-Based Reinforcement Learning (NeurIPS / Bytez)](https://bytez.com/docs/neurips/71440/paper?_c=eyJ2IjoxLCJyZWxhdGVkIjpbImNvZGUiLCJyZWZlcmVuY2VzIiwiY29uZmVyZW5jZSJdfQ%3D%3D)

- **Description:** NeurIPS research paper investigating continuous-time ODE dynamics models in model-based RL to drastically improve sample efficiency on underactuated pendulum tasks.
- **Cross-Platform Mirror Note:** OpenReview PDF at [Orig #84].

#### `C6.02` [Reinforcement Learning-Based Transition Control of Rotary Inverted Pendulum (MDPI)](https://www.mdpi.com/2227-7390/14/18/3319)

- **Description:** MDPI Mathematics paper designing a deep reinforcement learning policy specifically for managing the tricky transition region between swing-up and upright balancing.

#### `C6.03` [QUBE-Servo 2 PPO Reinforcement Learning Script (MathWorks File Exchange #99364)](https://www.mathworks.com/matlabcentral/fileexchange/99364-reinforcement-learning-inverted-pendulum-with-qube-servo2/files/RL/design_RL_multi_control_PPO_ja.mlx)

- **Description:** MATLAB Live Script (`design_RL_multi_control_PPO_ja.mlx`) training a Proximal Policy Optimization (PPO) agent to balance the Quanser QUBE-Servo 2 inverted pendulum.
- **Cross-Platform Mirror Note:** Companion PID script in same submission at [Orig #70].

#### `C6.04` [Modeling, Simulation, and Control of Rotary Inverted Pendulum: An RL Approach (ResearchGate)](https://www.researchgate.net/publication/386174932_Modeling_Simulation_and_Control_of_a_Rotary_Inverted_Pendulum_A_Reinforcement_Learning-Based_Control_Approach)

- **Description:** ResearchGate preprint on modeling, simulation, and training continuous reinforcement learning agents to swing up and balance a rotary inverted pendulum.
- **Cross-Platform Mirror Note:** Published MDPI journal paper at [Orig #86].

#### `C6.05` [Improving Sample Efficiency and Robustness in Deep Reinforcement Learning (M. Hensel, TU Darmstadt)](https://www.ias.informatik.tu-darmstadt.de/uploads/Site/EditPublication/maximilian_hensel.pdf)

- **Description:** Academic thesis from TU Darmstadt investigating policy optimization, domain randomization, and sample efficiency on physical and simulated robotics control tasks.

#### `C6.06` [Comparative Study: RL Control Model vs. Conventional PID Control (ResearchGate)](https://www.researchgate.net/figure/Compare-reinforcement-learning-control-model-with-conventional-PID-control-model-and_fig2_373540444)

- **Description:** Benchmarking diagram and analysis comparing the dynamic response, robustness to external impulse disturbances, and overshoot between RL neural policies and classical PID controllers.

#### `C6.07` [Sim-to-Real Reinforcement Learning for Rotary Double-Inverted Pendulum (MDPI)](https://www.mdpi.com/2227-7390/13/12/1996)

- **Description:** MDPI Mathematics paper demonstrating sim-to-real transfer of deep reinforcement learning policies onto an underactuated rotary double-inverted pendulum mechanism.

#### `C6.08` [Research Projects — Yunlong Song (Robotics & RL)](https://yunlong-song.com/project.html)

- **Description:** Personal research portfolio of Yunlong Song (University of Zurich / MIT) showcasing agile robotics control, model-based RL, and neural policy deployment.

#### `C6.09` [Efficient Exploration in Continuous-Time Model-Based RL (OpenReview PDF)](https://openreview.net/pdf?id=VkhvDfY2dB)

- **Description:** Full-text PDF of the NeurIPS paper detailing continuous-time model-based reinforcement learning and exploration heuristics on continuous benchmark environments.
- **Cross-Platform Mirror Note:** Bytez publication viewer at [Orig #24].

#### `C6.10` [Stable Baselines3 Comprehensive Documentation (PDF)](https://stable-baselines3.readthedocs.io/_/downloads/en/master/pdf/)

- **Description:** Official PDF manual for Stable-Baselines3, the standard PyTorch implementation of deep reinforcement learning algorithms (PPO, SAC, TD3, A2C).

#### `C6.11` [Modeling, Simulation, and Control of Rotary Inverted Pendulum: An RL Approach (MDPI)](https://www.mdpi.com/2673-3951/5/4/95)

- **Description:** Published open-access paper in MDPI AI detailing physics modeling, simulated gym environment creation, and actor-critic RL training for rotary inverted pendulums.
- **Cross-Platform Mirror Note:** ResearchGate version at [Orig #79].

#### `C6.12` [Sensitivity-Based Robustness Evaluation of RL Controller for Furuta Pendulum (MDPI)](https://www.mdpi.com/2673-4591/157/1/2)

- **Description:** MDPI Engineering paper analyzing parameter sensitivity (variations in arm inertia, link mass, damping) of neural network RL policies deployed on the Furuta pendulum.

#### `C6.13` [Real Life Reinforcement Learning: Building and Training a CartPole (Video & GitHub)](https://www.youtube.com/watch?v=Y6FVBbqjR40) | [GitHub Repository](https://github.com/Armandpl/furuta)

- **Description:** Comprehensive video guide by Armand Laplace demonstrating end-to-end design, physical fabrication, and PPO reinforcement learning training on physical pendulum hardware, linking the official [Armandpl/furuta](https://github.com/Armandpl/furuta) repository.

#### `C6.14` [Using the Reinforcement Learning Toolbox to Balance an Inverted Pendulum (Quanser)](https://www.quanser.com/blog/artificial-intelligence/using-the-reinforcement-learning-toolbox-to-balance-an-inverted-pendulum/)

- **Description:** Official Quanser tutorial showing integration between MATLAB Reinforcement Learning Toolbox and Quanser QUBE-Servo hardware for training neural balance policies.

#### `C6.15` [Deep Reinforcement Learning for Dynamical Systems (G. C. Andrei Master's Thesis)](https://w3.cran.univ-lorraine.fr/mayank-shekhar.jha/sites/w3.cran.univ-lorraine.fr.mayank-shekhar.jha/files/Claudiu_Stage_diffuse_version.pdf)

- **Description:** Master's thesis from Université de Lorraine & Politecnico di Torino evaluating deep reinforcement learning algorithms on physical continuous dynamic systems.

#### `C6.16` [Soft Actor-Critic (SAC) Algorithm Guide & Implementation (CleanRL)](https://docs.cleanrl.dev/rl-algorithms/sac/)

- **Description:** CleanRL clean single-file PyTorch implementation guide for the Soft Actor-Critic (SAC) continuous maximum-entropy reinforcement learning algorithm.

#### `C6.17` [Proximal Policy Optimization (PPO) Algorithm Guide & Implementation (CleanRL)](https://docs.cleanrl.dev/rl-algorithms/ppo/)

- **Description:** CleanRL canonical single-file implementation documentation for continuous action space Proximal Policy Optimization (PPO).

#### `C6.18` [Reinforcement Learning for Robotics: Study and Research Notes (Yu Zhang)](https://yuzhangai.com/blogs/rl-notes/posts/roadmap.html)

- **Description:** Comprehensive study notes covering robotics RL roadmaps, reward function engineering, continuous action spaces, and sim-to-real transfer techniques.

#### `C6.19` [Implementation of 5 RL Algorithms for CartPole Gym Environment (jankrepl/CartPole_v0)](https://github.com/jankrepl/CartPole_v0)

- **Description:** GitHub repository implementing 5 different classical and deep reinforcement learning algorithms (DQN, Policy Gradients, Actor-Critic, etc.) on the inverted pendulum CartPole benchmark.

#### `C6.20` [RL Agent Learns Physics Using Monte Carlo Methods on CartPole (YouTube)](https://www.youtube.com/watch?v=RrhVLzdmJ34)

- **Description:** Educational visual demonstration showing an agent discovering pendulum balancing dynamics using first-principles Monte Carlo reinforcement learning methods.

---

### 7. Sim-to-Real Transfer, Domain Randomization & Physics Simulators (13 Resources)

*High-fidelity physics engines (MuJoCo, Drake, NVIDIA Isaac Sim / Isaac Lab, MATLAB Simscape Multibody), domain randomization techniques (Bayesian optimization, neural posterior domain randomization), and closing the sim-to-real transfer gap.*

#### `C7.01` [Controlling the Furuta Pendulum: Proof of Concept Through Virtual Prototyping (SciELO)](https://www.scielo.org.mx/scielo.php?script=sci_arttext&pid=S1665-64232024000300327)

- **Description:** SciELO Mexico open-access publication demonstrating virtual prototyping and multibody co-simulation of the Furuta pendulum before physical deployment.
- **Cross-Platform Mirror Note:** Journal portal on JART at [Orig #60].

#### `C7.02` [Microduck: Training Walking RL PPO Policy with Mjlab (YouTube)](https://www.youtube.com/watch?v=ZDvCdIFXANw)

- **Description:** Tutorial on setting up MuJoCo-based physics environments using Mjlab for training GPU-accelerated PPO continuous control policies.

#### `C7.03` [Controlling the Furuta Pendulum: Proof of Concept Through Virtual Prototyping (JART)](https://jart.icat.unam.mx/index.php/jart/article/view/2347)

- **Description:** Journal of Applied Research and Technology (UNAM) article detailing CAD integration, multibody physics modeling, and virtual prototyping of rotary pendulums.
- **Cross-Platform Mirror Note:** SciELO portal at [Orig #1].

#### `C7.04` [Inverted Pendulum with Animation (Simulink Example)](https://www.mathworks.com/help/simulink/slref/inverted-pendulum-with-animation.html)

- **Description:** MathWorks official documentation example demonstrating dynamic modeling and synchronized 3D graphical animation of an inverted pendulum in Simulink.

#### `C7.05` [Matching MuJoCo Simulation to Real Hardware for Sim-to-Real Transfer (Kevin Wood #4)](https://www.youtube.com/watch?v=gBeU6EHJLJk)

- **Description:** Detailed video walkthrough calibrating MuJoCo physics parameters (viscous damping, dry friction, actuator latency) against physical Furuta pendulum telemetry.

#### `C7.06` [Neural Posterior Domain Randomization (F. Muratore et al., PMLR 2022)](https://proceedings.mlr.press/v164/muratore22a/muratore22a.pdf)

- **Description:** Proceedings of Machine Learning Research (PMLR) paper by Fabio Muratore et al. formulating Bayesian domain randomization for robust sim-to-real transfer on physical robotics platforms.

#### `C7.07` [Data-Efficient Domain Randomization with Bayesian Optimization (alphaXiv)](https://www.alphaxiv.org/abs/2003.02471)

- **Description:** AlphaXiv research preprint utilizing Bayesian optimization to automatically discover domain randomization distributions that close the sim-to-real gap efficiently.

#### `C7.08` [Explore Isaac Sim and Isaac Lab for Robotic Workflows on DGX Spark (Arm Learning)](https://learn.arm.com/learning-paths/laptops-and-desktops/dgx_spark_isaac_robotics/1_introduction_isaac/)

- **Description:** Arm Developer guide introducing NVIDIA Isaac Sim and Isaac Lab architectures for GPU-accelerated robotics physics and reinforcement learning pipelines.

#### `C7.09` [Getting Started With Isaac Lab: Architecture & Core Workflows (NVIDIA Docs)](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-lab/latest/an-introduction-to-robot-learning-and-isaac-lab/04-tying-it-all-together/01-tying-it-together.html)

- **Description:** NVIDIA official documentation detailing environment configuration, GPU-parallel vectorization, and robot policy learning in Isaac Lab.

#### `C7.10` [Introduction to Isaac Lab Reinforcement Learning (Toward Humanoids / Medium)](https://medium.com/correll-lab/introduction-to-isaac-lab-reinforcement-learning-with-the-isaac-humanoid-062a4c3f6b99)

- **Description:** Hands-on guide explaining asset ingestion, reward function configuration, and observation space design in NVIDIA Isaac Lab.

#### `C7.11` [Drake: MultibodyPlant Dynamics in MathematicalProgram (Stack Overflow)](https://stackoverflow.com/questions/71085209/multibodyplant-s-dynamics-as-part-of-a-python-function-lambda-constraint-in-a-ma)

- **Description:** Technical programming discussion on embedding Drake's MultibodyPlant symbolic and autodiff dynamics constraints into direct collocation trajectory optimization.

#### `C7.12` [Drake C++ API Documentation: Class List](https://drake.mit.edu/doxygen_cxx/annotated.html)

- **Description:** Complete class index for the Drake robotics simulation and verification engine, covering MultibodyPlant, SceneGraph, and mathematical solvers.

#### `C7.13` [Isaac Lab Environments Integration — skrl Documentation](https://skrl.readthedocs.io/en/1.4.2/api/envs/isaaclab.html)

- **Description:** Documentation for the skrl reinforcement learning library demonstrating high-throughput vectorized policy training on NVIDIA Isaac Lab environments.

---

### 8. Embedded Hardware Implementation, Microcontrollers & Robotics Middleware (10 Resources)

*Embedded electronics, microcontrollers (STM32 ARM Cortex-M, Arduino Nano, ESP32 dual-processor architecture), real-time control loops, micro-ROS integration, micro-ROS vs. UART latency trade-offs, ros2_control, actuators (MyActuator BLDC, DC motors), and optical/magnetic encoders.*

#### `C8.01` [Assembling a Rotary Inverted Pendulum with CUI Encoder & MyActuator (Kevin Wood #2)](https://www.youtube.com/watch?v=nJnQZFKTkG0)

- **Description:** Hardware assembly guide detailing high-resolution CUI AMT optical/magnetic rotary encoders, MyActuator brushless BLDC actuator integration, and mechanical alignment.

#### `C8.02` [3 Types of SLAM Explained Clearly: LiDAR vs. Visual vs. Visual-Inertial (YouTube)](https://www.youtube.com/watch?v=LNsmo4UQIFU)

- **Description:** Conceptual tutorial on spatial state estimation, sensor fusion, and inertial measurement unit (IMU) processing relevant to mobile and inverted robotic systems.

#### `C8.03` [ferrolho/rotary-inverted-pendulum: £20 Arduino Nano Neural Pendulum (GitHub)](https://github.com/ferrolho/rotary-inverted-pendulum)

- **Description:** Landmark open-source hardware project by Henrique Ferrolho deploying a 689-parameter neural network directly onto an 8-bit Arduino Nano to balance a low-cost rotary inverted pendulum.
- **Cross-Platform Mirror Note:** Media coverage at [Orig #90].

#### `C8.04` [ferrolho £20 Arduino Nano Neural Inverted Pendulum Review (RoboAIRadar)](https://roboairadar.com/news/cms0i8h0b000e04jxaibz2nb5)

- **Description:** Technical overview and engineering analysis of the Ferrolho low-cost Arduino Nano neural control system and embedded inference implementation.
- **Cross-Platform Mirror Note:** GitHub repository at [Orig #50].

#### `C8.05` [micro-ROS Project Overview (Open Source Robotics)](https://robotics.growbotics.ai/projects/software/micro-ros)

- **Description:** Introduction to the micro-ROS framework bringing standard ROS 2 DDS/eProsima Micro XRCE-DDS communications directly to microcontrollers.

#### `C8.06` [Micro-ROS on STM32: Complete How-To Guide for MCUs (Atlantis Robotics)](https://thinkrobotics.com/blogs/learn/micro-ros-on-stm32-complete-how-to-guide-for-ros-2-on-mcus)

- **Description:** Comprehensive implementation guide for running micro-ROS on STM32 microcontrollers using FreeRTOS, configuring serial/USB transports, and publishing sensor telemetry.

#### `C8.07` [Less is More: Why We Abandoned Micro-ROS for UART (The Newton Capstone)](https://thenewtoncapstone.github.io/blog/posts/issues-with-microros/)

- **Description:** Critical engineering case study explaining real-time latency bottlenecks, memory footprint overhead, and packet loss issues with micro-ROS, and why lightweight binary UART was preferred.

#### `C8.08` [ESP32 Micro-ROS Mobile Robot Project (GitHub)](https://github.com/EmDonato/Esp32_micro_Ros_mobile_robot_Leonardo)

- **Description:** Complete Arduino IDE & ESP32 implementation demonstrating micro-ROS node initialization, publisher/subscriber setup, and real-time motor control over Wi-Fi/Serial.

#### `C8.09` [micro-ROS Support in ros2_control (GitHub Issue #2171)](https://github.com/ros-controls/ros2_control/issues/2171)

- **Description:** Architecture discussion on integrating micro-ROS hardware interfaces into the standard ROS 2 ros2_control loop for real-time motor command dispatch.

#### `C8.10` [TeeterTotterBot: Dual-Processor ESP32 & STM32 Balancing Robot (GitHub)](https://github.com/ammaarrahmed/TeeterTotterBot)

- **Description:** Open-source balancing robot utilizing a dual-microcontroller architecture: an STM32 dedicated to hard real-time PID feedback control and an ESP32 dedicated to ROS 2 wireless communication.

---

### 9. Real-Time Dashboards, Telemetry, Data Logging & Visualization (17 Resources)

*Live operator interfaces, MATLAB App Designer serial streaming, Simulink Real-Time (SLRT / Speedgoat), modern robotics telemetry platforms (Foxglove Studio, MCAP container format, Rerun SDK, RViz), ROS 2 web bridges, and NVIDIA Holoscan edge streaming.*

#### `C9.01` [Inverted Pendulum Simulation and Animation Video Export (MathWorks File Exchange #182938)](https://www.mathworks.com/matlabcentral/fileexchange/182938-inverted-pendulum-simulation-and-animation-output-mp4-jpg?s_tid=prof_contriblnk)

- **Description:** MATLAB tool for rendering and exporting high-resolution MP4 and JPG animations of simulated inverted pendulum state trajectories.

#### `C9.02` [Creating an SLRT App with App Designer to Control & Monitor Real-Time Applications (MATLAB Answers)](https://www.mathworks.com/matlabcentral/answers/517628-how-can-i-create-an-slrt-app-with-app-designer-to-control-tune-and-monitor-my-real-time-applicatio)

- **Description:** Step-by-step tutorial on building a MATLAB App Designer dashboard that interfaces with Simulink Real-Time (SLRT) target machines for live parameter tuning and telemetry.

#### `C9.03` [B&R Automation Studio Target for Simulink (TM140 Training Manual)](https://kyb.fei.tuke.sk/laboratoria/prezentacie/br%20automation%20studio%20target%20for%20simulink.pdf)

- **Description:** Industrial training manual detailing code generation, real-time targets, and communication between MATLAB/Simulink and B&R industrial automation hardware.

#### `C9.04` [Add App Designer App to Inverted Pendulum Model (Simulink Real-Time Docs)](https://www.mathworks.com/help/slrealtime/ug/add-app-des-app-to-inverted-pendulum-model.html)

- **Description:** MathWorks tutorial explaining how to connect an interactive App Designer GUI to a physical or simulated inverted pendulum running under Simulink Real-Time.

#### `C9.05` [Implementing Real-Time Serial Logging in MATLAB App Designer (MATLAB Answers #847220)](https://www.mathworks.com/matlabcentral/answers/847220-implementing-real-time-serial-logging-to-app-designer)

- **Description:** Detailed guide on non-blocking serial communication, byte stream parsing, and real-time time-series strip-chart plotting inside MATLAB App Designer.

#### `C9.06` [Real-Time Testing Video Series (MathWorks Videos)](https://www.mathworks.com/videos/search.html?q=real-time%20testing&fq%5B%5D=marketing-capability:algorithm-development&page=1)

- **Description:** Curated video playlist covering hardware-in-the-loop (HIL) testing, rapid control prototyping, and real-time signal logging in MATLAB and Simulink.

#### `C9.07` [Open Source Foxglove Alternatives: RViz and Rerun Comparison (ReductStore)](https://www.reduct.store/blog/comparison-rviz-foxglove-rerun)

- **Description:** In-depth technical comparison evaluating Foxglove Studio, Rerun SDK, and RViz for robotic telemetry visualization, high-frequency logging, and multimodal sensor review.

#### `C9.08` [Hello Robot Stretch Core Framework Architecture Docs](https://docs.hello-robot.com/stretch4-body-repo/core-framework/primer_core)

- **Description:** Documentation of modular robotic system architectures, separating real-time low-level joint motor drivers from high-level visualization and planning pipelines.

#### `C9.09` [The MCAP Dilemma: Why Robot Data Has Three Jobs (Hugging Face Blog)](https://huggingface.co/blog/gagandeepreehal/the-mcap-dilemma)

- **Description:** Architecture essay explaining the design advantages of the MCAP container format (self-contained schemas, indexed chunks, zero-copy reads) for robotics telemetry.

#### `C9.10` [Foxglove Deep Dive: Robotics Data Platform, MCAP & Alternatives (Black Scarab)](https://www.blackscarab.ai/insights/foxglove-robotics-data-platform-deep-dive)

- **Description:** Technical and commercial overview of Foxglove Studio for robot log playback, custom dashboard visualization, and data lake integration.

#### `C9.11` [Foxglove Fleet Management & Edge Telemetry Platform](https://foxglove.dev/product/fleet)

- **Description:** Product documentation for Foxglove Fleet, enabling automatic upload, indexing, and visualization of robot telemetry bags and MCAP files.

#### `C9.12` [Importing Data into Foxglove Studio (Foxglove Docs)](https://docs.foxglove.dev/docs/data/importing-data)

- **Description:** Official guide on importing MCAP, ROS 1 bag, ROS 2 bag, and custom CSV logs into Foxglove Studio for interactive timeline inspection.

#### `C9.13` [NVIDIA Holoscan Operators (HoloHub Ecosystem)](https://nvidia-holoscan.github.io/holohub/operators/)

- **Description:** Collection of low-latency streaming and sensor processing operators for NVIDIA Holoscan, designed for real-time edge AI and robotics.

#### `C9.14` [Visualizing Datasets with the Rerun SDK (DEV Community)](https://dev.to/rerunio/visualize-objectron-dataset-need-visuals-a3c)

- **Description:** Practical guide demonstrating how to log time-varying spatial data, coordinate frames, and 3D bounding boxes using the lightweight Rerun visualization SDK.

#### `C9.15` [Zero2Robot Ch. 0.4: Teleoperation & Recording Your First Dataset](https://zero2robot.com/ch0.4-record/)

- **Description:** Educational curriculum chapter on recording continuous robot state-action trajectories, clock synchronization, and data format management for robot learning.

#### `C9.16` [ROS 2 Fleet Management Dashboard via ROSBridge (RoboCloud)](https://robocloud-dashboard.vercel.app/learn/blog/ros-bridge-integration)

- **Description:** Architecture guide on bridging ROS 2 WebSocket topics through `rosbridge_suite` to web-based dashboards for real-time monitoring.

#### `C9.17` [Rerun SDK Code Snippets and Documentation Index (GitHub)](https://github.com/rerun-io/rerun/blob/main/docs/snippets/INDEX.md)

- **Description:** Comprehensive code snippet index demonstrating Python, C++, and Rust APIs for streaming real-time plots, transforms, and matrices into Rerun.

---

### 10. Open-Source Physical Builds, Laboratory Workbooks & Curated Vaults (30 Resources)

*Replicable open-source hardware builds (Armandpl/furuta, ferrolho £20 Arduino build, bgkatz direct-drive build, SabirHusnain IP-Control), Kevin Wood robotics build logs, Quanser student and instructor lab courseware, project review notes, and shared Google Drive archives.*

#### `C10.01` [Armandpl/furuta Git Commit History (GitHub)](https://github.com/Armandpl/furuta/commits?after=c360dc5b2bad0cea04ef70733956f5b52a243c07+139)

- **Description:** Commit log tracking development iterations, firmware improvements, and CAD revisions of the open-source Furuta pendulum robot by Armand Laplace.

#### `C10.02` [Armandpl/furuta Software Instruction Link Troubleshooting (Issue #115)](https://github.com/Armandpl/furuta/issues/115)

- **Description:** Community issue resolving broken setup documentation links and providing clear instructions for configuring the Python/Gymnasium control environment.

#### `C10.03` [Pier Fabre GitHub Profile (Robotics & Open-Source Projects)](https://github.com/pierfabre)

- **Description:** Developer profile contributing to physical mechatronics designs, motor driver boards, and inverted pendulum hardware projects.

#### `C10.04` [Physical AI Robotics Projects (Kevin Wood Robotics)](https://kevinwoodrobotics.com/product/physical-ai-robotics-projects/)

- **Description:** Course and project portfolio by Kevin Wood detailing hardware component selection, motor sizing, 3D printing, and sim-to-real transfer.

#### `C10.05` [CIROS Education Robotics: Developing First Melfa Basic IV Program (YouTube)](https://www.youtube.com/watch?v=6xdgw8B1SCE)

- **Description:** Educational video on industrial robotics programming and simulation environments for mechatronics education.

#### `C10.06` [Building Another Furuta Pendulum Hardware Build Log (YouTube)](https://www.youtube.com/watch?v=1GOXuu1OZ3k)

- **Description:** Makerspace video walkthrough demonstrating physical machining, 3D printing, bearing assembly, and wiring of a DIY rotary inverted pendulum.
- **Course Handout Reference:** Formally recommended in Ain Shams University MCT411 / MCT413s Project Description §9.1 (Build Examples and Tutorials). See also companion Dagor controller build log at [`C10.20`](#c1020-furuta-pendulum-build-with-dagor-brushless-controller-dagordev).

#### `C10.07` [Desktop Inverted Pendulum Part 2: Control Loop Implementation (Robot Daycare)](https://robot-daycare.com/posts/2016-08-26-desktop-inverted-pendulum-part-2-control/)

- **Description:** Engineering blog post detailing real-time sensor sampling, microcontroller firmware structure, and control loop debugging on a desktop inverted pendulum.

#### `C10.08` [Quanser QUBE-Servo 2 User Manual](https://manuals.plus/m/ec3a4b23ac24cda93b3b79da157bec38b85cd363bd2365b4ebc718ad27be92a5)

- **Description:** Hardware user manual for the standard Quanser QUBE-Servo 2 benchmark apparatus, detailing electrical specifications, pinouts, encoder CPR, and motor ratings.

#### `C10.09` [Control of a Rotary Inverted Pendulum System Using Computer Vision (S. Á. Jakobsson Thesis)](https://skemman.is/bitstream/1946/47667/1/T_899_MEIS_Thesis-Sigur%C3%B0urJakobsson_20240317.pdf)

- **Description:** Master's thesis from Reykjavík University investigating contactless pendulum angle feedback and stabilization using high-speed computer vision.

#### `C10.10` [SabirHusnain/IP-Control: Inverted Pendulum Modeling and Control (GitHub)](https://github.com/SabirHusnain/IP-Control)

- **Description:** Open-source GitHub repository containing MATLAB/Simulink models and scripts for simulating and tuning inverted pendulum controllers.

#### `C10.11` [README Documentation Standards & Conventions (Wikipedia)](https://en.wikipedia.org/wiki/README)

- **Description:** Reference overview on software documentation structure, setup guides, and repository onboarding best practices.

#### `C10.12` [Technosys Advanced Control Systems Educational Equipment](https://www.technosysindia.in/advanced-control-systems-automation-control.html)

- **Description:** Commercial laboratory apparatus catalogue covering industrial and academic rotary inverted pendulum training stations.

#### `C10.13` [ICCAS 2026 International Conference on Control Program](https://ras.papercept.net/conferences/conferences/ICCAS2026/program/ICCAS2026_ContentListWeb_2.html)

- **Description:** Conference schedule and technical session listing for recent advancements in automatic control, robotics, and cyber-physical systems.

#### `C10.14` [OEMC Architecture Development and Pendulum Control (TU Eindhoven Master's Thesis)](https://pure.tue.nl/ws/portalfiles/portal/397418965/1976346-OEMCArchitectureDevelopmentAndPendulumControl.pdf.pdf)

- **Description:** Master's thesis from Eindhoven University of Technology designing an Open Embedded Motion Control (OEMC) architecture for precision pendulum control.

#### `C10.15` [20th IFAC World Congress Proceedings Table of Contents](https://www.proceedings.com/content/036/036872webtoc.pdf)

- **Description:** Proceedings index and session table of contents from the 20th IFAC World Congress (Toulouse, France), listing key papers in nonlinear mechanics and robotics.

#### `C10.16` [GitHub Topics: inverted-pendulum](https://github.com/topics/inverted-pendulum)

- **Description:** Curated index of open-source inverted pendulum repositories across GitHub spanning C++, Python, Arduino, MATLAB, and Julia implementations.

#### `C10.17` [Quanser QUBE-Servo Student Workbook (Scribd)](https://www.scribd.com/document/580305998/QUBE-Servo-Workbook-Student)

- **Description:** Official student laboratory courseware covering system identification, bump testing, pole placement, and LQR balancing on the QUBE-Servo.

#### `C10.18` [Quanser QUBE-Servo 2 Swing-Up Control Instructor Guide (Scribd)](https://www.scribd.com/document/860697901/QUBE-Servo-2-Swing-Up-Control-Workbook-Instructor)

- **Description:** Instructor laboratory manual detailing energy-based swing-up parameter tuning, switching condition thresholds, and experimental verification procedures.

#### `C10.19` [Armandpl/furuta: Building and Training a Rotary Inverted Pendulum Robot (GitHub)](https://github.com/Armandpl/furuta)

- **Description:** Main repository for the popular 3D-printable rotary inverted pendulum robot, featuring CAD STL files, electronic BOM, motor drivers, and Gymnasium RL environments.
- **Original Index:** `#48` *(Duplicate entries removed: `#172`)*
- **Original Label in Raw List:** *"GitHub - Armandpl/furuta: Building and Training a Rotary Inverted Pendulum robot · GitHub"*

#### `C10.20` [Furuta Pendulum Build with Dagor Brushless Controller (Dagor.dev)](https://www.dagor.dev/blog/furuta-pendulum)

- **Description:** Detailed build log by David Gonzalez featuring a custom Furuta pendulum powered by field-oriented control (FOC) on the open-source Dagor brushless controller.

#### `C10.21` [Mechatronic Systems Book Edition (MDPI)](https://mdpi-res.com/bookfiles/book/12853/Mechatronic_Systems.pdf?v=1790731160)

- **Description:** Complete academic reprint book compilation published by MDPI covering mechatronic system modeling, sensor integration, and advanced motion control.

#### `C10.22` [Quanser QUBE-Servo 2 Courseware for MATLAB/Simulink (Scribd)](https://www.scribd.com/document/486299841/made-for-science-Quanser-qube-servo-2-CoursewareStud-MATLAB-pdf)

- **Description:** Laboratory course manual outlining experiment workflows, Simulink QUARC blocks, and state estimation for the QUBE-Servo 2.
#### `C10.23` [Furuta Pendulum Literature Review Working Document (Google Docs)](https://docs.google.com/document/d/1FafBFPmdkxIdFKBmrCwpmW74ychCuQ6cblw2YOzWr3g/edit?tab=t.0)

- **Description:** Collaborative project working document containing literature notes, bibliographic summaries, and comparative controller analyses.

#### `C10.24` [bgkatz/Furuta-Pendulum: Custom Rotary Inverted Pendulum (GitHub)](https://github.com/bgkatz/Furuta-Pendulum)

- **Description:** Hardware repository by Ben Katz featuring compact direct-drive brushless motor integration, optical encoder capture, and high-frequency balancing firmware.

#### `C10.25` [Inverted Pendulum Arduino Balancing Robot (YouTube)](https://www.youtube.com/watch?v=UL99bJTtESk)

- **Description:** Step-by-step video demonstration building and tuning an Arduino-controlled self-balancing inverted pendulum robot.

#### `C10.26` [Rotary Inverted Pendulum: Requirements, Use, and Source Availability (Open Source Robotics)](https://opensourcerobotics.dev/robots/rotary-inverted-pendulum/)

- **Description:** Comprehensive hardware directory evaluating bill-of-materials costs, manufacturing difficulty, and software ecosystems for open-source rotary pendulums.

#### `C10.27` [Reinforcement Learning on Hardware from Sim-to-Real: Rotary Inverted Pendulum (Kevin Wood)](https://kevinwoodrobotics.com/product/reinforcement-learning-on-hardware-from-sim-to-real-rotary-inverted-pendulum/)

- **Description:** Complete project documentation and digital asset bundle covering CAD, hardware selection, and Isaac Sim / MuJoCo sim-to-real transfer.

#### `C10.28` [Kevin Wood Robotics & AI YouTube Channel](https://www.youtube.com/channel/UC0jeEKE9IzfYKDuIbQLJIog/about)

- **Description:** YouTube creator channel featuring comprehensive video series on physical robotics builds, sim-to-real reinforcement learning, and actuator calibration.

#### `C10.29` [Learn Robotics — Free Courses, Exercises & Research Guides (Open Source Robotics)](https://opensourcerobotics.dev/learn/)

- **Description:** Curated portal of free online robotics engineering courses, mathematical foundations, and practical project roadmaps.

#### `C10.30` [Furuta Pendulum Project Assignment 2 Shared Resources (Google Drive)](https://drive.google.com/drive/folders/1QhM0O3nyDqAKWpBcj2MteviHZyJ3M7FN)

- **Description:** Course shared Google Drive folder containing assignment instructions, template files, dataset logs, and reference project materials.

---

### 11. Course Specification Benchmark References & Ain Shams University Legacy Projects (10 Resources)

*Official reference videos and benchmark resources prescribed in Section 9 of the Ain Shams University Mechatronics Engineering Department project specification document (`Project Description.pdf` for MCT411 / MCT413s, Fall 2026). Encompasses physical hardware build walkthroughs, STM32 embedded LQR balancing, sim-to-real reinforcement learning, and historical Ain Shams University senior hybrid control project final submissions.*

> [!NOTE]
> **Course Handout Citation Context:**  
> These entries originate directly from **Section 9 ("Useful Resources and Reference Videos")** of the Faculty of Engineering, Ain Shams University course project handout for **MCT411: Hybrid Control Systems** and **MCT413s: Control Systems Design (2)** (Fall 2026, pp. 6–7). They serve as department-approved benchmarks for mechanical design, embedded control architectures, and project deliverable expectations.

#### `C11.01` [Ben Katz — Furuta Pendulum Hardware Demonstration (YouTube)](https://www.youtube.com/watch?v=XKzzWe15DEw)

- **Description:** High-performance direct-drive rotary inverted pendulum demonstration by Ben Katz, showcasing dynamic energy swing-up and vertical stabilization using a custom brushless pancake gimbal motor and high-resolution optical/magnetic encoder.
- **Course Handout Index:** `§9.1` (Build Examples and Tutorials)
- **Source Citation:** Ben Katz — *"Furuta Pendulum"*
- **Companion Code / Repository:** See [`C10.24`](file:///mnt/04048B17048B0ABE/Vaults/Undergraduate%20-%20Senior%202%20-%20S1/Hybrid%20Control/_resources/Furuta%20Pendulum/docs/resources.md#c1024-bgkatzfuruta-pendulum-custom-rotary-inverted-pendulum-github) for Ben Katz's open-source GitHub repository ([bgkatz/Furuta-Pendulum](https://github.com/bgkatz/Furuta-Pendulum)).

#### `C11.02` [Robotics GEE — Rotary Inverted Pendulum Control Using LQR | STM32 Embedded Control System (YouTube)](https://www.youtube.com/watch?v=1rarHSgbDhA)

- **Description:** Complete embedded control demonstration showcasing an STM32 ARM Cortex-M microcontroller executing discrete Linear Quadratic Regulator (LQR) full state feedback to balance a rotary inverted pendulum, accompanied by hardware interfacing and mathematical modeling/simulation validation.
- **Course Handout Index:** `§9.1` (Build Examples and Tutorials) & `§9.2` (Simulation / Modelling walkthrough)
- **Source Citation:** Robotics GEE — *"Rotary Inverted Pendulum Control using LQR | STM32 Embedded Control System | Hardware Demo"*
- **Cross-Section Note:** Cited under both Section 9.1 and Section 9.2 in the course project description; consolidated into a single unified canonical entry.

#### `C11.03` [Faisal Maniar — Rotary Inverted Pendulum Hardware & Balancing (YouTube)](https://www.youtube.com/watch?v=rhg0wcAvvhc)

- **Description:** Practical physical build demonstration of a custom rotary inverted pendulum, illustrating mechanical fabrication, motor driving, encoder interfacing, and real-time stabilization.
- **Course Handout Index:** `§9.1` (Build Examples and Tutorials)
- **Source Citation:** Faisal Maniar — *"Rotary Inverted Pendulum"*

#### `C11.04` [Henrique Ferrolho — I Built This £4,500 Pendulum for £20 (YouTube)](https://www.youtube.com/watch?v=rKChjuuR7K8)

- **Description:** Video breakdown and hardware demonstration by Henrique Ferrolho detailing the construction of a £20 open-source rotary inverted pendulum using 3D-printed parts, standard DC motor, and an 8-bit Arduino Nano running a 689-parameter neural network controller trained via reinforcement learning.
- **Course Handout Index:** `§9.1` (Build Examples and Tutorials)
- **Source Citation:** Henrique Ferrolho — *"I built this £4,500 pendulum for £20"*
- **Companion Code & Analysis:** See [`C8.03`](file:///mnt/04048B17048B0ABE/Vaults/Undergraduate%20-%20Senior%202%20-%20S1/Hybrid%20Control/_resources/Furuta%20Pendulum/docs/resources.md#c803-ferrolhorotary-inverted-pendulum-20-arduino-nano-neural-pendulum-github) for Ferrolho's GitHub repository ([ferrolho/rotary-inverted-pendulum](https://github.com/ferrolho/rotary-inverted-pendulum)) and [`C8.04`](file:///mnt/04048B17048B0ABE/Vaults/Undergraduate%20-%20Senior%202%20-%20S1/Hybrid%20Control/_resources/Furuta%20Pendulum/docs/resources.md#c804-ferrolho-20-arduino-nano-neural-inverted-pendulum-review-roboairadar) for technical analysis.

> [!TIP]
> **Pre-Indexed Course Handout Resource (§9.1):**  
> David Gonzalez — *"Building another Furuta Pendulum"* ([https://www.youtube.com/watch?v=1GOXuu1OZ3k](https://www.youtube.com/watch?v=1GOXuu1OZ3k)) is already canonically indexed in the catalogue as **[`C10.06`]** (Orig `#22`). See also Gonzalez's Dagor brushless controller build log at **[`C10.20`]**.

#### `C11.05` [Cillk 11 — Rotary Inverted Pendulum (PID): Design, Build, Model, Swing Up & Stabilisation (YouTube)](https://www.youtube.com/watch?v=bY4t6yfBA24)

- **Description:** Complete end-to-end engineering tutorial documenting mechanical design, machining and assembly, mathematical modeling, energy-based swing-up controller formulation, and upright PID stabilization of a custom rotary inverted pendulum.
- **Course Handout Index:** `§9.1` (Build Examples and Tutorials)
- **Source Citation:** Cillk 11 — *"Rotary Inverted Pendulum (PID) — Design, Build, Model, Swing Up and Stabilisation"*

#### `C11.06` [Whisky Nguyễn — Rotary Inverted Pendulum Prototype & Control (YouTube)](https://www.youtube.com/watch?v=S5DLs4sHmQI)

- **Description:** Hardware construction and live balancing demonstration of a DIY rotary inverted pendulum apparatus, illustrating actuator coupling, encoder mounting, and closed-loop control.
- **Course Handout Index:** `§9.1` (Build Examples and Tutorials)
- **Source Citation:** Whisky Nguyễn — *"Rotary Inverted Pendulum"*

#### `C11.07` [Kevin Wood | Robotics & AI — Reinforcement Learning on Hardware from Sim-to-Real (YouTube)](https://www.youtube.com/watch?v=PTsIg7wUC9U)

- **Description:** Comprehensive project walkthrough by Kevin Wood demonstrating sim-to-real transfer of deep reinforcement learning policies trained in physics simulation (Isaac Sim / MuJoCo) and executed zero-shot on physical rotary inverted pendulum hardware.
- **Course Handout Index:** `§9.3` (Sim-to-Real and Digital Twins)
- **Source Citation:** Kevin Wood | Robotics & AI — *"Reinforcement Learning on Hardware from Sim-to-Real (Rotary Inverted Pendulum)"*
- **Companion Series & Resources:** See [`C7.05`] (MuJoCo calibration video #4), [`C10.27`] (product asset package), and [`C10.28`] (YouTube channel).

#### `C11.08` [Ain Shams University — Balancing the Furuta Pendulum | Rotary Inverted Pendulum (YouTube)](https://www.youtube.com/watch?v=D3wFRbZpMtU)

- **Description:** Prior Ain Shams University Mechatronics student project demonstration showcasing physical balancing and robust disturbance rejection on a custom rotary inverted pendulum testbed.
- **Course Handout Index:** `§9.4` (Examples of Previous Ain Shams University Mechatronics Student Projects)
- **Source Citation:** Ain Shams University Mechatronics — *"Balancing the Furuta Pendulum | Rotary Inverted Pendulum"*
- **Course Deliverable Alignment:** Demonstrates physical prototype stability and manual disturbance rejection required for Deliverable 1.

#### `C11.09` [Ain Shams University — Rotary Inverted Pendulum: MCT411 Hybrid Control Project Final Submission (Team 14) (YouTube)](https://www.youtube.com/watch?v=1aN-A-AvBck)

- **Description:** Official MCT411 Hybrid Control Systems final submission video by Ain Shams University Mechatronics Team 14, presenting their physical prototype, system modeling, Stateflow hybrid switching logic, swing-up to upright stabilization, and real-time dashboard.
- **Course Handout Index:** `§9.4` (Examples of Previous Ain Shams University Mechatronics Student Projects)
- **Source Citation:** Ain Shams University Mechatronics Team 14 — *"Rotary Inverted Pendulum — MCT411 — Hybrid Control Project Final Submission (Team 14)"*
- **Course Deliverable Alignment:** Exemplifies the full Phase 3 final submission requirements, demo video structure, and hybrid supervisory switching.

#### `C11.10` [Ain Shams University — Furuta Pendulum: MCT411 Hybrid Control Systems Project (YouTube)](https://www.youtube.com/watch?v=ilNLgx1Jl00)

- **Description:** Ain Shams University Mechatronics senior course project video for MCT411 Hybrid Control Systems, showing physical prototype integration, real-time sensing, closed-loop control execution, and hybrid state machine operation.
- **Course Handout Index:** `§9.4` (Examples of Previous Ain Shams University Mechatronics Student Projects)
- **Source Citation:** Ain Shams University Mechatronics — *"Furuta Pendulum — MCT411 Hybrid Control Systems"*
- **Course Deliverable Alignment:** Demonstrates experimental swing-up and stabilization performance under course benchmark testing.

---

## Cross-Platform Mirror & Identical Publication Analysis

The following list represent identical academic papers hosted on different repositories. Both URLs are preserved in the catalog, but they are cross-referenced here for complete transparency:

### Controlling the Furuta Pendulum: Proof of Concept Through Virtual Prototyping (2024)
- **Primary / Host A:** [SciELO Mexico (`#1`, `#98`)](https://www.scielo.org.mx/scielo.php?script=sci_arttext&pid=S1665-64232024000300327)
- **Mirror / Host B:** [JART / UNAM (`#60`, `#166`)](https://jart.icat.unam.mx/index.php/jart/article/view/2347)
- **Relationship:** Same research paper published in Journal of Applied Research and Technology and indexed on SciELO.

### A Multibody-Based Benchmarking Framework for the Control of the Furuta Pendulum
- **Primary / Host A:** [Univ. of Vigo Repository (`#25`, `#139`)](https://www.investigo.biblioteca.uvigo.es/xmlui/handle/11093/10496?locale-attribute=en)
- **Mirror / Host B:** [MDPI Machines (`#46`, `#171`)](https://www.mdpi.com/2076-0825/14/8/377)
- **Relationship:** Same research article in university thesis repository and final published MDPI open-access format.

### A Swinging Up Controller for the Furuta Pendulum Based on the Total Energy Control System (TECS)
- **Primary / Host A:** [DML-CZ Digital Library (`#52`, `#108`)](https://dml.cz/handle/10338.dmlcz/147844)
- **Mirror / Host B:** [Kybernetika Journal PDF (`#95`, `#167`)](https://www.kybernetika.cz/content/2019/2/402/paper.pdf)
- **Relationship:** Same journal paper published in Kybernetika (2019), hosted on Czech Math Library and direct journal server.

### Modeling, Simulation, and Control of a Rotary Inverted Pendulum: An RL Approach (2024)
- **Primary / Host A:** [ResearchGate (`#79`, `#206`)](https://www.researchgate.net/publication/386174932_Modeling_Simulation_and_Control_of_a_Rotary_Inverted_Pendulum_A_Reinforcement_Learning-Based_Control_Approach)
- **Mirror / Host B:** [MDPI AI (`#86`, `#128`)](https://www.mdpi.com/2673-3951/5/4/95)
- **Relationship:** Same research article available as author preprint on ResearchGate and published version on MDPI AI.

### System Identifications of a 2DOF Pendulum Controlled by QUBE-Servo (2020)
- **Primary / Host A:** [Polish Academy of Sciences (`#130`)](https://www.journals.pan.pl/Content/115031/PDF/AME_2020_131699.pdf)
- **Mirror / Host B:** [ResearchGate (`#131`)](https://www.researchgate.net/publication/375432201_System_identifications_of_a_2DOF_pendulum_controlled_by_QUBE-servo_and_its_unwanted_oscillation_factors)
- **Relationship:** Same research article in Archive of Mechanical Engineering (PAN) and uploaded to ResearchGate.

### Energy Based Control of Pendulum (Iwashiro & Furuta, 1996)
- **Primary / Host A:** [Semantic Scholar (`#5`, `#103`)](https://www.semanticscholar.org/paper/Energy-based-control-of-pendulum-Iwashiro-Furuta/bd3f868d1538ab724d305e70f0d9766071c415fe)
- **Mirror / Host B:** [ResearchGate (`#7`, `#105`)](https://www.researchgate.net/publication/3668724_Energy_based_control_of_pendulum)
- **Relationship:** Same seminal conference publication hosted on Semantic Scholar and ResearchGate.

### Modelling the Furuta Pendulum (M. Gäfvert, 1998)
- **Primary / Host A:** [Semantic Scholar (`#2`, `#99`)](https://www.semanticscholar.org/paper/Modelling-the-Furuta-Pendulum-G%C3%A4fvert/f976a3a26d55c9f4d0d380080482765f2a25758b)
- **Mirror / Host B:** [Lund University LUCRIS PDF (`#93`, `#168`)](https://lucris.lub.lu.se/ws/files/4453844/8727127.pdf)
- **Relationship:** Same foundational technical report; one is the academic metadata record, one is the direct institutional PDF.

### Swinging Up a Pendulum by Energy Control (K. J. Åström & K. Furuta, Automatica 2000)
- **Primary / Host A:** [UCSB Direct PDF (`#51`, `#191`)](https://web.ece.ucsb.edu/~hespanha/ece229/references/AstromFurutaAUTOM00.pdf)
- **Mirror / Host B:** [Semantic Scholar (`#54`, `#192`)](https://www.semanticscholar.org/paper/Swinging-up-a-pendulum-by-energy-control-%C3%85str%C3%B6m-Furuta/8a259e547088912d0323b0d4fa7d0dafde50e4ee)
- **Relationship:** Same seminal Automatica journal paper; one is the direct course reprint PDF at UCSB, one is the Semantic Scholar entry.

---
