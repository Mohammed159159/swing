# Modular Report Chapters (`docs/report/sections/`)

This directory contains individual modular chapters for assembling the final technical report.

---

## 📁 Chapter Breakdown

- `01_introduction.tex` / `.md`: Motivation, underactuated dynamics background, and project objectives.
- `02_modeling_and_sysid.tex` / `.md`: Euler-Lagrange derivations, electromechanical coupling, and parameter ID experiments ($b_1, R_m, k_t, J_0, J_1$).
- `03_multibody_digital_twin.tex` / `.md`: Simscape Multibody plant modeling and open-loop step response cross-validation ($\text{NRMSE} < 3\%$).
- `04_stability_and_linearization.tex` / `.md`: Jacobian state-space linearization, open-loop eigenvalues, and controllability/observability rank proofs.
- `05_controller_synthesis.tex` / `.md`: Classical PID with derivative filtering, Ackermann Pole Placement, and Bryson-weighted LQR Riccati design.
- `06_energy_swingup.tex` / `.md`: Åström-Furuta Lyapunov energy-based swing-up controller derivation and voltage conversion.
- `07_stateflow_supervisor.tex` / `.md`: Discrete-continuous hybrid supervisory state machine with $10^\circ$ hysteresis capture/fall-back guards.
- `08_hardware_and_firmware.tex` / `.md`: CAD design, 3D printing, electronics schematic, QEI timer firmware, and HIL serial telemetry protocol.
- `09_dashboard_and_telemetry.tex` / `.md`: MATLAB App Designer GUI, real-time serial parsing, and live synchronized 3D twin animation.
- `10_benchmarking_and_results.tex` / `.md`: Standardized disturbance impulse testing, quantitative metrics (IAE, ISE, ITAE, settling time, $u_{\text{RMS}}$), and comparative analysis.
- `11_shadow_track_rl.tex` / `.md`: MuJoCo/Gymnasium SAC/PPO continuous training, domain randomization, and embedded TinyML C++ distillation benchmark.
- `12_conclusion.tex` / `.md`: Summary of engineering outcomes, lessons learned, and future work.
