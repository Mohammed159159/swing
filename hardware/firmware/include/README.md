# Firmware Header Files (`hardware/firmware/include/`)

This directory houses C/C++ header definitions, pin maps, controller gain matrices, and data structures.

---

## 📁 Header Modules Breakdown

- `config.h`: Hardware pin assignments, PWM timer frequencies, SysTick period ($T_s = 2.0\text{ ms}$), encoder counts per revolution (PPR).
- `gains.h`: Pre-computed feedback gain vectors ($K_{\text{LQR}}, K_{\text{PP}}, K_{\text{PID}}$) exported from MATLAB `config/compute_gains.m`.
- `state_types.h`: State enumerations (`SAFE_STOP`, `SWING_UP`, `STABILIZE`) and system state structures ($\mathbf{x} = [\theta_0, \theta_1, \dot{\theta}_0, \dot{\theta}_1]^T$).
- `packet_def.h`: Binary struct definitions for serial packet frames and checksum validators.
