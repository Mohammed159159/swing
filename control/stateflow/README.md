# Hybrid Supervisory State Machine (`control/stateflow/`)

This directory contains the **Stateflow** supervisory finite state machine (FSM) governing discrete-continuous hybrid control switching for the Furuta Rotary Inverted Pendulum.

---

## 🎯 Hybrid State Machine Architecture

The supervisor coordinates transitions between three distinct operational modes:

```text
              [ Reset / Start ]
                     │
                     ▼
             ┌───────────────┐
             │   SAFE_STOP   │  (u = 0, actuator disabled)
             └───────┬───────┘
                     │ Enable cmd & arm centered
                     ▼
             ┌───────────────┐
             │   SWING_UP    │  (Astrom-Furuta energy control)
             └───────┬───────┘
                     │ Capture Guard: |theta1| < 15° AND |theta1_dot| < 2.5 rad/s
                     ▼
             ┌───────────────┐
             │   STABILIZE   │  (LQR / Pole Placement / PID balancing)
             └───────┬───────┘
                     │ Fall-Back Guard: |theta1| > 25°
                     ▼
                (SWING_UP)
```

Emergency trip transitions occur from any active state directly to `SAFE_STOP` if:
$$|\theta_0| > 150^\circ \quad \mathbf{OR} \quad |I_{\text{motor}}| > I_{\max} \quad \mathbf{OR} \quad \text{E-Stop Triggered}$$

---

## 🛡️ Hysteresis Rationale

- **Capture Guard:** $|\theta_1| < 15^\circ$ AND $|\dot{\theta}_1| < 2.5\text{ rad/s}$
- **Fall-Back Guard:** $|\theta_1| > 25^\circ$
- **Hysteresis Band ($10^\circ$):** Eliminates boundary chattering, high-frequency mode switching, and limit cycle instabilities at the boundary of the linear capture basin.

---

## 📁 Expected Files

| File | Description |
|---|---|
| `furuta_supervisor.slx` | Simulink / Stateflow chart implementing the complete hybrid supervisory logic |
| `stateflow_params.m` | Configuration script defining threshold constants, basin angles, and timeout limits |
