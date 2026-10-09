# MATLAB App Designer Dashboard (`gui/app/`)

This directory houses the standalone **MATLAB App Designer** (`.mlapp`) application providing operator control, live telemetry scopes, and supervisor mode switches.

---

## 🎯 Dashboard Features & Layout

1. **Telemetry & Live Scopes:**
   - Real-time strip charts for arm angle $\theta_0(t)$ and pendulum angle $\theta_1(t)$.
   - Actuator control voltage plot $u(t)$ with saturation limits ($\pm V_{\max}$).
   - Motor current monitor $i_a(t)$ and estimated power expenditure.

2. **Supervisory State Indication:**
   - Visual status lamp displaying current Stateflow state:
     - 🔴 `SAFE_STOP`
     - 🟡 `SWING_UP`
     - 🟢 `STABILIZE`

3. **Interactive Control & Safety Overrides:**
   - Master **Emergency Stop** (E-Stop) latch button (immediately forces $u=0$).
   - Controller Selection Dropdown: `PID`, `Pole Placement`, `LQR`, `RL (TinyML)`.
   - Horizontal Arm Reference Slider: sets $\theta_{0,\text{ref}}$ for setpoint tracking experiments.
   - Disturbance Logging Trigger: records timestamped CSV/MAT data on demand.

4. **Synchronized 3D Viewport:**
   - Embedded 3D axes panel rendering the synchronized Furuta pendulum digital twin updated at $\ge 30\text{ fps}$.
