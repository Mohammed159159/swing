# Hardware Experimental Telemetry Logs (`data/hardware/`)

This directory archives real-time experimental logs, telemetry runs, and disturbance response captures acquired directly from the physical Furuta pendulum prototype.

---

## 🎯 Target Datasets & Milestones

1. **Parameter Identification Logs (Weeks 3–5):**
   - Locked-rotor armature resistance measurements.
   - Motor stall-torque and no-load speed characterization ($k_t, k_b$).
   - Pendulum free-decay encoder logs for logarithmic decrement calculation of $b_1$.
   - Compound pendulum oscillation logs for inertia $J_1$ calculation.

2. **Real-Time Balancing & Swing-Up Runs (Weeks 10–12):**
   - Telemetry streams recorded during automated Åström-Furuta swing-up transitions.
   - Steady-state upright balance runs for PID, Pole Placement, and LQR controllers.
   - Standardized impulse disturbance recovery captures (calibrated $10^\circ$ deflection or pendulum striker).

3. **Final Benchmark & Verification Data (Phase 3 Gate — Week 13):**
   - Disturbance rejection runs comparing settling times, peak excursions, and $u_{\text{RMS}}$ control efforts.
   - Synchronized digital twin telemetry validation bags.

---

## 📊 Recorded Fields (Aligned with HIL Telemetry Protocol)

Logs streamed via the serial telemetry bridge must contain:
- `timestamp_ms`: Microcontroller hardware timer timestamp $[ms]$
- `theta0_rad`: Arm position $[\text{rad}]$ from base encoder
- `theta1_rad`: Pendulum position $[\text{rad}]$ from pivot encoder
- `theta0_dot`: Filtered arm angular velocity $[\text{rad/s}]$
- `theta1_dot`: Filtered pendulum angular velocity $[\text{rad/s}]$
- `voltage_cmd`: Commanded motor voltage $[\text{V}]$
- `motor_current`: Motor current sensor reading $[A]$ (if instrumented)
- `state_id`: Active state (`0: SAFE_STOP`, `1: SWING_UP`, `2: STABILIZE`)
