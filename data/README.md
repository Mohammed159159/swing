# Experimental & Simulation Data (`data/`)

This directory archives all generated experimental data, simulation trajectories, disturbance logs, and real-time telemetry captures for the Furuta Rotary Inverted Pendulum project.

---

## 📁 Directory Structure

```text
data/
├── simulation/     # Open-loop step responses, ODE cross-validation, and simulated controller runs
└── hardware/       # Real-time CSV, MAT, and serial telemetry bags from physical hardware
```

---

## 🏷️ Data Management Standards

1. **Formats:**
   - `.mat` files for high-precision MATLAB workspace structures and timeseries logs.
   - `.csv` files for cross-platform data interchange, analysis, and Python scripts.
2. **Naming Convention:**
   - `[TYPE]_[CONTROLLER]_[TEST_ID]_[YYYYMMDD_HHMM].[ext]`
   - Example: `step_response_openloop_run01_20261015_1430.mat`
   - Example: `disturbance_lqr_impulse10deg_run03_20261102_1620.csv`
3. **Primary Owners:**
   - Simulation traces: Pair 1 (Dynamics & Modeling) and Pair 2 (Control Syntheses).
   - Hardware logs: Pair 3 (Hardware & Firmware) and Pair 4 (Benchmarking & Dashboard).
