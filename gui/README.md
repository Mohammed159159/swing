# Operator GUI & Telemetry Dashboard (`gui/`)

This directory houses the user interfaces, serial communication background workers, and real-time visualization tools, fulfilling the **20% Working Hardware Prototype with GUI/Dashboard** course requirement.

---

## 📁 Subdirectory & File Breakdown

```text
gui/
├── app/                    # MATLAB App Designer application (.mlapp)
├── telemetry_bridge.m      # High-speed serial / UDP binary packet transceiver worker
└── visualizer_3d.m         # Real-time Simscape / MATLAB 3D animation canvas hook
```

---

## 🎯 Architecture & Responsibilities

1. **Telemetry Bridge (`telemetry_bridge.m`):**
   - Asynchronous serial reader processing binary packets from the microcontroller at $\ge 115200\text{ baud}$ (target $460800\text{ baud}$).
   - Extracts encoder positions ($\theta_0, \theta_1$), filtered velocities ($\dot{\theta}_0, \dot{\theta}_1$), voltage $u$, and state flag.
   - Pushes deserialized data to App Designer UI queues without blocking graphics rendering.

2. **3D Visualizer (`visualizer_3d.m`):**
   - Renders a live 3D multibody representation of the Furuta pendulum synchronized with incoming hardware joint angles.
   - Satisfies the 10% Synchronized Digital Twin deliverable during physical HIL demonstrations.

3. **Primary Owners:** Pair 4 (Benchmarking & Dashboard) with support from Pair 3 (Hardware & Firmware).
