# Hardware Mechatronics Stack (`hardware/`)

This directory contains the physical mechatronics design, fabrication models, electrical wiring schematics, and embedded microcontroller firmware for the physical Furuta pendulum prototype.

---

## 📁 Subdirectory Breakdown

```text
hardware/
├── cad/            # SolidWorks / Fusion 360 source models, STEP assemblies, and 3D print STLs
├── electronics/    # Electrical schematics, motor driver wiring, pinout mappings, and BOM
└── firmware/       # Embedded C/C++ firmware (QEI timer interrupts, PWM generation, serial protocol)
```

---

## 🎯 Engineering Requirements (HANDBOOK.md §5 & WBS 5.0)

1. **Mechanical Design:**
   - Preloaded dual ball bearings on base vertical shaft to prevent radial and axial wobble.
   - 3D printed horizontal arm (PETG / PLA+) or aluminum rod ($L_0 \approx 100\text{--}150\text{ mm}$).
   - Low-friction miniature bearings at the pendulum pivot ($L_1 \approx 180\text{--}250\text{ mm}$).
   - Mechanical hard stops or slip-ring design to protect secondary encoder wiring.

2. **Actuation & Instrumentation:**
   - DC Motor with planetary gearbox or direct-drive gimbal motor with zero backlash.
   - High-frequency H-bridge driver ($\ge 20\text{ kHz}$) operating without audible switching noise.
   - Dual optical quadrature encoders: Base motor $\ge 360\text{ PPR}$; Pendulum pivot $\ge 600\text{--}1024\text{ PPR}$ ($2400\text{--}4096$ counts/rev in $4\times$ quadrature).

3. **Primary Owners:** Pair 3 (Hardware & Firmware) with support from Pair 1 (Modeling) and Pair 4 (Dashboard).
