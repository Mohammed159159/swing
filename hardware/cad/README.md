# Computer-Aided Design & 3D Fabrication (`hardware/cad/`)

This directory houses mechanical CAD models, fabrication drawings, exchange assemblies, and 3D printing mesh files for the physical Furuta pendulum platform.

---

## 📁 Expected Files & Formats

1. **Native CAD Models:**
   - SolidWorks (`.sldprt`, `.sldasm`) or Autodesk Fusion 360 (`.f3d`) design source files.
2. **Neutral Exchange Assemblies:**
   - `furuta_assembly.step`: Standard STEP AP214/AP242 exchange format for importing into Simscape Multibody.
3. **3D Printing Mesh Files (`.stl`):**
   - `base_mount.stl`: Motor clamp and dual bearing housing.
   - `arm_hub.stl`: Rigid coupling clamping horizontal arm to motor shaft.
   - `horizontal_arm.stl`: Lightweight rigid arm link ($L_0 \approx 100\text{--}150\text{ mm}$).
   - `pendulum_hub.stl`: Low-friction bearing housing attaching unactuated pendulum rod to arm tip.
   - `pendulum_rod.stl`: Pendulum cylindrical rod ($L_1 \approx 180\text{--}250\text{ mm}$).

---

## ⚙️ 3D Printing Specifications

- **Recommended Filament:** PETG or Tough PLA / PLA+ (ABS optional).
- **Infill:** $\ge 50\%$ gyroid or cubic infill for horizontal arm and bearing hubs to maximize bending and torsional stiffness.
- **Wall Thickness:** $\ge 4$ perimeters ($1.6\text{ mm}$) to ensure dimensional accuracy around press-fit bearing pockets.
- **Tolerances:** Bearing seats modeled with $+0.1\text{ mm}$ to $+0.15\text{ mm}$ clearance for snug push-fit.
