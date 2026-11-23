# Deferred Backlog (Postponed Items)

Items intentionally excluded from the current implementation iteration so they are tracked for upcoming iterations.

---

## 1. Next Iteration: Workspace & Simscape Model Initialization
- [x] **Create root `startup.m`:** Completed in Step 1A (`startup.m` adds `config/`, `sim/{runners,scenarios}/`, `digital-twin/{models,validation}/`, `control/`, and `tests/`, and populates `params` and `plant` safely).
- [ ] **Configure Simscape `PreLoadFcn` callbacks:**
  - Add relative `config/` path fallback + `if ~exist('params', 'var'), params = load_parameters(); end` to `digital-twin/models/simscape/robot_compatible.slx` (R2024a).
  - Apply/document the same `PreLoadFcn` snippet for `digital-twin/models/simscape/robot.slx` (requires MATLAB R2025b to save).

---

## 2. Later: Documentation & Convention Alignment
- [ ] **Repository refactoring documentation sync (`digital-twin/models/`, `digital-twin/validation/`, `sim/`):**
  - Update `README.md`, `HANDBOOK.md`, `docs/logs/ROADMAP.md`, `control/README.md`, `control/controllers/README.md`, `control/evaluation/README.md`, and `digital-twin/README.md` to reflect:
    - Removal of `control/controllers/open-loop/` and move of `control/model/` -> `digital-twin/models/analytical/`.
    - Unified `digital-twin/models/{analytical,simscape,urdf,mujoco}` and `digital-twin/validation/` layout (plus `mujuco` -> `mujoco` spelling fix).
    - New root `sim/{runners,scenarios}/` layer.
- [ ] **State-vector ordering alignment:** Reconcile `HANDBOOK.md` §7.1.4 ($[\theta_0, \theta_1, \dot{\theta}_0, \dot{\theta}_1]^T$) with `digital-twin/models/analytical/furuta_state_space.m` / `config/build_plant.m` ($[\theta_0, \dot{\theta}_0, \theta_1, \dot{\theta}_1]^T$).
- [ ] **Update `config/README.md`:** Replace `run('config/load_parameters.m')` and "Symbolic Jacobian linearization" notes with the function-based `params = load_parameters()` / `plant = build_plant(params)` API and `extract_urdf_params.m`.
- [ ] **Gearbox ratio $N$ alignment:** Note in `HANDBOOK.md` §7.1.2 that `furuta_state_space.m` §5 explicitly models gearbox ratio $N$ ($\tau = \frac{N k_t}{R_m} V_a - \frac{N^2 k_t k_b}{R_m} \dot{\theta}_0$).
- [ ] **2D-vs-3D centrifugal term note for Phase 1 validation:** Update/close note now that `furuta_state_space.m` includes the full 3D radial sweep & transverse inertia terms $\frac{1}{2}\hat{J}_1\sin^2(\theta_1)\dot{\theta}_0^2$.

---

## 3. Phase 1 CAD Finalization & Parameter ID
- [ ] **Replace temporary URDF values:** Re-run `extract_urdf_params('digital-twin/models/simscape/robot.urdf')` once the final URDF fixes the temporary `x = 0.004 m` arm joint offset.
- [ ] **Replace `% TODO` baseline estimates in `config/load_parameters.m`:** Update `I0, L0, m1, l1, J1, N, R_m, k_t, k_b, b0, b1, V_max, theta0_ppr, theta1_ppr, Ts` from final CAD, motor datasheet, and experimental SysID tests.
