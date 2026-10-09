---
created: Friday, October 9th 2026, 6:16:36 pm
modified: Friday, October 9th 2026, 6:16:36 pm
---
# **Comprehensive Technical Review and Reinforcement Learning Integration for the Rotary Inverted Pendulum**

## **Theoretical Modeling, Euler-Lagrange Dynamics, and Parameter Identification**

The Furuta pendulum, or rotary inverted pendulum, is an underactuated mechanical benchmark characterized by two degrees of freedom and a single control actuator [1]. First introduced by Katsuhisa Furuta and colleagues in 1991 at the Tokyo Institute of Technology, the platform comprises a horizontally rotating base arm driven by an actuator (generalized coordinate $\theta_0$) and an unactuated secondary pendulum link (generalized coordinate $\theta_1$) that rotates freely in the vertical plane at the tip of the horizontal arm [1]. The mechanism exhibits severe nonlinearities, state configurations defined on non-Euclidean manifolds ($\mathbb{S}^1 \times \mathbb{S}^1$), and open-loop instability about its upright equilibrium point [1]. Although originally developed as a laboratory benchmark, its balancing and stabilization dynamics directly mirror complex engineering systems, including thrust-vector control in rocketry, bipedal humanoid robotic balance, crane cargo anti-sway suppression, and personal balancing transporters [1].

```text
Actuated Base Motor (Torque \tau)
       |
       v
Horizontal Arm Joint (\theta_0, length L_0)
       |
       v
Unactuated Pivot Joint (\theta_1)
       |
       v
Inverted Pendulum Link (CoM l_1, mass m_1)
```

The physical assembly couples the actuated horizontal joint to the unactuated vertical pivot, transmitting rotational inertia through centrifugal, Coriolis, and gravitational forces.

### **Nonlinear Euler-Lagrange Formulation**

The nonlinear equations of motion governing the rotary inverted pendulum are derived from analytical mechanics using the Euler-Lagrange equations of the second kind [2]:

$$\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}} \right) - \frac{\partial \mathcal{L}}{\partial q} = Q$$

where $q = [\theta_0, \theta_1]^T$ represents the vector of generalized coordinates, $\mathcal{L} = T - V$ denotes the Lagrangian of the mechanical system, $T$ is the total kinetic energy, $V$ is the total potential energy, and $Q$ is the generalized force vector accounting for non-conservative actuator inputs and viscous friction torques [2].

Assuming rigid link geometry, the horizontal arm is characterized by mass $m_0$, length $L_0$, and mass moment of inertia $J_0$ about its central axis of rotation [1]. The pendulum link is parameterized by mass $m_1$, total length $L_1$, center of mass distance $l_1 = L_1/2$, and moment of inertia $J_1$ about its center of mass [3]. Establishing a reference Cartesian frame at the base pivot, the position vector defining an arbitrary differential element along the pendulum rod is written as [3]:

$$\mathbf{r}_1 = \begin{bmatrix} L_0 \cos\theta_0 - r_p \sin\theta_0 \sin\theta_1 \\ L_0 \sin\theta_0 + r_p \cos\theta_0 \sin\theta_1 \\ r_p \cos\theta_1 \end{bmatrix}$$

where $r_p$ defines the radial distance measured along the pendulum link from the pivot joint [3]. Differentiating the position vector with respect to time yields the instantaneous velocity field across both bodies, enabling analytical integration of the system kinetic energy $T$ and gravitational potential energy $V$ [2]:

$$T = \frac{1}{2} J_0 \dot{\theta}_0^2 + \frac{1}{2} m_1 \left( L_0^2 \dot{\theta}_0^2 + l_1^2 \dot{\theta}_1^2 + l_1^2 \dot{\theta}_0^2 \sin^2\theta_1 + 2 L_0 l_1 \dot{\theta}_0 \dot{\theta}_1 \cos\theta_1 \right) + \frac{1}{2} J_1 \dot{\theta}_1^2$$

$$V = m_1 g l_1 (1 - \cos\theta_1)$$

Evaluating the partial derivatives across the generalized coordinates yields two coupled second-order nonlinear differential equations [3]. Expressed in standard robotic manipulator form, the dynamics satisfy [3]:

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) + F(\dot{q}) = \begin{bmatrix} \tau \\ 0 \end{bmatrix}$$

where $M(q)$ denotes the positive-definite symmetric generalized inertia matrix, $C(q,\dot{q})$ encapsulates Coriolis and centripetal terms, $G(q)$ is the gravitational torque vector, and $F(\dot{q})$ defines viscous joint damping [3]:

$$M(q) = \begin{bmatrix} \alpha + \beta \sin^2\theta_1 & \gamma \cos\theta_1 \\ \gamma \cos\theta_1 & \beta \end{bmatrix}$$

$$C(q,\dot{q}) = \begin{bmatrix} \beta \dot{\theta}_1 \sin(2\theta_1) & -\gamma \dot{\theta}_1 \sin\theta_1 + \frac{1}{2}\beta \dot{\theta}_0 \sin(2\theta_1) \\ -\frac{1}{2}\beta \dot{\theta}_0 \sin(2\theta_1) & 0 \end{bmatrix}$$

$$G(q) = \begin{bmatrix} 0 \\ -\delta \sin\theta_1 \end{bmatrix}, \quad F(\dot{q}) = \begin{bmatrix} b_0 \dot{\theta}_0 \\ b_1 \dot{\theta}_1 \end{bmatrix}$$

The physical constants are lumped into concise algebraic groupings [3]:

$$\alpha = J_0 + m_1 L_0^2, \quad \beta = J_1 + m_1 l_1^2, \quad \gamma = m_1 L_0 l_1, \quad \delta = m_1 g l_1$$

### **Actuator Dynamics and Electromechanical Coupling**

The input torque $\tau$ applied to the horizontal arm originates from a direct-current servomotor attached to the stationary foundation [1]. Modeling the electrical armature circuit yields the voltage-current relationship:

$$V_m(t) = R_m i_a(t) + L_m \frac{d i_a(t)}{dt} + k_m \dot{\theta}_0(t)$$

where $V_m$ is the applied armature voltage, $R_m$ is the winding terminal resistance, $L_m$ represents armature inductance, $i_a$ denotes armature current, and $k_m$ represents the motor back-electromotive force (back-EMF) constant [5]. Because the electrical time constant $\tau_e = L_m / R_m$ is negligible relative to the dominant mechanical time constants of the physical arm assembly, armature inductance is conventionally set to zero [5].

Solving algebraically for armature current and combining the result with the motor torque relationship $\tau = k_t i_a$, where $k_t$ represents the motor torque constant, gives the electromechanical torque input [5]:

$$\tau = \frac{k_t}{R_m} V_m - \frac{k_t k_m}{R_m} \dot{\theta}_0$$

Substituting this direct electromechanical relation into the horizontal joint dynamic equation demonstrates that back-EMF introduces an intrinsic viscous damping term, modifying the effective base damping coefficient to $b_{0,\text{eff}} = b_0 + \frac{k_t k_m}{R_m}$ when the system is driven via motor armature voltage $u(t) = V_m(t)$ [5].

### **Equilibrium Points, Linearization, and Controllability**

In the unforced operational regime ($\tau = 0$), setting angular velocities and accelerations to zero reveals two distinct equilibrium manifolds: the stable hanging downward equilibrium $x_{\text{down}} = [0, \pi, 0, 0]^T$ and the open-loop unstable upright equilibrium $x_{\text{up}} = [0, 0, 0, 0]^T$ [1].

Linearizing the nonlinear equations of motion about the unstable upright equilibrium through a first-order Taylor series expansion ($\sin\theta_1 \approx \theta_1$, $\cos\theta_1 \approx 1$, and dropping second-order velocity products) yields the continuous-time linear state-space system [2]:

$$\dot{x} = A x + B u, \quad y = C x$$

where the state vector is chosen as $x = [\theta_0, \theta_1, \dot{\theta}_0, \dot{\theta}_1]^T$ and the control input is $u = V_m$ [6]. Evaluating the analytical Jacobians results in the system matrices [6]:

$$A = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & \frac{\gamma \delta}{\alpha \beta - \gamma^2} & -\frac{\beta \left(b_0 + \frac{k_t k_m}{R_m}\right)}{\alpha \beta - \gamma^2} & \frac{\gamma b_1}{\alpha \beta - \gamma^2} \\ 0 & \frac{\alpha \delta}{\alpha \beta - \gamma^2} & \frac{\gamma \left(b_0 + \frac{k_t k_m}{R_m}\right)}{\alpha \beta - \gamma^2} & -\frac{\alpha b_1}{\alpha \beta - \gamma^2} \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 0 \\ \frac{\beta \frac{k_t}{R_m}}{\alpha \beta - \gamma^2} \\ -\frac{\gamma \frac{k_t}{R_m}}{\alpha \beta - \gamma^2} \end{bmatrix}$$

The term $\alpha \beta - \gamma^2 = J_0(J_1 + m_1 l_1^2) + J_1 m_1 L_0^2$ represents the determinant of the inertia tensor evaluated at the upright configuration and is strictly positive for all real physical mass distributions, guaranteeing invertibility [3]. Evaluating the characteristic polynomial $\det(sI - A) = 0$ reveals the open-loop spectrum $\lambda(A) = \{0, -\lambda_1, +\lambda_2, -\lambda_3\}$ [1]. The presence of a real, positive root $+\lambda_2 \approx \sqrt{\frac{\alpha \delta}{\alpha \beta - \gamma^2}}$ confirms that the unforced upright equilibrium is an unstable hyperbolic saddle point [1]. The zero eigenvalue reflects rotational invariance of the horizontal base arm along the horizontal plane [2].

System controllability is verified by constructing the Kalman controllability rank matrix [1]:

$$\mathcal{C} = \begin{bmatrix} B & AB & A^2 B & A^3 B \end{bmatrix}$$

Computing the determinant confirms $\det(\mathcal{C}) \neq 0$ and $\operatorname{rank}(\mathcal{C}) = 4$, establishing full-state controllability [1]. Given that physical rotary pendulums measure both the horizontal base angle $\theta_0$ and the link deflection $\theta_1$ using optical or magnetic rotary encoders, the measurement output matrix is $C = \begin{bmatrix} I_{2\times 2} & 0_{2\times 2} \end{bmatrix}$, which similarly satisfies the Kalman observability rank condition $\operatorname{rank}(\mathcal{O}) = 4$ [1].

### **Physical Parameter Identification Methods**

Accurate control design requires precise identification of plant parameters to mitigate the reality gap between numerical simulations and physical hardware [1]. Geometric dimensions and static link masses are obtained directly using digital calipers and precision analytical scales [6]. Determining mass moments of inertia requires decoupled dynamic testing [6]. The pendulum link inertia $J_1$ is measured using a bifilar torsional pendulum rig, calculating inertia from the oscillation period:

$$J_1 = \frac{m_1 g r^2 T_{\text{osc}}^2}{4 \pi^2 L_{\text{fil}}}$$

where $r$ represents bifilar suspension radius and $L_{\text{fil}}$ denotes filament length, or by integrating three-dimensional CAD geometric meshes assigned known material densities [11].

Joint friction parameters require targeted experimental estimation [8]. The unactuated pendulum joint viscous damping coefficient $b_1$ is identified by decoupling the pendulum link from the motor assembly, displacing it to an initial deflection angle $\theta_1(0) \approx 30^\circ$, and recording the unforced decay trajectory [8]. Applying the logarithmic decrement formulation across $n$ successive oscillation periods yields [8]:

$$\delta_{\text{dec}} = \frac{1}{n} \ln\left( \frac{\theta_{1}(t)}{\theta_{1}(t + n T_d)} \right), \quad \zeta = \frac{\delta_{\text{dec}}}{\sqrt{4\pi^2 + \delta_{\text{dec}}^2}}, \quad b_1 = 2 \zeta \sqrt{m_1 g l_1 (J_1 + m_1 l_1^2)}$$

Motor electrical resistance $R_m$ is determined via four-point Kelvin resistance sensing with a locked rotor [8]. The motor torque constant $k_t$ and back-EMF constant $k_m$ are determined by driving the motor under steady-state no-load conditions across varying terminal voltages while recording rotational velocity ($V_m \approx k_m \omega_{\text{no-load}}$), complemented by static torque measurements under locked-rotor conditions ($k_t = \tau_{\text{stall}} / i_{\text{stall}}$) [8].

| Parameter Symbol | Physical Description | Quanser QUBE-Servo 2 Reference Value | Custom / 3D-Printed Prototype Range | Identification Strategy |
| :--- | :--- | :--- | :--- | :--- |
| **$L_0$** | Rotary arm pivot-to-pivot length | $0.0850 - 0.0895\text{ m}$ | $0.100 - 0.200\text{ m}$ | Precision Caliper Metrology |
| **$m_0$** | Horizontal arm total mass | $0.0950\text{ kg}$ | $0.080 - 0.187\text{ kg}$ | Digital Balance Scale |
| **$J_0$** | Arm pivot moment of inertia | $2.28 \times 10^{-4}\text{ kg}\cdot\text{m}^2$ | $5.57 \times 10^{-4} - 2.50 \times 10^{-3}\text{ kg}\cdot\text{m}^2$ | CAD Mass Properties / Bifilar Rig |
| **$L_1$** | Pendulum link overall length | $0.129\text{ m}$ | $0.150 - 0.300\text{ m}$ | Precision Caliper Metrology |
| **$l_1$** | Pendulum center of mass location | $0.0645\text{ m}$ | $0.075 - 0.150\text{ m}$ | Knife-Edge Center of Gravity Balance |
| **$m_1$** | Pendulum link total mass | $0.0240\text{ kg}$ | $0.020 - 0.127\text{ kg}$ | Digital Balance Scale |
| **$J_1$** | Pendulum center of mass inertia | $1.33 \times 10^{-4}\text{ kg}\cdot\text{m}^2$ | $1.20 \times 10^{-4} - 1.20 \times 10^{-3}\text{ kg}\cdot\text{m}^2$ | Free Oscillation Frequency Period |
| **$R_m$** | Motor armature terminal resistance | $8.40\ \Omega$ | $1.20 - 12.0\ \Omega$ | Locked-Rotor Digital Multimeter |
| **$k_t$** | Motor electromechanical torque constant | $0.042\text{ N}\cdot\text{m/A}$ | $0.025 - 0.085\text{ N}\cdot\text{m/A}$ | Dynamometer Stall Torque Slope |
| **$k_m$** | Motor back-EMF voltage constant | $0.040\text{ V}/(\text{rad/s})$ | $0.025 - 0.085\text{ V}/(\text{rad/s})$ | Steady-State No-Load Saturation Slope |
| **$b_0$** | Arm joint viscous damping factor | $5.00 \times 10^{-5}\text{ N}\cdot\text{m}\cdot\text{s/rad}$ | $1.00 \times 10^{-4} - 2.40 \times 10^{-3}\text{ N}\cdot\text{m}\cdot\text{s/rad}$ | Unloaded Motor Deceleration Curve |
| **$b_1$** | Pendulum pivot viscous damping factor | $1.00 \times 10^{-4}\text{ N}\cdot\text{m}\cdot\text{s/rad}$ | $1.50 \times 10^{-4} - 2.40 \times 10^{-3}\text{ N}\cdot\text{m}\cdot\text{s/rad}$ | Logarithmic Decrement Free Decay |

## **Classical, Optimal, and Nonlinear Control Syntheses**

Stabilizing an underactuated mechanical plant across its complete operational space requires decomposing the problem into two distinct regimes: large-scale nonlinear energy injection to swing the pendulum up from its hanging resting state, and high-bandwidth linear state feedback to balance it within a localized capture basin around the vertical equilibrium [1].

The control architecture routes sensor feedback through a high-level switching mechanism. When the pendulum rests in the downward hemisphere, the energy-shaping controller commands arm accelerations to pump mechanical energy into the link. Once state trajectories enter the linearized capture basin ($|\theta_1| < \epsilon$), supervisory logic transfers authority to a linear balancing controller (such as an LQR or pole-placement state-feedback law) that stabilizes the upright position while regulating arm drift.

### **Energy-Based Swing-Up Control**

Because linear controllers cannot recover the pendulum from the downward equilibrium ($\theta_1 = \pi$), swing-up relies on nonlinear energy control, grounded in the foundational formulations developed by Karl Johan Åström and Katsuhisa Furuta [10]. Neglecting high-order coupling during the initial pump cycles, the instantaneous mechanical energy of the pendulum link, referenced to zero at the upright equilibrium, is expressed as [10]:

$$E(\theta_1, \dot{\theta}_1) = \frac{1}{2} J_p \dot{\theta}_1^2 + m_1 g l_1 (1 - \cos\theta_1)$$

where $J_p = J_1 + m_1 l_1^2$ defines the total link inertia about the pivot joint [10]. At the inverted target state ($\theta_1 = 0, \dot{\theta}_1 = 0$), the target reference energy is $E_r = 0$ (or $E_r = 2 m_1 g l_1$ when referencing potential energy to the lowest hanging configuration) [10]. Differentiating total energy with respect to time along the system trajectories reveals [10]:

$$\dot{E} = J_p \dot{\theta}_1 \ddot{\theta}_1 + m_1 g l_1 \dot{\theta}_1 \sin\theta_1 = -m_1 l_1 \dot{\theta}_1 \cos\theta_1 \cdot a_{\text{pivot}}$$

where $a_{\text{pivot}} = L_0 \ddot{\theta}_0$ represents the linear acceleration induced at the pendulum pivot by the base arm [10]. Selecting a positive-definite Lyapunov candidate based on energy error, $V_E = \frac{1}{2}(E - E_r)^2$, its time derivative evaluates to:

$$\dot{V}_E = (E - E_r)\dot{E} = -(E - E_r) m_1 l_1 \dot{\theta}_1 \cos\theta_1 \cdot a_{\text{pivot}}$$

To guarantee negative semi-definiteness ($\dot{V}_E \le 0$), the base pivot acceleration is commanded to drive the energy error asymptotically to zero [10]:

$$a_{\text{pivot}} = k_e (E - E_r) \operatorname{sign}(\dot{\theta}_1 \cos\theta_1)$$

where $k_e$ is an empirical tuning gain [10]. Mapping commanded acceleration to motor terminal voltage and introducing control saturation to prevent excessive current draw yields the bounded energy-shaping swing-up law [10]:

$$u_{\text{swing}} = \operatorname{sat}_{V_{\max}}\left( k_E (E - E_r) \operatorname{sign}(\dot{\theta}_1 \cos\theta_1) \right)$$

Physical implementations often incorporate an auxiliary proportional-derivative feedback term on the base arm angle ($\theta_0$) to prevent unbounded arm drift during consecutive swing cycles, ensuring the base remains centered within its safe rotational limits [5].

### **Linear Balancing Controllers: PID, Pole Placement, and LQR**

Once the swing-up trajectory brings the pendulum within the vicinity of the vertical apex, control is transferred to a linear feedback architecture designed using the linearized state-space model.

Dual-loop Proportional-Integral-Derivative (PID) control addresses the underactuated nature of the system by running two parallel control loops into a single actuator voltage command [1]:

$$u_{\text{PID}}(t) = K_{p,1} \theta_1(t) + K_{d,1} \dot{\theta}_1(t) + K_{p,0} \theta_0(t) + K_{i,0} \int_0^t \theta_0(\tau) d\tau + K_{d,0} \dot{\theta}_0(t)$$

The primary inner loop focuses on pendulum balance through high derivative action ($K_{d,1}$), which provides the damping needed to stabilize the unstable pole. The secondary outer loop regulates the horizontal base arm back to its zero setpoint via weak proportional-integral action ($K_{p,0}, K_{i,0}$) [8]. Because the two degrees of freedom are dynamically coupled, tuning these gains requires careful coordination to prevent cross-axis limit cycles and high-frequency chatter [9].

Full-state feedback pole placement synthesizes a state gain matrix $K_{\text{PP}} = [k_1, k_2, k_3, k_4]$ such that the closed-loop state matrix $A_{\text{cl}} = A - B K_{\text{PP}}$ exhibits a designer-specified set of eigenvalues $\mu = \{\mu_1, \mu_2, \mu_3, \mu_4\}$ in the open left-half complex plane [1]. Using Ackermann’s formula or the Bass-Gura algorithm, dominant poles are selected to enforce a second-order damping ratio $\zeta \ge 0.707$ and a 2% settling time $t_s \le 1.0\text{ s}$ [1]. The remaining poles are placed three to five times further into the left-half plane to ensure fast non-dominant decay without unnecessarily amplifying sensor noise or driving the motor into voltage saturation [1].

The Linear Quadratic Regulator (LQR) provides an optimal state-feedback gain matrix $u = -K_{\text{LQR}} x$ that minimizes an infinite-horizon quadratic performance index balancing state regulation against control effort [1]:

$$J = \int_{0}^{\infty} \left( x(t)^T Q x(t) + u(t)^T R u(t) \right) dt$$

where $Q = \operatorname{diag}([q_{\theta_0}, q_{\theta_1}, q_{\dot{\theta}_0}, q_{\dot{\theta}_1}]) \ge 0$ penalizes state deviations, and $R > 0$ penalizes control effort [1]. Solving the continuous-time Algebraic Riccati Equation (ARE) yields the unique positive-definite solution matrix $P$ [6]:

$$A^T P + P A - P B R^{-1} B^T P + Q = 0 \implies K_{\text{LQR}} = R^{-1} B^T P$$

Weight selection typically follows Bryson’s rule as a baseline, setting diagonal elements to the inverse squares of the maximum acceptable state excursions and actuator limits [1]:

$$Q_{ii} = \frac{1}{(x_{i,\max})^2}, \quad R = \frac{1}{(u_{\max})^2}$$

Emphasizing pendulum angular deflection ($q_{\theta_1} \gg q_{\theta_0}$) enforces aggressive disturbance rejection, while tuning $R$ prevents the motor from saturating during aggressive recovery maneuvers [6]. LQR provides guaranteed stability margins under nominal conditions—specifically, infinite gain margin and at least $60^\circ$ of phase margin—making it a robust foundation for balancing control [6].

| Controller Strategy | Mathematical Principle | Primary Advantages | Critical Vulnerabilities | Parameter Tuning Burden |
| :--- | :--- | :--- | :--- | :--- |
| **Åström-Furuta Energy Shaping** [cite: 10, 16] | Lyapunov energy error dissipation ($V_E = \frac{1}{2}\tilde{E}^2$) | Wide basin of attraction; reliably swings link up from rest [10] | Singular at zero velocity; requires bounding to prevent arm drift [5] | Low: requires scaling gain $k_E$ and target energy $E_r$ [10] |
| **Dual-Loop PID** [cite: 8, 14] | Decoupled tracking with low-pass derivative filtering | Model-free design; intuitive manual gain adjustments [9] | Vulnerable to cross-axis coupling; sensitive to velocity noise [14] | High: requires tuning five interacting scalar gains [9] |
| **Full-State Pole Placement** [cite: 1, 20] | Exact closed-loop eigenvalue placement ($\det(sI - A + BK) = 0$) | Explicit control over settling time, rise time, and modal damping [1] | Highly sensitive to model inaccuracies; prone to saturation [1] | Moderate: requires selecting four stable pole locations [1] |
| **Linear Quadratic Regulator (LQR)** [cite: 1, 6, 21] | Optimal control via Riccati solution ($A^T P + P A - P B R^{-1} B^T P + Q = 0$) | Guaranteed stability margins; balances tracking and actuator effort [6] | Performance degrades near severe kinematic nonlinearities [21] | Low-to-Moderate: requires populating diagonal $Q$ and $R$ matrices [6] |

## **Hybrid Supervisory Architectures and Stateflow Implementation**

Because linear stabilization laws are only valid within a local neighborhood of the upright equilibrium, and energy-based swing-up laws cannot maintain balance at the apex, a hybrid supervisory architecture is required to govern the continuous-time dynamics. The supervisory controller is modeled as an event-driven finite state machine (FSM) implemented in MATLAB Stateflow, coordinating transitions between continuous-time and discrete-time control laws across three primary operating states [1].

In the SWING_UP state, the system executes the Åström-Furuta energy shaping law to pump mechanical energy into the pendulum link [1]. As the link approaches the upright position, the supervisory logic monitors state trajectories until they satisfy a multidimensional entry guard, triggering a transition into the STABILIZE state [1]. In this balancing state, authority transfers to the linear feedback controller (such as an LQR gain matrix) to balance the pendulum and regulate arm drift [1]. If an external disturbance knocks the pendulum outside its recoverable envelope, an exit guard triggers an automatic fallback transition back to SWING_UP [1]. Finally, an overriding SAFE_STOP state provides immediate fail-safe shutdown by disabling actuator output whenever hardware limits or thermal safety thresholds are violated [1].

```text
               [ |	heta_1| < 12 deg & |d(	heta_1)| < 1.5 rad/s ]
      SWING_UP -------------------------------------------------> STABILIZE
         ^                                                           |
         |                   [ |	heta_1| > 25 deg ]                 |
         +-----------------------------------------------------------+
         |                                                           |
         |         [ Arm limit exceeded OR Overcurrent trip ]        |
         +----------------------------+------------------------------+
                                      |
                                      v
                                  SAFE_STOP
                             (Actuator Disabled)
```

Direct switching between continuous controllers can induce severe high-frequency chattering along the boundary of the capture basin, causing violent motor vibrations and rapid loss of balance [1]. To resolve this, the Stateflow architecture incorporates geometric hysteresis and temporal dwell-time filtering into its transition guards [1]. The transition from SWING_UP to STABILIZE requires the pendulum to fall within a tight capture envelope defined by angle, angular rate, and total energy error [1]:

$$\text{Guard}_{\text{Swing}\to\text{Stab}} = \left( \vert{}\theta_1\vert{} \le \theta_{\text{threshold}} \right) \land \left( \vert{}\dot{\theta}_1\vert{} \le \dot{\theta}_{\text{threshold}} \right) \land \left( \vert{}E - E_r\vert{} \le E_{\text{margin}} \right)$$

Typical empirical values for physical hardware are $\theta_{\text{threshold}} \approx 0.20\text{ rad}$ ($11.5^\circ$), $\dot{\theta}_{\text{threshold}} \approx 1.50\text{ rad/s}$, and $E_{\text{margin}} \approx 0.15 E_r$ [10]. Conversely, the exit transition from STABILIZE back to SWING_UP is assigned a wider threshold, setting $\theta_{\text{escape}} \approx 0.45\text{ rad}$ ($25.8^\circ$) [14]. This angular separation creates a $14^\circ$ hysteresis band that prevents limit cycling across the switching boundary [14]. In addition, the Stateflow chart enforces a temporal dwell-time filter requiring the state trajectory to remain continuously inside the capture basin for at least $\Delta t_{\text{dwell}} \ge 20\text{ ms}$ before committing to the balancing controller [14].

Overriding transitions to SAFE_STOP protect the hardware by monitoring four continuous safety conditions: horizontal arm overtravel exceeding soft limits ($|\theta_0| \ge 150^\circ$ on rigs without slip rings), motor overspeed ($|\dot{\theta}_0| \ge \omega_{\max}$), sustained actuator saturation ($|u(t)| = V_{\max}$ continuously for over $2.0\text{ s}$ while the motor is stalled), and manual activation of the emergency stop switch [1].

## **Digital Twin Modeling, Simscape Multibody, and Real-Time Dashboard Frameworks**

Digital twins bridge the gap between theoretical multi-body dynamics and physical hardware deployment, serving as high-fidelity models for simulation and real-time kinematic mirrors during Hardware-in-the-Loop (HIL) operation [1].

```text
Physical Plant (Encoders & Motor Driver)
       |
       |  Bidirectional Serial / UDP Telemetry (100 Hz - 1 kHz)
       v
Real-Time Dashboard (App Designer / Instrumentation)
       |
       |  Mirrored Joint Angles & Actuator Feedback
       v
Simscape Multibody Dynamic Digital Twin (3D Physics & Visualizer)
```

Sensor data streams from the physical rig through an asynchronous communications pipeline into a MATLAB App Designer dashboard, which updates strip charts, tracks Stateflow modes, and drives a 3D Simscape Multibody twin to mirror the physical hardware's motion [1].

### **Simscape Multibody Dynamic Verification**

Simscape Multibody provides a numerical physics engine that simulates the Furuta pendulum without the geometric approximations required by analytical formulations. Links are modeled as extruded solid bodies defined by two-dimensional planar profiles matched to CAD geometry, allowing the engine to calculate mass, center of gravity, and the full inertia tensor directly from material densities. Revolute joint blocks connect the bodies, incorporating internal joint stiffness, viscous damping, and Coulomb friction to match the physical pivot bearings. Actuation torques enter the horizontal joint through a physical signal port interfaced to a DC motor block or dynamic torque controller.

Validating this digital twin requires comparing its open-loop trajectory against the analytical ODEs solved in MATLAB using numerical integrators like `ode45`. Releasing both models from identical non-zero initial conditions confirms that their transient response frequencies and energy decay rates match, verifying the accuracy of the multi-body parameters before closing the control loop [1].

### **Real-Time Instrumentation Dashboard**

A dashboard built in MATLAB App Designer provides unified telemetry logging, supervisory mode monitoring, and digital twin synchronization. Bidirectional communication with the embedded microcontroller runs over a high-speed serial connection (`serialport` interface) or a low-latency UDP socket at update rates between $100\text{ Hz}$ and $1\text{ kHz}$ [1].

The dashboard interface displays continuous strip charts showing pendulum deflection $\theta_1(t)$, arm angle $\theta_0(t)$, and motor voltage $u(t)$ alongside real-time indicators that reflect the active Stateflow mode (SWING_UP, STABILIZE, or SAFE_STOP). As data streams in, the dashboard computes online integral error performance metrics—specifically the Integral of Absolute Error (IAE), Integral of Squared Error (ISE), and Integral of Time-weighted Absolute Error (ITAE)—allowing quantitative performance tracking across disturbance tests [1]. Simultaneously, the incoming joint coordinates drive an embedded 3D visual canvas or Simulink 3D Animation viewport, rendering a synchronized digital twin that mirrors the physical pendulum's motion in real time [1].

### **Open-Source MATLAB File Exchange Curations**

Several community and MathWorks reference packages provide established starting points for modeling, simulation, and control design:

* *Furuta (Rotary) Pendulum by Alan Jennings* (File Exchange ID: 34521): Validates analytical nonlinear equations of motion against SimMechanics and Simscape Multibody models, demonstrating function-handle ODE solving and dynamic consistency verification [26].
* *Furuta Pendulum by John Caipa Roldan* (File Exchange ID: 23490): Provides symbolic linearization scripts and state-space parameter extractions for multi-body assemblies around the upright equilibrium point [32].
* *Full State Feedback of Furuta Pendulum by Vikash Gupta* (File Exchange ID: 25585): Contains foundational parameter initialization files and Simulink models implementing pole-placement and state-feedback balancing loops [20].
* *Reinforcement Learning Inverted Pendulum with QUBE-Servo 2 by MathWorks* (File Exchange ID: 99364): Provides a complete reference implementation using the MATLAB Reinforcement Learning Toolbox, including Gymnasium-compatible environment definitions, reward modeling, and deep actor-critic training routines [35].

## **Open-Source Hardware, Firmware Repositories, and Video Case Studies**

Reviewing open-source physical implementations highlights proven approaches for mechanical fabrication, embedded firmware architectures, and low-latency sensor processing pipelines [1].

Embedded firmware architectures require careful coordination between hardware timer peripherals, numerical filtering routines, and discrete control logic [1]. Dedicated quadrature hardware counters decode base and pivot encoder signals, passing raw positions to a first-order low-pass filter to generate clean velocity estimates [8]. The resulting state vector is processed by the hybrid controller to generate motor command voltages, which are converted to high-frequency pulse-width modulation (PWM) and directional signals driving an H-bridge or brushless motor controller [1].

```text
Optical / Magnetic Encoders (>= 2048 PPR)
       |
       v
Embedded Hardware Timer (Quadrature QEI Mode)
       |
       v
Velocity Filter: G(s) = (\omega_f * s) / (s + \omega_f)
       |
       v
Hybrid Stateflow / Discrete Controller Engine
       |
       v
PWM & Direction Logic -> Motor Driver H-Bridge
```

### **Survey of Open-Source Projects and Hardware Implementations**

A comparative analysis of notable open-source repositories and hardware reference designs reveals several common implementation strategies:

| Project / Primary Author | Mechanical & Actuation Architecture | Sensor Suite & Interface Logic | Firmware & Control Pipeline | Primary Technical Focus & Repository Context |
| :--- | :--- | :--- | :--- | :--- |
| **Ben Katz** [cite: 1, 14, 37] | Direct-drive brushless DC motor; CNC aluminum and 3D printed arm | Dual quadrature optical encoders; direct back-EMF phase tracking | Custom C/C++; direct voltage balance; energy shaping with $\cos^4(\theta_1)$ scaling [14] | Minimizing mechanical impedance; low-cogging direct-drive control [14] |
| **Henrique Ferrolho** [cite: 1, 15, 38] | £20 low-cost build; NEMA-17 stepper with TMC2209 silent step driver; 3D printed PLA | AS5600 12-bit magnetic rotary encoder ($I^2C$/analog) on pivot [39] | Arduino Nano (ATmega328P); TinyML SAC policy (689 parameters) compiled to C++ [15] | Low-cost builds; embedded TinyML execution on 8-bit microcontrollers [15] |
| **Armand du Parc Locmaria & Pierre Fabre** [cite: 13, 31] | High-torque direct-drive motor; rigid modular 3D printed arms | Dual high-count quadrature encoders with dedicated hardware counter ICs [13] | Python / PyTorch; Stable-Baselines3; Crocoddyl NMPC swing-up; Weights & Biases [13] | Reproducible RL benchmarking; NMPC vs. RL comparisons (`Armandpl/furuta`) [13] |
| **Kevin Wood Robotics** [cite: 1, 40, 41] | Direct-drive brushless motor; CNC aluminum chassis and 3D printed arm | High-resolution magnetic/optical encoders; digital IMU sensor feedback [40] | Python Gymnasium; MuJoCo physics engine; PPO policy deployed via real-time inference loop [40] | Systematic Sim-to-Real calibration; motor parameter identification [40] |
| **Robotics GEE** [cite: 1] | High-traction geared DC servomotor with high-mass base plate | Precision optical encoders with quadrature decoding channels | STM32 ARM Cortex-M4 microcontroller; embedded LQR optimal control in C [1] | Industrial embedded workflows; Simulink Embedded Coder deployment [1] |
| **Ain Shams University Mechatronics Cohort** [cite: 1] | Rigid machined aluminum chassis; DC servomotor; dual encoder mounts | Quadrature encoders ($\ge 360\text{ PPR}$); hardware emergency stop circuit [1] | Hybrid MATLAB Stateflow supervisory logic; HIL Simulink real-time tracking [1] | Multi-controller benchmarking (PID vs. Pole Placement vs. LQR); HIL digital twins [1] |

### **Embedded Computing Platforms and Velocity Estimation**

Deploying high-gain optimal controllers or high-frequency neural policies imposes strict real-time constraints on embedded microcontrollers [1]. Platforms such as the STM32 (ARM Cortex-M4/M7) provide dedicated hardware timers with Quadrature Encoder Interface (QEI) peripherals, an integrated hardware Floating Point Unit (FPU), and native support for MATLAB Embedded Coder, making them well-suited for high-bandwidth feedback loops [1]. Texas Instruments C2000 Delfino microcontrollers (such as the TMS320F28379D) include specialized enhanced Quadrature Encoder Pulse (eQEP) peripherals designed for high-frequency motion control [1]. Conversely, highly constrained 8-bit boards like the Arduino Nano (ATmega328P) lack hardware floating-point support, requiring distilled neural policies and pre-computed fixed-point arithmetic to maintain loop timing [15].

A primary operational bottleneck on physical hardware is encoder quantization noise [8]. The unactuated pendulum joint requires high angular resolution ($\ge 2048$ quadrature counts per revolution) [1]. Coarse encoder resolution introduces quantization steps that are magnified by the derivative gains in LQR and PID controllers, generating high-frequency voltage chatter that overheats motor windings [8]. Furthermore, calculating angular velocities using pure backward finite differences ($\dot{\theta} \approx \frac{\theta_k - \theta_{k-1}}{\Delta t}$) amplifies high-frequency noise [8]. Robust firmware implementations filter raw position readings through a first-order low-pass filter with cut-off frequency $\omega_f \approx 50\text{ rad/s}$ [8], a tracking differentiator, or a steady-state Kalman filter [13]:

$$G_{\text{filter}}(s) = \frac{\omega_f s}{s + \omega_f}$$

## **Reinforcement Learning: Algorithms, Gym Environments, and Sim-to-Real Transfer**

Reinforcement learning formulates the swing-up and balancing problem as a continuous Markov Decision Process (MDP) defined by the tuple $(\mathcal{S}, \mathcal{A}, \mathcal{P}, \mathcal{R}, \gamma)$ [13]. Unlike linear controllers that rely on localized operating models, deep reinforcement learning policies can learn end-to-end control laws that handle both swing-up and balance within a single parameterized neural network.

```text
Physics Simulation Engine (MuJoCo / Gymnasium)
       |
       |  Continuous Observation: s_t = [cos, sin, angular_rates]
       v
Neural Actor-Critic Policy (SAC / PPO)
       |
       |  Smooth Action: a_t = Normalized Voltage / Torque
       v
Reward Feedback + Action Continuity Regularization
       |
       |  Domain Randomization & Latency Buffering
       v
TinyML Distillation -> Embedded Microcontroller Execution
```

During simulation training, the physics engine updates system dynamics while domain randomization perturbs parameters and injects transport lag. The neural policy learns a mapping from trigonometric state observations to motor actions, optimizing a continuous reward that balances swing-up progress, balance stability, and actuator smoothness. Once converged, the policy is distilled into a compact neural network for embedded deployment [13].

### **Observation Manifolds and Reward Engineering**

Because planar rotations wrap around at $\pm \pi$, passing raw angular coordinates directly into a neural network creates artificial discontinuities that destabilize training [13]. To preserve continuity across the state manifold $\mathbb{S}^1 \times \mathbb{S}^1$, the observation space maps raw angles to continuous trigonometric projections [13]:

$$s_t = \begin{bmatrix} \cos\theta_0 & \sin\theta_0 & \cos\theta_1 & \sin\theta_1 & \dot{\theta}_0 & \dot{\theta}_1 \end{bmatrix}^T \in \mathbb{R}^6$$

The continuous action space is defined as normalized motor armature voltage $a_t \in [-1.0, 1.0]$, scaled internally to physical actuator bounds $[-V_{\max}, V_{\max}]$ [13].

Reward function design directly shapes the stability of the learned policy and the smoothness of the resulting control action. Discontinuous step rewards produce sparse gradient feedback and encourage aggressive bang-bang actuator behavior. Similarly, pure quadratic penalties create steep gradients near the vertical apex that can lead to divergent policy updates during large swings. Continuous cosine reward functions, by contrast, provide smooth, well-conditioned gradients across the entire $360^\circ$ swing profile [46]. A robust composite reward function incorporates state error, control effort, and action continuity regularization [13]:

$$R_t = c_1 \cos\theta_1 - c_2 \theta_0^2 - c_3 \dot{\theta}_1^2 - c_4 \dot{\theta}_0^2 - c_5 u_t^2 - c_6 (u_t - u_{t-1})^2$$

The action continuity penalty $-c_6 (u_t - u_{t-1})^2$ penalizes high-frequency command oscillations, reducing current spikes and preventing mechanical wear on the physical drive train [13].

### **Algorithmic Comparison: PPO vs. SAC vs. DDPG**

The leading model-free reinforcement learning algorithms present distinct trade-offs when applied to underactuated pendulum benchmarks:

* *Proximal Policy Optimization (PPO)*: An on-policy actor-critic algorithm that uses a clipped surrogate objective to prevent destabilizing policy updates. PPO exhibits stable, monotonic policy improvements and low hyperparameter sensitivity, making it resilient to unmodeled disturbances on physical hardware [40]. However, because it is on-policy, it cannot reuse past experience, requiring large amounts of training data and relying heavily on accelerated parallel physics environments [21].
* *Soft Actor-Critic (SAC)*: An off-policy actor-critic algorithm framed around maximum entropy reinforcement learning. SAC maximizes both cumulative reward and policy entropy $\mathcal{H}(\pi(\cdot\vert{}s))$, which prevents the policy from prematurely collapsing into suboptimal local limit cycles during swing-up [15]. Reusing past transitions via an off-policy replay buffer makes SAC an order of magnitude more sample-efficient than PPO, allowing it to fine-tune directly on physical hardware rigs [15].
* *Deep Deterministic Policy Gradient (DDPG)*: An off-policy actor-critic algorithm using deterministic action selection. While historically important, DDPG is vulnerable to severe Q-value overestimation bias and brittle hyperparameter tuning in underactuated domains, often lagging behind SAC and PPO in learning stability [49].
* *Hybrid Intelligent Architectures (Fuzzy-LQR / RL-LQR Fusion)*: These architectures combine nominal LQR gains with learned neural policies or fuzzy inference engines [8]. The analytical LQR guarantees stability within the linear capture basin, while the learned component dynamically compensates for friction, parameter drift, and external impacts [21].

### **Sim-to-Real Transfer and Domain Randomization**

Policies trained in idealized simulation environments often fail when deployed directly onto physical hardware due to the reality gap [15]. This performance gap is driven by unmodeled Coulomb friction, bearing stiction, motor saturation, voltage drops, and transport latency [15].

To enable reliable zero-shot sim-to-real transfer, physics environments (such as MuJoCo or Gymnasium) must incorporate systematic domain randomization (DR) and latency modeling during training [15]. At the start of each training episode, physical parameters are sampled uniformly across randomized distributions [15]:

$$m_1 \sim \mathcal{U}(0.85 m_{1,\text{nom}}, 1.15 m_{1,\text{nom}}), \quad l_1 \sim \mathcal{U}(0.90 l_{1,\text{nom}}, 1.10 l_{1,\text{nom}})$$

$$b_1 \sim \mathcal{U}(0.50 b_{1,\text{nom}}, 2.00 b_{1,\text{nom}}), \quad k_t \sim \mathcal{U}(0.90 k_{t,\text{nom}}, 1.10 k_{t,\text{nom}})$$

In addition, Gaussian white noise is added to simulated states ($\sigma_\theta \approx 0.005\text{ rad}$, $\sigma_{\dot{\theta}} \approx 0.05\text{ rad/s}$), along with uniform sensor offset biases [15]. Physical serial interfaces and microcontroller execution introduce control loop latencies between $10\text{ ms}$ and $30\text{ ms}$ [15]. Inserting a FIFO transport delay buffer into the simulation loop trains the policy to anticipate communication lag, preventing latency-induced instability on real hardware [15].

### **TinyML and Embedded Policy Distillation**

Deploying deep neural networks onto low-cost microcontrollers requires policy distillation and memory compression [15]. While training often uses large networks ($2 \times 256$ hidden units) to facilitate gradient descent, balancing control can be distilled into compact Multi-Layer Perceptrons (MLPs) [15]. Ferrolho demonstrated that a minimal network featuring two hidden layers (16 and 8 neurons with ReLU activations), totaling just 689 floating-point parameters, reliably balances a physical rotary pendulum when executed on an 8-bit Arduino Nano [15].

Trained weights and biases are exported as static constant arrays (`const float weights[]`) and compiled into bare-metal C++ firmware [15]. On 32-bit targets, execution can also leverage the CMSIS-NN acceleration library, ONNX runtime engines, or MATLAB Coder deployment pipelines generated directly from the Reinforcement Learning Toolbox [35].

## **Comparative Performance Benchmarks and Engineering Implementation Roadmap**

Selecting an appropriate control architecture requires balancing analytical modeling effort against computational footprint, disturbance rejection, and empirical tracking performance [1].

### **Performance Metrics Formulation**

Quantifying controller performance during nominal balancing and external disturbance rejection relies on standard transient and error indices [1]:

* **Peak Overshoot**: $M_p = \frac{\theta_{1,\max} - \theta_{1,ss}}{\theta_{1,ss}} \times 100\%$ [cite: 1, 9]
* **10-to-90% Rise Time**: $t_r$ [cite: 1, 9]
* **2% Settling Time**: $t_s$ [cite: 1, 9]
* **Error Integrals**:
  $$\text{IAE} = \int_{0}^{T} \vert{}\theta_1(t)\vert{} dt, \quad \text{ISE} = \int_{0}^{T} \theta_1^2(t) dt, \quad \text{ITAE} = \int_{0}^{T} t \vert{}\theta_1(t)\vert{} dt$$
  IAE measures cumulative error magnitude, ISE penalizes large transient peaks, and ITAE emphasizes persistent long-term oscillations [1].
* **Actuator Effort**: Peak voltage ($u_{\max}$) and Root-Mean-Square control effort [1]:
  $$u_{\text{RMS}} = \sqrt{\frac{1}{T} \int_{0}^{T} u^2(t) dt}$$

The following benchmark comparison synthesizes theoretical characteristics and experimental results across classical, optimal, and reinforcement learning paradigms on rotary inverted pendulum hardware [1]:

| Performance Index / Feature | Dual-Loop PID Control | Full-State Pole Placement | Linear Quadratic Regulator (LQR) | Hybrid FSM (Swing-Up + LQR) | Deep Reinforcement Learning (SAC) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Operational Domain** | Local ($\pm 10^\circ$) | Local ($\pm 12^\circ$) | Local ($\pm 15^\circ$) | Global ($360^\circ$ via FSM) | Global ($360^\circ$ unified) |
| **Rise Time ($t_r$)** | Moderate ($0.35 - 0.50\text{ s}$) | Fast ($0.20 - 0.35\text{ s}$) | Fast ($0.18 - 0.28\text{ s}$) | Fast ($0.18 - 0.28\text{ s}$) | Fast ($0.15 - 0.25\text{ s}$) |
| **Settling Time ($t_s$, 2%)** | Sluggish ($1.20 - 1.80\text{ s}$) | Moderate ($0.80 - 1.20\text{ s}$) | Short ($0.60 - 0.90\text{ s}$) | Short ($0.60 - 0.90\text{ s}$) | Short ($0.50 - 0.80\text{ s}$) |
| **Peak Overshoot ($M_p$)** | High ($18 - 30\%$) | Moderate ($10 - 20\%$) | Low ($5 - 12\%$) | Low ($5 - 12\%$) | Moderate ($8 - 15\%$) |
| **Disturbance Recovery** | Slow recovery; position drift | Stiff; sensitive to noise | Rapid recovery; well-damped | Rapid recovery; well-damped | Adaptive recovery; broad basins |
| **Control Effort ($u_{\text{RMS}}$)** | High; persistent hunting | Moderate-to-High | Optimal; minimum energy | Optimal during balance | Low-to-Moderate (regularized) |
| **Error Indices (IAE / ITAE)** | High | Moderate | Low | Low | Very Low |
| **Memory Footprint** | Extremely Low ($< 1\text{ KB}$) | Low ($< 2\text{ KB}$) | Low ($< 2\text{ KB}$) | Moderate ($< 10\text{ KB}$) | Low in inference ($< 5\text{ KB}$, TinyML) |
| **Sim-to-Real Vulnerability** | Low (intuitive tuning) | High (model mismatch) | Moderate (stable margins) | Moderate (switching guards) | High (requires Domain Randomization) |

### **Engineering Implementation Roadmap**

Successfully taking a rotary inverted pendulum from mathematical models to a functional physical prototype requires a systematic development process [1].

1. **Physical Parameter Identification and Digital Twin Construction**: In this initial stage, link masses and centers of gravity are measured, pivot inertias are determined through bifilar oscillation tests, and joint damping coefficients are extracted from unforced decay logs [6]. A corresponding multi-body model is assembled in Simscape Multibody using accurate CAD geometries and material densities, and its open-loop behavior is validated against analytical ODE simulations [1].
2. **Controller Synthesis and Numerical Validation**: Once the model is verified, work moves to controller synthesis and numerical validation [1]. The nonlinear equations of motion are linearized about the upright apex to extract state-space matrices, confirming full system controllability and observability [1]. Dual-loop PID, pole placement, and LQR balancing controllers are synthesized, using Bryson's rule to configure weighting matrices [1]. The Åström-Furuta energy-shaping swing-up law is implemented alongside them in Simulink, tuning the energy gain to achieve smooth ascent without over-rotation [1].
3. **Hybrid Supervisory Architecture for Hardware-in-the-Loop Deployment**: Next, the controllers are integrated into a hybrid supervisory architecture for hardware-in-the-loop deployment [1]. The supervisory state machine is configured in MATLAB Stateflow, incorporating capture basin guards, angular hysteresis deadbands, and fail-safe trip logic [1]. An instrumentation dashboard is built in MATLAB App Designer to stream real-time telemetry over a serial interface, track supervisory states, compute online error metrics, and synchronize the 3D Simscape digital twin [1]. Control routines are compiled onto the target microcontroller to validate hardware balancing and benchmark disturbance rejection performance across the classical and optimal controllers [1].
4. **Learned Control via Deep Reinforcement Learning Pipeline**: Finally, the project expands into learned control by developing a deep reinforcement learning pipeline [1]. A custom Gymnasium environment is wrapped around a MuJoCo simulation of the physical platform [13]. Using Soft Actor-Critic (SAC) or Proximal Policy Optimization (PPO), an agent is trained on a continuous cosine reward with action continuity penalties [13]. Domain randomization across link parameters and transport delay buffers are introduced to bridge the sim-to-real gap [15]. The trained policy is then distilled into a compact neural network (fewer than 1,000 parameters) and exported as static C++ arrays, enabling bare-metal TinyML execution on the microcontroller to benchmark learned end-to-end control against the hybrid Stateflow/LQR baseline directly on physical hardware [15].

## **Conclusion**

The Furuta rotary inverted pendulum is a versatile engineering platform that bridges fundamental classical mechanics, optimal state-space control, hybrid supervisory logic, and modern data-driven reinforcement learning [1]. While classical dual-loop PID provides an accessible, model-free baseline for balance, its susceptibility to cross-axis coupling and sensor noise limits its performance on coupled underactuated systems [8]. Full-state feedback pole placement gives direct control over closed-loop dynamics, but requires an accurate system model and can easily saturate physical actuators [1]. Linear Quadratic Regulation (LQR) resolves these limitations by systematically balancing tracking accuracy against actuator effort, offering robust gain and phase margins that make it an ideal choice for localized balance control [1].

Because linear controllers operate within a limited capture basin, practical hardware implementations require a hybrid control architecture [1]. Pairing an Åström-Furuta energy-shaping swing-up law with a supervisory Stateflow state machine provides reliable global recovery from the hanging position, using angular hysteresis and dwell-time filtering to eliminate boundary chattering and manage hardware safety trips [1]. Integrating this control stack with a Simscape Multibody digital twin and a real-time App Designer telemetry dashboard establishes a complete Hardware-in-the-Loop workflow that tracks system states, visualizes dynamics, and quantifies performance metrics like IAE and ITAE [1].

Extending the system to deep reinforcement learning demonstrates the viability of learned, end-to-end control on underactuated platforms [13]. Soft Actor-Critic (SAC) and Proximal Policy Optimization (PPO) can synthesize swing-up and balance behaviors into a single neural network when trained on continuous trigonometric observations and smooth cosine reward functions [13]. Bridging the reality gap, however, requires systematic domain randomization and transport delay modeling during simulation [15]. Once trained, policy distillation enables compact neural networks (under 1,000 parameters) to run on low-cost microcontrollers via TinyML, providing a direct comparison between optimal model-based control and learned data-driven policies on physical hardware [15].

#### **Works Cited**

1. Project Description `F26_3faaec4d6f9dd5653c19bb021509ddef.pdf`
2. On the Dynamics of the Furuta Pendulum - ResearchGate, [https://www.researchgate.net/publication/228421122_On_the_Dynamics_of_the_Furuta_Pendulum](https://www.researchgate.net/publication/228421122_On_the_Dynamics_of_the_Furuta_Pendulum)
3. Modelling the Furuta Pendulum Gäfvert, Magnus - Lucris, [https://lucris.lub.lu.se/ws/files/4453844/8727127.pdf](https://lucris.lub.lu.se/ws/files/4453844/8727127.pdf)
4. Dynamic Modeling and Simulation of an Underactuated System - arXiv, [https://arxiv.org/pdf/1706.04996](https://arxiv.org/pdf/1706.04996)
5. A swinging up controller for the furuta pendulum based on the total, [https://www.kybernetika.cz/content/2019/2/402/paper.pdf](https://www.kybernetika.cz/content/2019/2/402/paper.pdf)
6. Soft Constrained Warm Start based MPC-PD Approach for Real, [https://latamt.ieeer9.org/index.php/transactions/article/download/9660/2834](https://latamt.ieeer9.org/index.php/transactions/article/download/9660/2834)
7. Underactuated Trajectory-Tracking Control for Long-Exposure, [https://www.ias.informatik.tu-darmstadt.de/uploads/Team/FabioMuratore/IP_UnderactuatedTrajTrackingControlForLongExposurePhotography_I_report_final.pdf](https://www.ias.informatik.tu-darmstadt.de/uploads/Team/FabioMuratore/IP_UnderactuatedTrajTrackingControlForLongExposurePhotography_I_report_final.pdf)
8. QUBE-Servo Workbook (Student) | PDF | Electric Motor - Scribd, [https://www.scribd.com/document/580305998/QUBE-Servo-Workbook-Student](https://www.scribd.com/document/580305998/QUBE-Servo-Workbook-Student)
9. Agentic AI for Real-Time Adaptive PID Control of a Servo Motor - MDPI, [https://www.mdpi.com/2076-0825/14/9/459](https://www.mdpi.com/2076-0825/14/9/459)
10. QUBE-SERVO 2 Swing-Up Control Guide | PDF | Pendulum - Scribd, [https://www.scribd.com/document/860697901/QUBE-Servo-2-Swing-Up-Control-Workbook-Instructor](https://www.scribd.com/document/860697901/QUBE-Servo-2-Swing-Up-Control-Workbook-Instructor)
11. A Multibody-Based Benchmarking Framework for the Control of the, [https://www.mdpi.com/2076-0825/14/8/377](https://www.mdpi.com/2076-0825/14/8/377)
12. LQR controller for Rotary inverted pendulum tuning problems, [https://www.mathworks.com/matlabcentral/answers/2180502-lqr-controller-for-rotary-inverted-pendulum-tuning-problems](https://www.mathworks.com/matlabcentral/answers/2180502-lqr-controller-for-rotary-inverted-pendulum-tuning-problems)
13. Armandpl/furuta: Building and Training a Rotary Inverted Pendulum, [https://github.com/Armandpl/furuta](https://github.com/Armandpl/furuta)
14. Furuta Pendulum - Dagor-Brushless-Controller, [https://www.dagor.dev/blog/furuta-pendulum](https://www.dagor.dev/blog/furuta-pendulum)
15. ferrolho/rotary-inverted-pendulum - GitHub, [https://github.com/ferrolho/rotary-inverted-pendulum](https://github.com/ferrolho/rotary-inverted-pendulum)
16. Swinging up a pendulum by energy control, [https://web.ece.ucsb.edu/~hespanha/ece229/references/AstromFurutaAUTOM00.pdf](https://web.ece.ucsb.edu/~hespanha/ece229/references/AstromFurutaAUTOM00.pdf)
17. A swinging up controller for the Furuta pendulum based ... - DML-CZ, [https://dml.cz/handle/10338.dmlcz/147844](https://dml.cz/handle/10338.dmlcz/147844)
18. Revitalizing the Furuta pendulum, [https://research.tue.nl/files/4307900/626064.pdf](https://research.tue.nl/files/4307900/626064.pdf)
19. Swinging up a pendulum by energy control - Semantic Scholar, [https://www.semanticscholar.org/paper/Swinging-up-a-pendulum-by-energy-control-%C3%85str%C3%B6m-Furuta/8a259e547088912d0323b0d4fa7d0dafde50e4ee](https://www.semanticscholar.org/paper/Swinging-up-a-pendulum-by-energy-control-%C3%85str%C3%B6m-Furuta/8a259e547088912d0323b0d4fa7d0dafde50e4ee)
20. Full State Feedback of Furuta Pendulum - File Exchange - MathWorks, [https://www.mathworks.com/matlabcentral/fileexchange/25585-full-state-feedback-of-furuta-pendulum?s_tid=prof_contriblnk](https://www.mathworks.com/matlabcentral/fileexchange/25585-full-state-feedback-of-furuta-pendulum?s_tid=prof_contriblnk)
21. Mechatronic Systems - MDPI, [https://mdpi-res.com/bookfiles/book/12853/Mechatronic_Systems.pdf?v=1790731160](https://mdpi-res.com/bookfiles/book/12853/Mechatronic_Systems.pdf?v=1790731160)
22. Swing-up and Stabilisation of a Furuta Pendulum using Energy, [https://www.academia.edu/31024303/Swing_up_and_Stabilisation_of_a_Furuta_Pendulum_using_Energy_Control](https://www.academia.edu/31024303/Swing_up_and_Stabilisation_of_a_Furuta_Pendulum_using_Energy_Control)
23. B&R Automation Studio Target for Simulink®, [https://kyb.fei.tuke.sk/laboratoria/prezentacie/br%20automation%20studio%20target%20for%20simulink.pdf](https://kyb.fei.tuke.sk/laboratoria/prezentacie/br%20automation%20studio%20target%20for%20simulink.pdf)
24. Made For Science Quanser Qube Servo 2 CoursewareStud ... - Scribd, [https://www.scribd.com/document/486299841/made-for-science-Quanser-qube-servo-2-CoursewareStud-MATLAB-pdf](https://www.scribd.com/document/486299841/made-for-science-Quanser-qube-servo-2-CoursewareStud-MATLAB-pdf)
25. Controlling the Furuta pendulum: Proof of concept through virtual, [https://jart.icat.unam.mx/index.php/jart/article/view/2347](https://jart.icat.unam.mx/index.php/jart/article/view/2347)
26. Furuta (rotary) Pendulum - File Exchange - MATLAB Central, [https://www.mathworks.com/matlabcentral/fileexchange/34521-furuta-rotary-pendulum](https://www.mathworks.com/matlabcentral/fileexchange/34521-furuta-rotary-pendulum)
27. Rotary Inverted Pendulum: Rotating shaft is not rotating the arm at, [https://de.mathworks.com/matlabcentral/answers/2184949-rotary-inverted-pendulum-rotating-shaft-is-not-rotating-the-arm-attached-to-it/?s_tid=ans_lp_feed_leaf](https://de.mathworks.com/matlabcentral/answers/2184949-rotary-inverted-pendulum-rotating-shaft-is-not-rotating-the-arm-attached-to-it/?s_tid=ans_lp_feed_leaf)
28. Inverted Pendulum with Animation - MATLAB & Simulink - MathWorks, [https://www.mathworks.com/help/simulink/slref/inverted-pendulum-with-animation.html](https://www.mathworks.com/help/simulink/slref/inverted-pendulum-with-animation.html)
29. Add App Designer App to Inverted Pendulum Model - MathWorks, [https://www.mathworks.com/help/slrealtime/ug/add-app-des-app-to-inverted-pendulum-model.html](https://www.mathworks.com/help/slrealtime/ug/add-app-des-app-to-inverted-pendulum-model.html)
30. Implementing real time serial logging to app designer - MathWorks, [https://www.mathworks.com/matlabcentral/answers/847220-implementing-real-time-serial-logging-to-app-designer](https://www.mathworks.com/matlabcentral/answers/847220-implementing-real-time-serial-logging-to-app-designer)
31. Google Drive Resource Document, [https://drive.google.com/open?id=1FafBFPmdkxIdFKBmrCwpmW74ychCuQ6cblw2YOzWr3g](https://drive.google.com/open?id=1FafBFPmdkxIdFKBmrCwpmW74ychCuQ6cblw2YOzWr3g)
32. Furuta pendulum - File Exchange - MATLAB Central - MathWorks, [https://www.mathworks.com/matlabcentral/fileexchange/23490-furuta-pendulum?s_tid=FX_rc1_behav](https://www.mathworks.com/matlabcentral/fileexchange/23490-furuta-pendulum?s_tid=FX_rc1_behav)
33. Furuta pendulum - File Exchange - MATLAB Central - MathWorks, [https://www.mathworks.com/matlabcentral/fileexchange/23490-furuta-pendulum](https://www.mathworks.com/matlabcentral/fileexchange/23490-furuta-pendulum)
34. Full State Feedback of Furuta Pendulum - File Exchange - MathWorks, [https://www.mathworks.com/matlabcentral/fileexchange/25585-full-state-feedback-of-furuta-pendulum](https://www.mathworks.com/matlabcentral/fileexchange/25585-full-state-feedback-of-furuta-pendulum)
35. Reinforcement-Learning-Inverted-Pendulum-with ... - MathWorks, [https://www.mathworks.com/matlabcentral/fileexchange/99364-reinforcement-learning-inverted-pendulum-with-qube-servo2/files/PID/design_PID_control_ja.mlx](https://www.mathworks.com/matlabcentral/fileexchange/99364-reinforcement-learning-inverted-pendulum-with-qube-servo2/files/PID/design_PID_control_ja.mlx)
36. Reinforcement-Learning-Inverted-Pendulum-with ... - MathWorks, [https://www.mathworks.com/matlabcentral/fileexchange/99364-reinforcement-learning-inverted-pendulum-with-qube-servo2/files/RL/design_RL_multi_control_PPO_ja.mlx](https://www.mathworks.com/matlabcentral/fileexchange/99364-reinforcement-learning-inverted-pendulum-with-qube-servo2/files/RL/design_RL_multi_control_PPO_ja.mlx)
37. bgkatz/Furuta-Pendulum - GitHub, [https://github.com/bgkatz/Furuta-Pendulum](https://github.com/bgkatz/Furuta-Pendulum)
38. Inverted Pendulum - Arduino Balancing Robot - YouTube, [https://www.youtube.com/watch?v=UL99bJTtESk](https://www.youtube.com/watch?v=UL99bJTtESk)
39. Rotary Inverted Pendulum: requirements, use and source availability, [https://opensourcerobotics.dev/robots/rotary-inverted-pendulum/](https://opensourcerobotics.dev/robots/rotary-inverted-pendulum/)
40. Reinforcement Learning on Hardware from Sim-to-Real (Rotary), [https://kevinwoodrobotics.com/product/reinforcement-learning-on-hardware-from-sim-to-real-rotary-inverted-pendulum/](https://kevinwoodrobotics.com/product/reinforcement-learning-on-hardware-from-sim-to-real-rotary-inverted-pendulum/)
41. Kevin Wood | Robotics & AI - YouTube, [https://www.youtube.com/channel/UC0jeEKE9IzfYKDuIbQLJIog/about](https://www.youtube.com/channel/UC0jeEKE9IzfYKDuIbQLJIog/about)
42. Matching Mujoco Simulation to Real Hardware for Sim-to ... - YouTube, [https://www.youtube.com/watch?v=gBeU6EHJLJk](https://www.youtube.com/watch?v=gBeU6EHJLJk)
43. Inertia Wheel Pendulum: How It Works, Diagram & Examples, [https://www.firgelliauto.com/blogs/mechanisms/inertia-wheel-pendulum](https://www.firgelliauto.com/blogs/mechanisms/inertia-wheel-pendulum)
44. Modeling, Simulation, and Control of a Rotary Inverted Pendulum, [https://www.researchgate.net/publication/386174932_Modeling_Simulation_and_Control_of_a_Rotary_Inverted_Pendulum_A_Reinforcement_Learning-Based_Control_Approach](https://www.researchgate.net/publication/386174932_Modeling_Simulation_and_Control_of_a_Rotary_Inverted_Pendulum_A_Reinforcement_Learning-Based_Control_Approach)
45. Correlated Exploration in Deep Reinforcement Learning, [https://www.ias.informatik.tu-darmstadt.de/uploads/Site/EditPublication/maximilian_hensel.pdf](https://www.ias.informatik.tu-darmstadt.de/uploads/Site/EditPublication/maximilian_hensel.pdf)
46. Compare reinforcement learning control model with conventional, [https://www.researchgate.net/figure/Compare-reinforcement-learning-control-model-with-conventional-PID-control-model-and_fig2_373540444](https://www.researchgate.net/figure/Compare-reinforcement-learning-control-model-with-conventional-PID-control-model-and_fig2_373540444)
47. Sim-to-Real Reinforcement Learning for a Rotary Double-Inverted, [https://www.mdpi.com/2227-7390/13/12/1996](https://www.mdpi.com/2227-7390/13/12/1996)
48. Research Projects - Yunlong Song, [https://yunlong-song.com/project.html](https://yunlong-song.com/project.html)
49. Efficient Exploration in Continuous-time Model-based ... - OpenReview, [https://openreview.net/pdf?id=VkhvDfY2dB](https://openreview.net/pdf?id=VkhvDfY2dB)
50. Stable Baselines3 Documentation, [https://stable-baselines3.readthedocs.io/_/downloads/en/master/pdf/](https://stable-baselines3.readthedocs.io/_/downloads/en/master/pdf/)
51. Modeling, Simulation, and Control of a Rotary Inverted Pendulum, [https://www.mdpi.com/2673-3951/5/4/95](https://www.mdpi.com/2673-3951/5/4/95)
52. Neural Posterior Domain Randomization, [https://proceedings.mlr.press/v164/muratore22a/muratore22a.pdf](https://proceedings.mlr.press/v164/muratore22a/muratore22a.pdf)
53. Sensitivity-Based Robustness Evaluation of a Reinforcement ... - MDPI, [https://www.mdpi.com/2673-4591/157/1/2](https://www.mdpi.com/2673-4591/157/1/2)
54. Data-efficient Domain Randomization with Bayesian Optimization, [https://www.alphaxiv.org/abs/2003.02471](https://www.alphaxiv.org/abs/2003.02471)
55. ferrolho/rotary-inverted-pendulum：一个20英镑的开源旋转倒立摆, [https://roboairadar.com/news/cms0i8h0b000e04jxaibz2nb5](https://roboairadar.com/news/cms0i8h0b000e04jxaibz2nb5)
56. Agentic AI For Real-Time Adaptive PID Control of Servo Motor, [https://www.preprints.org/manuscript/202508.0355](https://www.preprints.org/manuscript/202508.0355)

---
