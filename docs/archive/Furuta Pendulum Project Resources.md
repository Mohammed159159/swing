# **Comprehensive Technical Review and Reinforcement Learning Integration for the Rotary Inverted Pendulum**

## **Theoretical Modeling, Euler-Lagrange Dynamics, and Parameter Identification**

The Furuta pendulum, or rotary inverted pendulum, is an underactuated mechanical benchmark characterized by two degrees of freedom and a single control actuator. First introduced by Katsuhisa Furuta and colleagues in 1991 at the Tokyo Institute of Technology, the platform comprises a horizontally rotating base arm driven by an actuator (generalized coordinate \\theta\_0) and an unactuated secondary pendulum link (generalized coordinate \\theta\_1) that rotates freely in the vertical plane at the tip of the horizontal arm. The mechanism exhibits severe nonlinearities, state configurations defined on non-Euclidean manifolds (S^1 \\times S^1), and open-loop instability about its upright equilibrium point. Although originally developed as a laboratory benchmark, its balancing and stabilization dynamics directly mirror complex engineering systems, including thrust-vector control in rocketry, bipedal humanoid robotic balance, crane cargo anti-sway suppression, and personal balancing transporters.  
`Actuated Base Motor (Torque \tau)`   
       `|`  
       `v`  
`Horizontal Arm Joint (\theta_0, length L_0)`  
       `|`  
       `v`  
`Unactuated Pivot Joint (\theta_1)`  
       `|`  
       `v`  
`Inverted Pendulum Link (CoM l_1, mass m_1)`

The physical assembly couples the actuated horizontal joint to the unactuated vertical pivot, transmitting rotational inertia through centrifugal, Coriolis, and gravitational forces.

### **Nonlinear Euler-Lagrange Formulation**

The nonlinear equations of motion governing the rotary inverted pendulum are derived from analytical mechanics using the Euler-Lagrange equations of the second kind:  
\\frac{d}{dt} \\left( \\frac{\\partial \\mathcal{L}}{\\partial \\dot{q}} \\right) \- \\frac{\\partial \\mathcal{L}}{\\partial q} \= Q  
where q \= \[\\theta\_0, \\theta\_1\]^T represents the vector of generalized coordinates, \\mathcal{L} \= T \- V denotes the Lagrangian of the mechanical system, T is the total kinetic energy, V is the total potential energy, and Q is the generalized force vector accounting for non-conservative actuator inputs and viscous friction torques.  
Assuming rigid link geometry, the horizontal arm is characterized by mass m\_0, length L\_0, and mass moment of inertia J\_0 about its central axis of rotation. The pendulum link is parameterized by mass m\_1, total length L\_1, center of mass distance l\_1 \= L\_1/2, and moment of inertia J\_1 about its center of mass. Establishing a reference Cartesian frame at the base pivot, the position vector defining an arbitrary differential element along the pendulum rod is written as:  
\\mathbf{r}\_1 \= \\begin{bmatrix} L\_0 \\cos\\theta\_0 \- r\_p \\sin\\theta\_0 \\sin\\theta\_1 \\\\ L\_0 \\sin\\theta\_0 \+ r\_p \\cos\\theta\_0 \\sin\\theta\_1 \\\\ r\_p \\cos\\theta\_1 \\end{bmatrix}  
where r\_p defines the radial distance measured along the pendulum link from the pivot joint. Differentiating the position vector with respect to time yields the instantaneous velocity field across both bodies, enabling analytical integration of the system kinetic energy T and gravitational potential energy V:  
T \= \\frac{1}{2} J\_0 \\dot{\\theta}\_0^2 \+ \\frac\[span\_13\](start\_span)\[span\_13\](end\_span)\[span\_15\](start\_span)\[span\_15\](end\_span){1}{2} m\_1 \\left( L\_0^2 \\dot{\\theta}\_0^2 \+ l\_1^2 \\dot{\\theta}\_1^2 \+ l\_1^2 \\dot{\\theta}\_0^2 \\sin^2\\theta\_1 \+ 2 L\_0 l\_1 \\dot{\\theta}\_0 \\dot{\\theta}\_1 \\cos\\theta\_1 \\right) \+ \\frac{1}{2} J\_1 \\dot{\\theta}\_1^2 V \= m\_1 g l\_1 (1 \- \\cos\\theta\_1)  
Evaluating the partial derivatives across the generalized coordinates yields two coupled second-order nonlinear differential equations. Expressed in standard robotic manipulator form, the dynamics satisfy:  
M(q)\\ddot{q} \+ C(q,\\dot{q})\\dot{q} \+ G(q) \+ F(\\dot{q}) \= \\begin{bmatrix} \\tau \\\\ 0 \\end{bmatrix}  
where M(q) denotes the positive-definite symmetric generalized inertia matrix, C(q,\\dot{q}) encapsulates Coriolis and centripetal terms, G(q) is the gravitational torque vector, and F(\\dot{q}) defines viscous joint damping:  
M(q) \= \\begin{bmatrix} \\alpha \+ \\beta \\sin^2\\theta\_1 & \\gamma \\cos\\theta\_1 \\\\ \\gamma \\cos\\theta\_1 & \\beta \\end{bmatrix} C(q,\\dot{q}) \= \\begin{bmatrix} \\beta \\dot{\\theta}\_1 \\sin(2\\theta\_1) & \-\\gamma \\dot{\\theta}\_1 \\sin\\theta\_1 \+ \\frac{1}{2}\\beta \\dot{\\theta}\_0 \\sin(2\\theta\_1) \\\\ \-\\frac{1}{2}\\beta \\dot{\\theta}\_0 \\sin(2\\theta\_1) & 0 \\end{bmatrix} G(q) \= \\begin{bmatrix} 0 \\\\ \-\\delta \\sin\\theta\_1 \\end{bmatrix}, \\quad F(\\dot{q}) \= \\begin{bmatrix} b\_0 \\dot{\\theta}\_0 \\\\ b\_1 \\dot{\\theta}\_1 \\end{bmatrix}  
The physical constants are lumped into concise algebraic groupings:

\\alpha \= J\_0 \+ m\_1 L\_\[span\_23\](start\_span)\[span\_23\](end\_span)0^2, \\quad \\beta \= J\_1 \+ m\_1 l\_1^2, \\quad \\gamma \= m\_1 L\_0 l\_1, \\quad \\delta \= m\_1 g l\_1

### **Actuator Dynamics and Electromechanical Coupling**

The input torque \\tau applied to the horizontal arm originates from a direct-current servomotor attached to the stationary foundation. Modeling the electrical armature circuit yields the voltage-current relationship:  
V\_m(t) \= R\_m i\_a(t) \+ L\_m \\frac{d i\_a(t)}{dt} \+ k\_m \\dot{\\theta\[span\_6\](start\_span)\[span\_6\](end\_span)}\_0(t)  
where V\_m is the applied armature voltage, R\_m is the winding terminal resistance, L\_m represents armature inductance, i\_a denotes armature current, and k\_m represents the motor back-electromotive force (back-EMF) constant. Because the electrical time constant \\tau\_e \= L\_m / R\_m is negligible relative to the dominant mechanical time constants of the physical arm assembly, armature inductance is conventionally set to zero.  
Solving algebraically for armature current and combining the result with the motor torque relationship \\tau \= k\_t i\_a, where k\_t represents the motor torque constant, gives the electromechanical torque input:  
\\tau \= \\frac{k\_t}{R\_m} V\_m \- \\frac{k\_t k\_m}{R\_m} \\dot{\\theta}\_0  
Substituting this direct electromechanical relation into the horizontal joint dynamic equation demonstrates that back-EMF introduces an intrinsic viscous damping term, modifying the effective base damping coefficient to b\_{0,\\text{eff}} \= b\_0 \+ \\frac{k\_t k\_m}{R\_m} when the system is driven via motor armature voltage u(t) \= V\_m(t).

### **Equilibrium Points, Linearization, and Controllability**

In the unforced operational regime (\\tau \= 0), setting angular velocities and accelerations to zero reveals two distinct equilibrium manifolds: the stable hanging downward equilibrium x\_{\\text{down}} \= \[0, \\pi, 0, 0\]^T and the open-loop unstable upright equilibrium x\_{\\text{up}} \= \[0, 0, 0, 0\]^T.  
Linearizing the nonlinear equations of motion about the unstable upright equilibrium through a first-order Taylor series expansion (\\sin\\theta\_1 \\approx \\theta\_1, \\cos\\theta\_1 \\approx 1, and dropping second-order velocity products) yields the continuous-time linear state-space system:  
\\dot{x} \= A x \+ B u, \\quad y \= C x  
where the state vector is chosen as x \= \[\\theta\_0, \\theta\_1, \\dot{\\theta}\_0, \\dot{\\theta}\_1\]^T and the control input is u \= V\_m. Evaluating the analytical Jacobians results in the system matrices:  
A \= \\begin{bmatrix} 0 & 0 & 1 & 0 \\\\ 0 & 0 & 0 & 1 \\\\ 0 & \\frac{\\gamma \\delta}{\\alpha \\beta \- \\gamma^2} & \-\\frac{\\beta \\left(b\_0 \+ \\frac{k\_t k\_m}{R\_m}\\right)}{\\alpha \\beta \- \\gamma^2} & \\frac{\\gamma b\_1}{\\alpha \\beta \- \\gamma^2} \\\\ 0 & \\frac{\\alpha \\delta}{\\alpha \\beta \- \\gamma^2} & \\frac{\\gamma \\left(b\_0 \+ \\frac{k\_t k\_m}{R\_m}\\right)}{\\alpha \\beta \- \\gamma^2} & \-\\frac{\\alpha b\_1}{\\alpha \\beta \- \\gamma^2} \\end{bmatrix}, \\quad B \= \\begin{bmatrix} 0 \\\\ 0 \\\\ \\frac{\\beta \\frac{k\_t}{R\_m}}{\\alpha \\beta \- \\gamma^2} \\\\ \-\\frac{\\gamma \\frac{k\_t}{R\_m}}{\\alpha \\beta \- \\gamma^2} \\end{bmatrix}  
The term \\alpha \\beta \- \\gamma^2 \= J\_0(J\_1 \+ m\_1 l\_1^2) \+ J\_1 m\_1 L\_0^2 represents the determinant of the inertia tensor evaluated at the upright configuration and is strictly positive for all real physical mass distributions, guaranteeing invertibility. Evaluating the characteristic polynomial \\det(sI \- A) \= 0 reveals the open-loop spectrum \\lambda(A) \= \\{0, \-\\lambda\_1, \+\\l\[span\_24\](start\_span)\[span\_24\](end\_span)ambda\_2, \-\\lambda\_3\\}. The presence of a real, positive root \+\\lambda\_2 \\approx \\sqrt{\\frac{\\alpha \\delta}{\\alpha \\beta \- \\gamma^2}} confirms that the unforced upright equilibrium is an unstable hyperbolic saddle point. The zero eigenvalue reflects rotational invariance of the horizontal base arm along the horizontal plane.  
System controllability is verified by constructing the Kalman controllability rank matrix:  
\\mathcal{C} \= \\begin{bmatrix} B & AB & A^2 B & A^3 B \\end{bmatrix}  
Computing the determinant confirms \\det(\\mathcal{C}) \\neq 0 and \\text{rank}(\\mathcal{C}) \= 4, establishing full-state controllability. Given that physical rotary pendulums measure both the horizontal base angle \\theta\_0 and the link deflection \\theta\_1 using optical or magnetic rotary encoders, the measurement output matrix is C \= \\begin{bmatrix} I\_{2\\times 2} & 0\_{2\\times 2} \\end{bmatrix}, which similarly satisfies the Kalman observability rank condition \\text{rank}(\\mathcal{O}) \= 4\.

### **Physical Parameter Identification Methods**

Accurate control design requires precise identification of plant parameters to mitigate the reality gap between numerical simulations and physical hardware. Geometric dimensions and static link masses are obtained directly using digital calipers and precision analytical scales. Determining mass moments of inertia requires decoupled dynamic testing. The pendulum link inertia J\_1 is measured using a bifilar torsional pendulum rig, calculating inertia from the oscillation period:  
J\_1 \= \\frac{m\_1 g r^2 T\_{\\text{osc}}^2}{4 \\pi^2 L\_{\\text{fil}}}  
where r represents bifilar suspension radius and L\_{\\text{fil}} denotes filament length, or by integrating three-dimensional CAD geometric meshes assigned known material densities.  
Joint friction parameters require targeted experimental estimation. The unactuated pendulum joint viscous damping coefficient b\_1 is identified by decoupling the pendulum link from the motor assembly, displacing it to an initial deflection angle \\theta\_1(0) \\approx 30^\\circ, and recording the unforced decay trajectory. Applying the logarithmic decrement formulation across n successive oscillation periods yields:  
\\delta\_{\\text{dec}} \= \\frac{1}{n} \\ln\\left( \\frac{\\theta\_{1}(t)}{\\theta\_{1}(t \+ n T\_d)} \\right), \\quad \\zeta \= \\frac{\\delta\_{\\text{dec}}}{\\sqrt{4\\pi^2 \+ \\delta\_{\\text{dec}}^2}}, \\quad b\_1 \= 2 \\zeta \\sqrt{m\_1 g l\_1 (J\_1 \+ m\_1 l\_1^2)}  
Motor electrical resistance R\_m is determined via four-point Kelvin resistance sensing with a locked rotor. The motor torque constant k\_t and back-EMF constant k\_m are determined by driving the motor under steady-state no-load conditions across varying terminal voltages while recording rotational velocity (V\_m \\approx k\_m \\omega\_{\\text{no-load}}), complemented by static torque measurements under locked-rotor conditions (k\_t \= \\tau\_{\\text{stall}} / i\_{\\text{stall}}).

| Parameter Symbol | Physical Description | Quanser QUBE-Servo 2 Reference Value | Custom / 3D-Printed Prototype Range | Identification Strategy |
| :---- | :---- | :---- | :---- | :---- |
| L\_0 | Rotary arm pivot-to-pivot length | 0.0850 \- 0.0895\\text{ m} | 0.100 \- 0.200\\text{ m} | Precision Caliper Metrology |
| m\_0 | Horizontal arm total mass | 0.0950\\text{ kg} | 0.080 \- 0.187\\text{ kg} | Digital Balance Scale |
| J\_0 | Arm pivot moment of inertia | 2.28 \\times 10^{-4}\\text{ kg}\\cdot\\text{m}^2 | 5.57 \\times 10^{-4} \- 2.50 \\times 10^{-3}\\text{ kg}\\cdot\\text{m}^2 | CAD Mass Properties / Bifilar Rig |
| L\_1 | Pendulum link overall length | 0.129\\text{ m} | 0.150 \- 0.300\\text{ m} | Precision Caliper Metrology |
| l\_1 | Pendulum center of mass location | 0.0645\\text{ m} | 0.075 \- 0.150\\text{ m} | Knife-Edge Center of Gravity Balance |
| m\_1 | Pendulum link total mass | 0.0240\\text{ kg} | 0.020 \- 0.127\\text{ kg} | Digital Balance Scale |
| J\_1 | Pendulum center of mass inertia | 1.33 \\times 10^{-4}\\text{ kg}\\cdot\\text{m}^2 | 1.20 \\times 10^{-4} \- 1.20 \\times 10^{-3}\\text{ kg}\\cdot\\text{m}^2 | Free Oscillation Frequency Period |
| R\_m | Motor armature terminal resistance | 8.40\\ \\Omega | 1.20 \- 12.0\\ \\Omega | Locked-Rotor Digital Multimeter |
| k\_t | Motor electromechanical torque constant | 0.042\\text{ N}\\cdot\\text{m/A} | 0.025 \- 0.085\\text{ N}\\cdot\\text{m/A} | Dynamometer Stall Torque Slope |
| k\_m | Motor back-EMF voltage constant | 0.040\\text{ V}/(\\text{rad/s}) | 0.025 \- 0.085\\text{ V}/(\\text{rad/s}) | Steady-State No-Load Saturation Slope |
| b\_0 | Arm joint viscous damping factor | 5.00 \\times 10^{-5}\\text{ N}\\cdot\\text{m}\\cdot\\text{s/rad} | 1.00 \\times 10^{-4} \- 2.40 \\times 10^{-3}\\text{ N}\\cdot\\text{m}\\cdot\\text{s/rad} | Unloaded Motor Deceleration Curve |
| b\_1 | Pendulum pivot viscous damping factor | 1.00 \\times 10^{-4}\\text{ N}\\cdot\\text{m}\\cdot\\text{s/rad} | 1.50 \\times 10^{-4} \- 2.40 \\times 10^{-3}\\text{ N}\\cdot\\text{m}\\cdot\\text{s/rad} | Logarithmic Decrement Free Decay |

## **Classical, Optimal, and Nonlinear Control Syntheses**

Stabilizing an underactuated mechanical plant across its complete operational space requires decomposing the problem into two distinct regimes: large-scale nonlinear energy injection to swing the pendulum up from its hanging resting state, and high-bandwidth linear state feedback to balance it within a localized capture basin around the vertical equilibrium.  
The control architecture routes sensor feedback through a high-level switching mechanism. When the pendulum rests in the downward hemisphere, the energy-shaping controller commands arm accelerations to pump mechanical energy into the link. Once state trajectories enter the linearized capture basin (\\vert{}\\theta\_1\\vert{} \< \\epsilon), supervisory logic transfers authority to a linear balancing controller (such as an LQR or pole-placement state-feedback law) that stabilizes the upright position while regulating arm drift.

### **Energy-Based Swing-Up Control**

Because linear controllers cannot recover the pendulum from the downward equilibrium (\\theta\_1 \= \\pi), swing-up relies on nonlinear energy control, grounded in the foundational formulations developed by Karl Johan Åström and Katsuhisa Furuta. Neglecting high-order coupling during the initial pump cycles, the instantaneous mechanical energy of the pendulum link, referenced to zero at the upright equilibrium, is expressed as:  
E(\\theta\_1, \\dot{\\theta}\_1) \= \\frac{1}{2} J\_p \\dot{\\theta}\_1^2 \+ m\_1 g l\_1 (1 \- \\cos\\theta\_1)  
where J\_p \= J\_1 \+ m\_1 l\_1^2 defines the total link inertia about the pivot joint. At the inverted target state (\\theta\_1 \= 0, \\dot{\\theta}\_1 \= 0), the target reference energy is E\_r \= 0 (or E\_r \= 2 m\_1 g l\_1 when referencing potential energy to the lowest hanging configuration). Differentiating total energy with respect to time along the system trajectories reveals:  
\\dot{E} \= J\_p \\dot{\\theta}\_1 \\ddot{\\theta}\_1 \+ m\_1 g l\_1 \\dot{\\theta}\_1 \\sin\\theta\_1 \= \-m\_1 l\_1 \\dot{\\theta}\_1 \\cos\\theta\_1 \\cdot a\_{\\text{pivot}}  
where a\_{\\text{pivot}} \= L\_0 \\ddot{\\theta}\_0 represents the linear acceleration induced at the pendulum pivot by the base arm. Selecting a positive-definite Lyapunov candidate based on energy error, V\_E \= \\frac{1}{2}(E \- E\_r)^2, its time derivative evaluates to:  
\\dot{V}\_E \= (E \- E\_r)\\dot{E} \= \-(E \- E\_r) m\_1 l\_1 \\dot{\\theta}\_1 \\cos\\theta\_1 \\cdot a\_{\\text{pivot}}  
To guarantee negative semi-definiteness (\\dot{V}\_E \\le 0), the base pivot acceleration is commanded to drive the energy error asymptotically to zero:  
a\_{\\text{pivot}} \= k\_e (E \- E\_r) \\operatorname{sign}(\\dot{\\theta}\_1 \\cos\\theta\_1)  
where k\_e is an empirical tuning gain. Mapping commanded acceleration to motor terminal voltage and introducing control saturation to prevent excessive current draw yields the bounded energy-shaping swing-up law:  
u\_{\\text{swing}} \= \\operatorname{sat}\_{V\_{\\max}}\\left( k\_E (E \- E\_r) \\operatorname{sign}(\\dot{\\theta}\_1 \\cos\\theta\_1) \\right)  
Physical implementations often incorporate an auxiliary proportional-derivative feedback term on the base arm angle (\\theta\_0) to prevent unbounded arm drift during consecutive swing cycles, ensuring the base remains centered within its safe rotational limits.

### **Linear Balancing Controllers: PID, Pole Placement, and LQR**

Once the swing-up trajectory brings the pendulum within the vicinity of the vertical apex, control is transferred to a linear feedback architecture designed using the linearized state-space model.  
Dual-loop Proportional-Integral-Derivative (PID) control addresses the underactuated nature of the system by running two parallel control loops into a single actuator voltage command:  
u\_{\\text{PID}}(t) \= K\_{p,1} \\theta\_1(t) \+ K\_{d,1} \\dot{\\theta}\_1(t) \+ K\_{p,0} \\theta\_0(t) \+ K\_{i,0} \\int\_0^t \\theta\_0(\\tau) d\\tau \+ K\_{d,0} \\dot{\\theta}\_0(t)  
The primary inner loop focuses on pendulum balance through high derivative action (K\_{d,1}), which provides the damping needed to stabilize the unstable pole. The secondary outer loop regulates the horizontal base arm back to its zero setpoint via weak proportional-integral action (K\_{p,0\[span\_107\](start\_span)\[span\_107\](end\_span)\[span\_113\](start\_span)\[span\_113\](end\_span)}, K\_{i,0}). Because the two degrees of freedom are dynamically coupled, tuning these gains requires careful coordination to prevent cross-axis limit cycles and high-frequency chatter.  
Full-state feedback pole placement synthesizes a state gain matrix K\_{\\text{PP}} \= \[k\_1, k\_2, k\_3, k\_4\] such that the closed-loop state matrix A\_{\\text{cl}} \= A \- B K\_{\\text{PP}} exhibits a designer-specified set of eigenvalues \\mu \= \\{\\mu\_1, \\mu\_2, \\mu\_3, \\mu\_4\\} in the open left-half complex plane. Using Ackermann’s formula or the Bass-Gura algorithm, dominant poles are selected to enforce a second-order damping ratio \\zeta \\ge 0.707 and a 2% settling time t\_s \\le 1.0\\text{ s}. The remaining poles are placed three to five times further into the left-half plane to ensure fast non-dominant decay without unnecessarily amplifying sensor noise or driving the motor into voltage saturation.  
The Linear Quadratic Regulator (LQR) provides an optimal state-feedback gain matrix u \= \-K\_{\\text{LQR}} x that minimizes an infinite-horizon quadratic performance index balancing state regulation against control effort:  
J \= \\int\_{0}^{\\infty} \\left( x(t)^T Q x(t) \+ u(t)^T R u(t) \\right) dt  
where Q \= \\text{diag}(\[q\_{\\theta\_0}, q\_{\\theta\_1}, q\_{\\dot{\\theta}\_0}, q\_{\\dot{\\theta}\_1}\]) \\ge 0 penalizes state deviations, and R \> 0 penalizes control effort. Solving the continuous-time Algebraic Riccati Equation (ARE) yields the unique positive-definite solution matrix P:  
A^T P \+ P A \- P B R^{-1} B^T P \+ Q \= 0 \\implies K\_{\\text{LQR}} \= R^{-1} B^T P  
Weight selection typically follows Bryson’s rule as a baseline, setting diagonal elements to the inverse squares of the maximum acceptable state excursions and actuator limits:  
Q\_{ii} \= \\frac{1}{(x\_{i,\\max})^2}, \\quad R \= \\frac{1}{(u\_{\\max})^2}  
Emphasizing pendulum angular deflection (q\_{\\theta\_1} \\gg q\_{\\theta\_0}) enforces aggressive disturbance rejection, while tuning R prevents the motor from saturating during aggressive recovery maneuvers. LQR provides guaranteed stability margins under nominal conditions—specifically, infinite gain margin and at least 60^\\circ of phase margin—making it a robust foundation for balancing control.

| Controller Strategy | Mathematical Principle | Primary Advantages | Critical Vulnerabilities | Parameter Tuning Burden |
| :---- | :---- | :---- | :---- | :---- |
| **Åström-Furuta Energy Shaping** | Lyapunov energy error dissipation (V\_E \= \\frac{1}{2}\\tilde{E}^2) | Wide basin of attraction; reliably swings link up from rest | Singular at zero velocity; requires bounding to prevent arm drift | Low: requires scaling gain k\_E and target energy E\_r |
| **Dual-Loop PID** | Decoupled tracking with low-pass derivative filtering | Model-free design; intuitive manual gain adjustments | Vulnerable to cross-axis coupling; sensitive to velocity noise | High: requires tuning five interacting scalar gains |
| **Full-State Pole Placement** | Exact closed-loop eigenvalue placement (\\de\[span\_119\](start\_span)\[span\_119\](end\_span)t(sI \- A \+ BK) \= 0\) | Explicit control over settling time, rise time, and modal damping | Highly sensitive to model inaccuracies; prone to saturation | Moderate: requires selecting four stable pole locations |
| **Linear Quadratic Regulator** | Optimal control via Riccati solution (A^T P \+ P A \- P B R^{-1} B^T P \+ Q \= 0\) | Guaranteed stability margins; balances tracking and actuator effort | Performance degrades near severe kinematic nonlinearities | Low-to-Moderate: requires populating diagonal Q and R matrices |

## **Hybrid Supervisory Architectures and Stateflow Implementation**

Because linear stabilization laws are only valid within a local neighborhood of the upright equilibrium, and energy-based swing-up laws cannot maintain balance at the apex, a hybrid supervisory architecture is required to govern the continuous-time dynamics. The supervisory controller is modeled as an event-driven finite state machine (FSM) implemented in MATLAB Stateflow, coordinating transitions between continuous-time and discrete-time control laws across three primary operating states.  
In the SWING\_UP state, the system executes the Åström-Furuta energy shaping law to pump mechanical energy into the pendulum link. As the link approaches the upright position, the supervisory logic monitors state trajectories until they satisfy a multidimensional entry guard, triggering a transition into the STAB\[span\_164\](start\_span)\[span\_164\](end\_span)ILIZE state. In this balancing state, authority transfers to the linear feedback controller (such as an LQR gain matrix) to balance the pendulum and regulate arm drift. If an external disturbance knocks the pendulum outside its recoverable envelope, an exit guard triggers an automatic fallback transition back to SWING\_UP. Finally, an overriding SAFE\_STOP state provides immediate fail-safe shutdown by disabling actuator output whenever hardware limits or thermal safety thresholds are violated.  
               `[ |theta_1| < 12 deg & |d(theta_1)| < 1.5 rad/s ]`  
      `SWING_UP -------------------------------------------------> STABILIZE`  
         `^             [span_103](start_span)[span_103](end_span)                                              |`  
         `|                   [ |theta_1| > 25 deg ]                  |`  
         `+-----------------------------------------------------------+`  
         `|                                                           |`  
         `|         [ Arm limit exceeded OR Overcurrent trip ]        |`  
         `+----------------------------+------------------------------+`  
                                      `|`  
                                      `v`  
                                  `SAFE_STOP`  
        `[span_53](start_span)[span_53](end_span)                     (Actuator Disabled)`

Direct switching between continuous controllers can induce severe high-frequency chattering along the boundary of the capture basin, causing violent motor vibrations and rapid loss of balance. To resolve this, the Stateflow architecture incorporates geometric hysteresis and temporal dwell-time filtering into its transition guards. The transition from SWING\_UP to STABILIZE requires the pendulum to fall within a tight capture envelope defined by angle, angular rate, and total energy error:  
\\text{Guard}\_{\\text{Swing}\\to\\text{Sta\[span\_104\](start\_span)\[span\_104\](end\_span)b}} \= \\left( \\vert{}\\theta\_1\\vert{} \\le \\theta\_{\\text{threshold}} \\right) \\land \\left( \\vert{}\\dot{\\theta}\_1\\vert{} \\le \\dot{\\theta}\_{\\text{\[span\_105\](start\_span)\[span\_105\](end\_span)threshold}} \\right) \\land \\left( \\vert{}E \- E\_r\\vert{} \\le E\_{\\text{margin}} \\right)  
Typical empirical values for physical hardware are \\theta\_{\\text{threshold}} \\approx 0.20\\text{ rad} (11.5^\\circ), \\dot{\\theta}\_{\\text{threshold}} \\approx 1.50\\text{ rad/s}, and E\_{\\text{margin}} \\approx 0.15 E\_r. Conversely, the exit transition from STABILIZE back to SWING\_UP is assigned a wider threshold, setting \\theta\_{\\text{escape}} \\approx 0.45\\text{ rad} (25.8^\\circ). This angular separation creates a 14^\\circ hysteresis band that prevents limit cycling across the switching boundary. In addition, the Stateflow chart enforces a temporal dwell-time filter requiring the state trajectory to remain continuously inside the capture basin for at least \\Delta t\_{\\text{dwell}} \\ge 20\\text{ ms} before committing to the balancing controller.  
Overriding transitions to SAFE\_STOP protect the hardware by monitoring four continuous safety conditions: horizontal arm overtravel exceeding soft limits (\\vert{}\\theta\_0\\vert{} \\ge 150^\\circ on rigs without slip rings), motor overspeed (\\vert{}\\dot{\\theta\[span\_122\](start\_span)\[span\_122\](end\_span)}\_0\\vert{} \\ge \\omega\_{\\max}), sustained actuator saturation (\\vert{}u(t)\\vert{} \= V\_{\\max} continuously for over 2.0\\text{\[span\_173\](start\_span)\[span\_173\](end\_span)\[span\_174\](start\_span)\[span\_174\](end\_span) s} while the motor is stalled), and manual activation of the emergency stop switch.

## **Digital Twin Modeling, Simscape Multibody, and Real-Time Dashboard Frameworks**

Digital twins bridge the gap between theoretical multi-body dynamics and physical hardware deployment, serving as high-fidelity models for simulation and real-time kinematic mirrors during Hardware-in-the-Loop (HIL) operation.  
`Physical Plant (Encoders & Motor Driver)`  
       `|`  
       `|  Bidirectional Serial / UDP Telemetry (100 Hz - 1 kHz)`  
       `v`  
`Real-Time Dashboard (App Designer / Instrumentation)`  
       `|`  
       `|  Mirrored Joint Angles & Actuator Feedback`  
       `v`  
`Simscape Multibody Dynamic Digital Twin (3D Physics & Visualizer)`

Sensor data streams from the physical rig through an asynchronous communications pipeline into a MATLAB App Designer dashboard, which updates strip charts, tracks Stateflow modes, and drives a 3D Simscape Multibody twin to mirror the physical hardware's motion.

### **Simscape Multibody Dynamic Verification**

Simscape Multibody provides a numerical physics engine that simulates the Furuta pendulum without the geometric approximations required by analytical formulations. Links are modeled as extruded solid bodies defined by two-dimensional planar profiles matched to CAD geometry, allowing the engine to calculate mass, center of gravity, and the full inertia tensor directly from material densities. Revolute joint blocks connect the bodies, incorporating internal joint stiffness, viscous damping, and Coulomb friction to match the physical pivot bearings. Actuation torques enter the horizontal joint through a physical signal port interfaced to a DC motor block or dynamic torque controller.  
Validating this digital twin requires comparing its open-loop trajectory against the analytical ODEs solved in MATLAB using numerical integrators like ode45. Releasing both models from identical non-zero initial conditions confirms that their transient response frequencies and energy decay rates match, verifying the accuracy of the multi-body parameters before closing the control loop.

### **Real-Time Instrumentation Dashboard**

A dashboard built in MATLAB App Designer provides unified telemetry logging, supervisory mode monitoring, and digital twin synchronization. Bidirectional communication with the embedded microcontroller runs over a high-speed serial connection (serialport interface) or a low-latency UDP socket at update rates between 100\\text{ Hz} and 1\\text{ kHz}.  
The dashboard interface displays continuous strip charts showing pendulum deflection \\theta\_1(t), arm angle \\theta\_0(t), and motor voltage u(t) alongside real-time indicators that reflect the active Stateflow mode (SWING\_UP, STABILIZE, or SAFE\_STOP). As data streams in, the dashboard computes online integral error performance metrics—specifically the Integral of Absolute Error (IAE), Integral of Squared Error (ISE), and Integral of Time-weighted Absolute Error (ITAE)—allowing quantitative performance tracking across disturbance tests. Simultaneously, the incoming joint coordinates drive an embedded 3D visual canvas or Simulink 3D Animation viewport, rendering a synchronized digital twin that mirrors the physical pendulum's motion in real time.

### **Open-Source MATLAB File Exchange Curations**

Several community and MathWorks reference packages provide established starting points for modeling, simulation, and control design:

> * *Furuta (Rotary) Pendulum by Alan Jennings* (File Exchange ID: 34521): Validates analytical nonlinear equations of motion against SimMechanics and Simscape Multibody models, demonstrating function-handle ODE solving and dynamic consistency verification.  
> * *Furuta Pendulum by John Caipa Roldan* (File Exchange ID: 23490): Provides symbolic linearization scripts and state-space parameter extractions for multi-body assemblies around the upright equilibrium point.  
> * *Full State Feedback of Furuta Pendulum by Vikash Gupta* (File Exchange ID: 25585): Contains foundational parameter initialization files and Simulink models implementing pole-placement and state-feedback balancing loops.  
> * *Reinforcement Learning Inverted Pendulum with QUBE-Servo 2 by MathWorks* (File Exchange ID: 99364): Provides a complete reference implementation using the MATLAB Reinforcement Learning Toolbox, including Gymnasium-compatible environment definitions, reward modeling, and deep actor-critic training routines.

## **Open-Source Hardware, Firmware Repositories, and Video Case Studies**

Reviewing open-source physical implementations highlights proven approaches for mechanical fabrication, embedded firmware architectures, and low-latency sensor processing pipelines.  
Embedded firmware architectures require careful coordination between hardware timer peripherals, numerical filtering routines, and discrete control logic. Dedicated quadrature hardware counters decode base and pivot encoder signals, passing raw positions to a first-order low-pass filter to generate clean velocity estimates. The resulting state vector is processed by the hybrid controller to generate motor command voltages, which are converted to high-frequency pulse-width modulation (PWM) and directional signals driving an H-bridge or brushless motor controller.  
`Optical / Magnetic Encoders (>= 2048 PPR)`  
       `|`  
       `v`  
`Embedded Hardware Timer (Quadrature QEI Mode)`  
       `|`  
       `v`  
`Velocity Filter: G(s) = (\omega_f * s) / (s + \omega_f)`  
       `|`  
       `v`  
`Hybrid Stateflow / Discrete Controller Engine`  
       `|`  
       `v`  
`PWM & Direction Logic -> Motor Driver H-Bridge`

### **Survey of Open-Source Projects and Hardware Implementations**

A comparative analysis of notable open-source repositories and hardware reference designs reveals several common implementation strategies:

| Project / Primary Author | Mechanical & Actuation Architecture | Sensor Suite & Interface Logic | Firmware & Control Pipeline | Primary Technical Focus & Repository Context |
| :---- | :---- | :---- | :---- | :---- |
| **Ben Katz** | Direct-drive brushless DC motor; CNC aluminum and 3D printed arm | Dual quadrature optical encoders; direct back-EMF phase tracking | Custom C/C++; direct voltage balance; energy shaping with \\cos^4(\\t\[span\_216\](start\_span)\[span\_216\](end\_span)\[span\_217\](start\_span)\[span\_217\](end\_span)heta\_1) scaling | Minimizing mechanical impedance; low-cogging direct-drive control |
| **Henrique Ferrolho** | £20 low-cost build; NEMA-17 stepper with TMC2209 silent step driver; 3D printed PLA | AS5600 12-bit magnetic rotary encoder (I^2C/analog) on pivot | Arduino Nano (ATmega328P); TinyML SAC policy (689 parameters) compiled to C++ | Low-cost builds; embedded TinyML execution on 8-bit microcontrollers |
| **Armand du Parc Locmaria & Pierre Fabre** | High-torque direct-drive motor; rigid modular 3D printed arms | Dual high-count quadrature encoders with dedicated hardware counter ICs | Python / PyTorch; Stable-Baselines3; Crocoddyl NMPC swing-up; Weights & Biases | Reproducible RL benchmarking; NMPC vs. RL comparisons (Armandpl/furuta) |
| **Kevin Wood Robotics** | Direct-drive brushless motor; CNC aluminum chassis and 3D printed arm | High-resolution magnetic/optical encoders; digital IMU sensor feedback | Python Gymnasium; MuJoCo physics engine; PPO policy deployed via real-time inference loop | Systematic Sim-to-Real calibration; motor parameter identification |
| **Robotics GEE** | High-traction geared DC servomotor with high-mass base plate | Precision optical encoders with quadrature decoding channels | STM32 ARM Cortex-M4 microcontroller; embedded LQR optimal control in C | Industrial embedded workflows; Simulink Embedded Coder deployment |
| **Ain Shams University Mechatronics Cohort** | Rigid machined aluminum chassis; DC servomotor; dual encoder mounts | Quadrature encoders (\\ge 360 PPR); hardware emergency stop circuit | Hybrid MATLAB Stateflow supervisory logic; HIL Simulink real-time tracking | Multi-controller benchmarking (PID vs. Pole Placement vs. LQR); HIL digital twins |

### **Embedded Computing Platforms and Velocity Estimation**

Deploying high-gain optimal controllers or high-frequency neural policies imposes strict real-time constraints on embedded microcontrollers. Platforms such as the STM32 (ARM Cortex-M4/M7) provide dedicated hardware timers with Quadrature Encoder Interface (QEI) peripherals, an integrated hardware Floating Point Unit (FPU), and native support for MATLAB Embedded Coder, making them well-suited for high-bandwidth feedback loops. Texas Instruments C2000 Delfino microcontrollers (such as the TMS320F28379D) include specialized enhanced Quadrature Encoder Pulse (eQEP) peripherals designed for high-frequency motion control. Conversely, highly constrained 8-bit boards like the Arduino Nano (ATmega328P) lack hardware floating-point support, requiring distilled neural policies and pre-computed fixed-point arithmetic to maintain loop timing. A primary operational bottleneck on physical hardware is encoder quantization noise. The unactuated pendulum joint requires high angular resolution (\\ge 2048 quadrature counts per revolution). Coarse encoder resolution introduces quantization steps that are magnified by the derivative gains in LQR and PID controllers, generating high-frequency voltage chatter that overheats motor windings. Furthermore, calculating angular velocities using pure backward finite differences (\\dot{\\theta} \\approx \\frac{\\theta\_k \- \\theta\_{k-1}}{\\Delta t}) amplifies high-frequency noise. Robust firmware implementations filter raw position readings through a first-order low-pass filter with cut-off frequency \\omega\_f \[span\_252\](start\_span)\[span\_252\](end\_span)\[span\_253\](start\_span)\[span\_253\](end\_span)\[span\_254\](start\_span)\[span\_254\](end\_span)\\approx 50\\text{ rad/s}, a tracking differentiator, or a steady-state Kalman filter:

G\_{\\text{filter}}(s) \= \\frac{\\omega\_f s}{s \+ \\omega\_f}

## **Reinforcement Learning: Algorithms, Gym Environments, and Sim-to-Real Transfer**

Reinforcement learning formulates the swing-up and balancing problem as a continuous Markov Decision Process (MDP) defined by the tuple (\\mathcal{S}, \\mathcal{A}, \\mathcal{P}, \\mathcal{R}, \\gamma). Unlike linear controllers that rely on localized operating models, deep reinforcement learning policies can learn end-to-end control laws that handle both swing-up and balance within a single parameterized neural network.  
`Physics Engine (MuJoCo / Gymnasium)`  
       `|`  
       `|  Continuous Observ[span_109](start_span)[span_109](end_span)[span_115](start_span)[span_115](end_span)ation: s_t = [cos, sin, angular_rates]`  
       `v`  
`Neural Actor-Critic Policy (SAC / PPO)`  
       `|`  
       `|  Smooth Action: a_t = Normalized Voltage / Torque`  
       `v`  
`Reward Feedback + Action Continuity Regularization`  
       `|`  
       `|  Domain Randomization & Latency Buffering`  
       `v`  
`TinyML Distillation -> Embedded Microcontroller Execution`

During simulation training, the physics engine updates system dynamics while domain randomization perturbs parameters and injects transport lag. The neural policy learns a mapping from trigonometric state observations to motor actions, optimizing a continuous reward that balances swing-up progress, balance stability, and actuator smoothness. Once converged, the policy is distilled into a compact neural network for embedded deployment.

### **Observation Manifolds and Reward Engineering**

Because planar rotations wrap around at \\pm \\pi, passing raw angular coordinates directly into a neural network creates artificial discontinuities that destabilize training. To preserve continuity across the state manifold S^1 \\times\[span\_262\](start\_span)\[span\_262\](end\_span)\[span\_264\](start\_span)\[span\_264\](end\_span) S^1, the observation space maps raw angles to continuous trigonometric projections:  
s\_t \= \\begin{bmatrix} \\cos\\theta\_0 & \\sin\\theta\_0 & \\cos\\theta\_1 & \\sin\\theta\_1 & \\dot{\\theta}\_0 & \\dot{\\theta}\_1 \\end{bmatrix}^T \\in \\mathbb{R}^6  
The continuous action space is defined as normalized motor armature voltage a\_t \\in\[span\_263\](start\_span)\[span\_263\](end\_span)\[span\_265\](start\_span)\[span\_265\](end\_span) \[-1.0, 1.0\], scaled internally to physical actuator bounds \[-V\_{\\max}, V\_{\\max}\].  
Reward function design directly shapes the stability of the learned policy and the smoothness of the resulting control action. Discontinuous step rewards produce sparse gradient feedback and encourage aggressive bang-bang actuator behavior. Similarly, pure quadratic penalties create steep gradients near the vertical apex that can lead to divergent policy updates during large swings. Continuous cosine reward functions, by contrast, provide smooth, well-conditioned gradients across the entire 360^\\circ swing profile. A robust composite reward function incorporates state error, control effort, and action continuity regularization:  
R\_t \= c\_1 \\cos\\theta\_1 \- c\_2 \\theta\_0^2 \- c\_3 \\dot{\\theta}\_1^2 \- c\_4 \\dot{\\theta}\_0^2 \- c\_5 u\_t^2 \- c\_6 (u\_t \- u\_{t-1})^2  
The action continuity penalty \-c\_6 (u\_t \- u\_{t-1})^2 penalizes high-frequency command oscillations, reducing current spikes and preventing mechanical wear on the physical drive train.

### **Algorithmic Comparison: PPO vs. SAC vs. DDPG**

The leading model-free reinforcement learning algorithms present distinct trade-offs when applied to underactuated pendulum benchmarks:

> * *Proximal Policy Optimization (PPO)*: An on-policy actor-critic algorithm that uses a clipped surrogate objective to prevent destabilizing policy updates. PPO exhibits stable, monotonic policy improvements and low hyperparameter sensitivity, making it resilient to unmodeled disturbances on physical hardware. However, because it is on-policy, it cannot reuse past experience, requiring large amounts of training data and relying heavily on accelerated parallel physics environments.  
> * *Soft Actor-Critic (SAC)*: An off-policy actor-critic algorithm framed around maximum entropy reinforcement learning. SAC maximizes both cumulative reward and policy entropy \\mathcal{H}(\\pi(\\cdot\\vert{}s)), which prevents the policy from prematurely collapsing into suboptimal local limit cycles during swing-up. Reusing past transitions via an off-policy replay buffer makes SAC an order of magnitude more sample-efficient than PPO, allowing it to fine-tune directly on physical hardware rigs.  
> * *Deep Deterministic Policy Gradient (DDPG)*: An off-policy actor-critic algorithm using deterministic action selection. While historically important, DDPG is vulnerable to severe Q-value overestimation bias and brittle hyperparameter tuning in underactuated domains, often lagging behind SAC and PPO in learning stability.  
> * *Hybrid Intelligent Architectures (Fuzzy-LQR / RL-LQR Fusion)*: These architectures combine nominal LQR gains with learned neural policies or fuzzy inference engines. The analytical LQR guarantees stability within the linear capture basin, while the learned component dynamically compensates for friction, parameter drift, and external impacts.

### **Sim-to-Real Transfer and Domain Randomization**

Policies trained in idealized simulation environments often fail when deployed directly onto physical hardware due to the reality gap. This performance gap is driven by unmodeled Coulomb friction, bearing stiction, motor saturation, voltage drops, and transport latency.  
To enable reliable zero-shot sim-to-real transfer, physics environments (such as MuJoCo or Gymnasium) must incorporate systematic domain randomization (DR) and latency modeling during training. At the start of each training episode, physical parameters are sampled uniformly across randomized distributions:  
m\_1 \\sim \\mathcal{U}(0.85 m\_{1,\\text{nom}}, 1.15 m\_{1,\\text{nom}}), \\quad l\_1 \\sim \\mathcal{U}(0.90 l\_{1,\\text{nom}}, 1.10 l\_{1,\\text{nom}}) b\_1 \\sim \\mathcal{U}(0.50 b\_{1,\\text{nom}}, 2.00 b\_{1,\\text{nom}}), \\quad k\_t \\sim \\mathcal{U}(0.90 k\_{t,\\text{nom}}, 1.10 k\_{t,\\text{nom}})  
In addition, Gaussian white noise is added to simulated states (\\sigma\_\\theta \\approx 0.005\\text{ rad}, \\sigma\_{\\dot{\\theta}} \\approx 0.05\\tex\[span\_300\](start\_span)\[span\_300\](end\_span)\[span\_302\](start\_span)\[span\_302\](end\_span)t{ rad/s}), along with uniform sensor offset biases. Physical serial interfaces and microcontroller execution introduce control loop latencies between 10\\text{ ms} and 30\\text{ ms}. Inserting a FIFO transport delay buffer into the simulation loop trains the policy to anticipate communication lag, preventing latency-induced instability on real hardware.

### **TinyML and Embedded Policy Distillation**

Deploying deep neural networks onto low-cost microcontrollers requires policy distillation and memory compression. While training often uses large networks (2 \\times 256 hidden units) to facilitate gradient descent, balancing control can be distilled into compact Multi-Layer Perceptrons (MLPs). Ferrolho demonstrated that a minimal network featuring two hidden layers (16 and 8 neurons with ReLU activations), totaling just 689 floating-point parameters, reliably balances a physical rotary pendulum when executed on an 8-bit Arduino Nano.  
Trained weights and biases are exported as static constant arrays (const float weights\[\]) and compiled into bare-metal C++ firmware. On 32-bit targets, execution can also leverage the CMSIS-NN acceleration library, ONNX runtime engines, or MATLAB Coder deployment pipelines generated directly from the Reinforcement Learning Toolbox.

## **Comparative Performance Benchmarks and Engineering Implementation Roadmap**

Selecting an appropriate control architecture requires balancing analytical modeling effort against computational footprint, disturbance rejection, and empirical tracking performance.

### **Performance Metrics Formulation**

Quantifying controller performance during nominal balancing and external disturbance rejection relies on standard transient and error indices:

> * Peak Overshoot: M\_p \= \\frac{\\theta\_{1,\\max} \- \\theta\_{1,ss}}{\\theta\_{1,ss}} \\times 100\\%  
> * 10-to-90% Rise Time: t\_r  
> * 2% Settling Time: t\_s  
> * Error Integrals: \\text{IAE} \= \\int\_{0}^{T} \\vert{}\\theta\_1(t)\\vert{} dt, \\quad \\text{ISE} \= \\int\_{0}^{T} \\theta\_1^2(t) dt, \\quad \\text{ITAE} \= \\int\_{0}^{T} t \\vert{}\\theta\_1(t)\\vert{} dt IAE measures cumulative error magnitude, ISE penalizes large transient peaks, and ITAE emphasizes persistent long-term oscillations.  
> * Actuator Effort: Peak voltage (u\_{\\max}) and Root-Mean-Square control effort: u\_{\\text{RMS}} \= \\sqrt{\\frac{1}{T} \\int\_{0}^{T} u^2(t) dt}

The following benchmark comparison synthesizes theoretical characteristics and experimental results across classical, optimal, and reinforcement learning paradigms on rotary inverted pendulum hardware:  
| Performance Index / Feature | Dual-Loop PID Control | Full-State Pole Placement | Linear Quadratic Regulator | Hybrid FSM (Swing-Up \+ LQR) | Deep Reinforcement Learning (SAC) | | :--- | :--- | :--- | :--- | :--- | :--- | | **Operational Domain** | Local (\\pm 10^\\circ) | Local (\\pm 12^\\circ) | Local (\\pm 15^\\circ) | Global (360^\\circ via FSM) | Global (360^\\circ unified) | | **Rise Time (t\_r)** | Moderate (0.35 \- 0.50\\text{ s}) | Fast (0.20 \- 0.35\\text{ s}) | Fast (0.18 \- 0.28\\text{ s}) | Fast (0.18 \- 0.28\\text{ s}) | Fast (0.15 \- 0.25\\text{ s}) | | **Settling Time (t\_s, 2%)** | Sluggish (1.20 \- 1.80\\text{ s}) | Moderate (0.80 \- 1.20\\text{ s}) | Short (0.60 \- 0.90\\text{ s}) | Short (0.60 \- 0.90\\text{ s}) | Short (0.50 \- 0.80\\text{ s}) | | **Peak Overshoot (M\_p)** | High (18 \- 30\\%) | Moderate (10 \- 20\\%) | Low (5 \- 12\\%) | Low (5 \- 12\\%) | Moderate (8 \- 15\\%) | | **Disturbance Recovery** | Slow recovery; position drift | Stiff; sensitive to noise | Rapid recovery; well-damped | Rapid recovery; well-damped | Adaptive recovery; broad basins | | **Control Effort (u\_{\\text{RMS}})** | High; persistent hunting | Moderate-to-High | Optimal; minimum energy | Optimal during balance | Low-to-Moderate (regularized) | | **Error Indices (IAE / ITAE)** | High | Moderate | Low | Low | Very Low | | **Memory Footprint** | Extremely Low (\< 1\\text{ KB}) | Low (\< 2\\text{ KB}) | Low (\< 2\\text{ KB}) | Moderate (\< 10\\\[span\_130\](start\_span)\[span\_130\](end\_span)\[span\_134\](start\_span)\[span\_134\](end\_span)text{\[span\_126\](start\_span)\[span\_126\](end\_span) KB}) | Low in inference (\< 5\\text{ KB}, TinyML) | | **Sim-to-Real Vulnerability** | Low (intuitive tuning) | High (model mismatch) | Moderate (stable margins) | Moderate (switching guards) | High (requires Domain Randomization) |

### **Engineering Implementation Roadmap**

Successfully taking a rotary inverted pendulum from mathematical models to a functional physical prototype requires a systematic development process.  
The development workflow begins with physical parameter identification and digital twin construction. In this initial stage, link masses and centers of gravity are measured, pivot inertias are determined through bifilar oscillation tests, and joint damping coefficients are extracted from unforced decay logs. A corresponding multi-body model is assembled in Simscape Multibody using accurate CAD geometries and material densities, and its open-loop behavior is validated against analytical ODE simulations.  
Once the model is verified, work moves to controller synthesis and numerical validation. The nonlinear equations of motion are linearized about the upright apex to extract state-space matrices, confirming full system controllability and observability. Dual-loop PID, pole placement, and LQR balancing controllers are synthesized, using Bryson's rule to configure weighting matrices. The Åström-Furuta energy-shaping swing-up law is implemented alongside them in Simulink, tuning the energy gain to achieve smooth ascent without over-rotation.  
Next, the controllers are integrated into a hybrid supervisory architecture for hardware-in-the-loop deployment. The supervisory state machine is configured in MATLAB Stateflow, incorporating capture basin guards, angular hysteresis deadbands, and fail-safe trip logic. An instrumentation dashboard is built in MATLAB App Designer to stream real-time telemetry over a serial interface, track supervisory states, compute online error metrics, and synchronize the 3D Simscape digital twin. Control routines are compiled onto the target microcontroller to validate hardware balancing and benchmark disturbance rejection performance across the classical and optimal controllers.  
Finally, the project expands into learned control by developing a deep reinforcement learning pipeline. A custom Gymnasium environment is wrapped around a MuJoCo simulation of the physical platform. Using Soft Actor-Critic (SAC) or Proximal Policy Optimization (PPO), an agent is trained on a continuous cosine reward with action continuity penalties. Domain randomization across link parameters and transport delay buffers are introduced to bridge the sim-to-real gap. The trained policy is then distilled into a compact neural network (fewer than 1,000 parameters) and exported as static C++ arrays, enabling bare-metal TinyML execution on the microcontroller to benchmark learned end-to-end control against the hybrid Stateflow/LQR baseline directly on physical hardware.

## **Conclusion**

The Furuta rotary inverted pendulum is a versatile engineering platform that bridges fundamental classical mechanics, optimal state-space control, hybrid supervisory logic, and modern data-driven reinforcement learning. While classical dual-loop PID provides an accessible, model-free baseline for balance, its susceptibility to cross-axis coupling and sensor noise limits its performance on coupled underactuated systems. Full-state feedback pole placement gives direct control over closed-loop dynamics, but requires an accurate system model and can easily saturate physical actuators. Linear Quadratic Regulation (LQR) resolves these limitations by systematically balancing tracking accuracy against actuator effort, offering robust gain and phase margins that make it an ideal choice for localized balance control.  
Because linear controllers operate within a limited capture basin, practical hardware implementations require a hybrid control architecture. Pairing an Åström-Furuta energy-shaping swing-up law with a supervisory Stateflow state machine provides reliable global recovery from the hanging position, using angular hysteresis and dwell-time filtering to eliminate boundary chattering and manage hardware safety trips. Integrating this control stack with a Simscape Multibody digital twin and a real-time App Designer telemetry dashboard establishes a complete Hardware-in-the-Loop workflow that tracks system states, visualizes dynamics, and quantifies performance metrics like IAE and ITAE.  
Extending the system to deep reinforcement learning demonstrates the viability of learned, end-to-end control on underactuated platforms. Soft Actor-Critic (SAC) and Proximal Policy Optimization (PPO) can synthesize swing-up and balance behaviors into a single neural network when trained on continuous trigonometric observations and smooth cosine reward functions. Bridging the reality gap, however, requires systematic domain randomization and transport delay modeling during simulation. Once trained, policy distillation enables compact neural networks (under 1,000 parameters) to run on low-cost microcontrollers via TinyML, providing a direct comparison between optimal model-based control and learned data-driven policies on physical hardware.

#### **Works cited**

1\. On the Dynamics of the Furuta Pendulum \- ResearchGate, https\://www\.researchgate.net/publication/228421122\_On\_the\_Dynamics\_of\_the\_Furuta\_Pendulum 2\. Modelling the Furuta Pendulum Gäfvert, Magnus \- Lucris, https\://lucris.lub.lu.se/ws/files/4453844/8727127.pdf 3\. Dynamic Modeling and Simulation of an Underactuated System \- arXiv, https\://arxiv.org/pdf/1706.04996 4\. a swinging up controller for the furuta pendulum based on the total, https\://www\.kybernetika.cz/content/2019/2/402/paper.pdf 5\. Soft Constrained Warm Start based MPC-PD Approach for Real, https\://latamt.ieeer9.org/index.php/transactions/article/download/9660/2834 6\. Underactuated Trajectory-Tracking Control for Long-Exposure, https\://www\.ias.informatik.tu-darmstadt.de/uploads/Team/FabioMuratore/IP\_UnderactuatedTrajTrackingControlForLongExposurePhotography\_I\_report\_final.pdf 7\. QUBE-Servo Workbook (Student) | PDF | Electric Motor \- Scribd, https\://www\.scribd.com/document/580305998/QUBE-Servo-Workbook-Student 8\. Agentic AI for Real-Time Adaptive PID Control of a Servo Motor \- MDPI, https\://www\.mdpi.com/2076-0825/14/9/459 9\. QUBE-SERVO 2 Swing-Up Control Guide | PDF | Pendulum \- Scribd, https\://www\.scribd.com/document/860697901/QUBE-Servo-2-Swing-Up-Control-Workbook-Instructor 10\. A Multibody-Based Benchmarking Framework for the Control of the, https\://www\.mdpi.com/2076-0825/14/8/377 11\. LQR controller for Rotary inverted pendulum tuning problems, https\://www\.mathworks.com/matlabcentral/answers/2180502-lqr-controller-for-rotary-inverted-pendulum-tuning-problems 12\. Swinging up a pendulum by energy control, https\://web.ece.ucsb.edu/\~hespanha/ece229/references/AstromFurutaAUTOM00.pdf 13\. A swinging up controller for the Furuta pendulum based ... \- DML-CZ, https\://dml.cz/handle/10338.dmlcz/147844 14\. Revitalizing the Furuta pendulum, https\://research.tue.nl/files/4307900/626064.pdf 15\. Swinging up a pendulum by energy control \- Semantic Scholar, https\://www\.semanticscholar.org/paper/Swinging-up-a-pendulum-by-energy-control-%C3%85str%C3%B6m-Furuta/8a259e547088912d0323b0d4fa7d0dafde50e4ee 16\. Furuta Pendulum \- Dagor-Brushless-Controller, https\://www\.dagor.dev/blog/furuta-pendulum 17\. Full State Feedback of Furuta Pendulum \- File Exchange \- MathWorks, https\://www\.mathworks.com/matlabcentral/fileexchange/25585-full-state-feedback-of-furuta-pendulum?s\_tid=prof\_contriblnk 18\. Mechatronic Systems \- MDPI, https\://mdpi-res.com/bookfiles/book/12853/Mechatronic\_Systems.pdf?v=1790731160 19\. Swing-up and Stabilisation of a Furuta Pendulum using Energy, https\://www\.academia.edu/31024303/Swing\_up\_and\_Stabilisation\_of\_a\_Furuta\_Pendulum\_using\_Energy\_Control 20\. B\&R Automation Studio Target for Simulink®, https\://kyb.fei.tuke.sk/laboratoria/prezentacie/br%20automation%20studio%20target%20for%20simulink.pdf 21\. Controlling the Furuta pendulum: Proof of concept through virtual, https\://jart.icat.unam.mx/index.php/jart/article/view/2347 22\. Furuta (rotary) Pendulum \- File Exchange \- MATLAB Central, https\://www\.mathworks.com/matlabcentral/fileexchange/34521-furuta-rotary-pendulum 23\. Rotary Inverted Pendulum: Rotating shaft is not rotating the arm at, https\://de.mathworks.com/matlabcentral/answers/2184949-rotary-inverted-pendulum-rotating-shaft-is-not-rotating-the-arm-attached-to-it/?s\_tid=ans\_lp\_feed\_leaf 24\. Inverted Pendulum with Animation \- MATLAB & Simulink \- MathWorks, https\://www\.mathworks.com/help/simulink/slref/inverted-pendulum-with-animation.html 25\. Add App Designer App to Inverted Pendulum Model \- MathWorks, https\://www\.mathworks.com/help/slrealtime/ug/add-app-des-app-to-inverted-pendulum-model.html 26\. Implementing real time serial logging to app designer \- MathWorks, https\://www\.mathworks.com/matlabcentral/answers/847220-implementing-real-time-serial-logging-to-app-designer 27\. , https\://drive.google.com/open?id=1FafBFPmdkxIdFKBmrCwpmW74ychCuQ6cblw2YOzWr3g 28\. Furuta pendulum \- File Exchange \- MATLAB Central \- MathWorks, https\://www\.mathworks.com/matlabcentral/fileexchange/23490-furuta-pendulum 29\. Reinforcement-Learning-Inverted-Pendulum-with ... \- MathWorks, https\://www\.mathworks.com/matlabcentral/fileexchange/99364-reinforcement-learning-inverted-pendulum-with-qube-servo2/files/PID/design\_PID\_control\_ja.mlx 30\. Armandpl/furuta: Building and Training a Rotary Inverted Pendulum, https\://github.com/Armandpl/furuta 31\. ferrolho/rotary-inverted-pendulum \- GitHub, https\://github.com/ferrolho/rotary-inverted-pendulum 32\. Inertia Wheel Pendulum: How It Works, Diagram & Examples, https\://www\.firgelliauto.com/blogs/mechanisms/inertia-wheel-pendulum 33\. Made For Science Quanser Qube Servo 2 CoursewareStud ... \- Scribd, https\://www\.scribd.com/document/486299841/made-for-science-Quanser-qube-servo-2-CoursewareStud-MATLAB-pdf 34\. Modeling, Simulation, and Control of a Rotary Inverted Pendulum, https\://www\.researchgate.net/publication/386174932\_Modeling\_Simulation\_and\_Control\_of\_a\_Rotary\_Inverted\_Pendulum\_A\_Reinforcement\_Learning-Based\_Control\_Approach 35\. Correlated Exploration in Deep Reinforcement Learning, https\://www\.ias.informatik.tu-darmstadt.de/uploads/Site/EditPublication/maximilian\_hensel.pdf 36\. Compare reinforcement learning control model with conventional, https\://www\.researchgate.net/figure/Compare-reinforcement-learning-control-model-with-conventional-PID-control-model-and\_fig2\_373540444 37\. Sim-to-Real Reinforcement Learning for a Rotary Double-Inverted, https\://www\.mdpi.com/2227-7390/13/12/1996 38\. Reinforcement Learning on Hardware from Sim-to-Real (Rotary, https\://kevinwoodrobotics.com/product/reinforcement-learning-on-hardware-from-sim-to-real-rotary-inverted-pendulum/ 39\. Research Projects \- Yunlong Song, https\://yunlong-song.com/project.html 40\. Efficient Exploration in Continuous-time Model-based ... \- OpenReview, https\://openreview.net/pdf?id=VkhvDfY2dB 41\. Stable Baselines3 Documentation, https\://stable-baselines3.readthedocs.io/\_/downloads/en/master/pdf/ 42\. Neural Posterior Domain Randomization, https\://proceedings.mlr.press/v164/muratore22a/muratore22a.pdf 43\. Sensitivity-Based Robustness Evaluation of a Reinforcement ... \- MDPI, https\://www\.mdpi.com/2673-4591/157/1/2 44\. Data-efficient Domain Randomization with Bayesian Optimization, https\://www\.alphaxiv.org/abs/2003.02471 45\. ferrolho/rotary-inverted-pendulum：一个20英镑的开源旋转倒立摆, https\://roboairadar.com/news/cms0i8h0b000e04jxaibz2nb5 46\. Agentic AI For Real-Time Adaptive PID Control of Servo Motor, https\://www\.preprints.org/manuscript/202508.0355