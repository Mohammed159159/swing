While there are no official, pre-packaged model weights or datasets specifically labeled for the physical two-degree-of-freedom **Furuta (Rotary) Inverted Pendulum** on Hugging Face—and searching "Furuta" primarily returns unrelated NLP and multimodal papers by AI researcher Hiroki Furuta—Hugging Face offers several tools, models, and libraries for reinforcement learning and robotics:

  

### 1. Pre-Trained Continuous Pendulum Models (Hugging Face Hub)

Hugging Face hosts numerous trained policy checkpoints for classic continuous underactuated pendulum benchmarks that share the same underlying continuous dynamics and trigonometric observation representations as the Furuta pendulum:

  

- **`sb3/ppo-Pendulum-v1` & `HumanCompatibleAI/ppo-Pendulum-v1`:** Pre-trained Stable-Baselines3 PPO actor-critic models configured for continuous torque balance and swing-up tasks.
    
      
    
- **`OpenDILabCommunity/Pendulum-v1-MuZero` & `Pendulum-v1-EfficientZero`:** Model-based reinforcement learning policies developed with the LightZero library, demonstrating sample-efficient continuous control.
    
      
    
- **`physicalai-bmi/efa-2-pendulum`:** Checkpoints demonstrating continuous observation-to-action mapping for physical AI benchmarks.
    
      
    

Studying the model cards and configuration files of these repositories provides exact reference hyperparameters (learning rate, entropy coefficient, clip ranges, network architecture) for underactuated inverted pendulum stabilization.

  

### 2. Hugging Face SB3 Integration (`huggingface_sb3`)

The official `huggingface_sb3` package connects Stable-Baselines3 pipelines (such as those used in Armand du Parc Locmaria's and Henrique Ferrolho's rotary pendulum repositories) directly with Hugging Face:

  

- **Automated Model Versioning:** Allows you to save, package, and push trained PPO or SAC `.zip` policies to the Hub with a single command (`package_to_hub()`).
    
      
    
- **Automated Evaluation Cards:** Automatically generates evaluation videos and logs metrics (mean episode reward, standard deviation) on the Hub repository page.
    
      
    
- **Team Collaboration:** Enables student teams to share, benchmark, and download policy checkpoints across different workstations or embedded deployment setups without manual file transfers.
    
      
    

### 3. Hugging Face LeRobot (`lerobot`)

Launched by Hugging Face, **LeRobot** is an open-source library specifically focused on end-to-end robot learning, imitation learning, and physical hardware deployment:

  

- Provides standardized PyTorch pipelines to train policies and stream actions directly to low-cost hardware actuators and microcontrollers.
    
      
    
- While widely known for robotic manipulators, its architecture supports custom environments, making it a viable modern alternative to legacy simulation stacks for deploying neural network policies onto physical motor-driven platforms.
    
      
    

### 4. Interactive Hugging Face Spaces for Project Demos

Hugging Face Spaces allows you to deploy interactive web applications using Gradio or Streamlit:

  

- Community spaces such as `pjt222/triple-pendulum` demonstrate interactive 3D simulations rendered directly in the browser.
    
      
    
- You can host a Gradio-based web dashboard that visualizes your MuJoCo digital twin or provides an interactive sandbox for testing RL policies and comparing them to classical LQR gains.
    
      
    

Would you like me to do a more in-depth research instead and propose a research plan?


---

Beyond MATLAB and Simscape, extending an underactuated robotics project—especially one incorporating reinforcement learning and optimal control—benefits from several specialized software ecosystems. These span multibody dynamic engines, trajectory optimization suites, modern telemetry tools, and embedded robotics frameworks.

---

### 1. Physics Engines and Multibody Simulators

Simulating the non-Euclidean configuration space ($S^1 \times S^1$) and fast dynamics of the Furuta pendulum requires numerical integrators capable of handling stiff equations and rapid velocities.

* **MuJoCo (Multi-Joint dynamics with Contact):**
* *Why it is useful:* MuJoCo has become the robotics standard for continuous control and sim-to-real workflows. Its generalized-coordinate formulation accurately handles kinematic loops and joint constraints without numerical drift.


* *Application:* You can define the Furuta pendulum via an XML (MJCF) model, calibrate motor torque constants, joint viscous damping, and rotor inertia, and achieve simulation speeds of tens of thousands of steps per second.




* **Drake (MIT Robot Locomotion Group):**
* *Why it is useful:* Drake is specifically built for nonlinear dynamics and underactuated robotics. Unlike general-purpose game engines, Drake emphasizes analytical mechanics, symbolic dynamics, and formal verification.


* *Application:* It features built-in tools for computing Regions of Attraction (ROA) via Sums-of-Squares (SOS) dynamic programming, evaluating Lyapunov stability certificates, and synthesizing time-varying LQR controllers along nonlinear trajectories.




* **Isaac Gym / Isaac Lab (NVIDIA Omniverse):**
* *Why it is useful:* Runs physics simulations entirely on GPU Tensor cores, bypassing CPU-GPU transfer bottlenecks.
* *Application:* Enables simulating thousands of pendulum instances simultaneously, reducing reinforcement learning training times for algorithms like PPO from hours to minutes while executing domain randomization sweeps in parallel.



---

### 2. Trajectory Optimization and Optimal Control Suites

While LQR stabilizes the pendulum within the linearized capture basin, generating minimum-effort swing-up trajectories can be treated as a numerical trajectory optimization problem rather than relying solely on Åström-Furuta energy shaping.

* **Crocoddyl & Pinocchio:**
* *Why it is useful:* Pinocchio is an ultra-fast rigid-body dynamics library based on spatial vector algebra; Crocoddyl builds on it to solve optimal control problems via Differential Dynamic Programming (DDP) and box-constrained iterative LQR (iLQR).


* *Application:* Crocoddyl computes optimal swing-up trajectories that respect actuator voltage and torque limits, and it outputs feedback gain matrices ($K_k$) that can be tracked via Linear Time-Varying (LTV) controllers or compiled directly into embedded C++.




* **CasADi:**
* *Why it is useful:* A symbolic framework for algorithmic differentiation and numerical optimization with native interfaces to high-performance nonlinear solvers like IPOPT.
* *Application:* Useful for setting up direct collocation or direct multiple shooting to compute swing-up paths and implement real-time nonlinear Model Predictive Control (NMPC).




* **`python-control`:**
* *Why it is useful:* A Python library replicating MATLAB Control System Toolbox commands (e.g., `ct.ss()`, `ct.lqr()`, `ct.place()`, Bode plots, Nyquist stability analysis).
* *Application:* Ideal if you want to perform rapid mathematical modeling, Riccati algebraic solving, and frequency-domain stability checks purely within Python scripts or Jupyter Notebooks.



---

### 3. Reinforcement Learning and MLOps Frameworks

A modular RL pipeline requires separation between the physics environment, the agent algorithm, and the training telemetry.

* **Stable-Baselines3 (SB3) & RL Baselines3 Zoo:**
* *Why it is useful:* The reference PyTorch implementation for model-free RL algorithms (PPO, SAC, TD3). The Zoo provides pre-configured hyperparameters, cross-validation scripts, and automatic logging.


* *Application:* Supports custom action-continuity penalties and state histories to prevent high-frequency chattering on physical actuators.




* **Weights & Biases (W&B):**
* *Why it is useful:* Real-time tracking of neural network training metrics, loss surfaces, reward convergence, and hyperparameter sweeps.
* *Application:* As used in open-source Furuta implementations (such as `Armandpl/furuta`), W&B allows tracking reward evolution across varying domain randomization bounds (e.g., varying mass, damping, or delay) to monitor sim-to-real transfer viability.




* **Optuna:**
* *Why it is useful:* A Bayesian hyperparameter optimization framework.
* *Application:* Tunes continuous reward weights ($c_1, c_2, \dots, c_6$), learning rates, buffer sizes, and entropy coefficients to optimize convergence and sample efficiency.



---

### 4. Real-Time Telemetry and Hardware-in-the-Loop Visualizers

While MATLAB App Designer provides a structured GUI dashboard, standalone high-performance telemetry visualizers can monitor high-frequency serial/UDP data streams without inducing computational latency.

* **PlotJuggler:**
* *Why it is useful:* A fast, lightweight desktop time-series visualizer specifically designed for robotics and control systems.


* *Application:* Connects directly to microcontroller serial ports (USB/UART), UDP streams, or ROS topics. It plots $\theta_0(t)$, $\theta_1(t)$, motor current, and PWM effort at rates up to $1\text{ kHz}$ with real-time zooming, FFT frequency spectrum analysis (crucial for diagnosing sensor noise), and mathematical curve transformation.




* **Foxglove Studio:**
* *Why it is useful:* A modern, modular visualization platform that supports WebSocket, serial, and MCAP log streaming.


* *Application:* Allows building custom dashboards that combine multi-signal strip charts with an interactive 3D URDF viewport displaying the physical pendulum's pose in real time.





---

### 5. Embedded Deployment and Low-Level Firmware Tooling

Bridging the gap between algorithms designed on a PC and bare-metal execution on microcontrollers requires high-performance mathematical and communication libraries.

* **Eigen (C++ Template Library):**
* *Why it is useful:* An optimized, header-only C++ matrix library.
* *Application:* Allows implementing discrete Kalman filter state estimation ($\hat{x}_{k+1} = A_d \hat{x}_k + B_d u_k + L (y_k - C \hat{x}_k)$) and state feedback ($u = -K x$) directly in embedded firmware with minimal memory overhead.


* **micro-ROS:**
* *Why it is useful:* Brings the Robot Operating System (ROS 2) directly onto 32-bit microcontrollers (STM32, ESP32, Teensy).
* *Application:* Exposes the Furuta pendulum states as native ROS 2 topics and accepts velocity/voltage setpoints over serial DDS, creating an interface to Python or C++ nodes running on an attached host PC.


* **TinyML Inference Engines (CMSIS-NN / TFLite Micro / Bare-Metal C++):**
* *Why it is useful:* Translates neural policies trained in Python into static C/C++ functions.


* *Application:* As demonstrated in embedded implementations, small policies (e.g., 2 hidden layers with 16 and 8 units) can be exported as flat C-arrays and executed in microsecond timeframes without needing a dynamic runtime environment.





---

### Summary of Alternative Tooling

| Functional Category                           | Primary Software / Framework      | Key Strength for Furuta Pendulum                                                                 |
| --------------------------------------------- | --------------------------------- | ------------------------------------------------------------------------------------------------ |
| **Physics Simulation**                        | MuJoCo, Drake, Isaac Gym          | Fast contact/generalized coordinate dynamics, SOS/Lyapunov stability analysis, GPU vectorization |
| **Optimal Control & Trajectory Optimization** | Crocoddyl, Pinocchio, CasADi      | iLQR/DDP optimal swing-up generation with state/torque bounds                                    |
| **Reinforcement Learning**                    | Stable-Baselines3, Gymnasium, W&B | Reproducible actor-critic pipelines (SAC/PPO), MLOps telemetry                                   |
| **Real-Time Telemetry & Monitoring**          | PlotJuggler, Foxglove Studio      | Low-latency serial/UDP streaming, live FFT noise analysis, 3D URDF mirroring                     |
| **Embedded & Real-Time Math**                 | Eigen, micro-ROS, CMSIS-NN        | Matrix operations for Kalman/LQR, ROS 2 messaging, TinyML policy execution                       |
