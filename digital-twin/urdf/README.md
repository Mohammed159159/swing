# Unified Robot Description Format (`digital-twin/urdf/`)

This directory contains the standard **URDF (Unified Robot Description Format)** kinematic and dynamic specification of the Furuta Rotary Inverted Pendulum robot.

---

## 🎯 Implementation Overview

1. **URDF Architecture (`furuta.urdf`):**
   - **Links:**
     - `base_link`: Inertial ground mount.
     - `arm_link`: Horizontal rotating bar with joint frame aligned to motor shaft.
     - `pendulum_link`: Unactuated cylindrical rod with pivot at arm tip.
   - **Joints:**
     - `arm_joint`: Continuous revolute joint about $+Z$ axis.
     - `pendulum_joint`: Continuous revolute joint about $+X$ or $+Y$ axis.
   - **Inertial Tags:**
     - Explicit mass ($m_0, m_1$) and inertia matrices ($I_{xx}, I_{yy}, I_{zz}$) derived from CAD models.
   - **Visual & Collision:**
     - Linked STL meshes stored in local directory or relative asset paths.

2. **Applications:**
   - Universal robot representation compatible with ROS / ROS 2, PyBullet, Gazebo, RViz, and MATLAB Robotics System Toolbox (`importrobot`).
