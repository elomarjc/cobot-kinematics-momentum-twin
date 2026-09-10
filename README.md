# Collaborative Robot (UR5e) Kinematics & Generalized Momentum Sensorless Collision Observer Digital Twin

[![Build Status](https://img.shields.io/badge/tests-passing-brightgreen.svg)](test/test_cobot_dynamics.mjs)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Target: Universal Robots](https://img.shields.io/badge/Target-Universal%20Robots%20A%2FS-0ea5e9.svg)](https://www.universal-robots.com/)
[![Live Demo](https://img.shields.io/badge/demo-Live%20GitHub%20Pages-success.svg)](https://elomarjc.github.io/cobot-kinematics-momentum-twin/)

An interactive, high-fidelity physics and kinematics digital twin of the 6-DOF **Universal Robots UR5e** articulated manipulator. Built in native ES6 and WebGL / Three.js, this platform demonstrates real-time **Damped Least-Squares (DLS) singularity-robust inverse kinematics**, Yoshikawa manipulability tracking, and a **sensorless Generalized Momentum Disturbance Observer** capable of detecting physical human collisions under ISO/TS 15066 in under 15 milliseconds without external torque sensors or sensor skins.

Developed as a showcase for robotics motion software engineering and control systems design at **Universal Robots A/S** (Odense, Denmark).

---

## Interactive Live Simulator

Experience the live interactive simulation in your browser:  
👉 **[https://elomarjc.github.io/cobot-kinematics-momentum-twin/](https://elomarjc.github.io/cobot-kinematics-momentum-twin/)**

- **Interactive 3D WebGL Manipulator:** Drag 3D orbit camera controls, manipulate target Cartesian coordinates $[X, Y, Z]$, and visualize joint link frames and end-effector trajectories.
- **Singularity Damping Visualization:** Move the robot arm into full extension or wrist folds; observe how the DLS solver dynamically damps the Jacobian inverse to avoid joint velocity explosions.
- **Sensorless Collision Injection:** Trigger a simulated $22\text{ Nm}$ human impact; watch the Generalized Momentum Observer isolate the external contact wrench, trip an ISO/TS 15066 Category 0 Emergency Stop, and halt joint motion.

---

## Theoretical & Mathematical Foundations

### 1. Denavit-Hartenberg Forward Kinematics

The UR5e kinematic chain is modeled using standard Denavit-Hartenberg (D-H) transformation matrices $A_i \in SE(3)$ parameterized by link length $a_i$, link twist $\alpha_i$, joint distance $d_i$, and joint angle $q_i$:

$$
A_i(q_i) = \begin{bmatrix} \cos q_i & -\sin q_i \cos \alpha_i & \sin q_i \sin \alpha_i & a_i \cos q_i \\ \sin q_i & \cos q_i \cos \alpha_i & -\cos q_i \sin \alpha_i & a_i \sin q_i \\ 0 & \sin \alpha_i & \cos \alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

The overall end-effector pose $T_0^6(q)$ is obtained by sequential concatenation:

$$
T_0^6(q) = \prod_{i=1}^6 A_i(q_i)
$$

### 2. Analytical Geometric Jacobian & Manipulability Index

The translational velocity $v_e \in \mathbb{R}^3$ of the end-effector is mapped to joint velocities $\dot{q} \in \mathbb{R}^6$ via the position Jacobian $J_v(q) \in \mathbb{R}^{3 \times 6}$:

$$
J_{v, i}(q) = z_i \times (p_e - p_i)
$$

where $z_i$ is the unit direction of joint axis $i$, and $(p_e - p_i)$ is the lever arm vector from joint frame $i$ to the end-effector.

Kinematic proximity to singularities (where $J_v$ loses rank) is tracked continuously using the **Yoshikawa Manipulability Measure**:

$$
w(q) = \sqrt{\det\left( J_v(q) J_v(q)^T \right)}
$$

As $w(q) \to 0$, standard pseudo-inverses ($J^+ = J^T (J J^T)^{-1}$) diverge, commanding infinite joint velocities.

### 3. Damped Least-Squares (DLS) Regularization

To guarantee finite, smooth, and bounded joint speeds near boundary and wrist singularities, the solver minimizes the objective function $||J_v \dot{q} - v_e||^2 + \lambda^2 ||\dot{q}||^2$:

$$
\dot{q} = J_v^T \left( J_v J_v^T + \lambda^2 I \right)^{-1} v_e
$$

The damping factor $\lambda^2$ is dynamically tuned based on the manipulability index $w(q)$:

$$
\lambda^2(w) = \begin{cases} \lambda_{\max}^2 \left( 1 - \left(\frac{w}{w_{\text{th}}}\right)^2 \right) & \text{if } w < w_{\text{th}} \\ 0 & \text{if } w \ge w_{\text{th}} \end{cases}
$$

This ensures uncompromised Cartesian trajectory accuracy in dexterous regions while guaranteeing bounded joint rates through singularity transitions.

### 4. Sensorless Generalized Momentum Disturbance Observer

To achieve ISO/TS 15066 human-robot collaborative safety without adding bulky external 6-axis force-torque load cells or tactile skins, the controller tracks the system generalized momentum:

$$
p(t) = M(q) \dot{q}
$$

where $M(q)$ is the robot inertia matrix. Differentiating with respect to time yields $\dot{p} = \tau_{\text{motor}} - g(q) + C^T(q, \dot{q})\dot{q} + \tau_{\text{ext}}$. By defining the residual vector $r(t) = \hat{\tau}_{\text{ext}}(t)$, the observer dynamics are formulated as:

$$
\hat{\tau}_{\text{ext}}(t) = K_o \left[ p(t) - \int_0^t \left( \tau_{\text{motor}} - g(q) + \hat{\tau}_{\text{ext}}(\sigma) \right) d\sigma - p(0) \right]
$$

where $K_o > 0$ is a diagonal observer gain matrix. This yields a decoupled, stable first-order low-pass filter:

$$
\dot{\hat{\tau}}_{\text{ext}} = K_o \left( \tau_{\text{ext}} - \hat{\tau}_{\text{ext}} \right)
$$

### 5. ISO/TS 15066 Category 0 Emergency Stop

The instantaneous disturbance torque magnitude $||\hat{\tau}_{\text{ext}}|| = \sqrt{\sum \hat{\tau}_{\text{ext}, i}^2}$ is compared at 100 Hz against the ISO/TS 15066 biomechanical threshold ($\tau_{\text{thresh}} = 12.0\text{ Nm}$):

$$
||\hat{\tau}_{\text{ext}}|| > \tau_{\text{thresh}} \implies \text{Emergency Stop Engaged (Cat 0)}
$$

Upon impact detection, motor velocity demands are clamped to zero and mechanical deceleration brakes engage in $< 15\text{ ms}$.

---

## Project Structure

```
cobot-kinematics-momentum-twin/
├── css/
│   └── style.css                     # Responsive styling & HUD dashboard
├── docs/
│   └── outreach/
│       └── email_and_linkedin.md     # Executive outreach package for Universal Robots
├── js/
│   ├── app.js                        # Main loop & UI orchestrator
│   ├── engine/
│   │   ├── dh-kinematics.js          # UR5e forward kinematics & analytical Jacobian
│   │   ├── dls-solver.js             # Damped Least-Squares inverse kinematics solver
│   │   ├── momentum-observer.js      # Generalized Momentum Disturbance Observer
│   │   └── robot-dynamics.js         # Inertia matrix M(q) & gravity torques g(q)
│   └── ui/
│       ├── cobot-3d-view.js          # Three.js 6-DOF WebGL articulated arm
│       └── observer-chart.js         # 60 FPS HTML5 Canvas strip-chart telemetry
├── test/
│   └── test_cobot_dynamics.mjs       # Automated Node.js unit test suite
├── index.html                        # Application entry point
├── package.json                      # ES module package configuration
└── README.md                         # Project documentation
```

---

## Verification & Automated Testing

Run the automated test suite verifying kinematic reach, singularity damping, and observer disturbance estimation:

```bash
npm test
```

All 4 test suites pass deterministically:
1. **UR5e Forward Kinematics:** Validates nominal horizontal workspace reach ($850\text{ mm}$) and 7-frame homogeneous transformations.
2. **Yoshikawa Manipulability Index:** Confirms manipulability degradation ($w(q) < 0.02$) at boundary configurations vs dexterous poses ($w(q) > 0.05$).
3. **DLS Velocity Bounding:** Verifies joint velocity limits ($\le 2.5\text{ rad/s}$) under extreme singularity demands.
4. **Generalized Momentum Collision Detection:** Verifies that nominal unperturbed motion generates negligible residuals ($< 0.1\text{ Nm}$), while a $25\text{ Nm}$ contact impulse triggers an ISO/TS 15066 emergency stop within $< 150\text{ ms}$.

---

## Executive Outreach

For hiring managers, engineering directors, and controls leads at **Universal Robots A/S**, please refer to:
- [`docs/outreach/email_and_linkedin.md`](docs/outreach/email_and_linkedin.md) for cold email pitches, technical briefing notes, and LinkedIn discussion copy.
