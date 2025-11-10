# QP-based Differential IK Controller (Intuitive Flow)

## 1. Motivation: Beyond the Pseudo-Inverse

The classic pseudo-inverse method for differential IK, $\dot{q} = J^+(q) V_{desired}$, is elegant but impractical for real hardware. It blatantly ignores physical limits on joints (position, velocity, acceleration) and becomes unstable near singularities, commanding dangerously high joint speeds.

By formulating the problem as a **Quadratic Program (QP)**, we can directly incorporate these constraints into the controller. This allows the robot to behave safely and predictably, gracefully reducing speed or deviating from the path only when necessary to respect its physical limits.

---

## 2. Controller Logic: Setup and Loop

To understand how the controller works, it's helpful to think like a programmer using a `setup()` function for one-time initializations and a `loop()` function for the repeating control cycle.

### `void setup()`: One-Time Path Generation

When the robot receives a new target pose, it performs a one-time setup routine to generate the complete path and motion profile. This doesn't happen in the real-time loop.

1. **Define the Geometric Path:** The controller first defines the simplest possible path in 3D space between the current end-effector pose, $T_0$, and the target pose, $T_{target}$. This path is a straight line for position and a direct interpolation for orientation.

   * Position: $p(s) = p_0 + s \cdot (p_{target} - p_0)$
   * Orientation: $R(s) = R_0 \cdot \text{exp}(s \cdot \log(R_0^T R_{target}))$
   
   Here, the path is parameterized by a single variable $s$ that goes from 0 to 1.

2. **Create a Motion Profile:** Next, the controller determines the *timing* for traversing this path. It computes a **trapezoidal velocity profile** for $s(t)$, which ensures smooth acceleration and deceleration. The maximum speed ($\dot{s}_{max}$) and acceleration ($\ddot{s}_{max}$) of this profile are calculated based on the robot's Cartesian limits to ensure the motion is feasible.
   
   $$\dot{s}_{max} = \min\left(\frac{v_{max}}{||\Delta p||}, \frac{\omega_{max}}{||\phi||}\right)$$
   
   $$\ddot{s}_{max} = \min\left(\frac{a_{max}}{||\Delta p||}, \frac{\alpha_{max}}{||\phi||}\right)$$

At the end of `setup()`, the controller has a complete plan for where the end-effector should be and how fast it should be moving along the path at any given time $t$.

### `void loop()`: The Real-Time Control Cycle

This is the core loop that runs repeatedly at the control frequency (e.g., 100 Hz). It's responsible for calculating and sending the actual joint commands.

1. **Read Robot State**: Get the current joint positions $q$ and velocities $\dot{q}_{meas}$ from the robot's encoders.

2. **Determine Desired Direction**: Look up the current time $t$ in the pre-computed trapezoidal profile to get the desired path velocity, $\dot{s}(t)$. This is used to calculate the desired Cartesian velocity, $V_{path}$. This velocity vector is then weighted and normalized to produce a **unit direction vector, $u$**, which represents the instantaneous direction the end-effector should be moving.
   
   $$u = \frac{W \cdot V_{path}}{||W \cdot V_{path}|| + \varepsilon_u} \quad \text{where} \quad W = \text{diag}(\lambda_\omega I_3, \lambda_v I_3)$$

3. **Formulate and Solve the QP**: This is the brain of the operation. Using the current state ($q, \dot{q}_{meas}$) and the desired direction ($u$), the controller builds and solves the Quadratic Program detailed in the next section. The solution provides the optimal joint velocities, $\dot{q}^*$, that best achieve the desired motion while respecting all constraints and secondary objectives.

4. **Send Command**: The optimal joint velocities $\dot{q}^*$ are sent to the robot's motor controllers.

5. **Check for Goal Arrival**: The controller checks if the motion is complete. The goal is reached when the trapezoidal profile has finished *and* the end-effector's actual pose is within a small tolerance of the target.

---

## 3. The Quadratic Program Formulation

The QP solved in the `loop()` is a constrained optimization that finds the best possible $\dot{q}$ at each instant.

### Decision Variables

The optimizer solves for the vector $z = [\dot{q} \in \mathbb{R}^7, \alpha \in \mathbb{R}, w \in \mathbb{R}^6]$:

* **$\dot{q}$**: The joint velocities to command.
* **$\alpha$**: A "gas pedal" scaling factor (0 to 1) that controls speed.
* **$w$**: A "slack" vector that allows path deviation to avoid violating limits.

### Objective Function

The controller's behavior is defined by minimizing a weighted sum of four clear objectives:

$$\min_{\dot{q}, \alpha, w} \quad \underbrace{\frac{1}{2} ||\dot{q}||_{Q_{\dot{q}}}^2}_{\text{1. Minimize Joint Effort}} + \underbrace{\frac{\lambda_{ns}}{2} ||P_n \dot{q} - \dot{q}_{\text{posture}}||^2}_{\text{2. Maintain Posture}} + \underbrace{\frac{\rho}{2} ||w||^2}_{\text{3. Stay on Path}} - \underbrace{\kappa \alpha}_{\text{4. Maximize Speed}}$$

1. **Minimize Joint Effort:** This term, $||\dot{q}||_{Q_{\dot{q}}}^2$, penalizes large joint velocities to keep the motion smooth and efficient. The weighting matrix $Q_{\dot{q}} = \text{diag}(W_j^2)$ uses the *square* of the weights from your `Wj` parameter.

2. **Maintain Posture:** This objective uses any extra ("redundant") motion available to keep the robot in a comfortable pose. It tries to make the robot's nullspace motion ($P_n \dot{q}$) match a desired posture-restoring velocity, $\dot{q}_{\text{posture}} = -K_p \odot (q - q_0)$.

3. **Stay on Path:** The term $\frac{\rho}{2} ||w||^2$ heavily penalizes using the slack variables $w$. This ensures the robot only deviates from the desired path when absolutely necessary.

4. **Maximize Speed:** The term $-\kappa \alpha$ rewards the optimizer for making $\alpha$ large. This tells the robot to "go as fast as you can" subject to all other constraints and objectives.

### Constraints

The optimization is subject to these hard physical and logical limits:

* **Task Constraint:** $J\dot{q} - \alpha u - w = 0$
* **Joint Limits:** $\dot{q}_{min} \leq \dot{q} \leq \dot{q}_{max}$ (enforcing position, velocity, and acceleration limits)
* **Speed Scaling:** $0 \leq \alpha \leq 1$
