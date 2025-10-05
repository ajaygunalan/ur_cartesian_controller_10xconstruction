# Differential Inverse Kinematics for 7-DOF Manipulator
## Mathematical Formulation with QP-Based Redundancy Resolution

## Executive Summary

This formulation provides motion control for a 7-DOF robot (6 revolute joints + 1 prismatic lift) using differential inverse kinematics with quadratic programming. The approach preserves task direction while optimally distributing motion between arm and lift under physical constraints.

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     ┌──────────────┐
│   Step 0    │────▶│    Step 1    │────▶│   Step 2    │────▶│    Step 3    │
│   Initial   │     │    Path      │     │   Timing    │     │  QP Solver   │
│   Setup     │     │  Planning    │     │  Generation │     │  Execution   │
└─────────────┘     └──────────────┘     └─────────────┘     └──────────────┘
       │                    │                    │                    │
    q₀, X₀               Δp, φ              s(t), ṡ(t)          ṗ*, α, w
  Robot State        Path Parameters      Time Profile      Optimal Solution
```

---

## Step 0: System Configuration & Initial State

### Robot Architecture
- **Configuration vector**: $q \in \mathbb{R}^7$ where $q = [q_1, q_2, ..., q_6, d]^T$
  - $q_1$ to $q_6$: revolute joint angles (radians)
  - $d$: prismatic lift displacement (meters)
- **Velocity vector**: $\dot{q} \in \mathbb{R}^7$

### Reference Frames
- World frame: $W$
- End-effector frame: $G$
- Initial pose: $X_0 = \{R_0, p_0\}$ where $R_0 \in SO(3)$, $p_0 \in \mathbb{R}^3$
- Target pose: $X^* = \{R^*, p^*\}$

### Kinematic Mappings
- Forward kinematics: $X_G(q) = \{R_G(q), p_G(q)\} \in SE(3)$
- Geometric Jacobian (spatial): $J_G(q) \in \mathbb{R}^{6 \times 7}$
- Spatial twist: $V_G = [\omega^T, v^T]^T = J_G(q)\dot{q} \in \mathbb{R}^6$
- Lift Jacobian column: $j_{\text{lift}} = [0, 0, 0, 0, 0, 1]^T$ (motion along $\hat{z}_W$)

### Physical Limits
- Joint position bounds: $q_{\min} \leq q \leq q_{\max}$
- Joint velocity bounds: $\dot{q}_{\min} \leq \dot{q} \leq \dot{q}_{\max}$
- Joint acceleration bounds: $\ddot{q}_{\min} \leq \ddot{q} \leq \ddot{q}_{\max}$
- Lift bounds: $d \in [0, 1]$ m, $|\dot{d}| \leq 0.25$ m/s

---

## Step 1: Path Planning in SE(3)

### Position Path
$$p(s) = p_0 + s\Delta p, \quad \Delta p = p^* - p_0, \quad L = \|\Delta p\|$$

### Orientation Path
1. Compute relative rotation: $R_{\Delta} = R_0^T R^*$
2. Extract axis-angle: $\phi = \text{Log}(R_{\Delta}) \in \mathbb{R}^3$, $\theta^* = \|\phi\|$
3. Interpolated rotation: $R(s) = R_0 \exp(s\hat{\phi})$

### Path-Parameterized Twist
For path parameter $s \in [0,1]$ with rate $\dot{s}$:
$$\omega_{\text{path}}(s, \dot{s}) = \dot{s}\phi, \quad v_{\text{path}}(s, \dot{s}) = \dot{s}\Delta p$$

---

## Step 2: Trapezoidal Timing Profile Generation

### Speed Limit Mapping
Transform Cartesian limits to path-space limits:

**Linear constraints:**
$$\dot{s}_{\max}^{\text{lin}} = \frac{v_{\max}^{\text{lin}}}{L}, \quad \ddot{s}_{\max}^{\text{lin}} = \frac{a_{\max}^{\text{lin}}}{L} \quad (L > 0)$$

**Angular constraints:**
$\dot{s}_{\max}^{\text{ang}} = \frac{\omega_{\max}}{\theta^*}, \quad \ddot{s}_{\max}^{\text{ang}} = \frac{\alpha_{\max}}{\theta^*} \quad (\theta^* > 0)$

**Combined limits:**
$$\dot{s}_{\max} = \min(\dot{s}_{\max}^{\text{lin}}, \dot{s}_{\max}^{\text{ang}}), \quad \ddot{s}_{\max} = \min(\ddot{s}_{\max}^{\text{lin}}, \ddot{s}_{\max}^{\text{ang}})$$

### Trapezoidal Velocity Profile

The velocity profile consists of three phases: acceleration, cruise, and deceleration.

**Profile Parameters:**
- Peak velocity: $\dot{s}_p = \dot{s}_{\max}$
- Acceleration time: $t_a = \frac{\dot{s}_{\max}}{\ddot{s}_{\max}}$
- Cruise time: $t_c = \frac{1 - \frac{\dot{s}_{\max}^2}{\ddot{s}_{\max}}}{\dot{s}_{\max}}$
- Total time: $T = 2t_a + t_c$

**Velocity Profile:**
$$\dot{s}_{\text{ref}}(t) = \begin{cases}
\ddot{s}_{\max} \cdot t, & 0 \leq t < t_a \quad \text{(acceleration)} \\
\dot{s}_{\max}, & t_a \leq t < t_a + t_c \quad \text{(cruise)} \\
\ddot{s}_{\max}(T - t), & t_a + t_c \leq t \leq T \quad \text{(deceleration)}
\end{cases}$$

**Position Profile:**
$$s(t) = \begin{cases}
\frac{1}{2}\ddot{s}_{\max} t^2, & 0 \leq t < t_a \\
\frac{1}{2}\ddot{s}_{\max} t_a^2 + \dot{s}_{\max}(t - t_a), & t_a \leq t < t_a + t_c \\
1 - \frac{1}{2}\ddot{s}_{\max}(T - t)^2, & t_a + t_c \leq t \leq T
\end{cases}$$

Note: Profile is valid when $1 \geq \frac{\dot{s}_{\max}^2}{\ddot{s}_{\max}}$. Ensure this condition holds by appropriate selection of limits.

---

## Step 3: Quadratic Programming Formulation

### Decision Variables
- $\dot{q} \in \mathbb{R}^7$: joint velocities
- $\alpha \in [0,1]$: task magnitude scaling
- $w \in \mathbb{R}^6$: task slack

### Task Direction
$$u = \frac{W_{\text{task}}V_{\text{path}}}{\|W_{\text{task}}V_{\text{path}}\| + \epsilon_u}$$
where $W_{\text{task}} = \text{diag}(\lambda_\omega I_3, \lambda_v I_3)$

### Objective Function
$$\min_{\dot{q}, \alpha, w} \quad J = J_{\text{effort}} + J_{\text{null}} + J_{\text{slack}} + J_{\text{speed}}$$

where:
- $J_{\text{effort}} = \frac{1}{2}\|W_j\dot{q}\|_2^2$ (weighted joint effort)
- $J_{\text{null}} = \frac{\lambda_{ns}}{2}\|P(\dot{q} + K(q - q_0))\|_2^2$ (null-space regulation)
- $J_{\text{slack}} = \frac{\rho}{2}\|w\|_2^2$ (task violation penalty)
- $J_{\text{speed}} = -\kappa\alpha$ (speed maximization reward)

with:
- $W_j = \text{diag}(w_1, ..., w_6, w_{\text{lift}})$: joint weight matrix
- $P = I_7 - J_G^+J_G$: null-space projector
- $K = \text{diag}(k_1, ..., k_7)$: posture gain matrix
- $q_0$: nominal reference posture

### Constraints

**Task constraint:**
$$J_G(q)\dot{q} - \alpha u = w$$

**Kinematic bounds:**
$$\dot{q}_{\min} \leq \dot{q} \leq \dot{q}_{\max}$$

**Position integration bounds:**
$$q_{\min} \leq q + h\dot{q} \leq q_{\max}$$

**Acceleration bounds:**
$$\ddot{q}_{\min} \leq \frac{\dot{q} - \dot{q}^{\text{meas}}}{h} \leq \ddot{q}_{\max}$$

**Scaling bounds:**
$$0 \leq \alpha \leq 1$$

---

## Parameter Guidelines

### Weights and Gains
- Joint weights: $w_i = 1$ (arm joints), $w_{\text{lift}} \in \{0.1, 1, 10\}$ for lift preference
- Null-space weight: $\lambda_{ns} = 10^{-2}$
- Slack penalty: $\rho = 10^3$
- Speed reward: $\kappa \in [0.05, 0.2]$
- Posture gains: $K = \text{diag}(0.5, ..., 0.5)$ s$^{-1}$

### Numerical Thresholds
- Direction normalization: $\epsilon_u = 10^{-9}$
- Zero velocity detection: $\epsilon_v = 10^{-4}$
- Position tolerance: $\epsilon_p = 10^{-3}$ m
- Orientation tolerance: $\epsilon_\omega = 10^{-2}$ rad

### Control Parameters
- Time step: $h \in [0.002, 0.01]$ s
- Task weights: $\lambda_\omega = \lambda_v = 1$ (initially equal)

---

## Mathematical Properties

1. **Direction preservation**: Task direction $u$ maintained, magnitude optimally scaled by $\alpha$
2. **Feasibility**: Slack variable $w$ ensures solution exists even under tight constraints
3. **Redundancy resolution**: Null-space term drives configuration toward $q_0$ without affecting task
4. **Smooth degradation**: As limits tighten, $\alpha$ continuously decreases from 1 to 0
5. **Motion allocation**: Weight $w_{\text{lift}}$ naturally distributes vertical motion between lift and arm

---

## Convergence Criteria

Motion terminates when both position and orientation errors fall below thresholds:
- Position: $\|p - p^*\|_\infty \leq \epsilon_p$
- Orientation: $\|\text{Log}(R^T R^*)\|_\infty \leq \epsilon_\omega$