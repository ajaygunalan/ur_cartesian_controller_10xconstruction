# ur_cartesian_controller

QP-based differential IK controller for 7-DOF UR manipulator (6 revolute + 1 prismatic lift).

## Features

- **QP-based differential IK** matching `diff_ik_7dof.md` formulation
- **Direction-preserving** task execution with α scaling (0–1)
- **Trapezoidal velocity profile** on SE(3) paths
- **Nullspace posture control** for redundancy resolution
- Publishes to `/forward_velocity_controller/commands`

## Dependencies

See [doc/dependencies.md](doc/dependencies.md) for installation.

**One-time setup:**
```bash
cd /tmp
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
sudo make install
```

## Build

```bash
cd ~/10x_ws
colcon build --packages-select ur_cartesian_controller
source install/setup.bash
```

## Run

### 1. Start simulation

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e
```

### 2. Activate velocity controller

```bash
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller
```

### 3. Launch cartesian controller

```bash
ros2 run ur_cartesian_controller cartesian_controller
```

### 4. Send end-effector commands

**Demo - 3x3 grid (varies lift height):**
```bash
ros2 run ur_cartesian_controller demo_grid.py
```

**Manual - Move to point (keep current orientation):**
```bash
ros2 topic pub --once /cartesian_controller/target_point geometry_msgs/PointStamped \
  '{header: {frame_id: "world"}, point: {x: 0.4, y: 0.2, z: 0.3}}'
```

**Manual - Move to pose (position + orientation):**
```bash
ros2 topic pub --once /cartesian_controller/target_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"},
    pose: {
      position: {x: 0.4, y: 0.1, z: 0.5},
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
    }
  }'
```

## Visualization - Plot Cartesian Velocity

Record and save Cartesian velocity as video files showing the trapezoidal velocity profile:

```bash
# Terminal 1-3: Start simulation, controller, and cartesian controller (as above)

# Terminal 4: Record velocity (run from desired save location)
ros2 run ur_cartesian_controller plot_velocity.py

# Terminal 5: Run motion demo
ros2 run ur_cartesian_controller demo_grid.py

# When motion complete, press ESC in Terminal 4 to stop and save
# Videos saved: linear_velocity_<timestamp>.mp4, angular_velocity_<timestamp>.mp4
```

**Output videos show:**
- Linear velocity: vx, vy, vz + ||v|| norm (m/s)
- Angular velocity: ωx, ωy, ωz + ||ω|| norm (rad/s)
- Trapezoidal profile clearly visible in norm plots

**Requirements:** `pip install matplotlib`

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cartesian_controller/target_point` | `geometry_msgs/PointStamped` | Target position (keeps current orientation) |
| `/cartesian_controller/target_pose` | `geometry_msgs/PoseStamped` | Target pose (position + orientation) |
| `/cartesian_controller/cartesian_velocity` | `geometry_msgs/TwistStamped` | Cartesian velocity output (V = J·dq) |
| `/forward_velocity_controller/commands` | `std_msgs/Float64MultiArray` | Joint velocities output [7] |
| `/joint_states` | `sensor_msgs/JointState` | Current joint states input |

## Parameters

All parameters in `diff_ik_7dof.md` are configurable via ROS params. Defaults are:

- `control_hz`: 100
- `lambda_omega`, `lambda_v`: 1.0 (task weights)
- `Wj`: [1, 1, 1, 1, 1, 1, 1] (joint effort weights; increase `Wj[0]` to prefer lift)
- `lambda_ns`: 0.01 (nullspace weight)
- `rho`: 1000 (slack penalty)
- `kappa`: 0.1 (speed reward)

## Math

Full formulation in `/home/ajay/10x_ws/src/ur_cartesian_controller/diff_ik_7dof.md`
