#!/usr/bin/env python3
"""
Plot Cartesian velocity of 7-DOF manipulator end-effector.
Computes V = J(q) * dq and plots linear & angular velocities.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import PyKDL as kdl
from kdl_parser_py import urdf as kdl_parser

class CartesianVelocityPlotter(Node):
    def __init__(self):
        super().__init__('cartesian_velocity_plotter')

        # Get URDF from parameter server
        self.declare_parameter('robot_description', '')
        urdf_string = self.get_parameter('robot_description').value

        if not urdf_string:
            self.get_logger().error('robot_description parameter not found!')
            return

        # Parse URDF and build KDL chain
        (ok, tree) = kdl_parser.treeFromString(urdf_string)
        if not ok:
            self.get_logger().error('Failed to parse URDF')
            return

        chain = tree.getChain('world', 'tool0')
        self.get_logger().info(f'KDL chain: {chain.getNrOfJoints()} joints')

        # Create KDL solvers
        self.jac_solver = kdl.ChainJntToJacSolver(chain)
        self.njoints = chain.getNrOfJoints()

        # Joint names (elevator + 6 UR joints)
        self.joint_names = [
            'elevator_joint',
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # Data storage (last 500 samples ~ 5 seconds at 100 Hz)
        self.max_samples = 500
        self.times = deque(maxlen=self.max_samples)
        self.v_linear = deque(maxlen=self.max_samples)  # [vx, vy, vz]
        self.v_angular = deque(maxlen=self.max_samples) # [wx, wy, wz]
        self.v_linear_norm = deque(maxlen=self.max_samples)
        self.v_angular_norm = deque(maxlen=self.max_samples)

        self.start_time = None

        # Subscribe to joint states
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Cartesian velocity plotter initialized')

    def joint_state_callback(self, msg):
        """Compute Cartesian velocity from joint velocities."""
        try:
            # Map joint names to indices
            name_to_idx = {name: idx for idx, name in enumerate(msg.name)}

            # Extract joint positions and velocities in correct order
            q = kdl.JntArray(self.njoints)
            dq = kdl.JntArray(self.njoints)

            for i, joint_name in enumerate(self.joint_names):
                if joint_name not in name_to_idx:
                    return
                idx = name_to_idx[joint_name]
                q[i] = msg.position[idx]
                dq[i] = msg.velocity[idx] if msg.velocity else 0.0

            # Compute Jacobian J(q)
            jac = kdl.Jacobian(self.njoints)
            self.jac_solver.JntToJac(q, jac)

            # Convert KDL Jacobian to numpy (KDL ordering: [angular; linear])
            J = np.zeros((6, self.njoints))
            for i in range(6):
                for j in range(self.njoints):
                    J[i, j] = jac[i, j]

            # Convert joint velocities to numpy
            dq_vec = np.array([dq[i] for i in range(self.njoints)])

            # Compute Cartesian velocity: V = J * dq
            # V = [angular_velocity (wx,wy,wz); linear_velocity (vx,vy,vz)]
            V = J @ dq_vec

            # Extract angular and linear parts (swap to match typical convention)
            omega = V[0:3]  # Angular velocity (rad/s)
            v_lin = V[3:6]  # Linear velocity (m/s)

            # Store data
            if self.start_time is None:
                self.start_time = self.get_clock().now()

            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

            self.times.append(current_time)
            self.v_linear.append(v_lin)
            self.v_angular.append(omega)
            self.v_linear_norm.append(np.linalg.norm(v_lin))
            self.v_angular_norm.append(np.linalg.norm(omega))

        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')

def plot_velocity(node):
    """Create animated plot of Cartesian velocity."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Cartesian End-Effector Velocity', fontsize=16)

    # Linear velocity components
    ax1 = axes[0]
    ax1.set_ylabel('Linear Velocity (m/s)')
    ax1.set_title('Linear Velocity Components')
    ax1.grid(True, alpha=0.3)
    line_vx, = ax1.plot([], [], 'r-', label='vx', linewidth=2)
    line_vy, = ax1.plot([], [], 'g-', label='vy', linewidth=2)
    line_vz, = ax1.plot([], [], 'b-', label='vz', linewidth=2)
    ax1.legend(loc='upper right')
    ax1.set_ylim(-1.0, 1.0)

    # Angular velocity components
    ax2 = axes[1]
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title('Angular Velocity Components')
    ax2.grid(True, alpha=0.3)
    line_wx, = ax2.plot([], [], 'r-', label='ωx', linewidth=2)
    line_wy, = ax2.plot([], [], 'g-', label='ωy', linewidth=2)
    line_wz, = ax2.plot([], [], 'b-', label='ωz', linewidth=2)
    ax2.legend(loc='upper right')
    ax2.set_ylim(-3.0, 3.0)

    # Velocity magnitudes
    ax3 = axes[2]
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity Magnitude')
    ax3.set_title('Velocity Magnitudes')
    ax3.grid(True, alpha=0.3)
    line_vnorm, = ax3.plot([], [], 'k-', label='||v|| (m/s)', linewidth=2)
    line_wnorm, = ax3.plot([], [], 'm-', label='||ω|| (rad/s)', linewidth=2)
    ax3.legend(loc='upper right')
    ax3.set_ylim(0, 3.0)

    def init():
        """Initialize animation."""
        for line in [line_vx, line_vy, line_vz, line_wx, line_wy, line_wz, line_vnorm, line_wnorm]:
            line.set_data([], [])
        return line_vx, line_vy, line_vz, line_wx, line_wy, line_wz, line_vnorm, line_wnorm

    def update(frame):
        """Update plot with new data."""
        if len(node.times) < 2:
            return line_vx, line_vy, line_vz, line_wx, line_wy, line_wz, line_vnorm, line_wnorm

        # Convert deques to arrays
        times = np.array(node.times)
        v_lin = np.array(node.v_linear)
        v_ang = np.array(node.v_angular)
        v_lin_norm = np.array(node.v_linear_norm)
        v_ang_norm = np.array(node.v_angular_norm)

        # Update linear velocity plot
        line_vx.set_data(times, v_lin[:, 0])
        line_vy.set_data(times, v_lin[:, 1])
        line_vz.set_data(times, v_lin[:, 2])

        # Update angular velocity plot
        line_wx.set_data(times, v_ang[:, 0])
        line_wy.set_data(times, v_ang[:, 1])
        line_wz.set_data(times, v_ang[:, 2])

        # Update magnitude plot
        line_vnorm.set_data(times, v_lin_norm)
        line_wnorm.set_data(times, v_ang_norm)

        # Auto-scale x-axis
        if len(times) > 0:
            for ax in axes:
                ax.set_xlim(max(0, times[-1] - 10), times[-1] + 0.5)

        # Auto-scale y-axis for magnitudes
        if len(v_lin_norm) > 0:
            max_v = max(np.max(v_lin_norm) if len(v_lin_norm) > 0 else 0.1, 0.1)
            max_w = max(np.max(v_ang_norm) if len(v_ang_norm) > 0 else 0.1, 0.1)

            # Update y-limits with some margin
            ax1.set_ylim(-max_v * 1.2, max_v * 1.2)
            ax2.set_ylim(-max_w * 1.2, max_w * 1.2)
            ax3.set_ylim(0, max(max_v, max_w) * 1.2)

        return line_vx, line_vy, line_vz, line_wx, line_wy, line_wz, line_vnorm, line_wnorm

    # Create animation
    ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=50, cache_frame_data=False)

    plt.tight_layout()
    plt.show()

def main():
    rclpy.init()
    node = CartesianVelocityPlotter()

    # Start ROS2 spinning in background
    from threading import Thread
    spin_thread = Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # Run plotting in main thread
    try:
        plot_velocity(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
