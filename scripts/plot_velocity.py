#!/usr/bin/env python3
"""
Record Cartesian velocity and save as videos.
Creates 2 video files showing time-series velocity data.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime
import time

class VelocityRecorder(Node):
    def __init__(self):
        super().__init__('velocity_recorder')

        # Data storage
        self.times = []
        self.vx = []
        self.vy = []
        self.vz = []
        self.wx = []
        self.wy = []
        self.wz = []

        self.start_time = None

        # Subscribe to Cartesian velocity
        self.sub = self.create_subscription(
            TwistStamped,
            '/cartesian_controller/cartesian_velocity',
            self.velocity_callback,
            10
        )

        self.get_logger().info('Velocity recorder ready. Waiting for data...')

    def velocity_callback(self, msg):
        """Store velocity data."""
        try:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
                self.get_logger().info('Recording started!')

            t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

            vx = float(msg.twist.linear.x)
            vy = float(msg.twist.linear.y)
            vz = float(msg.twist.linear.z)
            wx = float(msg.twist.angular.x)
            wy = float(msg.twist.angular.y)
            wz = float(msg.twist.angular.z)

            self.times.append(t)
            self.vx.append(vx)
            self.vy.append(vy)
            self.vz.append(vz)
            self.wx.append(wx)
            self.wy.append(wy)
            self.wz.append(wz)

            # Print norms every 50 samples (~0.5s at 100Hz)
            if len(self.times) % 50 == 0:
                v_norm = np.sqrt(vx**2 + vy**2 + vz**2)
                w_norm = np.sqrt(wx**2 + wy**2 + wz**2)
                print(f"[t={t:.2f}s] ||v||={v_norm:.4f} m/s, ||ω||={w_norm:.4f} rad/s, samples={len(self.times)}")

        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')

    def save_videos(self, output_dir='.', fps=30):
        """Save velocity data as video files."""
        if len(self.times) < 2:
            self.get_logger().warn('No data to save!')
            return

        # Convert to numpy
        t = np.array(self.times)
        vx = np.array(self.vx)
        vy = np.array(self.vy)
        vz = np.array(self.vz)
        wx = np.array(self.wx)
        wy = np.array(self.wy)
        wz = np.array(self.wz)

        # Compute norms
        v_norm = np.sqrt(vx**2 + vy**2 + vz**2)
        w_norm = np.sqrt(wx**2 + wy**2 + wz**2)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # ===== Video 1: Linear Velocity =====
        fig1, ax1 = plt.subplots(figsize=(12, 6))
        line_vx, = ax1.plot([], [], 'r-', label='vx', linewidth=2)
        line_vy, = ax1.plot([], [], 'g-', label='vy', linewidth=2)
        line_vz, = ax1.plot([], [], 'b-', label='vz', linewidth=2)
        line_vnorm, = ax1.plot([], [], 'k--', label='||v|| (norm)', linewidth=2.5)

        ax1.set_xlim(0, t[-1])
        ax1.set_ylim(min(vx.min(), vy.min(), vz.min()) * 1.2,
                     max(vx.max(), vy.max(), vz.max(), v_norm.max()) * 1.2)
        ax1.set_xlabel('Time (s)', fontsize=14)
        ax1.set_ylabel('Linear Velocity (m/s)', fontsize=14)
        ax1.set_title('End-Effector Linear Velocity', fontsize=16, fontweight='bold')
        ax1.legend(fontsize=12, loc='best')
        ax1.grid(True, alpha=0.3)

        def init1():
            line_vx.set_data([], [])
            line_vy.set_data([], [])
            line_vz.set_data([], [])
            line_vnorm.set_data([], [])
            return line_vx, line_vy, line_vz, line_vnorm

        def animate1(i):
            line_vx.set_data(t[:i], vx[:i])
            line_vy.set_data(t[:i], vy[:i])
            line_vz.set_data(t[:i], vz[:i])
            line_vnorm.set_data(t[:i], v_norm[:i])
            return line_vx, line_vy, line_vz, line_vnorm

        anim1 = animation.FuncAnimation(
            fig1, animate1, init_func=init1,
            frames=len(t), interval=1000/fps, blit=True
        )

        filename1 = f'{output_dir}/linear_velocity_{timestamp}.mp4'
        self.get_logger().info(f'Saving linear velocity video: {filename1}')
        anim1.save(filename1, writer='ffmpeg', fps=fps, dpi=150)
        plt.close(fig1)
        self.get_logger().info(f'✓ Saved: {filename1}')

        # ===== Video 2: Angular Velocity =====
        fig2, ax2 = plt.subplots(figsize=(12, 6))
        line_wx, = ax2.plot([], [], 'r-', label='ωx', linewidth=2)
        line_wy, = ax2.plot([], [], 'g-', label='ωy', linewidth=2)
        line_wz, = ax2.plot([], [], 'b-', label='ωz', linewidth=2)
        line_wnorm, = ax2.plot([], [], 'm--', label='||ω|| (norm)', linewidth=2.5)

        ax2.set_xlim(0, t[-1])
        ax2.set_ylim(min(wx.min(), wy.min(), wz.min()) * 1.2,
                     max(wx.max(), wy.max(), wz.max(), w_norm.max()) * 1.2)
        ax2.set_xlabel('Time (s)', fontsize=14)
        ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=14)
        ax2.set_title('End-Effector Angular Velocity', fontsize=16, fontweight='bold')
        ax2.legend(fontsize=12, loc='best')
        ax2.grid(True, alpha=0.3)

        def init2():
            line_wx.set_data([], [])
            line_wy.set_data([], [])
            line_wz.set_data([], [])
            line_wnorm.set_data([], [])
            return line_wx, line_wy, line_wz, line_wnorm

        def animate2(i):
            line_wx.set_data(t[:i], wx[:i])
            line_wy.set_data(t[:i], wy[:i])
            line_wz.set_data(t[:i], wz[:i])
            line_wnorm.set_data(t[:i], w_norm[:i])
            return line_wx, line_wy, line_wz, line_wnorm

        anim2 = animation.FuncAnimation(
            fig2, animate2, init_func=init2,
            frames=len(t), interval=1000/fps, blit=True
        )

        filename2 = f'{output_dir}/angular_velocity_{timestamp}.mp4'
        self.get_logger().info(f'Saving angular velocity video: {filename2}')
        anim2.save(filename2, writer='ffmpeg', fps=fps, dpi=150)
        plt.close(fig2)
        self.get_logger().info(f'✓ Saved: {filename2}')

        self.get_logger().info(f'Done! Samples: {len(t)}, Duration: {t[-1]:.2f}s')

def main():
    from threading import Thread, Event
    import sys
    import tty
    import termios

    rclpy.init()
    node = VelocityRecorder()

    output_dir = '.'  # Save in current directory

    # Flag to signal shutdown
    stop_event = Event()

    # ROS spin in background thread
    def spin_thread():
        while rclpy.ok() and not stop_event.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception as e:
                print(f"Spin error (continuing): {e}")

    thread = Thread(target=spin_thread, daemon=True)
    thread.start()

    print("="*60)
    print("🎥 Cartesian Velocity Recorder")
    print("="*60)
    print("Recording velocity data...")
    print("Send motion commands to your robot")
    print()
    print("Press ESC to stop and save videos")
    print("="*60)
    print()

    # Wait for ESC key
    try:
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not stop_event.is_set():
                if sys.stdin in [sys.stdin]:
                    import select
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        if key == '\x1b':  # ESC key
                            break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    except (KeyboardInterrupt, EOFError):
        pass

    # Signal shutdown
    print('\n' + '='*60)
    print('Stopping recording...')
    print('='*60)
    stop_event.set()
    thread.join(timeout=2.0)

    # Print summary
    if len(node.times) > 0:
        vx_arr = np.array(node.vx)
        vy_arr = np.array(node.vy)
        vz_arr = np.array(node.vz)
        wx_arr = np.array(node.wx)
        wy_arr = np.array(node.wy)
        wz_arr = np.array(node.wz)
        v_norm = np.sqrt(vx_arr**2 + vy_arr**2 + vz_arr**2)
        w_norm = np.sqrt(wx_arr**2 + wy_arr**2 + wz_arr**2)

        print(f"\n📊 Recording Summary:")
        print(f"  Duration: {node.times[-1]:.2f}s")
        print(f"  Samples: {len(node.times)}")
        print(f"  Linear velocity  ||v||: max={v_norm.max():.4f} m/s, avg={v_norm.mean():.4f} m/s")
        print(f"  Angular velocity ||ω||: max={w_norm.max():.4f} rad/s, avg={w_norm.mean():.4f} rad/s")
        print()

    # Save videos
    node.save_videos(output_dir, fps=30)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
