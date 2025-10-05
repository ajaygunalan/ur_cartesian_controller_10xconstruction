#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import time

class GridDemo(Node):
    def __init__(self):
        super().__init__('grid_demo')
        self.pub = self.create_publisher(PointStamped, '/cartesian_controller/target_point', 10)
        time.sleep(1)  # Let publisher settle

    def send_point(self, x, y, z):
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        self.pub.publish(msg)
        self.get_logger().info(f'Sent: x={x:.2f}, y={y:.2f}, z={z:.2f}')

def main():
    rclpy.init()
    node = GridDemo()

    # 3x3 raster scan grid
    z_heights = [1.5, 1.0, 0.5]       # 3 rows: high, mid, low
    x_positions = [-0.3, 0.05, 0.4]   # 3 columns: left, center, right
    y = 0.2                            # fixed lateral position

    for z in z_heights:
        for x in x_positions:
            node.send_point(x, y, z)
            time.sleep(2.5)  # Wait for motion to complete

    node.get_logger().info('Demo complete!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
