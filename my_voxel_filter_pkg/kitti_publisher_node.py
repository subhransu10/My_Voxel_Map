import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import os
import time
from glob import glob

class KittiPublisher(Node):
    def __init__(self):
        super().__init__('kitti_publisher')

        self.publisher = self.create_publisher(PointCloud2, '/lidar_points', 10)

        # Update this path to your actual folder
        self.data_path = os.path.expanduser('/home/suba/my_ros2_ws/src/my_voxel_filter_pkg/00/velodyne')
        self.files = sorted(glob(os.path.join(self.data_path, '*.bin')))

        if not os.path.exists(self.data_path):
            self.get_logger().error(f"Data path does not exist: {self.data_path}")
            return

        if not self.files:
            self.get_logger().error(f"No .bin files found in: {self.data_path}")
            return

        self.index = 0
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.get_logger().info(f"Found {len(self.files)} KITTI scans in: {self.data_path}")

    def timer_callback(self):
        if self.index >= len(self.files):
            self.get_logger().info("All frames published. Shutting down timer.")
            self.timer.cancel()
            return

        file = self.files[self.index]
        try:
            pointcloud = np.fromfile(file, dtype=np.float32).reshape(-1, 4)
        except Exception as e:
            self.get_logger().error(f"Failed to load {file}: {e}")
            self.index += 1
            return

        if pointcloud.shape[0] == 0:
            self.get_logger().warn(f"Empty point cloud in: {file}")
            self.index += 1
            return

        if self.index == 0:
            self.get_logger().info(f"First scan shape: {pointcloud.shape}")

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, pointcloud)
        self.publisher.publish(cloud_msg)
        self.get_logger().info(f"Published frame {self.index + 1}/{len(self.files)}: {os.path.basename(file)}")

        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = KittiPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()