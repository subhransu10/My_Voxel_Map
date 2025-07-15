import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
from collections import deque
import open3d as o3d

# Helper: Convert numpy array to Open3D PointCloud
def numpy_to_o3d(points_np):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)
    return pcd

# Helper: Downsample point cloud
def voxel_downsample(points_np, voxel_size):
    pcd = numpy_to_o3d(points_np)
    down_pcd = pcd.voxel_down_sample(voxel_size)
    return np.asarray(down_pcd.points)

class VoxelFilterNode(Node):
    def __init__(self):
        super().__init__('voxel_filter_node')

        # Declare and load parameters
        self.declare_parameter("voxel_resolution", 0.2)
        self.declare_parameter("consistency_threshold", 3)
        self.declare_parameter("frame_buffer_size", 10)
        self.declare_parameter("voxel_age_threshold_ns", int(2e9))  # 2 seconds

        self.voxel_size = self.get_parameter("voxel_resolution").value
        self.consistency_threshold = self.get_parameter("consistency_threshold").value
        self.buffer_size = self.get_parameter("frame_buffer_size").value
        self.voxel_age_threshold_ns = self.get_parameter("voxel_age_threshold_ns").value

        # Data structures
        self.voxel_buffer = deque(maxlen=self.buffer_size)
        self.voxel_counts = {}     # key: voxel index, value: count
        self.voxel_last_seen = {}  # key: voxel index, value: last seen timestamp (ns)

        # ROS interfaces
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_points',
            self.pointcloud_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.occupancy_pub = self.create_publisher(PointCloud2, '/voxel_occupancy_grid', 10)

        self.get_logger().info("Voxel Filter Node Initialized with aging and occupancy grid")

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy (x, y, z)
        points = np.array([
            [p[0], p[1], p[2]] for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

        if points.size == 0:
            self.get_logger().warn("Received empty point cloud frame.")
            return

        # Downsample current frame
        downsampled = voxel_downsample(points, self.voxel_size)
        self.voxel_buffer.append(downsampled)

        # Current timestamp
        now_ns = self.get_clock().now().nanoseconds

        # Update voxel counts and timestamps
        for frame in self.voxel_buffer:
            keys = np.floor(frame / self.voxel_size).astype(np.int32)
            for key in map(tuple, keys):
                self.voxel_counts[key] = self.voxel_counts.get(key, 0) + 1
                self.voxel_last_seen[key] = now_ns

        # Apply aging and filtering
        static_voxels = []
        for key, count in self.voxel_counts.items():
            age_ns = now_ns - self.voxel_last_seen.get(key, 0)
            if count >= self.consistency_threshold and age_ns < self.voxel_age_threshold_ns:
                static_voxels.append(key)

        # Cleanup old voxels
        self.voxel_counts = {
            k: v for k, v in self.voxel_counts.items()
            if now_ns - self.voxel_last_seen.get(k, 0) < self.voxel_age_threshold_ns
        }
        self.voxel_last_seen = {
            k: v for k, v in self.voxel_last_seen.items()
            if now_ns - v < self.voxel_age_threshold_ns
        }

        # Create output point cloud
        static_points = np.array(static_voxels, dtype=np.float32) * self.voxel_size

        # Publish filtered points (static map)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id

        filtered_msg = point_cloud2.create_cloud_xyz32(header, static_points.tolist())
        self.publisher.publish(filtered_msg)

        # Also publish as occupancy grid style
        self.occupancy_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoxelFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
