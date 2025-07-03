import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
from collections import deque
import open3d as o3d

# Helper to convert numpy point cloud to Open3D
def numpy_to_o3d(points_np):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)
    return pcd

# Helper to downsample point cloud to voxel grid
def voxel_downsample(points_np, voxel_size):
    pcd = numpy_to_o3d(points_np)
    down_pcd = pcd.voxel_down_sample(voxel_size)
    return np.asarray(down_pcd.points)

class VoxelFilterNode(Node):
    def __init__(self):
        super().__init__('voxel_filter_node')

        self.declare_parameter("voxel_resolution", 0.2)
        self.declare_parameter("consistency_threshold", 3)
        self.declare_parameter("frame_buffer_size", 10)

        # Load parameters
        self.voxel_size = self.get_parameter("voxel_resolution").value
        self.consistency_threshold = self.get_parameter("consistency_threshold").value
        self.buffer_size = self.get_parameter("frame_buffer_size").value

        # Buffer to hold previous voxelized point clouds
        self.voxel_buffer = deque(maxlen=self.buffer_size)

        # Sub and Pub
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_points',
            self.pointcloud_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)

        self.get_logger().info("Voxel Filter Node Initialized")

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy (x, y, z)
        points = np.array([
            [p[0], p[1], p[2]] for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

        # Voxel downsample current frame
        downsampled = voxel_downsample(points, self.voxel_size)
        self.voxel_buffer.append(downsampled)

        # Accumulate point occurrences
        voxel_dict = {}
        for frame in self.voxel_buffer:
            # Quantize points to voxel grid
            keys = np.floor(frame / self.voxel_size).astype(np.int32)
            for key in map(tuple, keys):
                voxel_dict[key] = voxel_dict.get(key, 0) + 1

        # Keep only "static" voxels
        static_voxels = [k for k, v in voxel_dict.items() if v >= self.consistency_threshold]
        static_points = np.array(static_voxels, dtype=np.float32) * self.voxel_size

        # Publish static point cloud
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id  # Usually "base_link" or "lidar_link"
        cloud_msg = point_cloud2.create_cloud_xyz32(header, static_points.tolist())
        self.publisher.publish(cloud_msg)


        #To summarize the things done in this code:
        #every incoming point cloud is voxel-downsampled and a rolling buffer stores N past frames.
        #each voxel gets a "hit count" based on how many frames it appears in.
        #only voxels seen >= thtreshold times are considered "static" and result is published as /filtered_points
        
def main(args=None):
    rclpy.init(args=args)
    node = VoxelFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
