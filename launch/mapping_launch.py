from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static Transform: map -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),

        # KITTI Publisher Node
        Node(
            package='my_voxel_filter_pkg',
            executable='kitti_publisher_node',
            name='kitti_publisher',
            output='screen'
        ),

        # Voxel Filter Node
        Node(
            package='my_voxel_filter_pkg',
            executable='voxel_filter_node',
            name='voxel_filter_node',
            output='screen'
        ),

        # RViz2 with Config (optional but recommended)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/suba/my_ros2_ws/src/my_voxel_filter_pkg/config/rviz_voxel_filter.rviz'],
            output='screen'
        ),
    ])