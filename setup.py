from setuptools import find_packages, setup

package_name = 'my_voxel_filter_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/mapping_launch.py']),
    ('share/' + package_name + '/config', ['config/voxel_params.yaml', 'config/rviz_voxel_filter.rviz']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'open3d', 'ros-numpy'],
    zip_safe=True,
    maintainer='suba',
    maintainer_email='subhransusouravp@gmail.com',
    description='A ROS 2 package for point cloud filtering ',
    license='Apache-2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voxel_filter_node = my_voxel_filter_pkg.voxel_filter_node:main',
            'kitti_publisher_node = my_voxel_filter_pkg.kitti_publisher_node:main',
        ],
    },
)
