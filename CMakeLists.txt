cmake_minimum_required(VERSION 3.8)
project(my_voxel_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common octree)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# map_builder
add_executable(map_builder src/map_builder.cpp)
ament_target_dependencies(map_builder rclcpp pcl_conversions)
target_link_libraries(map_builder ${PCL_LIBRARIES})

# octree_visualizer_node
add_executable(octree_visualizer_node src/octree_visualizer_node.cpp)
ament_target_dependencies(octree_visualizer_node rclcpp visualization_msgs geometry_msgs)
target_link_libraries(octree_visualizer_node ${PCL_LIBRARIES})

add_executable(voxel_pcd_publisher src/voxel_pcd_publisher.cpp)
ament_target_dependencies(voxel_pcd_publisher rclcpp sensor_msgs pcl_conversions)
target_link_libraries(voxel_pcd_publisher ${PCL_LIBRARIES})

install(TARGETS 
  map_builder
  octree_visualizer_node
  voxel_pcd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

