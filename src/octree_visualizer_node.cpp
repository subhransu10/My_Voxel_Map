#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <pcl/point_types.h>
#include <fstream>
#include <vector>

class OctreeVisualizer : public rclcpp::Node {
public:
  OctreeVisualizer()
      : Node("octree_visualizer_node"), resolution_(0.2f) {
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("occupied_voxels", 1);
    this->declare_parameter<std::string>("map_file", "octree_map.ot");

    std::string map_file;
    this->get_parameter("map_file", map_file);
    loadAndVisualizeOctree(map_file);
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  float resolution_;

  void loadAndVisualizeOctree(const std::string& filename) {
    std::ifstream in(filename, std::ios::binary | std::ios::ate);
    if (!in.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
      return;
    }

    std::streamsize size = in.tellg();
    in.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    in.read(buffer.data(), size);
    in.close();

    pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ> octree(resolution_);
    auto dummy_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    dummy_cloud->width = 1;
    dummy_cloud->height = 1;
    dummy_cloud->points.push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
    octree.setInputCloud(dummy_cloud);
    octree.deserializeTree(buffer);
    
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> centers;
    octree.getOccupiedVoxelCenters(centers);
    RCLCPP_INFO(this->get_logger(), "Loaded octree contains %lu voxels", centers.size());

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxel_centers;
    octree.getOccupiedVoxelCenters(voxel_centers);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "octree";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = resolution_;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto& pt : voxel_centers) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      marker.points.push_back(p);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing %zu occupied voxels", marker.points.size());
    marker_pub_->publish(marker);
  }
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctreeVisualizer>());
  rclcpp::shutdown();
  return 0;
}
