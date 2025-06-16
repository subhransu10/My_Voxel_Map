#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

class VoxelPublisher : public rclcpp::Node
{
public:
  VoxelPublisher()
  : Node("voxel_pcd_publisher")
  {
    occupied_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_occupied", 10);
    free_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_free", 10);

    loadPointClouds();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&VoxelPublisher::publishClouds, this));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::PointCloud2 occupied_msg_;
  sensor_msgs::msg::PointCloud2 free_msg_;

  void loadPointClouds()
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr occupied(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr free(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("occupied_voxels.pcd", *occupied) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load occupied_voxels.pcd");
      return;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("free_voxels.pcd", *free) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load free_voxels.pcd");
      return;
    }

    occupied->is_dense = true;
    occupied->width = occupied->size();
    occupied->height = 1;

    free->is_dense = true;
    free->width = free->size();
    free->height = 1;

    pcl::toROSMsg(*occupied, occupied_msg_);
    pcl::toROSMsg(*free, free_msg_);

    occupied_msg_.header.frame_id = "map";
    free_msg_.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "Loaded %ld occupied and %ld free voxels",
                occupied->size(), free->size());
  }

  void publishClouds()
  {
    auto stamp = this->now();
    occupied_msg_.header.stamp = stamp;
    free_msg_.header.stamp = stamp;

    occupied_pub_->publish(occupied_msg_);
    free_pub_->publish(free_msg_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Publishing voxel_occupied and voxel_free");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelPublisher>());
  rclcpp::shutdown();
  return 0;
}
