#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <vector>
#include <set>
#include <tuple>
#include <cmath>
#include <iostream>

namespace fs = std::filesystem;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string pcd_folder = "/home/suba/Desktop/Newproject/dufomap/zenodo.org/records/8160051/files/00/pcd";
  float resolution = 0.2f;

  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<fs::path> pcd_files;

  for (const auto &entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd") {
      pcd_files.push_back(entry.path());
    }
  }

  std::sort(pcd_files.begin(), pcd_files.end());
  for (const auto &path : pcd_files) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.string(), *temp) == -1) {
      PCL_ERROR("Couldn't read file %s\n", path.string().c_str());
      continue;
    }
    *accumulated += *temp;
  }

  std::cout << "Loaded " << accumulated->size() << " total points from " << pcd_files.size() << " files.\n";

  pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ> octree(resolution);
  octree.setInputCloud(accumulated);
  octree.addPointsFromInputCloud();

  std::set<std::tuple<int, int, int>> free_voxel_coords;
  std::vector<pcl::PointXYZ> free_voxels;

  Eigen::Vector3f sensor_origin(0.0f, 0.0f, 5.0f); // elevated

  std::size_t total = accumulated->points.size();
  std::size_t max_points = 500000; // <-- TEMPORARY limit to avoid hanging

  for (std::size_t i = 0; i < std::min(max_points, total); ++i) {
    const auto& pt = accumulated->points[i];
    Eigen::Vector3f endpoint(pt.x, pt.y, pt.z);
    pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector ray_voxels;

    octree.getApproxIntersectedVoxelCentersBySegment(sensor_origin, endpoint, ray_voxels);

    if (ray_voxels.empty()) {
      std::cerr << "[WARN] Empty ray for pt (" << pt.x << "," << pt.y << "," << pt.z << ")\n";
      continue;
    }

    for (const auto& voxel_pt : ray_voxels) {
      pcl::PointXYZ voxel(voxel_pt.x, voxel_pt.y, voxel_pt.z);
      if (!octree.isVoxelOccupiedAtPoint(voxel)) {
        int ix = static_cast<int>(std::floor(voxel.x / resolution));
        int iy = static_cast<int>(std::floor(voxel.y / resolution));
        int iz = static_cast<int>(std::floor(voxel.z / resolution));
        auto key = std::make_tuple(ix, iy, iz);
        if (free_voxel_coords.insert(key).second) {
          free_voxels.push_back(voxel);
        }
      }
    }

    if (i % 100000 == 0 && i > 0)
      std::cout << "Processed " << i << " points for raycasting...\n";
  }

  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> occupied;
  octree.getOccupiedVoxelCenters(occupied);
  std::cout << "Map contains " << occupied.size() << " occupied voxels.\n";
  std::cout << "Free voxels raycasted: " << free_voxels.size() << "\n";

  // Save occupied
  pcl::PointCloud<pcl::PointXYZRGB> occ_cloud;
  for (const auto& pt : occupied) {
    pcl::PointXYZRGB occ;
    occ.x = pt.x; occ.y = pt.y; occ.z = pt.z;
    occ.r = static_cast<uint8_t>(255 * (pt.z + 5.0f) / 10.0f);
    occ.g = 0; occ.b = 255 - occ.r;
    occ_cloud.push_back(occ);
  }
  pcl::io::savePCDFileBinary("occupied_voxels.pcd", occ_cloud);

  // Save free
  pcl::PointCloud<pcl::PointXYZRGB> free_cloud;
  for (const auto& pt : free_voxels) {
    pcl::PointXYZRGB f;
    f.x = pt.x; f.y = pt.y; f.z = pt.z;
    f.r = f.g = f.b = 160;
    free_cloud.push_back(f);
  }
  pcl::io::savePCDFileBinary("free_voxels.pcd", free_cloud);

  std::cout << "Saved occupied_voxels.pcd and free_voxels.pcd\n";
  rclcpp::shutdown();
  return 0;
}
