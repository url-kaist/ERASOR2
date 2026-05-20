#ifndef ERASOR2_MAPGEN_H
#define ERASOR2_MAPGEN_H

#include <math.h>

#include <filesystem>

#include "rosparam_server.hpp"

#define NUM_MAP_CLOUD_LARGE_ENOUGH 30000000

const char separator  = ' ';
const int print_width = 10;

class Mapgen : public RosParamServer {
 public:
  bool is_initial_ = true;
  int count_       = 0;

  // For large-scale map building
  // ROS can only publish point cloud whose volume is under the 1 GB
  std::vector<pcl::PointCloud<pcl::PointXYZI> > submaps;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_curr_wrt_world_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map_;

  std::string save_path_;

  explicit Mapgen(const erasor2::Config &cfg) : RosParamServer(cfg) {
    cloud_curr_wrt_world_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    const std::string rule(60, '-');
    std::cout << "\033[1;36m" << rule << "\n"
              << "[mapgen] configuration\n"
              << rule << "\033[0m\n"
              << "  sequence        : " << sequence_ << "\n"
              << "  frame range     : " << start_frame_ << ".." << end_frame_ << "\n"
              << "  accum. interval : " << accum_interval_ << "\n"
              << "  voxel size      : " << voxel_size_ << "\n"
              << "  large-scale     : " << (is_large_scale_ ? "yes" : "no") << "\n";
    printExtrinsic();
    std::cout << "\033[1;36m" << rule << "\033[0m" << std::endl;
  }

  ~Mapgen() {}

  void accumPointCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud, const Eigen::Matrix4f &pose) {
    // 1. Append the current pose to the path. Path used to be a
    // nav_msgs::Path of geometry_msgs::PoseStamped; rerun consumes a plain
    // sequence of Matrix4f and renders it as a 3D line strip.
    nav_path_.emplace_back(pose);

    // NOTE: `tf_h_of_ground_to_be_zero_` is not necessary, but we follow the legacy of ERASOR 1.0
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(cloud, *ptr_transformed, tf_h_of_ground_to_be_zero_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr world_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*ptr_transformed, *world_transformed, pose);

    erasor_utils::voxelize_preserving_labels_by_nanoflann(
        world_transformed, *cloud_curr_wrt_world_, voxel_size_);

    if (is_initial_) {
      *cloud_map_ = *cloud_curr_wrt_world_;
      is_initial_ = false;
    } else {
      *cloud_map_ += *cloud_curr_wrt_world_;

      if (is_large_scale_) {
        static int cnt_voxel = 0;
        if (cnt_voxel++ % 500 == 0) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
          *ptr_map = *cloud_map_;
          std::cout << "\033[1;32m Voxelizing submap...\033[0m" << std::endl;
          erasor_utils::voxelize_preserving_labels_by_nanoflann(ptr_map, *cloud_map_, voxel_size_);
          submaps.emplace_back(*cloud_map_);
          cloud_map_->clear();
        }
      }
    }

    if ((count_ % viz_interval_) == 0) {
      erasor2::viz::setFrame(count_);
      CurrCloudPublisher.publish(*cloud_curr_wrt_world_);
      MapCloudPublisher.publish(*cloud_map_);
      PathPublisher.publish(nav_path_);
      // tf2 'world → body' becomes a rerun Transform3D under world/body.
      PosePublisher.publish(pose);
    }
    ++count_;
  }

  void saveAccumMap(const std::string &raw_map_path, const std::string &voxelized_map_path) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_accum(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_accum->points.reserve(NUM_MAP_CLOUD_LARGE_ENOUGH);
    std::cout << "\033[1;32mOn saving map cloud...it may take few seconds...\033[0m" << std::endl;
    if (is_large_scale_) {
      // Prvious submaps
      for (const auto &submap : submaps) {
        *cloud_accum += submap;
      }
      // Remain map
      *cloud_accum += *cloud_map_;
    } else {
      *cloud_accum = *cloud_map_;
    }

    std::filesystem::path dir = std::filesystem::path(raw_map_path).parent_path();
    if (!dir.empty() && !std::filesystem::exists(dir)) {
      std::filesystem::create_directories(dir);
    }
    pcl::io::savePCDFileASCII(raw_map_path, *cloud_accum);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
    ptr_map->points.reserve(cloud_accum->points.size());

    std::cout << "\033[1;32mStart to copy pts...\033[0m" << std::endl;
    *ptr_map = *cloud_accum;
    std::cout << "\033[1;32mOn voxelizing...\033[0m" << std::endl;
    erasor_utils::voxelize_preserving_labels_by_nanoflann(
        ptr_map, *cloud_voxelized, map_voxel_size_);

    cloud_voxelized->width  = cloud_voxelized->points.size();
    cloud_voxelized->height = 1;
    std::cout << "[Debug]: (" << cloud_voxelized->width << ", " << cloud_voxelized->height
              << ") => " << cloud_voxelized->points.size() << std::endl;
    std::cout << "\033[1;32mSaving the map to pcd...\033[0m" << std::endl;
    pcl::io::savePCDFileASCII(voxelized_map_path, *cloud_voxelized);
    std::cout << "\033[1;32mComplete to save the map!:";
    std::cout << voxelized_map_path << "\033[0m" << std::endl;
  }
};

#endif
