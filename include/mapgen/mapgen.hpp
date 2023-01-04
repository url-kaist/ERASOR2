#ifndef ERASOR2_MAPGEN_H
#define ERASOR2_MAPGEN_H

#include "rosparam_server.hpp"
#include <math.h>

#define NUM_MAP_CLOUD_LARGE_ENOUGH 30000000

const char separator   = ' ';
const int  print_width = 10;

class Mapgen : public RosParamServer {
public:
    bool is_initial_ = true;
    int  count_      = 0;

    // For large-scale map building
    // ROS can only publish point cloud whose volume is under the 1 GB
    std::vector<pcl::PointCloud<pcl::PointXYZI> > submaps;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_curr_wrt_world_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map_;

    std::string save_path_;

    Mapgen() {
        cloud_curr_wrt_world_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);

        std::cout << "\033[1;32m";
        std::cout << "[MAPGEN]: Voxelization size - " << voxel_size_ << std::endl;
        std::cout << "[MAPGEN]: Target seq -  " << sequence_ << std::endl;
        std::cout << "[MAPGEN]: From " << start_frame_ << ", " << end_frame_ << std::endl;
        std::cout << "[MAPGEN]: Is the map large-scale? " << (is_large_scale_ ? "Yes" : "No") << "\033[0m" << std::endl;
    }

    ~Mapgen() {}

    void accumPointCloud(
            const pcl::PointCloud<pcl::PointXYZI> &cloud,
            const Eigen::Matrix4f &pose) {
        // 1. Set pose
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp    = ros::Time::now();
        pose_stamped.header.frame_id = "/map";
        pose_stamped.pose            = erasor_utils::eigen2geoPose(pose);

        nav_path_.header = pose_stamped.header;
        nav_path_.poses.emplace_back(pose_stamped);

        // NOTE: `tf_h_of_ground_to_be_zero_` is not necessary, but we follow the legacy of ERASOR 1.0
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(cloud, *ptr_transformed, tf_h_of_ground_to_be_zero_);

        std::cout << std::setprecision(3) << std::left << setw(print_width) << setfill(separator) << "=> [Pose] "
                  << pose(0, 3) << ", " << pose(1, 3) << ", " << pose(2, 3) << std::endl;

        pcl::PointCloud<pcl::PointXYZI>::Ptr world_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*ptr_transformed, *world_transformed, pose);

        erasor_utils::voxelize_preserving_labels_by_nanoflann(world_transformed, *cloud_curr_wrt_world_, voxel_size_);

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
            CurrCloudPublisher.publish(erasor_utils::cloud2msg(*cloud_curr_wrt_world_));
            MapCloudPublisher.publish(erasor_utils::cloud2msg(*cloud_map_));
            PathPublisher.publish(nav_path_);
        }
        ++count_;
    }

    void saveAccumMap(const std::string &raw_map_path, const std::string &voxelized_map_path) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_accum(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_accum->points.reserve(NUM_MAP_CLOUD_LARGE_ENOUGH);
        std::cout << "\033[1;32m On saving map cloud...it may take few seconds...\033[0m" << std::endl;
        if (is_large_scale_) {
            // Prvious submaps
            for (const auto &submap: submaps) {
                *cloud_accum += submap;
            }
            // Remain map
            *cloud_accum += *cloud_map_;
        } else {
            *cloud_accum = *cloud_map_;
        }

        pcl::io::savePCDFileASCII(raw_map_path, *cloud_accum);

        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
        ptr_map->points.reserve(cloud_accum->points.size());

        std::cout << "\033[1;32mStart to copy pts...\033[0m" << std::endl;
        *ptr_map = *cloud_accum;
        std::cout << "\033[1;32mOn voxelizing...\033[0m" << std::endl;
        erasor_utils::voxelize_preserving_labels_by_nanoflann(ptr_map, *cloud_voxelized, map_voxel_size_);

        cloud_voxelized->width  = cloud_voxelized->points.size();
        cloud_voxelized->height = 1;
        std::cout << "[Debug]: (" << cloud_voxelized->width << ", " << cloud_voxelized->height << ") => "
                  << cloud_voxelized->points.size()
                  << std::endl;
        std::cout << "\033[1;32mSaving the map to pcd...\033[0m" << std::endl;
        pcl::io::savePCDFileASCII(voxelized_map_path, *cloud_voxelized);
        std::cout << "\033[1;32mComplete to save the map!:";
        std::cout << voxelized_map_path << "\033[0m" << std::endl;
    }
};

#endif

