#ifndef ERASOR2_MAPGEN_H
#define ERASOR2_MAPGEN_H

#include "rosparam_server.hpp"
#include <math.h>

#define NUM_MAP_CLOUD 30000000

const char separator   = ' ';
const int  print_width = 10;

class Mapgen : public RosParamServer {
public:
    bool is_initial  = true;
    int  count       = 0;
    int  accum_count = 0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_curr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map;

    // For large-scale map building
    // ROS can publish point cloud whose volume is under the 1 GB
    std::vector<pcl::PointCloud<pcl::PointXYZI> > cloud_maps;

    nav_msgs::Path odom_path;
    float          leaf_size;
    std::string    save_path_;

    Mapgen() {
        cloud_curr.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_map.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_curr->points.reserve(200000); // Vel 64 HDE - almost 130,000
        cloud_map->points.reserve(NUM_MAP_PC_LARGE_ENOUGH);

        std::cout << "\033[1;32m";
        std::cout << "[MAPGEN]: Voxelization size - " << voxel_size_ << std::endl;
        std::cout << "[MAPGEN]: Target seq -  " << sequence_ << std::endl;
        std::cout << "[MAPGEN]: From " << init_idx_ << ", " << end_idx_ << std::endl;
        std::cout << "[MAPGEN]: Is the map large-scale? " << (is_large_scale_ ? "Yes" : "No") << "\033[0m" << std::endl;
    }

    ~Mapgen() {}

    void setSavePath() {
        save_path_ = "";
    }

    void accumPointCloud(
            const erasor::node &data, nav_msgs::Path &path) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header          = data.header;
        pose_stamped.header.frame_id = "/map";
        pose_stamped.pose            = data.odom;

        odom_path.header = pose_stamped.header;
        odom_path.poses.push_back(pose_stamped);
//    std::cout<<erasor_utilsgeoPose2eigen(data.odom)<<std::endl;
        path = odom_path;

        pcl::PointCloud<pcl::PointXYZI> cloud = erasor_utils::cloudmsg2cloud<pcl::PointXYZI>(data.lidar);

        // To remove some noisy points in the vicinity of the vehicles
        pcl::PointCloud<pcl::PointXYZI> inliers, outliers; // w.r.t origin
        float                           max_dist_square = pow(CAR_BODY_SIZE, 2);
        for (auto const                 &pt: cloud.points) {
            double dist_square = pow(pt.x, 2) + pow(pt.y, 2);
            if (dist_square < max_dist_square) {
                inliers.push_back(pt);
            } else {
                outliers.push_back(pt);
            }
        }
        cloud = outliers;

        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(cloud, *ptr_transformed, tf_h_of_ground_to_be_zero);

        Eigen::Matrix4f pose = erasor_utils::geoPose2eigen(data.odom);
        std::cout << std::setprecision(3) << std::left << setw(print_width) << setfill(separator) << "=> [Pose] "
                  << pose(0, 3) << ", " << pose(1, 3) << ", " << pose(2, 3) << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*ptr_transformed, *world_transformed, pose);

        erasor_utils::voxelize_preserving_labels(world_transformed, cloud_curr, 0.2);

        if (is_initial) {
            cloud_map  = cloud_curr;
            is_initial = false;
        } else {
            cloud_map += cloud_curr;

            if (is_large_scale) {
                static int cnt_voxel = 0;
                if (cnt_voxel++ % 500 == 0) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
                    *ptr_map = cloud_map;
                    std::cout << "\033[1;32m Voxelizing submap...\033[0m" << std::endl;
                    erasor_utils::voxelize_preserving_labels(ptr_map, cloud_map, leafsize);

                    cloud_maps.push_back(*cloud_map);
                    cloud_map->clear();
                }
            }
            ++accum_count;
        }

    }

    void getPointClouds(pcl::PointCloud<pcl::PointXYZI>::Ptr map_out,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr curr_out) {
        *map_out  = cloud_map;
        *curr_out = cloud_curr;
    }

    void saveNaiveMap(const std::string &original_dir, const std::string &map_dir) {
        pcl::PointCloud<pcl::PointXYZI> cloud_src;

        std::cout << "\033[1;32m On saving map cloud...it may take few seconds...\033[0m" << std::endl;
        if (is_large_scale) {
            // Prvious submaps
            for (const auto &submap: cloud_maps) {
                cloud_src += submap;
            }
            // Remain map
            cloud_src += cloud_map;
        } else {
            cloud_src = cloud_map;
        }

        pcl::io::savePCDFileASCII(original_dir, cloud_src);

        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>      cloud_out;
        ptr_map->points.reserve(cloud_src.points.size());

        std::cout << "\033[1;32m Start to copy pts...\033[0m" << std::endl;
        *ptr_map = cloud_src;
        std::cout << "[Debug]: " << cloud_src.width << ", " << cloud_src.height << ", " << cloud_src.points.size()
                  << std::endl;
        std::cout << "\033[1;32m On voxelizing...\033[0m" << std::endl;
        erasor_utils::voxelize_preserving_labels(ptr_map, cloud_out, leafsize);

        cloud_out.width  = cloud_out.points.size();
        cloud_out.height = 1;
        std::cout << "[Debug]: " << cloud_out.width << ", " << cloud_out.height << ", " << cloud_out.points.size()
                  << std::endl;
        std::cout << "\033[1;32m Saving the map to pcd...\033[0m" << std::endl;
        pcl::io::savePCDFileASCII(map_dir, cloud_out);
        std::cout << "\033[1;32m Complete to save the map!:";
        std::cout << map_dir << "\033[0m" << std::endl;

    }
};

#endif

