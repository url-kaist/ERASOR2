#ifndef ERASOR_UTILS_H
#define ERASOR_UTILS_H

#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cmath>
#include <string>
#include <map>
#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>

#include<omp.h>
#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include "signal.h"
#include "nanoflann/nanoflann.hpp"
#include "nanoflann/nanoflann_utils.hpp"
#include "tools/hash_voxel_grid.hpp"

// Point-wise label
#define NOT_INTEREST 0 // ground and noisy points in the estimated labels
#define GROUND_LABEL -1
#define NOISE_LABEL -2
#define NOT_VOLUME_OF_INTEREST -3
#define VOLUME_OF_INTEREST -4

// Gridmap characteristics
#define NOT_OBSERVED 100.0
#define GROUND_EXISTS 101.0 // Must be larger than `NOT_OBSERVED`
#define TEMPORARILY_OCCUPIED 102.0 // Must be larger than `GROUND_EXISTS`

// Just for visualization
#define NOT_UPDATED -2.3

#define IS_STATIC 100000
#define IS_DYNAMIC 100001
#define IS_NOISE_YET_POTENTIAL_DYNAMIC 100002

#define DYNAMIC_LABEL 251
#define STATIC_LABEL 9

using namespace std;

using num_t = float;
// construct a kd-tree index:

namespace erasor_utils {
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
        PointCloud<num_t>, 3 /* dim */
    >;

    template<typename T>
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
    {
        sensor_msgs::PointCloud2 cloud_ROS;
        pcl::toROSMsg(cloud, cloud_ROS);
        cloud_ROS.header.frame_id = frame_id;
        return cloud_ROS;
    }

    template<typename T>
    pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
    {
        pcl::PointCloud<T> cloudresult;
        pcl::fromROSMsg(cloudmsg,cloudresult);
        return cloudresult;
    }

    template<typename T>
    void cloudmsg2cloudptr(sensor_msgs::PointCloud2 cloudmsg,boost::shared_ptr< pcl::PointCloud< T > > cloudPtr)
    {
        pcl::fromROSMsg(cloudmsg,*cloudPtr);
    }


    template<typename T>
    int load_pcd(std::string pcd_name, boost::shared_ptr<pcl::PointCloud<T> > dst) {
        std::cout << "Loading point cloud..." << std::endl;
        if (pcl::io::loadPCDFile<T>(pcd_name, *dst) == -1) {
            PCL_ERROR ("Couldn't read file!!! \n");
            return (-1);
        }
//        std::cout << "Complete!" << std::endl;
//        std::cout << "Loaded " << dst->size() << " data points from " << pcd_name << std::endl;
        return 0;
    }

    template<typename T>
    int load_pcd(std::string pcd_name, pcl::PointCloud<T>& dst) {
        if (pcl::io::loadPCDFile<T>(pcd_name, dst) == -1) {
            PCL_ERROR ("Couldn't read file!!! \n");
            return (-1);
        }
        std::cout << "Loaded " << dst.size() << " data points from " << pcd_name << std::endl;
        return 0;
    }

    template<typename T>
    void save_dyn_label(const std::string abs_dir, const int frame_num, 
                      const pcl::PointCloud<T> &cloud_raw,
                      const pcl::PointCloud<T> &cloud_est_dyn,
                        const pcl::PointCloud<T> &cloud_est_potential_dyn){

    // Following SemanticKITTI's MOS Label format, Dynamic => 251 | Static => 0 
    // This function is derived from `savelabel` in patchwork
    // Save the estimate dynamic points into a `.label` file

    const float SQR_EPSILON = 0.1; // for better gathering of dynamic points

    int num_cloud_raw = cloud_raw.points.size();
    std::vector<uint32_t> labels(num_cloud_raw, 0); // 0: static or points

    int N_dyn = cloud_est_dyn.points.size();
    int N_potential_dyn = cloud_est_potential_dyn.points.size();

    PointCloud<num_t> cloud;

    cloud.pts.resize(N_dyn + N_potential_dyn);
    for (size_t i = 0; i < N_dyn; i++) {
        cloud.pts[i].x = cloud_est_dyn.points[i].x;
        cloud.pts[i].y = cloud_est_dyn.points[i].y;
        cloud.pts[i].z = cloud_est_dyn.points[i].z;
    }
    for (size_t i = 0; i < N_potential_dyn; i++) {
        cloud.pts[i].x = cloud_est_potential_dyn.points[i].x;
        cloud.pts[i].y = cloud_est_potential_dyn.points[i].y;
        cloud.pts[i].z = cloud_est_potential_dyn.points[i].z;
    }
    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
            PointCloud<num_t>, 3 >;

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

    int num_valid = 0;
    for (int j = 0; j < cloud_raw.points.size(); ++j) {
        const auto query_pcl = cloud_raw.points[j];
        const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};

        size_t num_results = 1;
        std::vector<uint32_t> ret_index(num_results);
        std::vector<num_t> out_dist_sqr(num_results);

        num_results = index.knnSearch(
                &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

        ret_index.resize(num_results);
        out_dist_sqr.resize(num_results);
        if(out_dist_sqr[0] < SQR_EPSILON) { // it is in the same voxel point
            labels[j] = 251; // DYNAMIC_LABEL
            ++num_valid;
        }
    }

    // Must be equal to the # of above-ground points
//    std::cout << "# of valid points: " << num_valid << std::endl;

    //  To follow the KITTI format, # of zeros are set to 6
    const int NUM_ZEROS = 6;

    std::string count_str = std::to_string(frame_num);
    std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
    std::string abs_label_path = abs_dir + "/" + count_str_padded + ".label";

    std::cout << "\033[1;32m" << abs_label_path << "\033[0m" << std::endl;
    std::ofstream output_file(abs_label_path, std::ios::out | std::ios::binary);
    output_file.write(reinterpret_cast<char*>(&labels[0]), num_cloud_raw * sizeof(uint32_t));

    }

    std::string format(float f, int digits);

    bool load_labels(const std::string& label_name, std::vector<uint32_t>& labels);

    void findCorrespondences(const pcl::PointCloud<pcl::PointXYZI> &query_cloud, const pcl::PointCloud<pcl::PointXYZI> &target_cloud,
                     vector<int>& correspondences);

    // For visualization of TPs and FPs
    void findEmptyCorrespondences(const pcl::PointCloud<pcl::PointXYZI> &query_cloud, const pcl::PointCloud<pcl::PointXYZI> &target_cloud,
                     vector<int>& correspondences, const float margin=0.02);

    void radiusSearch(const pcl::PointCloud<pcl::PointXYZI> &query_cloud, const pcl::PointCloud<pcl::PointXYZI> &target_cloud,
                     const float radius, vector<pair<int, vector<int>>>& correspondences);

    void radiusSearch(const pcl::PointCloud<pcl::PointXYZI> &query_cloud, const pcl::PointCloud<pcl::PointXYZI> &target_cloud,
                     const float radius, vector<int>& target_idxes);

    void radiusSearchWithAdaptiveRadii(const pcl::PointCloud<pcl::PointXYZI> &query_cloud, const pcl::PointCloud<pcl::PointXYZI> &target_cloud,
                     const vector<float>& radii, vector<int>& target_idxes);


    void fillGTLabel(const pcl::PointCloud<pcl::PointXYZI> &gt_cloud, pcl::PointCloud<pcl::PointXYZI> &est_cloud,
                     const float margin=0.02);

    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose);

    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose);

    void parseStaticAndDynamic(
            const pcl::PointCloud<pcl::PointXYZI> &cloud, pcl::PointCloud<pcl::PointXYZI> &dynamic_points,
            pcl::PointCloud<pcl::PointXYZI> &static_points);

    void voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size);

    void pcl2nanoflann(const pcl::PointCloud<pcl::PointXYZI>& src_cloud, PointCloud<num_t>& cloud);

    void voxelize_preserving_labels_by_nanoflann(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
                                                 pcl::PointCloud<pcl::PointXYZI> &dst, const double leaf_size,
                                                 const int minimum_num_pts_per_voxel=0, const bool verbose=false);

    void count_stat_dyn(const pcl::PointCloud<pcl::PointXYZI> &cloudIn, int &num_static, int &num_dynamic);

    void signal_callback_handler(int signum);

    visualization_msgs::Marker setVisualMarker(const float voxel_size, const float pos_x, const float pos_y);

    void calcMinMaxXY(const vector<pcl::PointCloud<pcl::PointXYZI>> &pcs, float &min_x, float &min_y, float &max_x,
                      float &max_y);

    void calcMinMaxZ(const pcl::PointCloud<pcl::PointXYZI> &pcs, float &min_z, float &max_z);

    void calcMinMaxZWithoutGround(const pcl::PointCloud<pcl::PointXYZI> &pcs, float &min_z, float &max_z);

    float calcMeanZOfGround(const pcl::PointCloud<pcl::PointXYZI>& pcs);

//    void calcMinMaxZ(const pcl::PointCloud<pcl::PointXYZI>& pcs, float& min_z, float& max_z);

    int getNumGroundPoints(const pcl::PointCloud<pcl::PointXYZI>& pc);

    std::tuple<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointXYZI>>
            clusterIndices2PointCloud(const vector<Eigen::Vector3f>& positions,
                                      const vector<vector<size_t>>& cluster_indices);

    vector<uint8_t> getRandomColor();
}
#endif // ERASOR_UTILS_H
