#ifndef ERASOR2_ROSPARAM_SERVER_H
#define ERASOR2_ROSPARAM_SERVER_H

#include "tools/erasor_utils.hpp"
#include <math.h>
#include <boost/format.hpp>
#include <std_msgs/Int8.h>
#include <cstdlib>
#include <algorithm>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

class RosParamServer
{
public:
    ros::NodeHandle nh_;

    // Hardware extrinsics
    vector<float> ext_rot_vec_;
    vector<float> ext_trans_vec_;
    Eigen::Matrix3f ext_rot_;
    Eigen::Vector3f ext_trans_;
    Eigen::Matrix4f extrinsic_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tf_h_of_ground_to_be_zero = Eigen::Matrix4f::Identity();
    float sensor_height_;
    float robot_body_size_;

    // range / resolution 했을 때 짝수가 되게!
    float grid_resolution_;
    float range_of_interest_;
    float egocentric_grid_resolution_;
    float ground_likelihood_thr_;
    float min_z_voi_;
    float max_z_voi_;
    float scan_ratio_threshold_;
    float th_bin_max_h_;

    bool verbose_;
    bool is_large_scale_;

    int init_idx_;
    int end_idx_;
    int viz_interval_;

    float voxel_size_;

    string dataset_name_;
    string cloud_dir_;
    string cloud_format_;
    string pose_path_;
    string sequence_;

    ros::Publisher PathPublisher;

    ros::Publisher MapCloudPublisher;
    ros::Publisher DynMapPublisher;
    ros::Publisher CurrCloudPublisher;
    ros::Publisher DynCurrCloudPublisher;

    ros::Publisher EgocentricGridPublisher;
    ros::Publisher GridPublisher;
    ros::Publisher VizMarkerPublisher;

    RosParamServer()
    {
        nh_.param<string>("dataloader/dataset_name", dataset_name_, "");
        nh_.param<string>("dataloader/cloud_dir", cloud_dir_, "");
        nh_.param<string>("dataloader/cloud_format", cloud_format_,"");
        nh_.param<string>("dataloader/pose_path", pose_path_, "");
        nh_.param<string>("dataloader/sequence", sequence_, "");

        nh_.param<int>("init_idx", init_idx_, 0);
        nh_.param<int>("end_idx", end_idx_, -1);
        nh_.param<int>("viz_interval", viz_interval_, 10);

        nh_.param<bool>("is_large_scale", is_large_scale_, true);

        nh_.param<float>("voxel_size", voxel_size_, (float) 0.05);
//        nh_.param<std::string>("save_path", save_path, "/");

        nh_.param<float>("/erasor2/min_z_voi", min_z_voi_, -1.6);
        nh_.param<float>("/erasor2/max_z_voi", max_z_voi_, 1.3);
        nh_.param<float>("/erasor2/scan_ratio_threshold", scan_ratio_threshold_, 0.3);
        nh_.param<float>("/erasor2/th_bin_max_h", th_bin_max_h_, 0.3);

        // Extrinsic
        nh_.param<float>("extrinsic/robot_body_size", robot_body_size_, 2.7);
        nh_.param<float>("extrinsic/sensor_height", sensor_height_, 1.7);
        nh_.param<vector<float>>("extrinsic/rotation", ext_rot_vec_, vector<float>());
        nh_.param<vector<float>>("extrinsic/translation", ext_trans_vec_, vector<float>());
        ext_rot_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(ext_rot_vec_.data(), 3, 3);
        ext_trans_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(ext_trans_vec_.data(), 3, 1);

        MapCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/map", 100, true);
        DynMapPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/dyn_points_all", 100, true);
        CurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/curr_scan", 100, true);
        DynCurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/dyn_points", 100, true);

        EgocentricGridPublisher = nh_.advertise<grid_map_msgs::GridMap>("/erasor2/gridmap_egocentric", 100, true);
        GridPublisher = nh_.advertise<grid_map_msgs::GridMap>("/erasor2/gridmap", 100, true);
        VizMarkerPublisher = nh_.advertise<visualization_msgs::Marker>("/erasor2/target_grid_loc", 100, true);

        // On ERASOR2
//        nh_.param<string>("/data_dir", DATA_DIR, "/");
//        nh_.param<float>("/voxel_size", VOXEL_SIZE, 0.075);
//        nh_.param<float>("/grid_resolution", grid_resolution, 0.4);
//        nh_.param<float>("/egocentric_grid_resolution", egocentric_grid_resolution, 0.2);
//        nh_.param<float>("/range_of_interest", range_of_interest, 10.0);
//        nh_.param<float>("/ground_likelihood_thr", ground_likelihood_thr, 0.5);
//        nh_.param<int>("/interval", INTERVAL, 2);
//        nh_.param<bool>("/stop_for_each_frame", STOP_FOR_EACH_FRAME, false);
//        nh_.param<bool>("/verbose", verbose, false);

        tf_h_of_ground_to_be_zero(2, 3) = sensor_height_;
        usleep(100);
    }
};
#endif

