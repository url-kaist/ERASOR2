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
    Eigen::Matrix4f tf_h_of_ground_to_be_zero_ = Eigen::Matrix4f::Identity();
    float sensor_height_;
    float robot_body_size_;

    // range / resolution 했을 때 짝수가 되게!
    float grid_resolution_;
    float range_of_interest_;
    float egocentric_grid_resolution_;
    float region_proposal_thr_;
    float min_z_voi_;
    float max_z_voi_;
    float scan_ratio_threshold_;
    float binary_scan_ratio_threshold_;
    float min_len_empty_space_;
    float min_z_diff_thr_;

    float increment_gain_; // Should be larger than one
    float increment_;
    float ratio_num_pts_;

    float negative_log_odds_;
    float obj_score_soft_thr_;
    float obj_score_hard_thr_;
    float hard_thr_radius_;
    float xy_size_thr_;

    float voxel_size_for_pose_correction_;
    float max_corr_dist_for_pose_correction_;

    float minimum_area_thr_;
    float ratio_of_unknown_prior_;

    bool verbose_;
    bool is_large_scale_;
    bool stop_for_each_frame_;

    bool viz_set_scan_and_pose_ = false;
    bool viz_set_submap_ = false;
    bool viz_update_ = false;
    bool viz_detect_ = false;
    bool viz_over_seg_ = false;

    int start_frame_;
    int end_frame_;
    int viz_interval_;
    int accum_interval_;
    int expansion_range_;

    int neighboring_width_;
    int neighboring_height_;
    int kernel_size_;
    int update_interval_;
    int minimum_num_pts_;
    int minimum_num_per_voxel_;

    int window_size_;
    float vor_cand_score_thr_;
    bool use_adaptive_voxel_size_;
    float dist_thr_gain_;

    int num_omp_cores_;

    float voxel_size_;
    float map_voxel_size_;

    bool run_traj_clustering_;
    bool distinguish_temporal_trajectories_;
    bool correct_poses_by_submap_matching_;
    bool correct_poses_;
    bool save_map_;
    string dataset_name_;
    string abs_save_dir_;
    string abs_data_dir_;
    string cloud_dir_;
    string cloud_format_;
    string pose_path_;
    string sequence_;
    string instance_seg_method_;

    // ROS msgs
    nav_msgs::Path nav_path_;

    // ROS publishers
    ros::Publisher PathPublisher;

    ros::Publisher CurrVoIPublisher;
    ros::Publisher MapVoIPublisher;

    ros::Publisher MapCloudPublisher;
    ros::Publisher DynMapPublisher;
    ros::Publisher NoiseMapPublisher;
    ros::Publisher CurrCloudPublisher;
    ros::Publisher DynCurrCloudPublisher;
    ros::Publisher DynInstCurrCloudPublisher;
    ros::Publisher RejectedDynCurrCloudPublisher;
    ros::Publisher OutlierCurrCloudPublisher;
    ros::Publisher AcceptedMovingObjScorePublisher;
    ros::Publisher RejectedMovingObjScorePublisher;
    ros::Publisher NonGroundCurrCloudPublisher;
    ros::Publisher GroundCurrCloudPublisher;
    ros::Publisher NoiseCurrCloudPublisher;
    ros::Publisher StaticCloudPublisher;
    ros::Publisher DynamicCloudPublisher;

    ros::Publisher EgocentricGridPublisher;
    ros::Publisher GridPublisher;
    ros::Publisher AdaptiveRangePublisher;

    ros::Publisher PosePublisher;

    RosParamServer()
    {
        nh_.param<int>("/num_omp_cores", num_omp_cores_, 8);
        nh_.param<int>("/start_frame", start_frame_, 0);
        nh_.param<int>("/end_frame", end_frame_, -1);
        nh_.param<int>("/viz_interval", viz_interval_, 10);
        nh_.param<bool>("/is_large_scale", is_large_scale_, true);
        nh_.param<bool>("/dataloader/run_traj_clustering", run_traj_clustering_, false);
        nh_.param<bool>("/dataloader/distinguish_temporal_trajectories", distinguish_temporal_trajectories_, false);
        nh_.param<string>("/dataloader/dataset_name", dataset_name_, "");
        nh_.param<string>("/dataloader/abs_data_dir", abs_data_dir_, "");
        nh_.param<string>("/dataloader/cloud_dir", cloud_dir_, "");
        nh_.param<string>("/dataloader/cloud_format", cloud_format_,"");
        nh_.param<string>("/dataloader/pose_path", pose_path_, "");
        nh_.param<string>("/dataloader/sequence", sequence_, "");
        nh_.param<string>("/dataloader/abs_save_dir", abs_save_dir_, "/");
        nh_.param<string>("/dataloader/instance_seg_method", instance_seg_method_, "cais");
        nh_.param<int>("/dataloader/accum_interval", accum_interval_, 2);
        nh_.param<int>("/dataloader/expansion_range", expansion_range_, 20);
        nh_.param<float>("/dataloader/voxel_size", voxel_size_, (float) 0.05);
        nh_.param<float>("/dataloader/map_voxel_size", map_voxel_size_, (float) 0.2);

        nh_.param<bool>("/pose_corrector/correct_poses_by_submap_matching", correct_poses_by_submap_matching_, false);
        nh_.param<float>("/pose_corrector/voxel_size", voxel_size_for_pose_correction_, 1.0);
        nh_.param<float>("/pose_corrector/max_corr_dist", max_corr_dist_for_pose_correction_, 2.0);

        /* ERASOR2 parameters */
        nh_.param<float>("/erasor2/min_z_voi", min_z_voi_, -1.6);
        nh_.param<float>("/erasor2/max_z_voi", max_z_voi_, 1.3);
        nh_.param<float>("/erasor2/scan_ratio_threshold", scan_ratio_threshold_, 0.3);
        nh_.param<float>("/erasor2/binary_scan_ratio_threshold", binary_scan_ratio_threshold_, 0.3);
        nh_.param<float>("/erasor2/min_len_empty_space", min_len_empty_space_, 1.8);
        nh_.param<float>("/erasor2/min_z_diff_thr", min_z_diff_thr_, 0.3);

        nh_.param<float>("/erasor2/log_odds/increment_gain", increment_gain_, 0.3);
        nh_.param<float>("/erasor2/log_odds/increment", increment_, 0.3);
        nh_.param<float>("/erasor2/region_proposal_thr", region_proposal_thr_, 0.5);
        nh_.param<int>("/erasor2/kernel_size", kernel_size_, 3);
        nh_.param<int>("/erasor2/update_interval", update_interval_, 1);

        nh_.param<float>("/erasor2/ratio_num_pts", ratio_num_pts_, 0.95);
        nh_.param<int>("/erasor2/minimum_num_pts", minimum_num_pts_, 3);
        nh_.param<int>("/erasor2/minimum_num_per_voxel", minimum_num_per_voxel_, 1);

        nh_.param<float>("/erasor2/moving_object_detection/negative_log_odds", negative_log_odds_, -2.0);
        nh_.param<float>("/erasor2/moving_object_detection/obj_score_soft_thr", obj_score_soft_thr_, 0.8);
        nh_.param<float>("/erasor2/moving_object_detection/obj_score_hard_thr", obj_score_hard_thr_, 20.0);
        nh_.param<float>("/erasor2/moving_object_detection/hard_thr_radius", hard_thr_radius_, 12.0);
        nh_.param<float>("/erasor2/moving_object_detection/xy_size_thr", xy_size_thr_, 30.0);

        nh_.param<float>("/erasor2/over_segmentation/minimum_area_thr", minimum_area_thr_, 0.5);
        nh_.param<float>("/erasor2/over_segmentation/ratio_of_unknown_prior", ratio_of_unknown_prior_, 0.25);

        nh_.param<int>("/erasor2/volumetric_outlier_removal/window_size", window_size_, 1);
        nh_.param<bool>("/erasor2/volumetric_outlier_removal/use_adaptive_voxel_size", use_adaptive_voxel_size_, false);
        nh_.param<float>("/erasor2/volumetric_outlier_removal/vor_cand_score_thr", vor_cand_score_thr_, 3.0);
        nh_.param<float>("/erasor2/volumetric_outlier_removal/dist_thr_gain", dist_thr_gain_, 1.0);

        nh_.param<bool>("/erasor2/save_map", save_map_, false);

        nh_.param<float>("/erasor2/grid_resolution", grid_resolution_, 0.4);
        nh_.param<float>("/erasor2/egocentric_grid_resolution", egocentric_grid_resolution_, 0.2);
        nh_.param<float>("/erasor2/range_of_interest", range_of_interest_, 10.0);
//        nh_.param<int>("/interval", INTERVAL, 2);
        nh_.param<bool>("/stop_for_each_frame", stop_for_each_frame_, false);
//        nh_.param<bool>("/verbose", verbose, false);

        nh_.param<bool>("/erasor2/viz_flag/set_scan_and_pose", viz_set_scan_and_pose_, false);
        nh_.param<bool>("/erasor2/viz_flag/set_submap", viz_set_submap_, false);
        nh_.param<bool>("/erasor2/viz_flag/update", viz_update_, false);
        nh_.param<bool>("/erasor2/viz_flag/detect", viz_detect_, false);
        nh_.param<bool>("/erasor2/viz_flag/over_seg", viz_over_seg_, false);

        neighboring_width_ = static_cast<int>(2.0 * range_of_interest_ / grid_resolution_);
        neighboring_height_ = static_cast<int>(2.0 * range_of_interest_ / grid_resolution_);

        /* Extrinsic */
        nh_.param<float>("extrinsic/robot_body_size", robot_body_size_, 2.7);
        nh_.param<float>("extrinsic/sensor_height", sensor_height_, 1.73);
        nh_.param<vector<float>>("extrinsic/rotation", ext_rot_vec_, vector<float>());
        nh_.param<vector<float>>("extrinsic/translation", ext_trans_vec_, vector<float>());
        ext_rot_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(ext_rot_vec_.data(), 3, 3);
        ext_trans_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(ext_trans_vec_.data(), 3, 1);

        cout << "Extrinsic - Rot" << endl;
        cout << ext_rot_ << endl;
        cout << "Extrinsic - Trans" << endl;
        cout << ext_trans_ << endl;
        extrinsic_.block<3, 3>(0, 0) = ext_rot_;
        extrinsic_.topRightCorner<3, 1>() = ext_trans_;

        PathPublisher  = nh_.advertise<nav_msgs::Path>("/path", 100);

        CurrVoIPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/curr_voi", 100, true);
        MapVoIPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/map_voi", 100, true);

        MapCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/map", 100, true);
        DynMapPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/dyn_points_all", 100, true);
        NoiseMapPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/noise_points_all", 100, true);

        CurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/curr_scan", 100, true);
        DynCurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/dyn_points", 100, true);
        DynInstCurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/dyn_inst_points", 100, true);
        RejectedDynCurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/rejected_dyn_points", 100, true);
        OutlierCurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/outlier_points", 100, true);
        AcceptedMovingObjScorePublisher = nh_.advertise<visualization_msgs::MarkerArray>("/erasor2/accetped_moving_obj_scores", 100, true);
        RejectedMovingObjScorePublisher = nh_.advertise<visualization_msgs::MarkerArray>("/erasor2/rejected_moving_obj_scores", 100, true);

        NonGroundCurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/non_ground_points", 100, true);
        GroundCurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/ground_points", 100, true);
        NoiseCurrCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/noise_points", 100, true);

        StaticCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/static", 100, true);
        DynamicCloudPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/dynamic", 100, true);

        EgocentricGridPublisher = nh_.advertise<grid_map_msgs::GridMap>("/erasor2/gridmap_egocentric", 100, true);
        GridPublisher = nh_.advertise<grid_map_msgs::GridMap>("/erasor2/gridmap", 100, true);
        AdaptiveRangePublisher = nh_.advertise<visualization_msgs::Marker>("/erasor2/adaptive_range", 100, true);

        PosePublisher = nh_.advertise<geometry_msgs::PoseStamped>("/erasor2/pose", 100, true);

//        pub_map_rejected  = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/map_rejected", 100);
//        pub_curr_rejected = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/curr_rejected", 100);
//        pub_map_init      = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/map_init", 100);
//        pub_curr_init     = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/curr_init", 100);
//
//        pub_map_marker     = nh_.advertise<jsk_recognition_msgs::PolygonArray>("/erasor2/map_marker", 100);
//        pub_curr_marker    = nh_.advertise<jsk_recognition_msgs::PolygonArray>("/erasor2/curr_marker", 100);
//        pub_viz_bin_marker = nh_.advertise<jsk_recognition_msgs::PolygonArray>("/erasor2/polygons_marker", 100);
//        pub_h_marker  = nh_.advertise<visualization_msgs::MarkerArray>("/pseudo_occupancy_array", 100, true);
//
//        pub_ground   = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/ground", 100);
//        pub_arranged = nh_.advertise<sensor_msgs::PointCloud2>("/erasor2/arranged", 100);
//
        std::string home_dir = std::getenv("HOME");
        abs_data_dir_ = home_dir + abs_data_dir_;
        abs_save_dir_ = home_dir + abs_save_dir_;
        pose_path_ = home_dir + pose_path_;

        tf_h_of_ground_to_be_zero_(2, 3) = sensor_height_;
        usleep(100);
    }

    ~RosParamServer() {}
};
#endif

