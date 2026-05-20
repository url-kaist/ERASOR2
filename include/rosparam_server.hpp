#ifndef ERASOR2_ROSPARAM_SERVER_H
#define ERASOR2_ROSPARAM_SERVER_H

// Despite the legacy filename, this header is now ROS-free. It keeps the
// `RosParamServer` name purely so the call sites that say
// `class ERASOR2 : public RosParamServer` don't have to change in this
// migration step. Rename in a follow-up once the dust settles.
//
// Field semantics are preserved exactly: every name that used to be
// populated by `nh_.param("/...", x_, default)` is now populated by
// reading the same key out of a YAML config. Every `ros::Publisher` field
// is now a `viz::Publisher` (rerun) shim with the same `.publish(...)`
// surface.

#include <math.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <boost/format.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "erasor2/Config.hpp"
#include "erasor2/RerunLogger.hpp"
#include "erasor2/grid_map.hpp"

#include "tools/erasor_utils.hpp"

class RosParamServer {
 public:
  // Hardware extrinsics
  std::vector<float> ext_rot_vec_;
  std::vector<float> ext_trans_vec_;
  Eigen::Matrix3f ext_rot_;
  Eigen::Vector3f ext_trans_;
  Eigen::Matrix4f extrinsic_                 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tf_h_of_ground_to_be_zero_ = Eigen::Matrix4f::Identity();
  float sensor_height_;
  float robot_body_size_;

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

  float increment_gain_;
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
  bool viz_set_submap_        = false;
  bool viz_update_            = false;
  bool viz_detect_            = false;
  bool viz_over_seg_          = false;

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
  std::string dataset_name_;
  std::string abs_save_dir_;
  std::string abs_data_dir_;
  std::string cloud_dir_;
  std::string cloud_format_;
  std::string pose_path_;
  std::string sequence_;
  std::string instance_seg_method_;

  // Replaces nav_msgs::Path. Stored as a sequence of body poses; rerun
  // logs it as a 3D line strip at publish time.
  std::vector<Eigen::Matrix4f> nav_path_;

  // Replaces ros::Publisher fields. Each one has the same .publish(...)
  // surface so existing call sites keep their syntax.
  erasor2::viz::Publisher PathPublisher;

  erasor2::viz::Publisher CurrVoIPublisher;
  erasor2::viz::Publisher MapVoIPublisher;

  erasor2::viz::Publisher MapCloudPublisher;
  erasor2::viz::Publisher DynMapPublisher;
  erasor2::viz::Publisher NoiseMapPublisher;
  erasor2::viz::Publisher CurrCloudPublisher;
  erasor2::viz::Publisher DynCurrCloudPublisher;
  erasor2::viz::Publisher DynInstCurrCloudPublisher;
  erasor2::viz::Publisher RejectedDynCurrCloudPublisher;
  erasor2::viz::Publisher OutlierCurrCloudPublisher;
  erasor2::viz::TextArrayPublisher AcceptedMovingObjScorePublisher;
  erasor2::viz::TextArrayPublisher RejectedMovingObjScorePublisher;
  erasor2::viz::Publisher NonGroundCurrCloudPublisher;
  erasor2::viz::Publisher GroundCurrCloudPublisher;
  erasor2::viz::Publisher NoiseCurrCloudPublisher;
  erasor2::viz::Publisher StaticCloudPublisher;
  erasor2::viz::Publisher DynamicCloudPublisher;

  erasor2::viz::GridMapPublisher EgocentricGridPublisher;
  erasor2::viz::GridMapPublisher GridPublisher;
  erasor2::viz::Publisher AdaptiveRangePublisher;

  erasor2::viz::Publisher PosePublisher;

  // The legacy ctor read params from the ROS parameter server. The new
  // path is: caller loads a Config (via Config::fromYaml or hand-builds
  // one), then constructs this base. Single overload — using a default
  // value rather than two ctors avoids ambiguous-resolution at derived
  // classes that also define a default ctor.
  explicit RosParamServer(const erasor2::Config& cfg) {
    num_omp_cores_                     = cfg.num_omp_cores;
    start_frame_                       = cfg.start_frame;
    end_frame_                         = cfg.end_frame;
    viz_interval_                      = cfg.viz_interval;
    is_large_scale_                    = cfg.is_large_scale;
    run_traj_clustering_               = cfg.run_traj_clustering;
    distinguish_temporal_trajectories_ = cfg.distinguish_temporal_trajectories;
    dataset_name_                      = cfg.dataset_name;
    abs_data_dir_                      = cfg.abs_data_dir;
    cloud_dir_                         = cfg.cloud_dir;
    cloud_format_                      = cfg.cloud_format;
    pose_path_                         = cfg.pose_path;
    sequence_                          = cfg.sequence;
    abs_save_dir_                      = cfg.abs_save_dir;
    instance_seg_method_               = cfg.instance_seg_method;
    accum_interval_                    = cfg.accum_interval;
    expansion_range_                   = cfg.expansion_range;
    voxel_size_                        = cfg.voxel_size;
    map_voxel_size_                    = cfg.map_voxel_size;

    correct_poses_by_submap_matching_  = cfg.correct_poses_by_submap_matching;
    voxel_size_for_pose_correction_    = cfg.voxel_size_for_pose_correction;
    max_corr_dist_for_pose_correction_ = cfg.max_corr_dist_for_pose_correction;

    min_z_voi_                   = cfg.min_z_voi;
    max_z_voi_                   = cfg.max_z_voi;
    scan_ratio_threshold_        = cfg.scan_ratio_threshold;
    binary_scan_ratio_threshold_ = cfg.binary_scan_ratio_threshold;
    min_len_empty_space_         = cfg.min_len_empty_space;
    min_z_diff_thr_              = cfg.min_z_diff_thr;

    increment_gain_        = cfg.increment_gain;
    increment_             = cfg.increment;
    region_proposal_thr_   = cfg.region_proposal_thr;
    kernel_size_           = cfg.kernel_size;
    update_interval_       = cfg.update_interval;
    ratio_num_pts_         = cfg.ratio_num_pts;
    minimum_num_pts_       = cfg.minimum_num_pts;
    minimum_num_per_voxel_ = cfg.minimum_num_per_voxel;

    negative_log_odds_  = cfg.negative_log_odds;
    obj_score_soft_thr_ = cfg.obj_score_soft_thr;
    obj_score_hard_thr_ = cfg.obj_score_hard_thr;
    hard_thr_radius_    = cfg.hard_thr_radius;
    xy_size_thr_        = cfg.xy_size_thr;

    minimum_area_thr_       = cfg.minimum_area_thr;
    ratio_of_unknown_prior_ = cfg.ratio_of_unknown_prior;

    window_size_             = cfg.window_size;
    use_adaptive_voxel_size_ = cfg.use_adaptive_voxel_size;
    vor_cand_score_thr_      = cfg.vor_cand_score_thr;
    dist_thr_gain_           = cfg.dist_thr_gain;

    save_map_                   = cfg.save_map;
    grid_resolution_            = cfg.grid_resolution;
    egocentric_grid_resolution_ = cfg.egocentric_grid_resolution;
    range_of_interest_          = cfg.range_of_interest;
    stop_for_each_frame_        = cfg.stop_for_each_frame;
    verbose_                    = cfg.verbose;

    viz_set_scan_and_pose_ = cfg.viz_set_scan_and_pose;
    viz_set_submap_        = cfg.viz_set_submap;
    viz_update_            = cfg.viz_update;
    viz_detect_            = cfg.viz_detect;
    viz_over_seg_          = cfg.viz_over_seg;

    neighboring_width_  = cfg.neighboring_width;
    neighboring_height_ = cfg.neighboring_height;

    robot_body_size_           = cfg.robot_body_size;
    sensor_height_             = cfg.sensor_height;
    ext_rot_vec_               = cfg.ext_rot_vec;
    ext_trans_vec_             = cfg.ext_trans_vec;
    ext_rot_                   = cfg.ext_rot;
    ext_trans_                 = cfg.ext_trans;
    extrinsic_                 = cfg.extrinsic;
    tf_h_of_ground_to_be_zero_ = cfg.tf_h_of_ground_to_be_zero;

    std::cout << "Extrinsic - Rot\n" << ext_rot_ << "\n";
    std::cout << "Extrinsic - Trans\n" << ext_trans_ << "\n";

    // Hook up the rerun publisher shims with their entity paths. Names
    // mirror the original ROS topic names so the rerun viewer tree is
    // visually similar to the old RViz layout.
    PathPublisher                 = erasor2::viz::Publisher("world/path");
    CurrVoIPublisher              = erasor2::viz::Publisher("erasor2/curr_voi");
    MapVoIPublisher               = erasor2::viz::Publisher("erasor2/map_voi");
    MapCloudPublisher             = erasor2::viz::Publisher("erasor2/map");
    DynMapPublisher               = erasor2::viz::Publisher("erasor2/dyn_points_all");
    NoiseMapPublisher             = erasor2::viz::Publisher("erasor2/noise_points_all");
    CurrCloudPublisher            = erasor2::viz::Publisher("erasor2/curr_scan");
    DynCurrCloudPublisher         = erasor2::viz::Publisher("erasor2/dyn_points");
    DynInstCurrCloudPublisher     = erasor2::viz::Publisher("erasor2/dyn_inst_points");
    RejectedDynCurrCloudPublisher = erasor2::viz::Publisher("erasor2/rejected_dyn_points");
    OutlierCurrCloudPublisher     = erasor2::viz::Publisher("erasor2/outlier_points");
    AcceptedMovingObjScorePublisher =
        erasor2::viz::TextArrayPublisher("erasor2/accepted_obj_scores");
    RejectedMovingObjScorePublisher =
        erasor2::viz::TextArrayPublisher("erasor2/rejected_obj_scores");
    NonGroundCurrCloudPublisher = erasor2::viz::Publisher("erasor2/non_ground_points");
    GroundCurrCloudPublisher    = erasor2::viz::Publisher("erasor2/ground_points");
    NoiseCurrCloudPublisher     = erasor2::viz::Publisher("erasor2/noise_points");
    StaticCloudPublisher        = erasor2::viz::Publisher("erasor2/static");
    DynamicCloudPublisher       = erasor2::viz::Publisher("erasor2/dynamic");

    EgocentricGridPublisher = erasor2::viz::GridMapPublisher("erasor2/gridmap_egocentric");
    GridPublisher           = erasor2::viz::GridMapPublisher("erasor2/gridmap");
    AdaptiveRangePublisher  = erasor2::viz::Publisher("erasor2/adaptive_range");
    PosePublisher           = erasor2::viz::Publisher("world/body");
  }

  ~RosParamServer() {}
};

#endif  // ERASOR2_ROSPARAM_SERVER_H
