#ifndef ERASOR2_CONFIG_HPP
#define ERASOR2_CONFIG_HPP

// Plain config struct loaded from YAML. Replaces RosParamServer's nh_.param
// reads. Field names mirror the original 1:1 so call sites only have to swap
// `params->foo_` for `config.foo_`.

#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace erasor2 {

struct Config {
  // Hardware extrinsics
  std::vector<float> ext_rot_vec;
  std::vector<float> ext_trans_vec;
  Eigen::Matrix3f ext_rot                   = Eigen::Matrix3f::Identity();
  Eigen::Vector3f ext_trans                 = Eigen::Vector3f::Zero();
  Eigen::Matrix4f extrinsic                 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tf_h_of_ground_to_be_zero = Eigen::Matrix4f::Identity();
  float sensor_height                       = 1.73f;
  float robot_body_size                     = 2.7f;

  // Grid / VoI
  float grid_resolution             = 0.4f;
  float range_of_interest           = 10.0f;
  float egocentric_grid_resolution  = 0.2f;
  float region_proposal_thr         = 0.5f;
  float min_z_voi                   = -1.6f;
  float max_z_voi                   = 1.3f;
  float scan_ratio_threshold        = 0.3f;
  float binary_scan_ratio_threshold = 0.3f;
  float min_len_empty_space         = 1.8f;
  float min_z_diff_thr              = 0.3f;

  // Log-odds
  float increment_gain = 0.3f;
  float increment      = 0.3f;
  float ratio_num_pts  = 0.95f;

  // Moving-object detection
  float negative_log_odds  = -2.0f;
  float obj_score_soft_thr = 0.8f;
  float obj_score_hard_thr = 20.0f;
  float hard_thr_radius    = 12.0f;
  float xy_size_thr        = 30.0f;

  // Pose correction
  float voxel_size_for_pose_correction    = 1.0f;
  float max_corr_dist_for_pose_correction = 2.0f;

  // Over-segmentation
  float minimum_area_thr       = 0.5f;
  float ratio_of_unknown_prior = 0.25f;

  // Flags
  bool verbose             = false;
  bool is_large_scale      = true;
  bool stop_for_each_frame = false;

  bool viz_set_scan_and_pose = false;
  bool viz_set_submap        = false;
  bool viz_update            = false;
  bool viz_detect            = false;
  bool viz_over_seg          = false;

  // Frame & loop ints
  int start_frame           = 0;
  int end_frame             = -1;
  int viz_interval          = 10;
  int accum_interval        = 2;
  int expansion_range       = 20;
  int neighboring_width     = 0;  // derived
  int neighboring_height    = 0;  // derived
  int kernel_size           = 3;
  int update_interval       = 1;
  int minimum_num_pts       = 3;
  int minimum_num_per_voxel = 1;
  int num_omp_cores         = 8;

  // Volumetric outlier removal
  int window_size              = 1;
  float vor_cand_score_thr     = 3.0f;
  bool use_adaptive_voxel_size = false;
  float dist_thr_gain          = 1.0f;

  // Voxel sizes
  float voxel_size     = 0.05f;
  float map_voxel_size = 0.2f;

  // Trajectory clustering / pose correction
  bool run_traj_clustering               = false;
  bool distinguish_temporal_trajectories = false;
  bool correct_poses_by_submap_matching  = false;
  bool correct_poses                     = false;
  bool save_map                          = false;

  // Dataset paths
  std::string dataset_name;
  std::string abs_save_dir = "/";
  std::string abs_data_dir;
  std::string cloud_dir;
  std::string cloud_format;
  std::string pose_path;
  std::string sequence;
  std::string instance_seg_method = "cais";

  // Visualization output (optional)
  bool rerun_enabled = true;
  bool rerun_spawn   = true;    // launch rerun viewer subprocess
  std::string rerun_save_path;  // if non-empty, write .rrd here

  // Load config from a YAML file. Throws std::runtime_error on parse failure.
  static Config fromYaml(const std::string& yaml_path);
};

}  // namespace erasor2

#endif  // ERASOR2_CONFIG_HPP
