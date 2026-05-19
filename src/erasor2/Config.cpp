#include "erasor2/Config.hpp"

#include <stdexcept>

namespace erasor2 {

namespace {

// Walk a dotted YAML path and return the node, or an Undefined node if any
// segment is missing. Lets us read "erasor2.log_odds.increment" against a
// plain `YAML::Node`.
YAML::Node walk(const YAML::Node& root, const std::string& dotted) {
  YAML::Node cur = YAML::Clone(root);
  size_t pos     = 0;
  while (pos < dotted.size()) {
    size_t dot      = dotted.find('.', pos);
    std::string key = dotted.substr(pos, dot == std::string::npos ? std::string::npos : dot - pos);
    if (!cur || !cur.IsMap() || !cur[key]) return YAML::Node(YAML::NodeType::Undefined);
    cur = cur[key];
    if (dot == std::string::npos) break;
    pos = dot + 1;
  }
  return cur;
}

template <typename T>
T get(const YAML::Node& root, const std::string& key, const T& fallback) {
  YAML::Node n = walk(root, key);
  if (!n || !n.IsDefined()) return fallback;
  try {
    return n.as<T>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

}  // namespace

Config Config::fromYaml(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load YAML " + yaml_path + ": " + e.what());
  }

  Config c;

  // Top-level loop / OMP
  c.num_omp_cores       = get<int>(root, "num_omp_cores", c.num_omp_cores);
  c.start_frame         = get<int>(root, "start_frame", c.start_frame);
  c.end_frame           = get<int>(root, "end_frame", c.end_frame);
  c.viz_interval        = get<int>(root, "viz_interval", c.viz_interval);
  c.is_large_scale      = get<bool>(root, "is_large_scale", c.is_large_scale);
  c.stop_for_each_frame = get<bool>(root, "stop_for_each_frame", c.stop_for_each_frame);

  // Dataloader
  c.run_traj_clustering = get<bool>(root, "dataloader.run_traj_clustering", c.run_traj_clustering);
  c.distinguish_temporal_trajectories = get<bool>(
      root, "dataloader.distinguish_temporal_trajectories", c.distinguish_temporal_trajectories);
  c.dataset_name = get<std::string>(root, "dataloader.dataset_name", c.dataset_name);
  c.abs_data_dir = get<std::string>(root, "dataloader.abs_data_dir", c.abs_data_dir);
  c.cloud_dir    = get<std::string>(root, "dataloader.cloud_dir", c.cloud_dir);
  c.cloud_format = get<std::string>(root, "dataloader.cloud_format", c.cloud_format);
  c.pose_path    = get<std::string>(root, "dataloader.pose_path", c.pose_path);
  c.sequence     = get<std::string>(root, "dataloader.sequence", c.sequence);
  c.abs_save_dir = get<std::string>(root, "dataloader.abs_save_dir", c.abs_save_dir);
  c.instance_seg_method =
      get<std::string>(root, "dataloader.instance_seg_method", c.instance_seg_method);
  c.accum_interval  = get<int>(root, "dataloader.accum_interval", c.accum_interval);
  c.expansion_range = get<int>(root, "dataloader.expansion_range", c.expansion_range);
  c.voxel_size      = get<float>(root, "dataloader.voxel_size", c.voxel_size);
  c.map_voxel_size  = get<float>(root, "dataloader.map_voxel_size", c.map_voxel_size);

  // Pose correction
  c.correct_poses_by_submap_matching = get<bool>(
      root, "pose_corrector.correct_poses_by_submap_matching", c.correct_poses_by_submap_matching);
  c.voxel_size_for_pose_correction =
      get<float>(root, "pose_corrector.voxel_size", c.voxel_size_for_pose_correction);
  c.max_corr_dist_for_pose_correction =
      get<float>(root, "pose_corrector.max_corr_dist", c.max_corr_dist_for_pose_correction);

  // ERASOR2 core
  c.min_z_voi            = get<float>(root, "erasor2.min_z_voi", c.min_z_voi);
  c.max_z_voi            = get<float>(root, "erasor2.max_z_voi", c.max_z_voi);
  c.scan_ratio_threshold = get<float>(root, "erasor2.scan_ratio_threshold", c.scan_ratio_threshold);
  c.binary_scan_ratio_threshold =
      get<float>(root, "erasor2.binary_scan_ratio_threshold", c.binary_scan_ratio_threshold);
  c.min_len_empty_space = get<float>(root, "erasor2.min_len_empty_space", c.min_len_empty_space);
  c.min_z_diff_thr      = get<float>(root, "erasor2.min_z_diff_thr", c.min_z_diff_thr);

  c.increment_gain      = get<float>(root, "erasor2.log_odds.increment_gain", c.increment_gain);
  c.increment           = get<float>(root, "erasor2.log_odds.increment", c.increment);
  c.region_proposal_thr = get<float>(root, "erasor2.region_proposal_thr", c.region_proposal_thr);
  c.kernel_size         = get<int>(root, "erasor2.kernel_size", c.kernel_size);
  c.update_interval     = get<int>(root, "erasor2.update_interval", c.update_interval);
  c.ratio_num_pts       = get<float>(root, "erasor2.ratio_num_pts", c.ratio_num_pts);
  c.minimum_num_pts     = get<int>(root, "erasor2.minimum_num_pts", c.minimum_num_pts);
  c.minimum_num_per_voxel =
      get<int>(root, "erasor2.minimum_num_per_voxel", c.minimum_num_per_voxel);

  c.negative_log_odds =
      get<float>(root, "erasor2.moving_object_detection.negative_log_odds", c.negative_log_odds);
  c.obj_score_soft_thr =
      get<float>(root, "erasor2.moving_object_detection.obj_score_soft_thr", c.obj_score_soft_thr);
  c.obj_score_hard_thr =
      get<float>(root, "erasor2.moving_object_detection.obj_score_hard_thr", c.obj_score_hard_thr);
  c.hard_thr_radius =
      get<float>(root, "erasor2.moving_object_detection.hard_thr_radius", c.hard_thr_radius);
  c.xy_size_thr = get<float>(root, "erasor2.moving_object_detection.xy_size_thr", c.xy_size_thr);

  c.minimum_area_thr =
      get<float>(root, "erasor2.over_segmentation.minimum_area_thr", c.minimum_area_thr);
  c.ratio_of_unknown_prior = get<float>(
      root, "erasor2.over_segmentation.ratio_of_unknown_prior", c.ratio_of_unknown_prior);

  c.window_size = get<int>(root, "erasor2.volumetric_outlier_removal.window_size", c.window_size);
  c.use_adaptive_voxel_size =
      get<bool>(root,
                "erasor2.volumetric_outlier_removal.use_adaptive_voxel_size",
                c.use_adaptive_voxel_size);
  c.vor_cand_score_thr = get<float>(
      root, "erasor2.volumetric_outlier_removal.vor_cand_score_thr", c.vor_cand_score_thr);
  c.dist_thr_gain =
      get<float>(root, "erasor2.volumetric_outlier_removal.dist_thr_gain", c.dist_thr_gain);

  c.save_map = get<bool>(root, "erasor2.save_map", c.save_map);

  c.grid_resolution = get<float>(root, "erasor2.grid_resolution", c.grid_resolution);
  c.egocentric_grid_resolution =
      get<float>(root, "erasor2.egocentric_grid_resolution", c.egocentric_grid_resolution);
  c.range_of_interest = get<float>(root, "erasor2.range_of_interest", c.range_of_interest);

  c.viz_set_scan_and_pose =
      get<bool>(root, "erasor2.viz_flag.set_scan_and_pose", c.viz_set_scan_and_pose);
  c.viz_set_submap = get<bool>(root, "erasor2.viz_flag.set_submap", c.viz_set_submap);
  c.viz_update     = get<bool>(root, "erasor2.viz_flag.update", c.viz_update);
  c.viz_detect     = get<bool>(root, "erasor2.viz_flag.detect", c.viz_detect);
  c.viz_over_seg   = get<bool>(root, "erasor2.viz_flag.over_seg", c.viz_over_seg);

  c.neighboring_width  = static_cast<int>(2.0f * c.range_of_interest / c.grid_resolution);
  c.neighboring_height = static_cast<int>(2.0f * c.range_of_interest / c.grid_resolution);

  // Extrinsic
  c.robot_body_size = get<float>(root, "extrinsic.robot_body_size", c.robot_body_size);
  c.sensor_height   = get<float>(root, "extrinsic.sensor_height", c.sensor_height);
  c.ext_rot_vec     = get<std::vector<float>>(root, "extrinsic.rotation", c.ext_rot_vec);
  c.ext_trans_vec   = get<std::vector<float>>(root, "extrinsic.translation", c.ext_trans_vec);
  if (c.ext_rot_vec.size() == 9) {
    c.ext_rot = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(c.ext_rot_vec.data());
  }
  if (c.ext_trans_vec.size() == 3) {
    c.ext_trans = Eigen::Map<const Eigen::Vector3f>(c.ext_trans_vec.data());
  }
  c.extrinsic.block<3, 3>(0, 0)      = c.ext_rot;
  c.extrinsic.topRightCorner<3, 1>() = c.ext_trans;
  c.tf_h_of_ground_to_be_zero(2, 3)  = c.sensor_height;

  // Rerun
  c.rerun_enabled   = get<bool>(root, "rerun.enabled", c.rerun_enabled);
  c.rerun_spawn     = get<bool>(root, "rerun.spawn", c.rerun_spawn);
  c.rerun_save_path = get<std::string>(root, "rerun.save_path", c.rerun_save_path);

  return c;
}

}  // namespace erasor2
