// Entry point for the ported ERASOR v1 algorithm. Mirrors the surface area
// of run_erasor2's main but with the v1 polar-grid pipeline:
//
//   1. Load the accumulated raw map PCD produced by `mapgen` for this seq.
//   2. For each frame in [start_frame, end_frame], pull the current scan +
//      pose from the dataloader, extract a VoI from the running map around
//      the pose, run ERASOR on (map_voi, scan_voi), and stitch the filtered
//      static estimate back into the running map.
//   3. Save the final running map as `<seq>_0_frame_<start>_to_<end>_estimated.pcd`
//      so evaluate.py / scripts/run_pipeline.py / scripts/run_benchmark.py
//      consume it without any path-shape changes.
//
// The orchestration loop mirrors OfflineMapUpdater::callback_node in the
// original github.com/LimHyungTae/ERASOR repo; the ROS subscriber +
// publisher plumbing is replaced by a synchronous loop over our existing
// SemanticKITTILoader.

#include "dataloader/dataloader.h"
#include "erasor1/erasor.hpp"
#include "erasor2/RerunLogger.hpp"
#include "erasor2/progress_bar.hpp"
#include "tools/erasor_utils.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

// Mirror of fetch_VoI from OfflineMapUpdater: split map_arranged into a
// VoI within `max_range` of (px, py) and the outskirts kept verbatim;
// transform the VoI to the body frame for ERASOR's egocentric grid.
void fetch_voi(const pcl::PointCloud<pcl::PointXYZI>& map_arranged,
               const Eigen::Matrix4f&                 body2origin,
               double                                 max_range,
               pcl::PointCloud<pcl::PointXYZI>&       voi_body,
               pcl::PointCloud<pcl::PointXYZI>&       outskirts) {
  voi_body.clear();
  outskirts.clear();
  const double px               = body2origin(0, 3);
  const double py               = body2origin(1, 3);
  const double max_dist_squared = (max_range + 0.5) * (max_range + 0.5);

  pcl::PointCloud<pcl::PointXYZI> voi_world;
  voi_world.reserve(map_arranged.size());
  for (const auto& pt : map_arranged.points) {
    const double dx = pt.x - px;
    const double dy = pt.y - py;
    if (dx * dx + dy * dy < max_dist_squared) {
      voi_world.points.emplace_back(pt);
    } else {
      outskirts.points.emplace_back(pt);
    }
  }
  // PCL's transformPointCloud divides points.size() by width to derive
  // height; with emplace_back-only construction width stays 0 and the
  // division SIGFPEs. Keep the invariant.
  voi_world.width  = static_cast<std::uint32_t>(voi_world.points.size());
  voi_world.height = voi_world.points.empty() ? 0 : 1;
  outskirts.width  = static_cast<std::uint32_t>(outskirts.points.size());
  outskirts.height = outskirts.points.empty() ? 0 : 1;
  if (voi_world.empty()) {
    voi_body.clear();
    return;
  }
  pcl::transformPointCloud(voi_world, voi_body, body2origin.inverse());
}

struct Config {
  std::string dataset_name;
  std::string abs_data_dir;
  std::string abs_save_dir;
  std::string sequence;
  int         start_frame;
  int         end_frame;
  int         accum_interval;
  double      voxel_size;
  // Sensor mount height above the body origin -- upstream calls this
  // tf_lidar2body. KITTI velodyne sits ~1.73m above ground. Without this
  // shift the LiDAR-frame scans have ground at z=-1.73, well outside the
  // [min_h=-1.3, max_h=3.2] body-frame window the v1 paper uses.
  double      lidar_z_offset;
  bool        rerun_enabled;
  bool        rerun_spawn;
  std::string rerun_save_path;
  erasor1::Params params;
};

template <typename T>
T get(const YAML::Node& n, const std::string& key, T fallback) {
  return n[key] ? n[key].as<T>() : fallback;
}

Config load_config(const std::string& path) {
  YAML::Node y = YAML::LoadFile(path);
  Config     c;
  c.start_frame    = get<int>(y, "start_frame", 0);
  c.end_frame      = get<int>(y, "end_frame", 0);
  c.lidar_z_offset = get<double>(y, "lidar_z_offset", 1.73);

  const auto dl    = y["dataloader"];
  c.dataset_name   = get<std::string>(dl, "dataset_name", std::string("SemanticKITTI"));
  c.abs_data_dir   = get<std::string>(dl, "abs_data_dir", std::string());
  c.abs_save_dir   = get<std::string>(dl, "abs_save_dir", std::string());
  c.sequence       = get<std::string>(dl, "sequence", std::string("05"));
  c.accum_interval = get<int>(dl, "accum_interval", 2);
  c.voxel_size     = get<double>(dl, "voxel_size", 0.2);

  const auto er  = y["erasor"];
  c.params.max_range            = get<double>(er, "max_range", 60.0);
  c.params.num_rings            = get<int>(er, "num_rings", 20);
  c.params.num_sectors          = get<int>(er, "num_sectors", 108);
  c.params.min_h                = get<double>(er, "min_h", -3.0);
  c.params.max_h                = get<double>(er, "max_h", 1.5);
  c.params.th_bin_max_h         = get<double>(er, "th_bin_max_h", 0.39);
  c.params.scan_ratio_threshold = get<double>(er, "scan_ratio_threshold", 0.22);
  c.params.num_lowest_pts       = get<int>(er, "num_lowest_pts", 5);
  c.params.minimum_num_pts      = get<int>(er, "minimum_num_pts", 4);
  c.params.rejection_ratio      = get<double>(er, "rejection_ratio", 0.33);
  c.params.gf_dist_thr          = get<double>(er, "gf_dist_thr", 0.05);
  c.params.gf_iter              = get<int>(er, "gf_iter", 3);
  c.params.gf_num_lpr           = get<int>(er, "gf_num_lpr", 10);
  c.params.gf_th_seeds_height   = get<double>(er, "gf_th_seeds_height", 0.5);
  c.params.map_voxel_size       = get<double>(er, "map_voxel_size", c.voxel_size);
  c.params.query_voxel_size     = get<double>(er, "query_voxel_size", 0.05);
  c.params.removal_interval     = get<int>(er, "removal_interval", 8);

  const auto rr   = y["rerun"];
  c.rerun_enabled = get<bool>(rr, "enabled", true);
  c.rerun_spawn   = get<bool>(rr, "spawn", true);
  c.rerun_save_path = get<std::string>(rr, "save_path", std::string());
  return c;
}

std::string voxel_str(double v) {
  std::ostringstream oss;
  oss << v;
  std::string s = oss.str();
  std::replace(s.begin(), s.end(), '.', '_');
  return s;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: run_erasor <config.yaml>\n";
    return 1;
  }
  Config cfg = load_config(argv[1]);
  if (cfg.rerun_enabled) {
    erasor2::viz::init("erasor1", cfg.rerun_spawn, cfg.rerun_save_path);
  }

  std::unique_ptr<DataLoader> loader;
  if (cfg.dataset_name == "SemanticKITTI") {
    loader = std::make_unique<SemanticKITTILoader>(cfg.abs_data_dir, cfg.sequence);
  } else {
    std::cerr << "[erasor1] dataset_name=" << cfg.dataset_name
              << " not yet wired into the v1 runner\n";
    return 2;
  }

  erasor1::ERASOR erasor(cfg.params);

  // mapgen writes:
  //   <abs_save_dir>/<seq>_<start>_to_<end>_w_interval_<i>_voxel_<vox>.pcd
  // We need the voxelised one (not _original.pcd).
  const std::string map_in_path = cfg.abs_save_dir + "/" + cfg.sequence + "_" +
                                  std::to_string(cfg.start_frame) + "_to_" +
                                  std::to_string(cfg.end_frame) + "_w_interval_" +
                                  std::to_string(cfg.accum_interval) + "_voxel_" +
                                  voxel_str(cfg.voxel_size) + ".pcd";
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged(new pcl::PointCloud<pcl::PointXYZI>);
  if (erasor_utils::load_pcd<pcl::PointXYZI>(map_in_path, map_arranged) != 0) {
    std::cerr << "[erasor1] Failed to load mapgen output: " << map_in_path
              << "\nRun mapgen first to produce the accumulated map PCD.\n";
    return 3;
  }
  std::cout << "[erasor1] Loaded mapgen output: " << map_in_path
            << " (" << map_arranged->size() << " pts)\n";

  const int   n_frames = (cfg.end_frame - cfg.start_frame) / cfg.accum_interval + 1;
  erasor2::ProgressBar bar("[erasor1] filter     ", n_frames, erasor2::color::kMagenta);

  int tick = 0;
  for (int k = cfg.start_frame; k <= cfg.end_frame; k += cfg.accum_interval) {
    bar.tick(++tick);

    // Match upstream OfflineMapUpdater::callback_node: only run ERASOR
    // every `removal_interval` ticks. In between, the running map is
    // left as-is, which is what the upstream behaviour does (the
    // `if (stack_count % removal_interval_ == 0)` gate at line 209
    // of the upstream code).
    if ((tick - 1) % cfg.params.removal_interval != 0) continue;

    pcl::PointCloud<pcl::PointXYZI> scan;
    Eigen::Matrix4f                 pose = Eigen::Matrix4f::Identity();
    if (cfg.dataset_name == "SemanticKITTI") {
      loader->getGTLabeledScan(k, scan);
      pose = loader->poses_gt_[k];
    } else {
      loader->getScanAndPose(k, scan, pose);
    }
    if (scan.empty()) continue;

    // Voxelise the query scan at `query_voxel_size` before polar binning
    // -- mirrors OfflineMapUpdater.cpp:238. Reduces raw-scan noise and
    // keeps the per-frame bin sizes bounded.
    pcl::PointCloud<pcl::PointXYZI> query_voi;
    if (cfg.params.query_voxel_size > 0.0) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(
          new pcl::PointCloud<pcl::PointXYZI>(scan));
      scan_ptr->width  = static_cast<std::uint32_t>(scan_ptr->points.size());
      scan_ptr->height = scan_ptr->points.empty() ? 0 : 1;
      erasor_utils::voxelize_preserving_labels_by_nanoflann(
          scan_ptr, query_voi, cfg.params.query_voxel_size);
    } else {
      query_voi = scan;
    }
    query_voi.width  = static_cast<std::uint32_t>(query_voi.points.size());
    query_voi.height = query_voi.points.empty() ? 0 : 1;

    pcl::PointCloud<pcl::PointXYZI> map_voi, outskirts;
    fetch_voi(*map_arranged, pose, cfg.params.max_range, map_voi, outskirts);

    // Apply lidar->body z-shift so the paper's body-frame thresholds
    // (`min_h=-1.3`, `max_h=3.2`) line up with our LiDAR-frame point
    // data. Ground sits at z=-1.73 in the velodyne frame; we lift by
    // +1.73 so it lands at z=0 inside the polar grid. Reversed below.
    for (auto& pt : query_voi.points) pt.z += static_cast<float>(cfg.lidar_z_offset);
    for (auto& pt : map_voi.points)   pt.z += static_cast<float>(cfg.lidar_z_offset);

    erasor.set_inputs(map_voi, query_voi);
    // Dispatch the v3 algorithm: two-pass scan-ratio with BLOCKED-state
    // gating and per-bin voxelisation on ground revert. This is the
    // path the upstream YAMLs ship with (`version: 3`) and produces the
    // paper Table II numbers.
    erasor.compare_vois_and_revert_ground_w_block(k);

    pcl::PointCloud<pcl::PointXYZI> static_estimate, complement;
    erasor.get_static_estimate(static_estimate, complement);

    // Transform the filtered VoI back to the world frame, then re-stitch
    // with the outskirts to form the next iteration's running map.
    pcl::PointCloud<pcl::PointXYZI> filtered_body = static_estimate + complement;
    // Undo the body-frame z-shift applied above before transforming
    // back to world with the (LiDAR-frame) pose.
    for (auto& pt : filtered_body.points) pt.z -= static_cast<float>(cfg.lidar_z_offset);
    pcl::PointCloud<pcl::PointXYZI> filtered_world;
    if (!filtered_body.empty()) {
      filtered_body.width  = static_cast<std::uint32_t>(filtered_body.points.size());
      filtered_body.height = 1;
      pcl::transformPointCloud(filtered_body, filtered_world, pose);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr next_map(new pcl::PointCloud<pcl::PointXYZI>);
    next_map->reserve(filtered_world.size() + outskirts.size());
    *next_map = filtered_world + outskirts;
    next_map->width  = static_cast<std::uint32_t>(next_map->points.size());
    next_map->height = 1;

    // The global voxelize-after-each-frame in the v2 path (commit
    // 91a48f6) is dropped here. The v3 algorithm voxelizes per-bin
    // inside `compare_vois_and_revert_ground_w_block` on ground revert,
    // and `removal_interval`-frame skipping keeps the cumulative growth
    // in check. Re-voxelizing the full running map at 0.2m here would
    // strip detail the GT also has and depress PR (the failure mode
    // the v2 path produced).
    if (false) {
      pcl::PointCloud<pcl::PointXYZI> voxed;
      erasor_utils::voxelize_preserving_labels_by_nanoflann(
          next_map, voxed, cfg.params.map_voxel_size);
      voxed.width  = static_cast<std::uint32_t>(voxed.points.size());
      voxed.height = voxed.points.empty() ? 0 : 1;
      *next_map    = voxed;
    }
    map_arranged = next_map;
  }
  bar.finish();

  // Distinguish v1 from run_erasor2's output so both can coexist in the
  // same abs_save_dir for a side-by-side benchmark. Naming mirrors v2's
  // shape with `_v1` slotted before `_0_frame_`:
  //   <seq>_v1_0_frame_<start>_to_<end>_estimated.pcd
  const std::string out_path = cfg.abs_save_dir + "/" + cfg.sequence +
                               "_v1_0_frame_" + std::to_string(cfg.start_frame) +
                               "_to_" + std::to_string(cfg.end_frame) +
                               "_estimated.pcd";
  std::filesystem::create_directories(std::filesystem::path(out_path).parent_path());

  // Final voxelize at `voxel_size` (the dataloader.voxel_size, default
  // 0.2). Mirrors OfflineMapUpdater::save_static_map line 186 which
  // is triggered from the upstream README's
  //     rostopic pub /saveflag std_msgs/Float32 "data: 0.2"
  // Without this, the saved map carries uneven density (fine-voxel
  // MAP_IS_HIGHER bins next to raw running-map regions) while the GT
  // PCD is uniform at 0.2m -- the evaluate.py kNN check then misses
  // matches and PR tanks.
  pcl::PointCloud<pcl::PointXYZI> final_map;
  erasor_utils::voxelize_preserving_labels_by_nanoflann(
      map_arranged, final_map, cfg.voxel_size);
  final_map.width  = static_cast<std::uint32_t>(final_map.points.size());
  final_map.height = final_map.points.empty() ? 0 : 1;
  pcl::io::savePCDFileASCII(out_path, final_map);
  std::cout << "\033[1;32m[erasor1] Saved static map: " << out_path
            << " (" << final_map.size() << " pts)\033[0m\n";

  erasor2::viz::shutdown();
  return 0;
}
