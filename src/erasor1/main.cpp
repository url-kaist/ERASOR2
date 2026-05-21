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

    pcl::PointCloud<pcl::PointXYZI> scan;
    Eigen::Matrix4f                 pose = Eigen::Matrix4f::Identity();
    if (cfg.dataset_name == "SemanticKITTI") {
      loader->getGTLabeledScan(k, scan);
      pose = loader->poses_gt_[k];
    } else {
      loader->getScanAndPose(k, scan, pose);
    }
    if (scan.empty()) continue;

    // Query VoI = the current scan in the body / sensor frame (the dataloader
    // returns scans already in the LiDAR frame for KITTI, so no extra
    // lidar->body transform is needed here).
    pcl::PointCloud<pcl::PointXYZI> query_voi = scan;
    query_voi.width  = static_cast<std::uint32_t>(query_voi.points.size());
    query_voi.height = 1;

    pcl::PointCloud<pcl::PointXYZI> map_voi, outskirts;
    fetch_voi(*map_arranged, pose, cfg.params.max_range, map_voi, outskirts);

    erasor.set_inputs(map_voi, query_voi);
    erasor.compare_vois_and_revert_ground(k);

    pcl::PointCloud<pcl::PointXYZI> static_estimate, complement;
    erasor.get_static_estimate(static_estimate, complement);

    // Transform the filtered VoI back to the world frame, then re-stitch
    // with the outskirts to form the next iteration's running map.
    pcl::PointCloud<pcl::PointXYZI> filtered_body = static_estimate + complement;
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

    // merge_bins inside ERASOR doubles a bin's point count for high
    // scan-ratio cells. Without periodic global voxelization the running
    // map ~doubles each frame, fetch_voi turns O(N) -> exponential. The
    // upstream code's voxelize-on-stride hook (OfflineMapUpdater.cpp:300)
    // does the same; we inline it here every frame at map_voxel_size.
    // Without periodic voxelization the running map blows up by ~2x per
    // frame because `merge_bins` concatenates src1.points + src2.points
    // in static cells. pcl::VoxelGrid integer-overflows at fine leaves
    // over a 200m-extent map, so we use the nanoflann-backed voxelizer
    // already in this repo. NOTE: This produces a working but lossy
    // result vs the paper -- see TODO in this file.
    if (cfg.params.map_voxel_size > 0.0f && !next_map->empty()) {
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

  // Match run_erasor2's estimated.pcd naming so evaluate.py just works.
  const std::string out_path = cfg.abs_save_dir + "/" + cfg.sequence +
                               "_0_frame_" + std::to_string(cfg.start_frame) +
                               "_to_" + std::to_string(cfg.end_frame) +
                               "_estimated.pcd";
  std::filesystem::create_directories(std::filesystem::path(out_path).parent_path());

  map_arranged->width  = static_cast<std::uint32_t>(map_arranged->points.size());
  map_arranged->height = 1;
  pcl::io::savePCDFileASCII(out_path, *map_arranged);
  std::cout << "\033[1;32m[erasor1] Saved static map: " << out_path
            << " (" << map_arranged->size() << " pts)\033[0m\n";

  erasor2::viz::shutdown();
  return 0;
}
