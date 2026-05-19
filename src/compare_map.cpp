#include <filesystem>
#include <iostream>
#include <map>
#include <string>

#include <pcl/registration/gicp.h>

#include "erasor2/Config.hpp"
#include "erasor2/RerunLogger.hpp"

#include "tools/erasor_utils.hpp"

using Cloud    = pcl::PointCloud<pcl::PointXYZI>;
using CloudPtr = Cloud::Ptr;

// Splits a labelled cloud into its dynamic and static halves using the
// label embedded in `intensity`.
static void splitDynStatic(const Cloud &in, Cloud &dynamic_out, Cloud &static_out) {
  erasor_utils::parseStaticAndDynamic(in, dynamic_out, static_out);
}

// Both should be voxelized with the same voxel size.
static void getPositiveClouds(const Cloud &raw_cloud,
                              const Cloud &est_cloud,
                              Cloud &tp,
                              Cloud &fp,
                              const float voxel_size = 0.2f) {
  tp.clear();
  fp.clear();
  tp.reserve(raw_cloud.size());
  fp.reserve(raw_cloud.size());

  CloudPtr raw_static(new Cloud);
  CloudPtr raw_dynamic(new Cloud);
  splitDynStatic(raw_cloud, *raw_dynamic, *raw_static);

  std::vector<int> tps, fps;
  erasor_utils::findEmptyCorrespondences(*raw_dynamic, est_cloud, tps, voxel_size * voxel_size);
  erasor_utils::findEmptyCorrespondences(*raw_static, est_cloud, fps, voxel_size * voxel_size);

  for (int idx : tps) tp.emplace_back(raw_dynamic->points[idx]);
  for (int idx : fps) fp.emplace_back(raw_static->points[idx]);
}

// TN: remained static points; FN: remained dynamic points; TP: removed
// dynamic points; FP: removed static points.
static void getTPFN(const Cloud &raw_cloud,
                    const Cloud &est_cloud,
                    Cloud &tp,
                    Cloud &fp,
                    Cloud &fn,
                    Cloud &tn,
                    const float voxel_size = 0.2f) {
  if (est_cloud.empty()) return;
  splitDynStatic(est_cloud, fn, tn);
  getPositiveClouds(raw_cloud, est_cloud, tp, fp, voxel_size);
}

static bool tryLoad(const std::string &path, CloudPtr dst) {
  if (path.empty()) return false;
  if (!std::filesystem::exists(path)) {
    std::cerr << "[compare_map] File not found: " << path << "\n";
    return false;
  }
  return erasor_utils::load_pcd(path, dst) == 0;
}

// Logs raw_tn / raw_fn for the input + tp/fp/fn/tn for one method.
static void logMethod(const std::string &method,
                      const Cloud &tp,
                      const Cloud &fp,
                      const Cloud &fn,
                      const Cloud &tn) {
  erasor2::viz::logCloud(method + "/TP", tp);
  erasor2::viz::logCloud(method + "/FP", fp);
  erasor2::viz::logCloud(method + "/FN", fn);
  erasor2::viz::logCloud(method + "/TN", tn);
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: compare_map <config.yaml>\n"
              << "Expected YAML keys (all optional except raw_path):\n"
              << "  compare_map:\n"
              << "    raw_path: ...\n"
              << "    octomap_path: ...\n"
              << "    pplremover_path: ...\n"
              << "    removert_path: ...\n"
              << "    auto_mos_path: ...\n"
              << "    erasor_path: ...\n"
              << "    erasor2_path: ...\n"
              << "    four_d_mos_path: ...\n"
              << "    scan2map_path: ...\n";
    return 1;
  }

  YAML::Node root = YAML::LoadFile(argv[1]);
  auto get_path   = [&](const char *key) -> std::string {
    if (auto cm = root["compare_map"]; cm && cm[key]) return cm[key].as<std::string>();
    return "";
  };

  // Initialize rerun (default: spawn the viewer).
  bool rerun_enabled =
      root["rerun"] && root["rerun"]["enabled"] ? root["rerun"]["enabled"].as<bool>() : true;
  bool rerun_spawn =
      root["rerun"] && root["rerun"]["spawn"] ? root["rerun"]["spawn"].as<bool>() : true;
  std::string rerun_save = root["rerun"] && root["rerun"]["save_path"]
                               ? root["rerun"]["save_path"].as<std::string>()
                               : "";
  if (rerun_enabled) erasor2::viz::init("compare_map", rerun_spawn, rerun_save);

  std::cout << "[compare_map] Loading clouds...\n";
  CloudPtr raw_cloud(new Cloud);
  CloudPtr est_octomap(new Cloud);
  CloudPtr est_pplremover(new Cloud);
  CloudPtr est_removert(new Cloud);
  CloudPtr est_auto_mos(new Cloud);
  CloudPtr est_erasor(new Cloud);
  CloudPtr est_erasor2(new Cloud);
  CloudPtr est_4dmos(new Cloud);
  CloudPtr est_scan2map(new Cloud);

  tryLoad(get_path("raw_path"), raw_cloud);
  tryLoad(get_path("octomap_path"), est_octomap);
  tryLoad(get_path("pplremover_path"), est_pplremover);
  tryLoad(get_path("removert_path"), est_removert);
  tryLoad(get_path("auto_mos_path"), est_auto_mos);
  tryLoad(get_path("erasor_path"), est_erasor);
  tryLoad(get_path("erasor2_path"), est_erasor2);
  tryLoad(get_path("four_d_mos_path"), est_4dmos);
  tryLoad(get_path("scan2map_path"), est_scan2map);

  Cloud raw_tp, raw_fp, raw_fn, raw_tn;
  Cloud o_tp, o_fp, o_fn, o_tn;
  Cloud p_tp, p_fp, p_fn, p_tn;
  Cloud r_tp, r_fp, r_fn, r_tn;
  Cloud a_tp, a_fp, a_fn, a_tn;
  Cloud e_tp, e_fp, e_fn, e_tn;
  Cloud e2_tp, e2_fp, e2_fn, e2_tn;
  Cloud f_tp, f_fp, f_fn, f_tn;
  Cloud s_tp, s_fp, s_fn, s_tn;

  std::cout << "[compare_map] Computing TP/FP/FN/TN ...\n";
  getTPFN(*raw_cloud, *raw_cloud, raw_tp, raw_fp, raw_fn, raw_tn);
  getTPFN(*raw_cloud, *est_octomap, o_tp, o_fp, o_fn, o_tn);
  getTPFN(*raw_cloud, *est_pplremover, p_tp, p_fp, p_fn, p_tn);
  getTPFN(*raw_cloud, *est_removert, r_tp, r_fp, r_fn, r_tn);
  getTPFN(*raw_cloud, *est_auto_mos, a_tp, a_fp, a_fn, a_tn);
  getTPFN(*raw_cloud, *est_erasor, e_tp, e_fp, e_fn, e_tn);
  getTPFN(*raw_cloud, *est_erasor2, e2_tp, e2_fp, e2_fn, e2_tn);
  getTPFN(*raw_cloud, *est_4dmos, f_tp, f_fp, f_fn, f_tn);
  getTPFN(*raw_cloud, *est_scan2map, s_tp, s_fp, s_fn, s_tn);

  std::cout << "[compare_map] Logging to rerun...\n";
  erasor2::viz::logCloud("raw/static", raw_tn);
  erasor2::viz::logCloud("raw/dynamic", raw_fn);
  logMethod("octomap", o_tp, o_fp, o_fn, o_tn);
  logMethod("pplremover", p_tp, p_fp, p_fn, p_tn);
  logMethod("removert", r_tp, r_fp, r_fn, r_tn);
  logMethod("auto_mos", a_tp, a_fp, a_fn, a_tn);
  logMethod("erasor", e_tp, e_fp, e_fn, e_tn);
  logMethod("erasor2", e2_tp, e2_fp, e2_fn, e2_tn);
  logMethod("four_d_mos", f_tp, f_fp, f_fn, f_tn);
  logMethod("scan2map", s_tp, s_fp, s_fn, s_tn);

  std::cout << "[compare_map] Done.\n";
  return 0;
}
