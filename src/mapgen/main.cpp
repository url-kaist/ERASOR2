#include "erasor2/Config.hpp"
#include "erasor2/RerunLogger.hpp"
#include "erasor2/progress_bar.hpp"

#include "dataloader/dataloader.h"
#include "mapgen/mapgen.hpp"

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: mapgen <config.yaml>\n";
    return 1;
  }
  const auto cfg = erasor2::Config::fromYaml(argv[1]);
  if (cfg.rerun_enabled) {
    erasor2::viz::init("erasor2_mapgen", cfg.rerun_spawn, cfg.rerun_save_path);
  }

  unique_ptr<Mapgen> mapgen(new Mapgen(cfg));

  std::unique_ptr<DataLoader> loader;
  string dataset_name = mapgen->dataset_name_;
  if (dataset_name == "SemanticKITTI") {
    loader =
        std::move(std::make_unique<SemanticKITTILoader>(mapgen->abs_data_dir_, mapgen->sequence_));
  } else if (dataset_name == "HeLiPR") {
    loader = std::move(
        std::make_unique<HeLiPRLoader>(mapgen->abs_data_dir_, mapgen->sequence_, "hdbscan"));
  }

  int start_frame    = mapgen->start_frame_;
  int end_frame      = mapgen->end_frame_;
  int accum_interval = mapgen->accum_interval_;

  int cnt = 0;
  erasor2::ProgressBar mapgen_bar(
      "[mapgen]  accumulate ", end_frame - start_frame + 1, erasor2::color::kGreen);
  for (int i = start_frame; i < end_frame + accum_interval; ++i) {
    signal(SIGINT, erasor_utils::signal_callback_handler);
    mapgen_bar.tick(i - start_frame + 1);

    if (accum_interval > 1 && ++cnt / accum_interval >= 1) {
      cnt = 0;
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr noise(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    if (dataset_name == "SemanticKITTI") {
      loader->getGTLabeledScan(i, *cloud_raw);
      pose = loader->poses_gt_[i];
    } else {
      loader->getScanAndPose(i, *cloud_raw, pose);
    }

    loader->rejectNeighboringPoints(*cloud_raw, mapgen->robot_body_size_, *cloud, *noise);
    mapgen->accumPointCloud(*cloud, pose);
  }
  mapgen_bar.finish();

  string abs_save_dir = mapgen->abs_save_dir_;
  string interval     = to_string(mapgen->accum_interval_);
  string voxel_size   = erasor_utils::format(mapgen->voxel_size_, 2);
  replace(voxel_size.begin(), voxel_size.end(), '.', '_');

  string original_path = abs_save_dir + "/" + mapgen->sequence_ + "_" + to_string(start_frame) +
                         "_to_" + to_string(end_frame) + "_w_interval_" + interval + "_voxel_" +
                         voxel_size + "_original.pcd";

  string map_path = abs_save_dir + "/" + mapgen->sequence_ + "_" + to_string(start_frame) + "_to_" +
                    to_string(end_frame) + "_w_interval_" + interval + "_voxel_" + voxel_size +
                    ".pcd";

  mapgen->saveAccumMap(original_path, map_path);

  erasor2::viz::shutdown();
  return 0;
}
