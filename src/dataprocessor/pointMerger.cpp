#include <signal.h>

#include <algorithm>

#include <yaml-cpp/yaml.h>

#include "dataprocessor/PointCloudProcessor.h"
#include "dataprocessor/utility.h"

static void signal_callback_handler(int signum) {
  std::cout << "Caught Ctrl + c " << std::endl;
  std::exit(signum);
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: merge_heliclouds <config.yaml>\n";
    return 1;
  }
  YAML::Node root = YAML::LoadFile(argv[1]);

  auto get_str = [&](std::initializer_list<const char *> keys, const std::string &fallback) {
    YAML::Node n = root;
    for (auto k : keys) {
      if (!n || !n.IsMap() || !n[k]) return fallback;
      n = n[k];
    }
    return n.as<std::string>(fallback);
  };

  std::string absPath       = get_str({"dataprocessor", "dataset_root"}, "/home/ericlab");
  std::string saveDir       = get_str({"dataloader", "abs_data_dir"}, "/home/ericlab");
  std::string trajectoryDir = get_str({"dataprocessor", "save_ins_to_LiDAR_root"}, "/home/ericlab");
  std::string saveFormat    = get_str({"dataprocessor", "saveFormat"}, "bin");

  std::vector<std::string> process_lidar_list{"Ouster", "Velodyne", "Livox", "Aeva"};
  if (auto dp = root["dataprocessor"]; dp && dp["process_lidar_list"]) {
    process_lidar_list = dp["process_lidar_list"].as<std::vector<std::string>>();
  }

  PointCloudProcessor OusterProcessor(absPath, "Ouster", saveDir, trajectoryDir, saveFormat);
  PointCloudProcessor VelodyneProcessor(absPath, "Velodyne", saveDir, trajectoryDir, saveFormat);
  PointCloudProcessor AviaProcessor(absPath, "Avia", saveDir, trajectoryDir, saveFormat);
  PointCloudProcessor AevaProcessor(absPath, "Aeva", saveDir, trajectoryDir, saveFormat);

  const auto &ts_o = OusterProcessor.timestamp_lists_;
  const auto &ts_v = VelodyneProcessor.timestamp_lists_;
  const auto &ts_a = AviaProcessor.timestamp_lists_;
  const auto &ts_e = AevaProcessor.timestamp_lists_;

  std::cout << std::fixed << OusterProcessor.timestamp_lists_[0] << std::endl;
  std::cout << VelodyneProcessor.timestamp_lists_[0] << std::endl;
  std::cout << AviaProcessor.timestamp_lists_[0] << std::endl;
  std::cout << AevaProcessor.timestamp_lists_[0] << std::endl;

  auto isDifferenceWithin = [](long long a, long long b, double diff_in_sec) {
    auto diff = static_cast<double>(std::abs(a - b)) / 1e9;
    return std::abs(diff) <= diff_in_sec;
  };

  auto findMinMaxSize = [](const std::vector<long long> &ts_o,
                           const std::vector<long long> &ts_v,
                           const std::vector<long long> &ts_a,
                           const std::vector<long long> &ts_e) {
    size_t minSize = std::min({ts_o.size(), ts_v.size(), ts_a.size(), ts_e.size()});
    size_t maxSize = std::max({ts_o.size(), ts_v.size(), ts_a.size(), ts_e.size()});

    return std::make_tuple(minSize, maxSize);
  };

  if (ts_o[0] < ts_v[0] && ts_o[0] < ts_a[0] && ts_o[0] < ts_e[0]) {
    std::cout << std::fixed << "Check Ouster comes first!" << std::endl;
  } else {
    std::runtime_error(
        "Currently, only KAIST05 is supported while strongly assuming that the Ouster comes first");
  }

  double diff_in_sec =
      0.08;  // why not 0.1? I think 0.1 may wrongly include the next frame, so I just set 0.08

  const auto &[minSize, maxSize] = findMinMaxSize(ts_o, ts_v, ts_a, ts_e);
  // Frame wrt Ouster is criteria
  // Please check the timestamp of each frame is within `diff_in_sec`
  for (int i = 0; i < minSize; ++i) {
    if (isDifferenceWithin(ts_o[i], ts_v[i], diff_in_sec) &&
        isDifferenceWithin(ts_o[i], ts_a[i], diff_in_sec) &&
        isDifferenceWithin(ts_o[i], ts_e[i], diff_in_sec)) {
      continue;
    } else {
      std::runtime_error("Something's wrong. Timestamp is not matched :(");
    }
  }

  // Visualization publishers were Rviz-only diagnostics; merging is the
  // load-bearing work, and the merged PCDs are saved to disk just below.
  // Drop the publishers entirely rather than wire them up to rerun for a
  // utility that runs to completion in a few minutes.

  string mergedSavePath = saveDir + "/Merged/velodyne/";
  pcl::PCDWriter pcdWriter;
  for (int i = 0; i < maxSize; ++i) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr ousterCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulatedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // 1. Transform each point cloud to the Ouster frame
    const Eigen::Matrix4f T_o = OusterProcessor.gt_poses_.at(i);

    pcl::PointXYZI minPt, maxPt;

    loadCloud(i, OusterProcessor.savePath, "bin", *ousterCloud);
    *accumulatedCloud = *ousterCloud;

    pcl::getMinMax3D(*accumulatedCloud, minPt, maxPt);
    std::cout << "\033[1;32mOuster" << std::endl;
    std::cout << "Min : " << minPt.x << ", " << minPt.y << ", " << minPt.z << std::endl;
    std::cout << "Min : " << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "\033[0m"
              << std::endl;

    std::cout << "\033[1;33mVelodyne" << std::endl;
    VelodyneProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;
    pcl::getMinMax3D(*transformedCloud, minPt, maxPt);
    std::cout << "Min : " << minPt.x << ", " << minPt.y << ", " << minPt.z << std::endl;
    std::cout << "Max : " << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "\033[0m"
              << std::endl;

    std::cout << "\033[1;34mAvia" << std::endl;
    AviaProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    pcl::getMinMax3D(*transformedCloud, minPt, maxPt);
    std::cout << "Min : " << minPt.x << ", " << minPt.y << ", " << minPt.z << std::endl;
    std::cout << "Max : " << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "\033[0m"
              << std::endl;

    std::cout << "\033[1;35mAeva" << std::endl;
    AevaProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    pcl::getMinMax3D(*transformedCloud, minPt, maxPt);
    std::cout << "Min : " << minPt.x << ", " << minPt.y << ", " << minPt.z << std::endl;
    std::cout << "Max : " << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "\033[0m"
              << std::endl;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
    sor.setInputCloud(accumulatedCloud);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(*sampledCloud);
    std::cout << accumulatedCloud->points.size() << " -----> " << sampledCloud->size() << std::endl;
    if (saveFormat == "pcd") {
      pcdWriter.writeBinary(mergedSavePath + padZeros(i, 6) + ".pcd", *sampledCloud);
    } else if (saveFormat == "bin") {
      saveToBinFile(mergedSavePath + padZeros(i, 6) + ".bin", *sampledCloud);
    }
  }

  return 0;
}
