#include <signal.h>

#include <yaml-cpp/yaml.h>

#include "dataprocessor/PointCloudProcessor.h"

static void signal_callback_handler(int signum) {
  std::cout << "Caught Ctrl + c " << std::endl;
  std::exit(signum);
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: helipr_to_kitti <config.yaml>\n";
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

  std::string home_dir = std::getenv("HOME");
  absPath              = home_dir + absPath;
  saveDir              = home_dir + saveDir;
  trajectoryDir        = home_dir + trajectoryDir;

  for (auto sensorType : process_lidar_list) {
    signal(SIGINT, signal_callback_handler);
    PointCloudProcessor processor(absPath, sensorType, saveDir, trajectoryDir, saveFormat);
    processor.ProcessBinFiles();
  }

  // PointCloudProcessor processor(absPath, sensorType, saveDir);

  return 0;
}
