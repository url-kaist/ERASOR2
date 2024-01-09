#include "dataprocessor/PointCloudProcessor.h"
#include <signal.h>
#include <algorithm>

void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    exit(signum);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_merger");
  ros::NodeHandle nh;

  std::string absPath;
  std::vector<std::string> process_lidar_list;
  std::string saveDir;
  std::string trajectoryDir;

  nh.param<std::string>("/dataprocessor/dataset_root", absPath, "/home/ericlab");
  nh.param<std::vector<std::string>>("/dataprocessor/process_lidar_list", process_lidar_list, {"Ouster", "Velodyne", "Livox", "Aeva"});
  nh.param<std::string>("/dataloader/abs_data_dir", saveDir, "/home/ericlab");
  nh.param<std::string>("/dataprocessor/save_ins_to_LiDAR_root", trajectoryDir, "/home/ericlab");

  PointCloudProcessor OusterProcessor(absPath, "Ouster", saveDir, trajectoryDir);
  PointCloudProcessor VelodyneProcessor(absPath, "Velodyne", saveDir, trajectoryDir);
  PointCloudProcessor AviaProcessor(absPath, "Avia", saveDir, trajectoryDir);
  PointCloudProcessor AevaProcessor(absPath, "Aeva", saveDir, trajectoryDir);

  const auto &t_o = OusterProcessor.timestamp_lists_;
  const auto &t_v = VelodyneProcessor.timestamp_lists_;
  const auto &t_a = AviaProcessor.timestamp_lists_;
  const auto &t_ae = AevaProcessor.timestamp_lists_;

  std::cout << std::fixed << OusterProcessor.timestamp_lists_[0] << std::endl;
  std::cout << VelodyneProcessor.timestamp_lists_[0] << std::endl;
  std::cout << AviaProcessor.timestamp_lists_[0] << std::endl;
  std::cout << AevaProcessor.timestamp_lists_[0] << std::endl;
  if (t_o[0] < t_v[0] && t_o[0] < t_a[0] && t_o[0] < t_ae[0]) {
      double t_wrong = t_o[0];
      std::cout << std::fixed << "Significant figures check:" << t_wrong << std::endl;
      double t_right = t_o[0] / 1e9;
      std::cout << std::fixed << "Significant figures check:" << t_right << std::endl;
  } else { std::runtime_error("Currently, only KAIST05 is supported"); }

  return 0;
}
