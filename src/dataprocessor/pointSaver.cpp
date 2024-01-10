#include "dataprocessor/PointCloudProcessor.h"
#include <signal.h>

void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    exit(signum);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_processor");
  ros::NodeHandle nh;

  std::string absPath;
  std::vector<std::string> process_lidar_list;
  std::string saveDir;
  std::string saveFormat;
  std::string trajectoryDir;

  nh.param<std::string>("/dataprocessor/dataset_root", absPath, "/home/ericlab");
  nh.param<std::vector<std::string>>("/dataprocessor/process_lidar_list", process_lidar_list, {"Ouster", "Velodyne", "Livox", "Aeva"});
  nh.param<std::string>("/dataloader/abs_data_dir", saveDir, "/home/ericlab");
  nh.param<std::string>("/dataprocessor/save_ins_to_LiDAR_root", trajectoryDir, "/home/ericlab");
  nh.param<std::string>("/dataprocessor/saveFormat", saveFormat, "bin");

  
  for(auto sensorType : process_lidar_list)
  {
    signal(SIGINT, signal_callback_handler);
    PointCloudProcessor processor(absPath, sensorType, saveDir, trajectoryDir, saveFormat);
    processor.ProcessBinFiles();
  }
  
  // PointCloudProcessor processor(absPath, sensorType, saveDir);

  return 0;
}
