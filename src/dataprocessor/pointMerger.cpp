#include "dataprocessor/PointCloudProcessor.h"
#include "dataprocessor/utility.h"
#include <signal.h>
#include <algorithm>

void signal_callback_handler(int signum)
{
  cout << "Caught Ctrl + c " << endl;
  exit(signum);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_merger");
  ros::NodeHandle nh;

  std::string              absPath;
  std::vector<std::string> process_lidar_list;
  std::string              saveDir;
  std::string              saveFormat = "bin";
  std::string              trajectoryDir;

  nh.param<std::string>("/dataprocessor/dataset_root", absPath, "/home/ericlab");
  nh.param<std::vector<std::string>>("/dataprocessor/process_lidar_list",
                                     process_lidar_list,
                                     {"Ouster", "Velodyne", "Livox", "Aeva"});
  nh.param<std::string>("/dataloader/abs_data_dir", saveDir, "/home/ericlab");
  nh.param<std::string>("/dataprocessor/save_ins_to_LiDAR_root", trajectoryDir, "/home/ericlab");

  PointCloudProcessor OusterProcessor(absPath, "Ouster", saveDir, trajectoryDir);
  PointCloudProcessor VelodyneProcessor(absPath, "Velodyne", saveDir, trajectoryDir);
  PointCloudProcessor AviaProcessor(absPath, "Avia", saveDir, trajectoryDir);
  PointCloudProcessor AevaProcessor(absPath, "Aeva", saveDir, trajectoryDir);

  const auto &ts_o  = OusterProcessor.timestamp_lists_;
  const auto &ts_v  = VelodyneProcessor.timestamp_lists_;
  const auto &ts_a  = AviaProcessor.timestamp_lists_;
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
  } else { std::runtime_error("Currently, only KAIST05 is supported while strongly assuming that the Ouster comes first"); }

  double diff_in_sec = 0.08; // why not 0.1? I think 0.1 may wrongly include the next frame, so I just set 0.08

  const auto &[minSize, maxSize] = findMinMaxSize(ts_o, ts_v, ts_a, ts_e);
  // Frame wrt Ouster is criteria
  // Please check the timestamp of each frame is within `diff_in_sec`
  for (int   i                   = 0; i < minSize; ++i) {
    if (isDifferenceWithin(ts_o[i], ts_v[i], diff_in_sec) && isDifferenceWithin(ts_o[i], ts_a[i], diff_in_sec)
      && isDifferenceWithin(ts_o[i], ts_e[i], diff_in_sec)) {
      continue;
    } else {
      std::runtime_error("Something's wrong. Timestamp is not matched :(");
    }
  }

  ros::Publisher CloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_cloud", 100, true);
  ros::Publisher VoxelPublisher = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_voxel", 100, true);

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
    std::cout << "Min : " << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "\033[0m" << std::endl;

    std::cout << "\033[1;33mVelodyne" << std::endl;
    VelodyneProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;
    pcl::getMinMax3D(*transformedCloud, minPt, maxPt);
    std::cout << "Min : " << minPt.x << ", " << minPt.y << ", " << minPt.z << std::endl;
    std::cout << "Max : " << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "\033[0m" << std::endl;

    std::cout << "\033[1;34mAvia" << std::endl;
    AviaProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    pcl::getMinMax3D(*transformedCloud, minPt, maxPt);
    std::cout << "Min : " << minPt.x << ", " << minPt.y << ", " << minPt.z << std::endl;
    std::cout << "Max : " << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "\033[0m" << std::endl;

    std::cout << "\033[1;35mAeva" << std::endl;
    AevaProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    pcl::getMinMax3D(*transformedCloud, minPt, maxPt);
    std::cout << "Min : " << minPt.x << ", " << minPt.y << ", " << minPt.z << std::endl;
    std::cout << "Max : " << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "\033[0m" << std::endl;

    pcl::VoxelGrid<pcl::PointXYZI>       sor;
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

    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(*accumulatedCloud, cloud_ROS);
    cloud_ROS.header.frame_id = "map";
    CloudPublisher.publish(cloud_ROS);

    pcl::toROSMsg(*sampledCloud, cloud_ROS);
    cloud_ROS.header.frame_id = "map";
    VoxelPublisher.publish(cloud_ROS);
  }

  return 0;
}
