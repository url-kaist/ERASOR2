#ifndef PointCloudProcessor_H
#define PointCloudProcessor_H

#include <fstream>
#include <iostream>
#include <sstream>

#include <boost/format.hpp>
#include <pcl/common/common.h>

#include "BsplineSE3.h"
#include "visualizer.h"

using namespace ov_core;
using namespace std;

template <typename T>
void saveToBinFile(const std::string &filename, const pcl::PointCloud<T> &cloud) {
  std::ofstream outFile(filename, std::ios::out | std::ios::binary);
  if (!outFile) {
    std::cerr << "Cannot open file for writing: " << filename << std::endl;
    return;
  }

  for (const auto &point : cloud) {
    outFile.write(reinterpret_cast<const char *>(&point.x), sizeof(point.x));
    outFile.write(reinterpret_cast<const char *>(&point.y), sizeof(point.y));
    outFile.write(reinterpret_cast<const char *>(&point.z), sizeof(point.z));
    if (std::is_same<T, pcl::PointXYZ>::value) {
      outFile.write(reinterpret_cast<const char *>(0), sizeof(point.z));
    } else if (std::is_same<T, pcl::PointXYZI>::value) {
      outFile.write(reinterpret_cast<const char *>(&point.intensity), sizeof(point.intensity));
    }
  }
  outFile.close();
}

template <typename T>
int loadCloud(size_t idx, string cloud_dir, string cloud_format, pcl::PointCloud<T> &cloud) {
  if (cloud_dir.back() == '/') {
    cloud_dir.pop_back();
  }
  string filename = (boost::format("%s/%06d.%s") % cloud_dir % idx % cloud_format).str();
  FILE *file      = fopen(filename.c_str(), "rb");
  if (!file) {
    std::cerr << "Error: failed to load " << filename << std::endl;
    return -1;
  }

  std::vector<float> buffer(2000000);
  size_t num_points =
      fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
  fclose(file);

  cloud.resize(num_points);
  if (std::is_same<T, pcl::PointXYZ>::value) {
    for (int i = 0; i < num_points; i++) {
      auto &pt = cloud.at(i);
      pt.x     = buffer[i * 4];
      pt.y     = buffer[i * 4 + 1];
      pt.z     = buffer[i * 4 + 2];
    }
  } else if (std::is_same<T, pcl::PointXYZI>::value) {
    for (int i = 0; i < num_points; i++) {
      auto &pt     = cloud.at(i);
      pt.x         = buffer[i * 4];
      pt.y         = buffer[i * 4 + 1];
      pt.z         = buffer[i * 4 + 2];
      pt.intensity = buffer[i * 4 + 3];
    }
  }
  return 0;
}

class PointCloudProcessor {
 public:
  PointCloudProcessor(std::string absPath,
                      std::string sensorType,
                      std::string saveDir,
                      std::string trajectoryDir,
                      std::string saveFormat = "bin");
  ~PointCloudProcessor();

  BsplineSE3 bsplineSE3;
  Visualizer visualizer;
  std::vector<Eigen::Quaterniond> scanQuat;
  std::vector<Eigen::Vector3d> scanTrans;
  std::vector<std::string> scanTimestamps;
  std::vector<Point3D> scanPoints;

  std::vector<pcl::PointCloud<OusterPointXYZIRT>> vecOuster;
  std::vector<pcl::PointCloud<PointXYZIRT>> vecVelodyne;
  std::vector<pcl::PointCloud<LivoxPointXYZI>> vecLivox;
  std::vector<pcl::PointCloud<AevaPointXYZIRT>> vecAeva;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> vecXYZI;

  std::vector<std::string> binFiles;
  std::vector<Eigen::VectorXd> trajPoints;
  Point3D lastPoint;
  int keyIndex = 0;
  int numSaved = 0;
  int numBins  = 0;

  std::string absPath;
  std::string sensorType;
  std::string binPath;
  std::string originPose;
  std::string saveDir;
  std::string savePath;  // 디스큐드 된 포인트 클라우드
  std::string saveFormat = "bin";
  std::string trajPath;
  std::string posesTxt;

  std::ofstream fout_pose;  // for poses.txt

  LiDARType LiDAR       = OUSTER;  // Ouster, Velodyne, Livox, Aeva (Same as the folder name)
  int distanceThreshold = 10;      // saved pointcloud distanceThreshold (m) (default: 10, >= 0)
  int numIntervals      = 1000;    // number of intervals for interpolation (default: 1000, > 1)
  int accumulatedSize =
      20;  // number of pointclouds to accumulate before processing (default: 20, > 1)
  bool downSampleFlag  = true;  // downsample pointclouds before processing (default: true)
  float downSampleSize = 0.4f;  // downsample size (m) (default: 0.4f)
  bool undistortFlag   = true;  // undistort pointclouds before processing (default: true)

  void gatherInput();
  void displayBanner();
  void displayInput();

  // pose save 를 위한 함수
  void loadAllPoses(const std::string &pose_path,
                    std::vector<Eigen::Matrix4f> &poses,
                    std::vector<long long> &timestamps_);
  void vec2tf4x4(const std::vector<float> &pose, Eigen::Matrix4f &tf4x4);
  std::pair<long long, std::vector<float>> splitLine(std::string input, char delimiter);

  std::vector<Eigen::Matrix4f> gt_poses_;
  std::vector<long long> timestamp_lists_;

  void interpolate(double timestamp,
                   double timeStart,
                   double dt,
                   const std::vector<Eigen::Quaterniond> &quaternions,
                   const std::vector<Eigen::Vector3d> &positions,
                   int numIntervals,
                   Eigen::Quaterniond &qOut,
                   Eigen::Vector3d &pOut);

  void getTransformedCloud(const int i,
                           const Eigen::Matrix4f T_criterion,
                           pcl::PointCloud<pcl::PointXYZI> &transformed);

  template <class T>
  void processPoint(T &point,
                    double timestamp,
                    double timeStart,
                    double dt,
                    Eigen::Quaterniond &qStart,
                    Eigen::Vector3d &pStart,
                    const std::vector<Eigen::Quaterniond> &quaternions,
                    const std::vector<Eigen::Vector3d> &positions,
                    int numIntervals,
                    Eigen::Quaterniond &qOut,
                    Eigen::Vector3d &pOut);

  template <class T>
  void accumulateScans(std::vector<pcl::PointCloud<T>> &vecCloud, Point3D &lastPoint);

  // void readBinFile(const std::string &filename, pcl::PointCloud<OusterPointXYZIRT> &cloud);
  // void readBinFile(const std::string &filename, pcl::PointCloud<PointXYZIRT> &cloud);
  // void readBinFile(const std::string &filename, pcl::PointCloud<LivoxPointXYZI> &cloud);
  // void readBinFile(const std::string &filename, pcl::PointCloud<AevaPointXYZIRT> &cloud, double
  // timeStart);

  void readBinFile(const std::string &filename,
                   pcl::PointCloud<OusterPointXYZIRT> &cloud,
                   pcl::PointCloud<pcl::PointXYZI> &save_cloud);
  void readBinFile(const std::string &filename,
                   pcl::PointCloud<PointXYZIRT> &cloud,
                   pcl::PointCloud<pcl::PointXYZI> &save_cloud);
  void readBinFile(const std::string &filename,
                   pcl::PointCloud<LivoxPointXYZI> &cloud,
                   pcl::PointCloud<pcl::PointXYZI> &save_cloud);
  void readBinFile(const std::string &filename,
                   pcl::PointCloud<AevaPointXYZIRT> &cloud,
                   double timeStart,
                   pcl::PointCloud<pcl::PointXYZI> &save_cloud);

  void processFile(const std::string &filename);
  void ProcessBinFiles();
  void loadTrajectory();
  void loadBinFileNames();
};

#endif
