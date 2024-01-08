#ifndef PointCloudProcessor_H
#define PointCloudProcessor_H

#include "BsplineSE3.h"
#include "visualizer.h"
#include <iostream>
#include <sstream>
#include <fstream>

#include "ros/ros.h"

using namespace ov_core;
using namespace std;

class PointCloudProcessor
{
public:
    PointCloudProcessor(std::string absPath, std::string sensorType, std::string saveDir, std::string trajectoryDir);
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

    std::vector<Eigen::VectorXd> trajPoints;
    Point3D lastPoint;
    int keyIndex = 0;
    int numBins = 0;

    std::string absPath;
    std::string sensorType;
    std::string binPath;
    std::string originPose;
    std::string saveDir;
    std::string savePath; // 디스큐드 된 포인트 클라우
    std::string trajPath;

    std::ofstream fout_pose; // for poses.txt

    LiDARType LiDAR = OUSTER;    // Ouster, Velodyne, Livox, Aeva (Same as the folder name)
    int distanceThreshold = 10;  // saved pointcloud distanceThreshold (m) (default: 10, >= 0)
    int numIntervals = 1000;     // number of intervals for interpolation (default: 1000, > 1)
    int accumulatedSize = 20;    // number of pointclouds to accumulate before processing (default: 20, > 1)
    bool downSampleFlag = true;  // downsample pointclouds before processing (default: true)
    float downSampleSize = 0.4f; // downsample size (m) (default: 0.4f)
    bool undistortFlag = true;   // undistort pointclouds before processing (default: true)

    void gatherInput();
    void displayBanner();
    void displayInput();

    // pose save 를 위한 함수 
    void loadAllPoses(const std::string &pose_path, std::vector<Eigen::Matrix4f> &poses, std::vector<float> &timestamps_);
    void vec2tf4x4(std::vector<float> &pose, Eigen::Matrix4f &tf4x4);
    std::vector<float> splitLine(const std::string &input, char delimiter);

    std::vector<Eigen::Matrix4f> gt_poses_;
    std::vector<float> timestamp_lists_;

    void interpolate(double timestamp, double timeStart, double dt,
                     const std::vector<Eigen::Quaterniond> &quaternions,
                     const std::vector<Eigen::Vector3d> &positions,
                     int numIntervals, Eigen::Quaterniond &qOut,
                     Eigen::Vector3d &pOut);

    template <class T>
    void processPoint(T &point, double timestamp, double timeStart, double dt, Eigen::Quaterniond &qStart, Eigen::Vector3d &pStart,
                      const std::vector<Eigen::Quaterniond> &quaternions,
                      const std::vector<Eigen::Vector3d> &positions,
                      int numIntervals, Eigen::Quaterniond &qOut,
                      Eigen::Vector3d &pOut);

    template <class T>
    void accumulateScans(std::vector<pcl::PointCloud<T>> &vecCloud, Point3D &lastPoint);

    // void readBinFile(const std::string &filename, pcl::PointCloud<OusterPointXYZIRT> &cloud);
    // void readBinFile(const std::string &filename, pcl::PointCloud<PointXYZIRT> &cloud);
    // void readBinFile(const std::string &filename, pcl::PointCloud<LivoxPointXYZI> &cloud);
    // void readBinFile(const std::string &filename, pcl::PointCloud<AevaPointXYZIRT> &cloud, double timeStart);

    void readBinFile(const std::string &filename, pcl::PointCloud<OusterPointXYZIRT> &cloud, pcl::PointCloud<pcl::PointXYZI> &save_cloud);
    void readBinFile(const std::string &filename, pcl::PointCloud<PointXYZIRT> &cloud, pcl::PointCloud<pcl::PointXYZI> &save_cloud);
    void readBinFile(const std::string &filename, pcl::PointCloud<LivoxPointXYZI> &cloud, pcl::PointCloud<pcl::PointXYZI> &save_cloud);
    void readBinFile(const std::string &filename, pcl::PointCloud<AevaPointXYZIRT> &cloud, double timeStart, pcl::PointCloud<pcl::PointXYZI> &save_cloud);


    void processFile(const std::string &filename);
    void loadAndProcessBinFiles();
    void loadTrajectory();
};

#endif