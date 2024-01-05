#include "PointCloudProcessor.h"

PointCloudProcessor::PointCloudProcessor()
{
  displayBanner();
  gatherInput();
  displayInput();
  visualizer.progressBar(0, 1, "", false);
  loadTrajectory();
  loadAndProcessBinFiles();
}

PointCloudProcessor::~PointCloudProcessor() {}

void PointCloudProcessor::displayBanner()
{
// Clear the screen
#ifdef _WIN32
  system("cls");
#else
  system("clear");
#endif

  // Set a fixed width for the banner
  const int bannerWidth = 80; // Adjust this width as needed

  // Define the banner text
  const std::string title = "HeLiPR Point Cloud Undistortion and Accumulation Tool";
  const std::string description = "This utility facilitates the undistortion of .bin point cloud files, "
                                  "converts them to .pcd format, and supports point cloud accumulation. "
                                  "Please follow the prompts to set up your processing parameters.";

  const std::string inputInfo = "Input: .bin files, trajectory file\n"
                                "Output: .pcd files\n"
                                "1. Bin files from HeLiPR should be needed.\n"
                                "2. trajectory file is tum form made by transformINStoLiDAR.py.";

  const std::string maintainerInfo = "Maintainer: Minwoo Jung (moonshot@snu.ac.kr, SNU RPM Lab), Revised: 2023/12/06";

  int titlePadding = (bannerWidth - title.length()) / 2;
  titlePadding = titlePadding < 0 ? 0 : titlePadding; // Ensure no negative padding

  // Display the banner with fixed width
  std::cout << cyan << std::string(bannerWidth, '=') << reset << "\n";
  std::cout << std::string(titlePadding, ' ') << yellow << title << reset << "\n";
  std::cout << cyan << std::string(bannerWidth, '*') << reset << "\n";

  // Wrap and display the description and input information
  std::string combinedInfo = description;
  std::istringstream words(combinedInfo);
  std::string word;
  std::string line;
  while (words >> word)
  {
    if (line.length() + word.length() + 1 > bannerWidth)
    {
      std::cout << line << std::endl;
      line = word;
    }
    else
    {
      line += (line.empty() ? "" : " ") + word;
    }
  }
  if (!line.empty())
    std::cout << line << std::endl;

  // Display Information
  std::cout << cyan << std::string(bannerWidth, '-') << reset << "\n";
  std::cout << inputInfo << "\n";
  std::cout << cyan << std::string(bannerWidth, '-') << reset << "\n";
  std::cout << maintainerInfo << "\n";
  std::cout << cyan << std::string(bannerWidth, '=') << reset << "\n\n";
}

void PointCloudProcessor::gatherInput()
{
  absPath = visualizer.getInput(cyan + "Enter the path to the directory containing the .bin files (end with folder/): " + reset);
  sensorType = visualizer.getInput(cyan + "Enter the sensor type: " + reset);
  
  binPath = absPath + "/LiDAR/" + sensorType + "/";
  originPose = absPath + "/LiDAR_GT/" + sensorType + "_gt.txt";

  while (!std::filesystem::exists(binPath) || binPath.substr(binPath.size() - 1) != "/")
  {
    absPath = visualizer.getInput(cyan + "Enter the path to the directory containing the .bin files (end with folder/): " + reset);
    sensorType = visualizer.getInput(cyan + "Enter the sensor type: " + reset);

    binPath = absPath + "/LiDAR/" + sensorType + "/";
    originPose = absPath + "/LiDAR_GT/" + sensorType + "_gt.txt";
  }

  loadAllPoses(originPose, gt_poses_, timestamp_lists_);

  trajPath = visualizer.getInput(cyan + "Enter the path to the trajectory file: " + reset);
  while (!std::filesystem::exists(trajPath) || trajPath.substr(trajPath.size() - 4) != ".txt")
  {
    std::cout << red << "The path does not exist or the file is not txt. Please try again." << reset << std::endl;
    trajPath = visualizer.getInput(cyan + "Enter the path to the trajectory file: " + reset);
  }

  bool pathCreated = false;
  while (!pathCreated)
  {
    saveDir = visualizer.getInput(cyan + "Enter the path to the directory to save the processed point clouds (end with '/'): " + reset);
    // Check if the path ends with '/'
    savePath = saveDir + "velodyne/";
    
    std::string poses_txt = saveDir + "poses.txt";
    fout_pose.open(poses_txt);
    std::cout << "poses.txt is created in " << poses_txt << std::endl;


    if (savePath.back() != '/')
    {
      std::cout << red << "The path must end with '/'. Please try again." << reset << std::endl;
    }
    else
    {
      // Check if the path exists
      if (!std::filesystem::exists(savePath))
      {
        std::cout << yellow << "The path does not exist. Creating a new directory." << reset << std::endl;
        // Attempt to create the directory
        if (!std::filesystem::create_directory(savePath))
        {
          std::cout << red << "Failed to create the directory. Please try again." << reset << std::endl;
        }
        else
        {
          pathCreated = true; // Directory created successfully
        }
      }
      else
      {
        pathCreated = true; // Path already exists
      }
    }
  }

  int lidarInput = visualizer.getInput<int>(cyan + "Enter the LiDAR type (0: Ouster, 1: Velodyne, 2: Livox, 3: Aeva): " + reset, 0);
  LiDAR = static_cast<LiDARType>(lidarInput);
  while (LiDAR < 0 || LiDAR > 3)
  {
    std::cout << red << "Invalid LiDAR type. Please try again." << reset << std::endl;
    lidarInput = visualizer.getInput<int>(cyan + "Enter the LiDAR type (0: Ouster, 1: Velodyne, 2: Livox, 3: Aeva): " + reset, 0);
    LiDAR = static_cast<LiDARType>(lidarInput);
  }

  distanceThreshold = visualizer.getInput<float>(cyan + "Enter the distance threshold (m) (default: 10, >= 0): " + reset, 10.0f);
  while (distanceThreshold < 0)
  {
    std::cout << red << "Invalid distance threshold. Please try again." << reset << std::endl;
    distanceThreshold = visualizer.getInput<float>(cyan + "Enter the distance threshold (m) (default: 10, >= 0): " + reset, 10.0f);
  }

  numIntervals = visualizer.getInput<int>(cyan + "Enter the number of intervals for interpolation (default: 1000, >= 1): " + reset, 1000);
  while (numIntervals < 1)
  {
    std::cout << red << "Invalid number of intervals. Please try again." << reset << std::endl;
    numIntervals = visualizer.getInput<int>(cyan + "Enter the number of intervals for interpolation (default: 1000, >= 1): " + reset, 1000);
  }

  accumulatedSize = visualizer.getInput<int>(cyan + "Enter the number of point clouds to accumulate before processing (default: 20, >= 1): " + reset, 20);
  while (accumulatedSize < 1)
  {
    std::cout << red << "Invalid accumulated size. Please try again." << reset << std::endl;
    accumulatedSize = visualizer.getInput<int>(cyan + "Enter the number of point clouds to accumulate before processing (default: 20, >= 1): " + reset, 20);
  }

  downSampleFlag = visualizer.getInput<bool>(cyan + "Enter the downsample flag (0: false, 1: true) (default: 1): " + reset, true);
  while (downSampleFlag < 0 || downSampleFlag > 1)
  {
    std::cout << red << "Invalid downsample flag. Please try again." << reset << std::endl;
    downSampleFlag = visualizer.getInput<bool>(cyan + "Enter the downsample flag (0: false, 1: true) (default: 1): " + reset, true);
  }

  if (downSampleFlag)
  {
    downSampleSize = visualizer.getInput<float>(cyan + "Enter the downsample size (m) (default: 0.4): " + reset, 0.4f);
    while (downSampleSize < 0)
    {
      std::cout << red << "Invalid downsample size. Please try again." << reset << std::endl;
      downSampleSize = visualizer.getInput<float>(cyan + "Enter the downsample size (m) (default: 0.4): " + reset, 0.4f);
    }
  }

  undistortFlag = visualizer.getInput<bool>(cyan + "Enter the undistort flag (0: false, 1: true) (default: 1): " + reset, true);
  while (undistortFlag < 0 || undistortFlag > 1)
  {
    std::cout << red << "Invalid undistort flag. Please try again." << reset << std::endl;
    undistortFlag = visualizer.getInput<bool>(cyan + "Enter the undistort flag (0: false, 1: true) (default: 1): " + reset, true);
  }
}

void PointCloudProcessor::displayInput()
{
  const int width = 20; // Set the width for the first column

  std::cout << green << std::string(40, '-') << reset << std::endl; // Divider line
  std::cout << green << std::left << std::setw(width) << "Parameter"
            << "Value" << reset << std::endl;
  std::cout << green << std::string(40, '-') << reset << std::endl; // Divider line
  std::cout << green << std::left << std::setw(width) << "binPath:" << binPath << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "trajPath:" << trajPath << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "savePath:" << savePath << reset << std::endl;
  std::string lidarTypeStr = visualizer.lidarTypeToString(LiDAR);
  std::cout << green << std::left << std::setw(width) << "LiDAR:" << lidarTypeStr << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Distance Threshold:" << distanceThreshold << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Num Intervals:" << numIntervals << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Accumulated Size:" << accumulatedSize << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Downsample Flag:" << (downSampleFlag ? "True" : "False") << reset << std::endl;
  if (downSampleFlag)
    std::cout << green << std::left << std::setw(width) << "Downsample Size:" << downSampleSize << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Undistort Flag:" << (undistortFlag ? "True" : "False") << reset << std::endl;
  std::cout << std::endl;
}

std::vector<float> PointCloudProcessor::splitLine(const std::string &input, char delimiter)
{
  std::vector<float> result;
  stringstream ss(input);
  string token;
  while (getline(ss, token, delimiter))
  {
    result.push_back(stof(token));
  }
  return result;  
}


void PointCloudProcessor::loadAllPoses(const std::string &pose_path, std::vector<Eigen::Matrix4f> &poses, std::vector<float> &timestamps_)
{
  poses.clear();
  poses.reserve(20000);
  std::ifstream in(pose_path);
  std::string line;

  int count = 0;
  while (std::getline(in, line)){
    std::vector<float> pose = splitLine(line, ' ');
    Eigen::Matrix4f tf4x4_sensor = Eigen::Matrix4f::Identity();
    vec2tf4x4(pose, tf4x4_sensor);
    poses.push_back(tf4x4_sensor);
    timestamps_.push_back(pose[0]);
    count++;
  }
  in.close();
  std::cout << "Total " << count << " poses are loaded" << std::endl;
}

void PointCloudProcessor::vec2tf4x4(std::vector<float> &pose, Eigen::Matrix4f &tf4x4)
{
  Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose[7], pose[4], pose[5], pose[6]).toRotationMatrix();
  tf4x4 << mat3(0, 0), mat3(0, 1), mat3(0, 2), pose[1],
            mat3(1, 0), mat3(1, 1), mat3(1, 2), pose[2],
            mat3(2, 0), mat3(2, 1), mat3(2, 2), pose[3],
            0, 0, 0, 1;
}


void PointCloudProcessor::interpolate(double timestamp, double timeStart, double dt,
                                      const std::vector<Eigen::Quaterniond> &quaternions,
                                      const std::vector<Eigen::Vector3d> &positions,
                                      int numIntervals, Eigen::Quaterniond &qOut,
                                      Eigen::Vector3d &pOut)
{

  // Calculate the index in the quaternions and positions vectors
  int idx = (timestamp - timeStart) / dt;
  if (idx < 0)
    idx = 0;
  if (idx >= numIntervals - 1)
    idx = numIntervals - 2;

  // Calculate the interpolation factor
  double alpha = (timestamp - (timeStart + idx * dt)) / dt;

  // Interpolate the quaternion using Spherical Linear Interpolation (SLERP)
  qOut = quaternions[idx].slerp(alpha, quaternions[idx + 1]);

  // Interpolate the position linearly
  pOut = (1 - alpha) * positions[idx] + alpha * positions[idx + 1];
}

template <class T>
void PointCloudProcessor::processPoint(T &point, double timestamp, double timeStart, double dt, Eigen::Quaterniond &qStart, Eigen::Vector3d &pStart,
                                       const std::vector<Eigen::Quaterniond> &quaternions,
                                       const std::vector<Eigen::Vector3d> &positions,
                                       int numIntervals, Eigen::Quaterniond &qOut,
                                       Eigen::Vector3d &pOut)
{
  Eigen::Quaterniond qScan;
  Eigen::Vector3d pScan;
  interpolate(timestamp, timeStart, dt, quaternions, positions, numIntervals, qScan, pScan);
  Eigen::Vector3d transformedPoint(point.x, point.y, point.z);
  transformedPoint = qStart * ((qScan * transformedPoint + pScan) - pStart);
  point.x = transformedPoint(0);
  point.y = transformedPoint(1);
  point.z = transformedPoint(2);
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<OusterPointXYZIRT> &cloud)
{
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);

  while (!file.eof())
  {
    OusterPointXYZIRT point;
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.t), sizeof(uint32_t));
    file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&point.ambient), sizeof(uint16_t));
    cloud.push_back(point);
  }
  file.close();
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<PointXYZIRT> &cloud)
{
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);

  while (!file.eof())
  {
    PointXYZIRT point;
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&point.time), sizeof(float));
    cloud.push_back(point);
  }
  file.close();
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<LivoxPointXYZI> &cloud)
{
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);

  while (!file.eof())
  {
    LivoxPointXYZI point;
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
    file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
    file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
    file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
    cloud.push_back(point);
  }
  file.close();
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<AevaPointXYZIRT> &cloud, double timeStart)
{
  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);

  while (!file.eof())
  {
    AevaPointXYZIRT point;
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.velocity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.time_offset_ns), sizeof(int32_t));
    file.read(reinterpret_cast<char *>(&point.line_index), sizeof(uint8_t));
    if (timeStart > 1691936557946849179 / 1e9)
      file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    cloud.push_back(point);
  }
  file.close();
}

template <class T>
void PointCloudProcessor::accumulateScans(std::vector<pcl::PointCloud<T>> &vecCloud, Point3D &lastPoint)
{
  pcl::PCDWriter pcdWriter;
  pcl::PointCloud<T> accumulatedCloud;
  if (vecCloud.size() == accumulatedSize)
  {
    if (euclidean_distance(lastPoint, scanPoints[0]) > distanceThreshold)
    {
      visualizer.progressBar(keyIndex, numBins, padZeros(keyIndex - accumulatedSize, 6) + ".pcd", true);
      lastPoint = scanPoints[0];
      for (int i = 0; i < accumulatedSize; i++)
      { 
        auto pclIter = vecCloud[i].points.end() - 1;
        for (; pclIter != vecCloud[i].points.begin(); pclIter--)
        {
          T point = *pclIter;
          Eigen::Vector3d transformedPoint(pclIter->x, pclIter->y, pclIter->z);
          transformedPoint = scanQuat[0].conjugate() * ((scanQuat[i] * transformedPoint + scanTrans[i]) - scanTrans[0]);
          point.x = transformedPoint(0);
          point.y = transformedPoint(1);
          point.z = transformedPoint(2);
          accumulatedCloud.push_back(point);
        }
      }
      if (downSampleFlag)
      {
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        pcl::PointCloud<pcl::PointXYZI>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
        copyPointCloud(accumulatedCloud, *sampledCloud);
        sor.setInputCloud(sampledCloud);
        sor.setLeafSize(downSampleSize, downSampleSize, downSampleSize);
        sor.filter(*sampledCloud);
        pcdWriter.writeBinary(savePath + padZeros(keyIndex - accumulatedSize, 6) + ".pcd", *sampledCloud);
      }
      else
        pcdWriter.writeBinary(savePath + padZeros(keyIndex - accumulatedSize, 6) + ".pcd", accumulatedCloud);
      Eigen::Matrix4f tf4x4 = Eigen::Matrix4f::Identity();
      tf4x4 = gt_poses_[keyIndex - accumulatedSize];
      fout_pose << tf4x4(0, 0) << " " << tf4x4(0, 1) << " " << tf4x4(0, 2) << " " << tf4x4(0, 3) << " "
                << tf4x4(1, 0) << " " << tf4x4(1, 1) << " " << tf4x4(1, 2) << " " << tf4x4(1, 3) << " "
                << tf4x4(2, 0) << " " << tf4x4(2, 1) << " " << tf4x4(2, 2) << " " << tf4x4(2, 3) << std::endl; 
    }
    vecCloud.erase(vecCloud.begin());
    scanQuat.erase(scanQuat.begin());
    scanTrans.erase(scanTrans.begin());
    scanTimestamps.erase(scanTimestamps.begin());
    scanPoints.erase(scanPoints.begin());
  }
}

void PointCloudProcessor::processFile(const std::string &filename) // 여기서의 filename 은 LiDAR 파일의 파일명을 의미
{
  pcl::PointCloud<OusterPointXYZIRT>::Ptr scanOuster(new pcl::PointCloud<OusterPointXYZIRT>);
  pcl::PointCloud<PointXYZIRT>::Ptr scanVelodyne(new pcl::PointCloud<PointXYZIRT>);
  pcl::PointCloud<LivoxPointXYZI>::Ptr scanLivox(new pcl::PointCloud<LivoxPointXYZI>);
  pcl::PointCloud<AevaPointXYZIRT>::Ptr scanAeva(new pcl::PointCloud<AevaPointXYZIRT>);

  Eigen::Quaterniond qStart;
  Eigen::Quaterniond qScan;
  Eigen::Vector3d pStart;
  Eigen::Vector3d pScan;

  std::vector<Eigen::Quaterniond> quaternions(numIntervals); //인터벌로 지정해준 수만큼의 쿼터니온 및 position 을 가져와준다. 여기서 나는 1000으로 지정했다.
  std::vector<Eigen::Vector3d> positions(numIntervals);

  // Extract the timestamp from the filename
  std::size_t startPos = filename.find_last_of("/") + 1;
  std::size_t endPos = filename.find_last_of(".");
  std::string timestampStr = filename.substr(startPos, endPos - startPos);

  double timeStart = std::stod(timestampStr) / 1e9; // Convert from nanoseconds to seconds
  // 파일 이름으로 부터, 현재의 시간을 sec 단위로 가져와주고, 이를 timeStart 에 저장해준다.
  // time check with trajectory
  if (timeStart < trajPoints[0](0) - 100 || timeStart > trajPoints[trajPoints.size() - 1](0) + 100)
  {
    std::cout << red << "The timestamp is out of range. Please check the trajectory file and bin files." << reset << std::endl;
    exit(0);
  }

  bool success_field = bsplineSE3.get_pose(timeStart, qStart, pStart); // 시작 orientation 및 시작 position 을 
  // bSplineSE3 에는 trajectory 의 정보가 들어가있고, 이를 이용해서, 현재의 시간에 대한 quaternion 과 position 을 가져온다.

  switch (LiDAR)
  {
  case OUSTER:
    readBinFile(filename, *scanOuster);
    break;
  case VELODYNE:
    readBinFile(filename, *scanVelodyne);
    break;
  case LIVOX:
    readBinFile(filename, *scanLivox);
    break;
  case AEVA:
    readBinFile(filename, *scanAeva, timeStart);
    break;
  }

  Point3D currPoint;
  currPoint.x = pStart(0); // 그 데이터가 취득된 시점에서의 라이다 위치
  currPoint.y = pStart(1);
  currPoint.z = pStart(2);

  scanPoints.push_back(currPoint);
  scanQuat.push_back(qStart);
  scanTrans.push_back(pStart);
  scanTimestamps.push_back(timestampStr);

  if (success_field)
  {
    qStart = qStart.conjugate();
    double timeEnd = timeStart + 0.105; // 그 스캔이 끝날 때 까지의 시간
    double dt = (timeEnd - timeStart) / numIntervals; // 하나의 스캔이 들어오기 까지의 시간을 잘 쪼개서 (numIntervals 가 한 스캔을 디스큐잉할 레졸루션 같은 거였구나...)
    bool exitFlag = false;
    for (int i = 0; i < numIntervals; ++i)
    {
      double t = timeStart + i * dt;
      if (!bsplineSE3.get_pose(t, quaternions[i], positions[i]))
      {
        exitFlag = true;
      }
    }

    switch (LiDAR)
    {
    case OUSTER:
      if (!exitFlag && undistortFlag)
      {
        #pragma omp parallel for
        for (auto &point : scanOuster->points)
        {
          double timestamp = timeStart + point.t / float(1000000000); // nanosec?
          processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
        }
      }
      vecOuster.push_back(*scanOuster);
      break;
    case VELODYNE:
      if (!exitFlag && undistortFlag)
      {
        #pragma omp parallel for
        for (auto &point : scanVelodyne->points)
        {
          double timestamp = timeStart + point.time;
          processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
        }
      }
      vecVelodyne.push_back(*scanVelodyne);
      break;
    case LIVOX:
      if (!exitFlag && undistortFlag)
      {
#pragma omp parallel for
        for (auto &point : scanLivox->points)
        {
          double timestamp = timeStart + point.offset_time / float(1000000000);
          processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
        }
      }
      vecLivox.push_back(*scanLivox);
      break;
    case AEVA:
      if (!exitFlag && undistortFlag)
      {
        #pragma omp parallel for
        for (auto &point : scanAeva->points)
        {
          double timestamp = timeStart + point.time_offset_ns / float(1000000000);
          processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
        }
      }
      vecAeva.push_back(*scanAeva);
      break;
    }
    keyIndex++;
  }
}

void PointCloudProcessor::loadAndProcessBinFiles()
{
  DIR *dir;
  struct dirent *ent;
  std::vector<std::string> binFiles;

  if ((dir = opendir(binPath.c_str())) != NULL)
  {
    while ((ent = readdir(dir)) != NULL)
    {
      std::string filename = ent->d_name;
      if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".bin") //binFiles 에 타임스탬프만 저장됨
      {
        binFiles.push_back(filename);
      }
    }
    closedir(dir);
  }

  std::sort(binFiles.begin(), binFiles.end()); // 이론적으로라면 파일 이름이 전부 들어가야 함...
  Point3D lastPoint;
  lastPoint.x = -999;
  lastPoint.y = -999;
  lastPoint.z = -999;
  numBins = binFiles.size(); // 라이다 스캔 폴더내의 모든 스캔 수, 예를 들면 12475개.... 이런식
  // int count = 0;

  for (const std::string &filename : binFiles) // 파일 이름은 12475 개의 파일 이름 1개 1개를 지칭
  {
    visualizer.progressBar(keyIndex, binFiles.size(), filename, false); // keyIndex 는 0 에서 시작!
    processFile(binPath + filename);
    switch (LiDAR)
    {
    case OUSTER:
      accumulateScans(vecOuster, lastPoint);
      break;
    case VELODYNE:
      accumulateScans(vecVelodyne, lastPoint);
      break;
    case LIVOX:
      accumulateScans(vecLivox, lastPoint);
      break;
    case AEVA:
      accumulateScans(vecAeva, lastPoint);
      break;
    }
  }

  fout_pose.close();
}

void PointCloudProcessor::loadTrajectory()
{
  std::string line;
  std::ifstream file(trajPath);
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    Eigen::VectorXd point(8); // trajectory 가 찍힌 sec, 그 때의 포지션 x, y, z, ox, oy, oz, ow 순으로 들어감
    if (!(iss >> point(0) >> point(1) >> point(2) >> point(3) >> point(4) >> point(5) >> point(6) >> point(7)))
    {
      break;
    }
    point(0) *= 1e-9;
    trajPoints.push_back(point);
  }
  file.close();
  bsplineSE3.feed_trajectory(trajPoints);
}