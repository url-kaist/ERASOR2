//
// Created by shapelim on 22.12.22.
//

#ifndef ERASOR2_DATALOADER_H
#define ERASOR2_DATALOADER_H

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <boost/format.hpp>
#include <experimental/filesystem>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tools/erasor_utils.hpp"

namespace fs = std::experimental::filesystem;

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

using namespace std;
struct PointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t,
                                                                         ring,
                                                                         ring)(float, time, time))

struct OusterPointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint32_t t;
  uint16_t reflectivity;
  uint16_t ring;
  uint16_t ambient;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    OusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(
        uint16_t,
        reflectivity,
        reflectivity)(uint16_t, ring, ring)(uint16_t, ambient, ambient))

struct AevaPointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  float reflectivity;
  float velocity;
  int32_t time_offset_ns;
  uint8_t line_index;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    AevaPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float,
        reflectivity,
        reflectivity)(float, velocity, velocity)(int32_t,
                                                 time_offset_ns,
                                                 time_offset_ns)(uint8_t, line_index, line_index))

struct LivoxPointXYZI {
  PCL_ADD_POINT4D;
  uint8_t reflectivity;
  uint8_t tag;
  uint8_t line;
  uint32_t offset_time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXYZI,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t,
                                                                          reflectivity,
                                                                          reflectivity)(
                                      uint8_t,
                                      tag,
                                      tag)(uint8_t, line, line)(uint32_t, offset_time, offset_time))

using pc_type   = PointXYZIRT;
using pc_type_o = OusterPointXYZIRT;
using pc_type_l = LivoxPointXYZI;
using pc_type_a = AevaPointXYZIRT;

class DataLoader {
 public:
  //    DataLoader(const string &cloud_dir, const string &cloud_format, const string &pose_path) :
  //    cloud_dir_(cloud_dir), cloud_format_(cloud_format), pose_path_(pose_path) {}

  DataLoader() {}

  ~DataLoader() {}

  template <typename T>
  inline void rejectNeighboringPoints(const pcl::PointCloud<T> &cloud_raw,
                                      const float neighboring_region_size,
                                      pcl::PointCloud<T> &inliers,
                                      pcl::PointCloud<T> &outliers) {
    float max_dist_square = pow(neighboring_region_size, 2);
    for (auto const &pt : cloud_raw.points) {
      double dist_square = pow(pt.x, 2) + pow(pt.y, 2);
      if (dist_square < max_dist_square) {
        outliers.emplace_back(pt);
      } else {
        inliers.emplace_back(pt);
      }
    }
  }

  /*
   * Common functions
   */
  inline size_t size() const;

  inline vector<float> splitLine(const string &input, char delimiter);

  inline void vec2tf4x4(vector<float> &pose, Eigen::Matrix4f &tf4x4);

  inline bool loadLabel(const std::string &label_name, vector<uint32_t> &labels);

  inline void countNumFrames(const string &pcd_dir, const string &pcd_format);

  inline void getPose(const size_t i, Eigen::Matrix4f &pose);

  /*
   * Virtual functions virtual로 정의된 함수에서는 파생 클래스가 함수를 재정의 할 수 있음
   */
  // Important! 'virtual' is necessary
  // + the functions must be declared, i.e. {} is needed
  virtual void loadAllPoses(const string pose_path, vector<Eigen::Matrix4f> &poses) {}

  template <typename T>
  int loadCloud(size_t idx, pcl::PointCloud<T> &cloud) const {}

  virtual void getGTLabeledScan(size_t i, pcl::PointCloud<pcl::PointXYZI> &cloud) {}

  // Estimated labels are added
  virtual void getScanAndPose(size_t i,
                              pcl::PointCloud<pcl::PointXYZI> &cloud,
                              Eigen::Matrix4f &pose) {
    cout << "[DefaultLoader] Default getScanandPose is Loaded" << endl;
  }

  virtual void loadEstGroundAndInstanceLabels(const int i,
                                              std::vector<uint32_t> &ground_label,
                                              std::vector<uint32_t> &instance_label) {}

  virtual void assignLabels(const std::vector<uint32_t> ground_labels,
                            const std::vector<uint32_t> instance_labels,
                            const float min_z_voi,
                            const float max_z_voi,
                            pcl::PointCloud<pcl::PointXYZI> &src_cloud,
                            uint32_t &max_instance) {}

  virtual void testInheritance() { cout << "Test inheritance" << endl; }

  int num_frames_;
  string seq_;
  string cloud_dir_;
  string cloud_format_;
  string pose_path_;
  string gt_label_dir_;
  string ground_label_dir_;
  string est_label_dir_;
  vector<Eigen::Matrix4f> poses_gt_;
  vector<string> timestamp_lists_;  // for HeLiPR
};
// ! 위에까지가 기본적인 데이터 로더의 형태였고, 이제 아래부터가 커스터마이징 된 데이터 로더!
class SemanticKITTILoader : public DataLoader {
 public:
  //    SemanticKITTILoader() = delete;
  SemanticKITTILoader();

  SemanticKITTILoader(const string &abs_data_dir,
                      const string &seq,
                      const string &instance_seq_method = "hdbscan");

  //    explicit SemanticKITTILoader(const string &cloud_dir, const string &cloud_format, const
  //    string &pose_path) : DataLoader(cloud_dir, cloud_format, pose_path) { cout << (1 + 3) <<
  //    endl; };
  //

  ~SemanticKITTILoader() {}

  void loadAllPoses(string pose_path, vector<Eigen::Matrix4f> &poses);
  //
  template <typename T>
  int loadCloud(size_t idx, pcl::PointCloud<T> &cloud) const {
    string filename = (boost::format("%s/%06d.%s") % cloud_dir_ % idx % cloud_format_).str();
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

  void loadGTLabel(const size_t idx, vector<uint32_t> &labels);

  void parseGTLabel(const vector<uint32_t> &labels,
                    vector<uint32_t> &semantic_labels,
                    vector<uint32_t> &obj_ids);

  //    inline Eigen::Matrix4f getPose(size_t i);

  // Only for SemanticKITTI
  void getGTLabeledScan(size_t i, pcl::PointCloud<pcl::PointXYZI> &cloud);

  // Estimated labels are added
  void getScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI> &cloud, Eigen::Matrix4f &pose);

  void loadEstGroundAndInstanceLabels(const int i,
                                      std::vector<uint32_t> &ground_label,
                                      std::vector<uint32_t> &instance_label);

  void assignLabels(const std::vector<uint32_t> ground_labels,
                    const std::vector<uint32_t> instance_labels,
                    pcl::PointCloud<pcl::PointXYZI> &src_cloud,
                    uint32_t &max_instance);

  //
  pcl::PointCloud<pcl::PointXYZ> getAllPositions() const {
    // For fetching loops
    pcl::PointCloud<pcl::PointXYZ> poses_cloud;
    poses_cloud.reserve(num_frames_);
    for (auto const &pose : poses_gt_) {
      pcl::PointXYZ pose_pt(pose(0, 3), pose(1, 3), pose(2, 3));
      poses_cloud.push_back(pose_pt);
    }
    return poses_cloud;
  }

  void testInheritance() { cout << "print from SemanticKITTIloader" << endl; }
};

class HeLiPRLoader : public DataLoader {
 public:
  HeLiPRLoader();
  HeLiPRLoader(const string &abs_data_dir,
               const string &sensor,
               const string &instance_seq_method = "cais");
  ~HeLiPRLoader() {}

  inline void countNumFrames(const string &pcd_dir,
                             const string &pcd_format,
                             vector<string> &files);
  void loadAllPoses(const string pose_path, vector<Eigen::Matrix4f> &poses);

  //
  // void getScanAndPose(size_t i, pcl::PointCloud<T>& cloud, Eigen::Matrix4f &pose);
  // template<typename T>
  void getScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI> &cloud, Eigen::Matrix4f &pose);
  // void getScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI>& cloud, Eigen::Matrix4f &pose);

  template <typename T>
  int loadCloud(size_t idx, pcl::PointCloud<T> &cloud) {
    //  string filename = (boost::format("%s/%s.%s") % cloud_dir_ % idx_timestamp %
    //  cloud_format_).str();

    string filename = (boost::format("%s/%06d.%s") % cloud_dir_ % idx % cloud_format_).str();

    if (cloud_format_ == "pcd") {
      pcl::PointCloud<pcl::PointXYZI> cloud_raw;
      pcl::io::loadPCDFile(filename, cloud_raw);
      cloud = cloud_raw;
    } else if (cloud_format_ == "bin") {
      FILE *file = fopen(filename.c_str(), "rb");
      if (!file) {
        std::cerr << "Error: failed to load " << filename << std::endl;
        return -1;
      }
      std::vector<float> buffer(4000000);
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
    }
    return 0;
  }

  void loadEstGroundAndInstanceLabels(const int i,
                                      std::vector<uint32_t> &ground_label,
                                      std::vector<uint32_t> &instance_label);

  void assignLabels(const std::vector<uint32_t> ground_labels,
                    const std::vector<uint32_t> instance_labels,
                    pcl::PointCloud<pcl::PointXYZI> &src_cloud,
                    uint32_t &max_instance);

  //
  pcl::PointCloud<pcl::PointXYZ> getAllPositions() const {
    // For fetching loops
    pcl::PointCloud<pcl::PointXYZ> poses_cloud;
    poses_cloud.reserve(num_frames_);
    for (auto const &pose : poses_gt_) {
      pcl::PointXYZ pose_pt(pose(0, 3), pose(1, 3), pose(2, 3));
      poses_cloud.push_back(pose_pt);
    }
    return poses_cloud;
  }

  // template<typename T>
  // void loadCloudOuster(string filename, pcl::PointCloud<T> &cloud);

  // template<typename T>
  // void loadCloudVelodyne(string filename, pcl::PointCloud<T> &cloud);

  // template<typename T>
  // void loadCloudAvia(string filename, pcl::PointCloud<T> &cloud);

  // template<typename T>
  // void loadCloudAeva(string filename, pcl::PointCloud<T> &cloud, uint64_t data);
  // void loadCloudXYZ(string filename, pcl::PointCloud<pcl::PointXYZ> &cloud);

  Eigen::Matrix4f T_OS2_VLP16;
  Eigen::Matrix4f T_OS2_Avia;
  Eigen::Matrix4f T_OS2_Aeva;
};

#endif  // ERASOR2_DATA_LOADER_H
