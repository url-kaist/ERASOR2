//
// Created by shapelim on 21. 10. 18..
//

#include "erasor2/Config.hpp"

#include "dataloader/dataloader.h"
#include "rosparam_server.hpp"
#include "tools/erasor_utils.hpp"

using namespace std;

using PointType = pcl::PointXYZI;

void loadGTLabel(const string &gt_label_dir, const size_t idx, vector<uint32_t> &labels) {
  string label_name = (boost::format("%s/%06d.label") % gt_label_dir % idx).str();
  //    cout << label_name << endl;
  std::ifstream label_input(label_name, std::ios::binary);
  if (!label_input.is_open()) {
    throw invalid_argument("Could not open the label!");
  }
  label_input.seekg(0, std::ios::end);
  uint32_t num_points = label_input.tellg() / sizeof(uint32_t);
  label_input.seekg(0, std::ios::beg);

  labels.resize(num_points);
  label_input.read((char *)&labels[0], num_points * sizeof(uint32_t));
}

void loadEstLabel(const string &est_label_dir, const int i, std::vector<uint32_t> &instance_label) {
  string inst_label_name = (boost::format("%s/%06d.label") % est_label_dir % i).str();
  erasor_utils::load_labels(inst_label_name, instance_label);

  static bool is_initial = true;
  if (is_initial) {
    std::vector<uint32_t> tmp_instance_label = instance_label;
    std::sort(tmp_instance_label.begin(), tmp_instance_label.end());
    auto last = std::unique(tmp_instance_label.begin(), tmp_instance_label.end());
    tmp_instance_label.erase(last, tmp_instance_label.end());
    std::cout << "\033[1;33m[NOTE] Est. label contains ";
    for (int j = 0; j < tmp_instance_label.size(); ++j) {
      std::cout << tmp_instance_label[j];
      if (j < tmp_instance_label.size() - 1) {
        std::cout << ", ";
      } else {
        std::cout << std::endl;
      }
    }
    std::cout << "(check the function `assignLabel()` in `dataloader.cpp`\033[0m" << std::endl;
    is_initial = false;
  }
}

inline vector<float> splitLine(const string &input, char delimiter) {
  vector<float> answer;
  stringstream ss(input);
  string temp;

  while (getline(ss, temp, delimiter)) {
    answer.push_back(stof(temp));
  }
  return answer;
}

inline void vec2tf4x4(vector<float> &pose, Eigen::Matrix4f &tf4x4) {
  for (int idx = 0; idx < 12; ++idx) {
    int i       = idx / 4;
    int j       = idx % 4;
    tf4x4(i, j) = pose[idx];
  }
}

void loadAllPoses(string pose_path, vector<Eigen::Matrix4f> &poses) {
  // Pose is transformed into lidar pose!
  // https://github.com/ZuoJiaxing/Kitti_Lidarmap_fromGt/blob/master/constructLaserMap.cpp

  Eigen::Matrix4f KITTI_CAM2LIDAR = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tf_origin       = Eigen::Matrix4f::Identity();

  KITTI_CAM2LIDAR << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03,
      -4.784029760483e-03, -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01,
      -7.337429464231e-02, 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03,
      -3.339968064433e-01, 0, 0, 0, 1;

  tf_origin << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;
  // Tr?
  //    KITTI_CAM2LIDAR << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03,
  //    -1.198459927713e-02,
  //                       -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01,
  //                       -5.403984729748e-02, 9.999738645903e-01, 4.859485810390e-04,
  //                       -7.206933692422e-03, -2.921968648686e-01, 0, 0, 0, 1;

  //    KITTI_CAM2LIDAR << 7.755449000000e-03, -9.999694000000e-01, -1.014303000000e-03,
  //    -7.275538000000e-03,
  //                       2.294056000000e-03, 1.032122000000e-03, -9.999968000000e-01,
  //                       -6.324057000000e-02,
  //                       9.999673000000e-01, 7.753097000000e-03, 2.301990000000e-03,
  //                       -2.670414000000e-01, 0, 0, 0, 1;
  poses.clear();
  poses.reserve(2000);
  std::ifstream in(pose_path);
  string line;

  int count = 0;
  while (std::getline(in, line)) {
    vector<float> pose        = splitLine(line, ' ');
    Eigen::Matrix4f tf4x4_cam = Eigen::Matrix4f::Identity();  // Crucial!
    vec2tf4x4(pose, tf4x4_cam);
    Eigen::Matrix4f tf4x4_lidar = tf_origin * tf4x4_cam * KITTI_CAM2LIDAR;
    //        Eigen::Matrix4f tf4x4_lidar = KITTI_CAM2LIDAR.inverse() * tf4x4_cam * KITTI_CAM2LIDAR;
    poses.emplace_back(tf4x4_lidar);
    count++;
  }
  in.close();
  std::cout << "Total " << count << " poses are loaded" << std::endl;

  if (count == 0) {
    throw invalid_argument("Fail to load poses. Please check the `pose_path_`");
  }
}

template <typename T>
int loadCloud(const string &cloud_dir, size_t idx, pcl::PointCloud<T> &cloud) {
  string filename = (boost::format("%s/%06d.bin") % cloud_dir % idx).str();
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

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: accum_4dmos <config.yaml> [target_mos_type]\n";
    return 1;
  }
  string target_mos_type =
      (argc >= 3) ? std::string(argv[2]) : std::string("benedikt_4dmos_labels");

  std::cout << "4D MOS mapping started" << std::endl;

  const auto cfg = erasor2::Config::fromYaml(argv[1]);
  unique_ptr<RosParamServer> params(new RosParamServer(cfg));

  cout << "From " << params->start_frame_ << " to " << params->end_frame_ << endl;
  cout << params->robot_body_size_ << endl;

  int start_frame    = params->start_frame_;
  int end_frame      = params->end_frame_;
  int accum_interval = params->accum_interval_;

  string abs_data_dir = params->abs_data_dir_;
  string sequence     = params->sequence_;

  string abs_gt_label_dir = abs_data_dir + "/" + sequence + "/labels";
  string abs_label_dir    = abs_data_dir + "/" + sequence + "/" + target_mos_type;
  string abs_cloud_dir    = abs_data_dir + "/" + sequence + "/velodyne";
  string abs_pose_path    = abs_data_dir + "/" + sequence + "/kiss_icp_poses.txt";

  cout << "\033[1;32m" << abs_gt_label_dir << "\n";
  cout << abs_label_dir << "\n";
  cout << abs_cloud_dir << "\n";
  cout << abs_pose_path << "\033[0m\n";

  vector<Eigen::Matrix4f> poses;
  loadAllPoses(abs_pose_path, poses);

  int cnt = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_accum(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  static_map_accum->reserve(2000000);
  static_map_voxelized->reserve(2000000);

  Eigen::Matrix4f tf_h_of_ground_to_be_zero = Eigen::Matrix4f::Identity();
  tf_h_of_ground_to_be_zero(2, 3)           = params->sensor_height_;

  for (int i = start_frame; i < end_frame + accum_interval; ++i) {
    signal(SIGINT, erasor_utils::signal_callback_handler);
    if (i % 10 == 0) {
      cout << "[DataLoader] " << i << "th frame comes!\n";
    }

    // if `accum_interval` == 1, the below condition is not used
    if (accum_interval > 1 && ++cnt / accum_interval >= 1) {
      cnt = 0;
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gt_label(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_static_transformed(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_static(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_dynamic(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr noise(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose                 = poses[i];
    vector<uint32_t> est_labels, gt_labels;
    loadGTLabel(abs_gt_label_dir, i, gt_labels);
    loadEstLabel(abs_label_dir, i, est_labels);
    loadCloud(abs_cloud_dir, i, *cloud_gt_label);

    est_static->reserve(gt_labels.size());
    est_dynamic->reserve(gt_labels.size());
    static_map_accum->reserve(gt_labels.size());

    for (int j = 0; j < cloud_gt_label->points.size(); ++j) {
      auto &pt     = cloud_gt_label->points[j];
      pt.intensity = static_cast<float>(gt_labels[j]);
    }

    // Filtering out dynamic & noisy points
    float max_dist_square = pow(params->robot_body_size_, 2);
    int count             = 0;
    for (auto const &pt : cloud_gt_label->points) {
      double dist_square = pow(pt.x, 2) + pow(pt.y, 2);
      if (dist_square < max_dist_square) {
        noise->points.emplace_back(pt);
      } else {
        if (est_labels[count] == 9.0) {
          est_static->points.emplace_back(pt);
        } else {  // Maybe 251 is dynamic label!
          est_dynamic->points.emplace_back(pt);
        }
      }
      ++count;
    }
    pcl::transformPointCloud(
        *est_static, *est_static_transformed, pose * tf_h_of_ground_to_be_zero);
    (*static_map_accum) += (*est_static_transformed);
  }

  erasor_utils::voxelize_preserving_labels_by_nanoflann(
      static_map_accum, *static_map_voxelized, params->voxel_size_);

  static_map_voxelized->width  = static_map_voxelized->points.size();
  static_map_voxelized->height = 1;
  std::cout << "[Debug]: (" << static_map_voxelized->width << ", " << static_map_voxelized->height
            << ") => " << static_map_voxelized->points.size() << std::endl;
  string static_map_path = params->abs_save_dir_ + "/" + sequence + "_from_" +
                           to_string(start_frame) + "_to_" + to_string(end_frame) + "_" +
                           target_mos_type + ".pcd";
  std::cout << "\033[1;32mSaving the map to pcd...: " << static_map_path << "\033[0m" << std::endl;
  pcl::io::savePCDFileASCII(static_map_path, *static_map_voxelized);

  cout << "[ERASOR2] Complete to set scans and poses\n";

  return 0;
}
