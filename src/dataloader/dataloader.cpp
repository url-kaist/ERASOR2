//
// Created by shapelim on 22.12.22.
//

#include "dataloader/dataloader.h"

inline size_t DataLoader::size() const {
    return num_frames_;
}

inline vector<float> DataLoader::splitLine(const string &input, char delimiter) {
    vector<float> answer;
    stringstream  ss(input);
    string        temp;

    while (getline(ss, temp, delimiter)) {
        answer.push_back(stof(temp));
    }
    return answer;
}

inline void DataLoader::vec2tf4x4(vector<float> &pose, Eigen::Matrix4f &tf4x4) {
    if (pose.size() == 12)
  {
     for (int idx = 0; idx < 12; ++idx) {
        int i = idx / 4;
        int j = idx % 4;
        tf4x4(i, j) = pose[idx];
    }
  }

  else if(pose.size() == 8) // ns time, x, y, z, qx, qy, qz, qw
  {
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose[7], pose[4], pose[5], pose[6]).toRotationMatrix();
    tf4x4 << mat3(0, 0), mat3(0, 1), mat3(0, 2), pose[1],
             mat3(1, 0), mat3(1, 1), mat3(1, 2), pose[2],
             mat3(2, 0), mat3(2, 1), mat3(2, 2), pose[3],
             0, 0, 0, 1;
  }
}

inline bool DataLoader::loadLabel(const std::string &label_name, vector<uint32_t> &labels) {
    std::ifstream label_input(label_name, std::ios::binary);
    if (!label_input.is_open()) {
        std::cerr << "Could not open the label!" << std::endl;
        return false;
    }
    label_input.seekg(0, std::ios::end);
    uint32_t num_points = label_input.tellg() / sizeof(uint32_t);
    label_input.seekg(0, std::ios::beg);

    labels.resize(num_points);
    label_input.read((char *) &labels[0], num_points * sizeof(uint32_t));

    label_input.close();
    return true;
}

inline void DataLoader::countNumFrames(const string &pcd_dir, const string &pcd_format) {
    for (num_frames_ = 0;; num_frames_++) {
        string filename = (boost::format("%s/%06d.%s") % pcd_dir % num_frames_ % pcd_format).str();
//        cout << filename << endl;
        if (!std::filesystem::exists(filename)) {
            break;
        }
    }
}

inline void DataLoader::getPose(const size_t i, Eigen::Matrix4f& pose) {
    pose = poses_gt_[i];
}

SemanticKITTILoader::SemanticKITTILoader(const string &abs_data_dir, const string &seq,
                                         const string& instance_seg_method) {
    // Follow the KITTI format
    seq_              = seq;
    cloud_dir_        = abs_data_dir + "/" + seq + "/velodyne";
    gt_label_dir_     = abs_data_dir + "/" + seq + "/labels";
//    pose_path_        = abs_data_dir + "/" + seq + "/poses.txt";
    if (seq_ == "19") {
        pose_path_ = abs_data_dir + "/" + seq + "/kiss_icp_poses.txt";
    } else {
        pose_path_ = abs_data_dir + "/" + seq + "/suma_pose.txt";
    }
    ground_label_dir_ = abs_data_dir + "/" + seq + "/patchwork";
//    ground_label_dir_ = abs_data_dir + "/" + seq + "/patchwork_filtered";
    if (instance_seg_method == "cais") {
        cout << "\033[1;32m[DataLoader] Selected isntance seg. method: CAIS\n";
        est_label_dir_    = abs_data_dir + "/" + seq + "/cais";
    } else if (instance_seg_method == "hdbscan") {
        cout << "\033[1;32m[DataLoader] Selected isntance seg. method: HDBSCAN\n";
        est_label_dir_    = abs_data_dir + "/" + seq + "/hdbscan";
    }

    cloud_format_     = "bin";
    cout << "\033[1;32m[DataLoader] Data directories are as follows:" << endl;
    cout << cloud_dir_ << endl;
    cout << gt_label_dir_ << endl;
    cout << pose_path_ << endl;
    cout << ground_label_dir_ << endl;
    cout << est_label_dir_ << "\033[0m" << endl;

    countNumFrames(cloud_dir_, cloud_format_);
    loadAllPoses(pose_path_, poses_gt_);
}

void SemanticKITTILoader::loadAllPoses(string pose_path, vector<Eigen::Matrix4f> &poses) {
    // Pose is transformed into lidar pose!
    // https://github.com/ZuoJiaxing/Kitti_Lidarmap_fromGt/blob/master/constructLaserMap.cpp

    Eigen::Matrix4f KITTI_CAM2LIDAR = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tf_origin = Eigen::Matrix4f::Identity();

    KITTI_CAM2LIDAR << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
            -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
            9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
            0, 0, 0, 1;

    tf_origin << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;
    // Tr?
//    KITTI_CAM2LIDAR << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
//                       -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
//                       9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
//                       0, 0, 0, 1;

//    KITTI_CAM2LIDAR << 7.755449000000e-03, -9.999694000000e-01, -1.014303000000e-03, -7.275538000000e-03,
//                       2.294056000000e-03, 1.032122000000e-03, -9.999968000000e-01, -6.324057000000e-02,
//                       9.999673000000e-01, 7.753097000000e-03, 2.301990000000e-03, -2.670414000000e-01,
//                       0, 0, 0, 1;
    poses.clear();
    poses.reserve(2000);
    std::ifstream in(pose_path);
    string        line;

    int count = 0;
    while (std::getline(in, line)) {
        vector<float>   pose      = splitLine(line, ' ');
        Eigen::Matrix4f tf4x4_cam = Eigen::Matrix4f::Identity(); // Crucial!
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

//inline Eigen::Matrix4f SemanticKITTILoader::getPose(size_t i) {
//    return poses_gt_[i];
//}

void SemanticKITTILoader::getGTLabeledScan(size_t i, pcl::PointCloud<pcl::PointXYZI>& cloud) {
    loadCloud(i, cloud);

    shared_ptr<vector<uint32_t>> label(new vector<uint32_t>);
    loadGTLabel(i, *label);

    if (cloud.points.size() != label->size()) {
        throw invalid_argument("Something's wrong! The numbers of points are not matched to each other");
    }

    // The intensity is replaced with the raw label data
    for (int j = 0; j < cloud.points.size(); ++j) {
        auto &pt = cloud.points[j];
        pt.intensity = static_cast<float>((*label)[j]);
    }
}

void SemanticKITTILoader::getScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI> &cloud, Eigen::Matrix4f &pose) {
    loadCloud(i, cloud);

    shared_ptr<vector<uint32_t>> label(new vector<uint32_t>);
    loadGTLabel(i, *label);

    if (cloud.points.size() != label->size()) {
        throw invalid_argument("Something's wrong! The numbers of points are not matched to each other");
    }

    shared_ptr<vector<uint32_t>> ground_label(new vector<uint32_t>);
    shared_ptr<vector<uint32_t>> instance_label(new vector<uint32_t>);
    uint32_t max_instance;
    loadEstGroundAndInstanceLabels(i, *ground_label, *instance_label);
    assignLabels(*ground_label, *instance_label,
                      cloud, max_instance);

    getPose(i, pose);
}

void SemanticKITTILoader::loadEstGroundAndInstanceLabels(const int i, std::vector<uint32_t>& ground_label,
                                                         std::vector<uint32_t>& instance_label) {
    string inst_label_name = (boost::format("%s/%06d.label") % est_label_dir_ % i).str();
    string ground_label_name = (boost::format("%s/%06d.label") % ground_label_dir_ % i).str();

    erasor_utils::load_labels(inst_label_name, instance_label);
    erasor_utils::load_labels(ground_label_name, ground_label);

    if (instance_label.size() != ground_label.size()) {
        throw invalid_argument("[Loading] Something's wrong!");
    }

    static bool is_initial = true;
    if (is_initial) {
        std::vector<uint32_t> tmp_ground_label = ground_label;
        std::sort(tmp_ground_label.begin(), tmp_ground_label.end());
        auto last = std::unique(tmp_ground_label.begin(), tmp_ground_label.end());
        tmp_ground_label.erase(last, tmp_ground_label.end());
        std::cout << "\033[1;33m[NOTE] Ground label contains ";
        for (int j = 0; j < tmp_ground_label.size(); ++j) {
            std::cout << tmp_ground_label[j];
            if (j < tmp_ground_label.size() - 1) {
                std::cout << ", ";
            } else {
                std::cout << std::endl;
            }
        }
        std::cout << "(check the function `assignLabel()` in `dataloader.cpp`\033[0m" << std::endl;
        is_initial = false;
    }
}

void SemanticKITTILoader::assignLabels(const std::vector<uint32_t> ground_labels, const std::vector<uint32_t> instance_labels,
                  pcl::PointCloud<pcl::PointXYZI>& src_cloud, uint32_t& max_instance) {
    max_instance = 0;
    for (int j = 0; j < src_cloud.points.size(); ++j) {
        // Follow Rhiney and Lucas's format.
        if (ground_labels[j]) {
            src_cloud.points[j].intensity = GROUND_LABEL;
        } else { // For non-ground points
            uint32_t inst_label            = instance_labels[j] >> 16;
            src_cloud.points[j].intensity = inst_label;
//                std::cout << inst_label << ", ";
            max_instance = max(max_instance, inst_label);
        }
    }
}

void SemanticKITTILoader::loadGTLabel(const size_t idx, vector<uint32_t> &labels) {
    string        label_name = (boost::format("%s/%06d.label") % gt_label_dir_ % idx).str();
//    cout << label_name << endl;
    std::ifstream label_input(label_name, std::ios::binary);
    if (!label_input.is_open()) {
        throw invalid_argument("Could not open the label!");
    }
    label_input.seekg(0, std::ios::end);
    uint32_t num_points = label_input.tellg() / sizeof(uint32_t);
    label_input.seekg(0, std::ios::beg);

    labels.resize(num_points);
    label_input.read((char *) &labels[0], num_points * sizeof(uint32_t));
}

void SemanticKITTILoader::parseGTLabel(const vector<uint32_t> &labels,
                                       vector<uint32_t> &semantic_labels, vector<uint32_t> &obj_ids) {
    int num_pts = labels.size();
    semantic_labels.resize(num_pts);
    obj_ids.resize(num_pts);

    int                 cnt = 0;
    for (const uint32_t label: labels) {
        uint32_t semantic_label = label & 0xFFFF;
        uint32_t id             = label >> 16;

        semantic_labels[cnt] = semantic_label;
        obj_ids[cnt]         = id;
        ++cnt;
    }
}

// ! HeLiPR Loader

HeLiPRLoader::HeLiPRLoader(const string &abs_data_dir, const string &sensor, const string& instance_seq_method)
{
  seq_ = sensor;
  cloud_dir_ = abs_data_dir + sensor + "/velodyne";
  gt_label_dir_ = "";

  pose_path_ = abs_data_dir + sensor + "/poses.txt";
  est_label_dir_ = "";
  ground_label_dir_ = "";
  cloud_format_ = "pcd";

  T_OS2_Avia << 0.999821044774776  ,      0.0159071210418969,	0.0102392346214582, 0.628232128507671, 
                -0.0160846635739714   ,    0.999717526758036,0.0174971509254758, -0.338773055024622, 
                -0.00995801301399956 ,    -0.0176587143630322,	0.999794482773264, -0.229651932062428, 
                0                   ,    0                   ,   0                   , 1;
  // T_OS2_Avia = Eigen::Matrix4f::Identity();

  T_OS2_Aeva <<0.999566194824045 ,       0.0248733202636163	,-0.0157714965695182, 0.541864230595932, 
                -0.0249842995614032  ,     0.999664174831162,	-0.00687912309516293, 0.369867831347708, 
                0.0155950934721411 ,      0.00727017869078270,	 0.999851957822456,-0.0869161258617896, 
                0                   ,    0                   ,   0                   , 1;
  // T_OS2_Aeva = Eigen::Matrix4f::Identity();

  T_OS2_VLP16 << 0.999752289753613,   -0.0215240513828855 ,    0.00566342162254459, -0.958469101466382, 
                  0.0215555750390671,    0.999752161633017    , -0.00556529377889104, 0.0449041087926452, 
                  -0.00554223034012042 ,    0.00568599350836049 ,   0.999968476083461  , -0.262799643355476, 
                  0 ,    0 ,   0 , 1;
  // T_OS2_VLP16 = Eigen::Matrix4f::Identity();


  cout << ANSI_COLOR_GREEN << " [DataLoader] Data directories are as follows:" << ANSI_COLOR_RESET << endl;
  cout << "cloud_dir_: " << cloud_dir_ << endl;
  cout << "gt_label_dir_: " << gt_label_dir_ << endl;
  cout << "pose_path_: " <<pose_path_ << endl;
  cout << "ground_label_dir_: " << ground_label_dir_ << endl;
  cout << "est_label_dir_: " << est_label_dir_ << "\033[0m" << endl;
  // 대충 abs_data_dir_ 을 "/media/se0yeon00/SY_Other/HeliPR/KAIST05" 정도로 생각하고

  countNumFrames(cloud_dir_, cloud_format_, timestamp_lists_); // 생성자에서 스캔 수 세고
  loadAllPoses(pose_path_, poses_gt_); // pose 를 일단 전부 저장해둘거임
}

inline void HeLiPRLoader::countNumFrames(const string &pcd_dir, const string &pcd_format, vector<string> &files) {
  // std::vector<std::string> files;
  for (const auto& entry :fs::directory_iterator(pcd_dir)){
    if (entry.path().extension() == "." + pcd_format){
      // file extension ".bin" remove, then push back
      string filename = entry.path().filename().string();
      filename = filename.substr(0, filename.size() - 4);
      // printf("filename: %s\n", filename.c_str());
      files.push_back(filename);
    }
  }

  std::sort(files.begin(), files.end());
  num_frames_ = files.size();
  printf("Total %d frames are loaded\n", num_frames_);
  // printf("First frame: %s\n", files[0].c_str());
}

void HeLiPRLoader::loadAllPoses(const string pose_path, vector<Eigen::Matrix4f> &poses){

  Eigen::Matrix4f tf_origin = Eigen::Matrix4f::Identity();
  tf_origin << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;

  poses.clear();
  poses.reserve(20000);
  std::ifstream in(pose_path);
  string line;

  int count = 0;
  while (std::getline(in, line)){
    vector<float> pose = splitLine(line, ' ');
    Eigen::Matrix4f tf4x4_sensor = Eigen::Matrix4f::Identity();
    vec2tf4x4(pose, tf4x4_sensor);

    Eigen::Matrix4f tf4x4_lidar = tf4x4_sensor;

    if (seq_ == "Avia")
    {
      tf4x4_lidar = T_OS2_Avia * tf4x4_sensor;
    }
    else if(seq_ == "Aeva")
    {
      tf4x4_lidar = T_OS2_Aeva * tf4x4_sensor;
    }
    else if(seq_ == "VLP16")
    {
      tf4x4_lidar = T_OS2_VLP16 * tf4x4_sensor;
    }

    poses.push_back(tf4x4_lidar);
    count++;
  }
  in.close();
  std::cout << "Total " << count << " poses are loaded" << std::endl;

}

void HeLiPRLoader::getScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI> &cloud, Eigen::Matrix4f &pose)
{ 
    printf("[HeLiPRLoader] my sensor type is %s\n", seq_.c_str());
    loadCloud(i, cloud);
    getPose(i, pose);
}



