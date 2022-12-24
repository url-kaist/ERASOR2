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
    for (int idx = 0; idx < 12; ++idx) {
        int i = idx / 4;
        int j = idx % 4;
        tf4x4(i, j) = pose[idx];
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

inline Eigen::Matrix4f DataLoader::getPose(size_t i) {
    return poses_gt_[i];
}

SemanticKITTILoader::SemanticKITTILoader(const string &abs_data_dir, const string &seq) {
    // Follow the KITTI format
    cloud_dir_    = abs_data_dir + "/" + seq + "/velodyne";
    gt_label_dir_ = abs_data_dir + "/" + seq + "/labels";
    cloud_format_ = "bin";
    pose_path_    = abs_data_dir + "/" + seq + "/poses.txt";

    countNumFrames(cloud_dir_, cloud_format_);
    loadAllPoses(pose_path_, poses_gt_);
}

void SemanticKITTILoader::loadAllPoses(string pose_path, vector<Eigen::Matrix4f> &poses) {
    // Pose is transformed into lidar pose!
    Eigen::Matrix4f KITTI_CAM2LIDAR;
    KITTI_CAM2LIDAR << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
            -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
            9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
            0, 0, 0, 1;

    Eigen::Matrix4f tf_origin;
    tf_origin << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;

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
        poses.emplace_back(tf4x4_lidar);
        count++;
    }
    in.close();
    std::cout << "Total " << count << " poses are loaded" << std::endl;
}

void SemanticKITTILoader::getScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI> &cloud, Eigen::Matrix4f &pose) {
    loadCloud(i, cloud);
//    cout << "# of cloud: " << cloud.points.size() << endl;
//    vector<uint32_t> label;
    shared_ptr <vector<uint32_t>> label(new vector <uint32_t>);

    loadGTLabel(i, *label);
    // HT: I decide to follow the format of ERASOR ver. 1
    // The intensity is replaced with the raw label data
    // shared_ptr <vector<uint32_t>> semantic_label(new vector <uint32_t>);
    // shared_ptr <vector<uint32_t>> obj_id(new vector <uint32_t>);
    // parseGTLabel(*label, *semantic_label, *obj_id);
    if (cloud.points.size() == label->size()) {
        for (int j = 0; j < cloud.points.size(); ++j) {
            auto &pt = cloud.points[j];
            pt.intensity = static_cast<float>((*label)[j]);
        }
    } else {
        throw invalid_argument("Something's wrong! The numbers of points are not matched to each other");
    }
    pose = getPose(i);
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