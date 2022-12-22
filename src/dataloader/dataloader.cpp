//
// Created by shapelim on 22.12.22.
//

#include "dataloader/dataloader.h"

size_t DataLoader::size() const {
    return num_frames_;
}
vector<float> DataLoader::splitLine(const string &input, char delimiter) {
    vector<float> answer;
    stringstream  ss(input);
    string        temp;

    while (getline(ss, temp, delimiter)) {
        answer.push_back(stof(temp));
    }
    return answer;
}

void DataLoader::vec2tf4x4(vector<float> &pose, Eigen::Matrix4f &tf4x4) {
    for (int idx = 0; idx < 12; ++idx) {
        int i = idx / 4;
        int j = idx % 4;
        tf4x4(i, j) = pose[idx];
    }
}

bool DataLoader::loadLabels(const std::string &label_name, vector<uint32_t> &labels) {
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

void DataLoader::countNumFrames(const string &pcd_dir, const string &pcd_format) {
//    std::cout << " hh" << std::endl;
    for (num_frames_ = 0;; num_frames_++) {
        string filename = (boost::format("%s/%06d.%s") % pcd_dir % num_frames_ % pcd_format).str();
        cout << filename << endl;
        if (!std::filesystem::exists(filename)) {
            break;
        }
    }
}
//
SemanticKITTILoader::SemanticKITTILoader(const string &cloud_dir, const string &cloud_format, const string &pose_path) {
    cloud_dir_    = cloud_dir;
    cloud_format_ = cloud_format;
    pose_path_    = pose_path;

    countNumFrames(cloud_dir, cloud_format);
//    loadAllPoses(pose_path_, poses_gt_);
}