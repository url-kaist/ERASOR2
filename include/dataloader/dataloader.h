//
// Created by shapelim on 22.12.22.
//
#include <iostream>
#include <iomanip>
#include <memory>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <filesystem>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#ifndef ERASOR2_DATALOADER_H
#define ERASOR2_DATALOADER_H

using namespace std;

class DataLoader {
public:
//    DataLoader(const string &cloud_dir, const string &cloud_format, const string &pose_path) :
//    cloud_dir_(cloud_dir), cloud_format_(cloud_format), pose_path_(pose_path) {}

    DataLoader() {}

    ~DataLoader() {}

    template<typename T>
    void rejectNeighboringPoints(const pcl::PointCloud<T> &cloud_raw,
                                 const float neighboring_region_size,
                                 pcl::PointCloud<T> &inliers,
                                 pcl::PointCloud<T> &outliers) {

        float           max_dist_square = pow(neighboring_region_size, 2);
        for (auto const &pt: cloud_raw.points) {
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
    size_t size() const;

    vector<float> splitLine(const string& input, char delimiter);

    void vec2tf4x4(vector<float> &pose, Eigen::Matrix4f &tf4x4);

    bool loadLabels(const std::string &label_name, vector<uint32_t> &labels);

    void countNumFrames(const string &pcd_dir, const string &pcd_format);

    /*
     * Virtual functions
     */
    // Important! It must require 'virtual'
//    virtual void setDirectories(const string &abs_dir) {}

//    virtual void loadAllPoses(const string pose_path, vector<Eigen::Matrix4f> &poses) {}
//
//    virtual void loadScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI>& cloud, Eigen::Matrix4f &pose);

    virtual void printTest() { cout << "hey" << endl; }

    int                     num_frames_;
    string                  cloud_dir_;
    string                  cloud_format_;
    string                  pose_path_;
    string                  gt_label_dir_;
    string                  ground_label_dir_;
    string                  instance_label_dir_;
    vector<Eigen::Matrix4f> poses_gt_;
};

class SemanticKITTILoader : public DataLoader {
public:
//    SemanticKITTILoader() = delete;
    SemanticKITTILoader();

    SemanticKITTILoader(const string &cloud_dir, const string &cloud_format, const string &pose_path);

//    explicit SemanticKITTILoader(const string &cloud_dir, const string &cloud_format, const string &pose_path)
//    : DataLoader(cloud_dir, cloud_format, pose_path) { cout << (1 + 3) << endl; };
//

    ~SemanticKITTILoader() {}
//
//    void loadAllPoses(string pose_path, vector<Eigen::Matrix4f> &poses) {
//        // Pose is transformed into lidar pose!
//        Eigen::Matrix4f KITTI_CAM2LIDAR;
//        KITTI_CAM2LIDAR << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
//                -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
//                9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
//                0, 0, 0, 1;
//
//        Eigen::Matrix4f tf_origin;
//        tf_origin << 0, 0, 1, 0,
//                -1, 0, 0, 0,
//                0, -1, 0, 0,
//                0, 0, 0, 1;
//
//        poses.clear();
//        poses.reserve(2000);
//        std::ifstream in(pose_path);
//        string        line;
//
//        int count = 0;
//        while (std::getline(in, line)) {
//            vector<float>   pose      = splitLine(line, ' ');
//            Eigen::Matrix4f tf4x4_cam = Eigen::Matrix4f::Identity(); // Crucial!
//            vec2tf4x4(pose, tf4x4_cam);
//            Eigen::Matrix4f tf4x4_lidar = tf_origin * tf4x4_cam * KITTI_CAM2LIDAR;
//            poses.emplace_back(tf4x4_lidar);
//            count++;
//        }
//        in.close();
//        std::cout << "Total " << count << " poses are loaded" << std::endl;
//    }
//
//    template<typename T>
//    int loadCloud(size_t idx, pcl::PointCloud<T> &cloud) const {
//        string filename = (boost::format("%s/%06d.bin") % cloud_dir_ % idx % cloud_format_).str();
//        FILE   *file    = fopen(filename.c_str(), "rb");
//        if (!file) {
//            std::cerr << "error: failed to load " << filename << std::endl;
//            return -1;
//        }
//
//        std::vector<float> buffer(2000000);
//        size_t             num_points =
//                                   fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) /
//                                   4;
//        fclose(file);
//
//        cloud.resize(num_points);
//        if (std::is_same<T, pcl::PointXYZ>::value) {
//            for (int i = 0; i < num_points; i++) {
//                auto &pt = cloud.at(i);
//                pt.x = buffer[i * 4];
//                pt.y = buffer[i * 4 + 1];
//                pt.z = buffer[i * 4 + 2];
//            }
//        } else if (std::is_same<T, pcl::PointXYZI>::value) {
//            for (int i = 0; i < num_points; i++) {
//                auto &pt = cloud.at(i);
//                pt.x         = buffer[i * 4];
//                pt.y         = buffer[i * 4 + 1];
//                pt.z         = buffer[i * 4 + 2];
//                pt.intensity = buffer[i * 4 + 3];
//            }
//        }
//    }
//
//    int loadGTLabel(const size_t idx, vector<uint32_t> &labels) {
//        string        label_name = (boost::format("%s/%06d.label") % gt_label_dir_ % idx).str();
//        std::ifstream label_input(label_name, std::ios::binary);
//        if (!label_input.is_open()) {
//            std::cerr << "Could not open the label!" << std::endl;
//            return -1;
//        }
//        label_input.seekg(0, std::ios::end);
//        uint32_t num_points = label_input.tellg() / sizeof(uint32_t);
//        label_input.seekg(0, std::ios::beg);
//
//        labels.resize(num_points);
//        label_input.read((char *) &labels[0], num_points * sizeof(uint32_t));
//    }
//
//    void parseGTLabel(const vector<uint32_t> &labels,
//                      vector<uint32_t> &semantic_labels, vector<uint32_t> &obj_ids) {
//        int num_pts = labels.size();
//        semantic_labels.resize(num_pts);
//        obj_ids.resize(num_pts);
//
//        int                 cnt = 0;
//        for (const uint32_t label: labels) {
//            uint32_t semantic_label = label & 0xFFFF;
//            uint32_t id             = label >> 16;
//
//            semantic_labels[cnt] = semantic_label;
//            obj_ids[cnt]         = id;
//        }
//    }
//
//    void loadScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI>& cloud, Eigen::Matrix4f &pose) {
//        loadCloud(i, cloud);
//        vector<uint32_t> labels, semantic_labels, obj_ids;
//        loadGTLabel(i, labels);
//        parseGTLabel(labels, semantic_labels, obj_ids);
//        if (cloud.points.size() == labels.size()) {
//            for (int i = 0; i < cloud.points.size(); ++i) {
//                auto& pt = cloud.points[i];
//                pt.intensity = semantic_labels[i];
//            }
//        } else {
//            throw invalid_argument("Something's wrong! The numbers of points are not matched to each other");
//        }
//
//        pose = getPose(i);
//    }
//
//    Eigen::Matrix4f getPose(size_t i) {
//        return poses_gt_[i];
//    }
//
//    pcl::PointCloud<pcl::PointXYZ> getAllPositions() const {
//        // For fetching loops
//        pcl::PointCloud<pcl::PointXYZ> poses_cloud;
//        poses_cloud.reserve(num_frames_);
//        for (auto const &pose: poses_gt_) {
//            pcl::PointXYZ pose_pt(pose(0, 3), pose(1, 3), pose(2, 3));
//            poses_cloud.push_back(pose_pt);
//        }
//        return poses_cloud;
//    }
//
    void printTest() { cout << "Don't worry!" << endl; }
};

//class BongeunsaLoader : public DataLoader {
//public:

//    BongeunsaLoader(string abs_dir) {
//        path_ = path;
//    }
//
//    ~BongeunsaLoader() {}
//
//};



#endif //ERASOR2_DATA_LOADER_H
