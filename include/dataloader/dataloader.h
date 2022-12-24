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
#include "tools/erasor_utils.hpp"

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
    inline void rejectNeighboringPoints(const pcl::PointCloud<T> &cloud_raw,
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
    inline size_t size() const;

    inline vector<float> splitLine(const string& input, char delimiter);

    inline void vec2tf4x4(vector<float> &pose, Eigen::Matrix4f &tf4x4);

    inline bool loadLabel(const std::string &label_name, vector<uint32_t> &labels);

    inline void countNumFrames(const string &pcd_dir, const string &pcd_format);

    inline Eigen::Matrix4f getPose(size_t i);

    /*
     * Virtual functions
     */
    // Important! 'virtual' is necessary
    // + the functions must be declared, i.e. {} is needed
    virtual void loadAllPoses(const string pose_path, vector<Eigen::Matrix4f> &poses) {}

    template<typename T>
    int loadCloud(size_t idx, pcl::PointCloud<T> &cloud) const {};

    virtual void getScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI>& cloud, Eigen::Matrix4f &pose) {}

    // Only for SemanticKITTI

    virtual void testInheritance() { cout << "Test inheritance" << endl; }

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

    SemanticKITTILoader(const string &abs_data_dir, const string &seq);

//    explicit SemanticKITTILoader(const string &cloud_dir, const string &cloud_format, const string &pose_path)
//    : DataLoader(cloud_dir, cloud_format, pose_path) { cout << (1 + 3) << endl; };
//

    ~SemanticKITTILoader() {}

    void loadAllPoses(string pose_path, vector<Eigen::Matrix4f> &poses);
//
    template<typename T>
    int loadCloud(size_t idx, pcl::PointCloud<T> &cloud) const {
        string filename = (boost::format("%s/%06d.%s") % cloud_dir_ % idx % cloud_format_).str();
        FILE   *file    = fopen(filename.c_str(), "rb");
        if (!file) {
            std::cerr << "Error: failed to load " << filename << std::endl;
            return -1;
        }

        std::vector<float> buffer(2000000);
        size_t             num_points =
                                   fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) /
                                   4;
        fclose(file);

        cloud.resize(num_points);
        if (std::is_same<T, pcl::PointXYZ>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
            }
        } else if (std::is_same<T, pcl::PointXYZI>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
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
                      vector<uint32_t> &semantic_labels, vector<uint32_t> &obj_ids);

    void getScanAndPose(size_t i, pcl::PointCloud<pcl::PointXYZI>& cloud, Eigen::Matrix4f &pose);
//
    pcl::PointCloud<pcl::PointXYZ> getAllPositions() const {
        // For fetching loops
        pcl::PointCloud<pcl::PointXYZ> poses_cloud;
        poses_cloud.reserve(num_frames_);
        for (auto const &pose: poses_gt_) {
            pcl::PointXYZ pose_pt(pose(0, 3), pose(1, 3), pose(2, 3));
            poses_cloud.push_back(pose_pt);
        }
        return poses_cloud;
    }

    void testInheritance() {
        cout << "print from SemanticKITTIloader" << endl;
    }

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
