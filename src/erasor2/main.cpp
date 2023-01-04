//
// Created by shapelim on 21. 10. 18..
//

#include "tools/erasor_utils.hpp"
#include "dataloader/dataloader.h"
#include "erasor2/erasor2.h"

using namespace std;

using PointType = pcl::PointXYZI;

int main(int argc, char **argv) {
    ros::init(argc, argv, "erasor2_main");
    ros::NodeHandle nh;

    std::cout << "ERASOR2 started" << std::endl;
    unique_ptr<ERASOR2> erasor2(new ERASOR2());
    unique_ptr<RosParamServer> params(new RosParamServer());
    std::cout << "Set ERASOR2 complete" << std::endl;

    std::unique_ptr<DataLoader> loader;
    string                      dataset_name = params->dataset_name_;
    if (dataset_name == "SemanticKITTI") {
        loader = std::move(std::make_unique<SemanticKITTILoader>(params->abs_data_dir_, params->sequence_));
    }

    std::cout << "Set dataloader complete" << std::endl;
    std::cout << "From " << params->start_frame_ << " to " << params->end_frame_ << std::endl;
    std::cout << params->robot_body_size_ << endl;

    int start_frame = params->start_frame_;
    int end_frame   = params->end_frame_;
    int accum_interval = params->accum_interval_;

    string map_path = params->abs_save_dir_ + "/" + params->sequence_ + "_" + to_string(start_frame) +
                      "_to_" + to_string(end_frame) + "_estimated.pcd";
    int cnt = 0;
    for (int i = start_frame; i < end_frame + accum_interval; ++i) {
        signal(SIGINT, erasor_utils::signal_callback_handler);
        if (i % 10 == 0) {
            std::cout << "[ERASOR2] " << i << "th frame comes!" << std::endl;
        }

        if (++cnt / accum_interval >= 1) {
            cnt = 0;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gt_label(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_label(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gt_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr noise(new pcl::PointCloud<pcl::PointXYZI>);

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        // A scan contains label in `intensity`
        if (dataset_name == "SemanticKITTI") {
            loader->getGTLabeledScan(i, *cloud_gt_label);
            loader->rejectNeighboringPoints(*cloud_gt_label, erasor2->robot_body_size_, *cloud_gt_filtered, *noise);
        }

        loader->getScanAndPose(i, *cloud_est_label, pose);
        loader->rejectNeighboringPoints(*cloud_est_label, erasor2->robot_body_size_, *cloud_est_filtered, *noise);

        if (dataset_name == "SemanticKITTI") {
            erasor2->setScanAndPose(pose, *cloud_gt_filtered, *cloud_est_filtered);
        } else {
            erasor2->setScanAndPose(pose, *cloud_est_filtered);
        }
    }
    std::cout << "[ERASOR2] Complete to set scans and poses" << std::endl;
    erasor2->setSubmap();
    erasor2->updateSteppableRegion();
//    erasor2->dilateAndErode();
    erasor2->reprojectGroundLikelihood();
    erasor2->saveStaticMap(map_path);
    erasor2->publishStaticMapResults();

    return 0;
}