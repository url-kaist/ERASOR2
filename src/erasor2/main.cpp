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
        loader = std::move(std::make_unique<SemanticKITTILoader>(params->abs_data_dir_,
                                                                 params->sequence_,
                                                                 params->instance_seg_method_));
    }

    else if (dataset_name == "HeLiPR") {
        loader = std::move(std::make_unique<HeLiPRLoader>(params->abs_data_dir_,
                                                          params->sequence_,
                                                          params->instance_seg_method_));
    }

    cout << "Set dataloader complete" << endl;
    cout << "From " << params->start_frame_ << " to " << params->end_frame_ << endl;
    cout << params->robot_body_size_ << endl;

    int start_frame = params->start_frame_;
    int end_frame   = params->end_frame_;
    int accum_interval = params->accum_interval_; // 여기서는 accumulation 의 interval 이 2로 설정되어 있기는 함.

    string map_path = params->abs_save_dir_ + "/" + params->sequence_ + "_" + to_string(start_frame) +
                      "_to_" + to_string(end_frame) + "_estimated.pcd";

    string dynamic_label_root = params->abs_save_dir_ + "/" + "mos";
    if(!std::filesystem::exists(params->abs_save_dir_)){
        std::filesystem::create_directory(params->abs_save_dir_);
    }
    if(!std::filesystem::exists(dynamic_label_root)){
        std::filesystem::create_directory(dynamic_label_root);
    }

    int cnt = 0;
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
        cout << i << "th pose: " << endl <<pose << endl;
        cout << i << "th cloud size: " << cloud_est_label->size() << endl;
        loader->rejectNeighboringPoints(*cloud_est_label, erasor2->robot_body_size_, *cloud_est_filtered, *noise);
        std::cout << "cloud_est_filtered size: " << cloud_est_filtered->size() << std::endl;

        if (dataset_name == "SemanticKITTI") {
            erasor2->setScanAndPose(pose, *cloud_gt_filtered, *cloud_est_filtered);
        } else {
            std::cout<<"please get this..." << std::endl;
            erasor2->setScanAndPose(pose, *cloud_est_filtered);
        }
    }
    cout << "[ERASOR2] Complete to set scans and poses\n";


    erasor2->setSubmap();
    erasor2->updateSteppableRegion();
// //    erasor2->dilateAndErode(); // not using
    erasor2->detectMovingObjects();
    erasor2->filterDynamicObjects(); // ? Original 
    // //erasor2->filterDynamicObjectsAndSaveLabel(dynamic_label_root, start_frame); // ? Modified for saving dynamic labels
    erasor2->saveDynamicLabels(dynamic_label_root, start_frame);
    // dynamic_label_root
    erasor2->saveStaticMap(map_path);

    erasor2->publishStaticMapResults();

    return 0;
}