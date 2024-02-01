//
// Created by shapelim on 21. 10. 18..
//

#include "tools/erasor_utils.hpp"
#include "dataloader/dataloader.h"
#include "dataprocessor/TrajectoryClustering.hpp"
#include "erasor2/erasor2.h"

using namespace std;

using PointType = pcl::PointXYZI;

vector<Eigen::Vector3f> getPoses(const DataLoader& loader, const int start_frame,
                                 const int end_frame, const  int accum_interval) {
    // For fetching loops
    vector<Eigen::Vector3f> positions;
    vector<Eigen::Matrix4f> poses_gt;
    Eigen::Matrix4f pose_tmp = Eigen::Matrix4f::Identity();
    for (int i = start_frame; i < end_frame + accum_interval; ++i) {
        pose_tmp = loader.poses_gt_[i];
        poses_gt.emplace_back(pose_tmp);
    }
    for (auto const& pose: poses_gt) {
        Eigen::Vector3f position_vec(pose(0, 3), pose(1, 3), pose(2, 3));
        positions.emplace_back(position_vec);
    }
    return positions;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "erasor2_main");
    ros::NodeHandle nh;

    std::cout << "ERASOR2 started" << std::endl;
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
    int accum_interval = params->accum_interval_; // 여기서는 accumulation 의 interval이 2로 설정되어 있기는 함.



    string dynamic_label_root = params->abs_save_dir_ + "/" + "mos";
    if(!std::filesystem::exists(params->abs_save_dir_)){
        std::filesystem::create_directory(params->abs_save_dir_);
    }
    if(!std::filesystem::exists(dynamic_label_root)){
        std::filesystem::create_directory(dynamic_label_root);
    }

    const auto pose_cloud = getPoses(*loader, start_frame, end_frame, accum_interval);


    vector<vector<size_t>> submap_indices;
    if (params->run_traj_clustering_) {
        submap_indices = clusterTrajectoryXYZ(pose_cloud);
        const auto &[clustered, unclustered] = erasor_utils::clusterIndices2PointCloud(pose_cloud, submap_indices);
        if (!unclustered.empty()) {
            throw runtime_error("Some poses are not clustered!");
        }

        // You can see the results by using `rviz/clustering_viz.rviz`
        ros::Publisher pub_raw_traj = nh.advertise<sensor_msgs::PointCloud2>("unclustered",100, true);
        ros::Publisher pub_clustered = nh.advertise<sensor_msgs::PointCloud2>("clustered",100, true);
        sensor_msgs::PointCloud2 output, cluster_output;
        pcl::toROSMsg(unclustered, output);
        pcl::toROSMsg(clustered, cluster_output);
        output.header.frame_id = "map";
        cluster_output.header.frame_id = "map";
        for (int i = 0; i < 5; ++i) {
            pub_raw_traj.publish(output);
            pub_clustered.publish(cluster_output);
          ros::spinOnce();
        }
    } else {
        submap_indices.clear();
        vector<size_t> indices_tmp;
        for (size_t i = start_frame; i < end_frame + accum_interval; i+= accum_interval) {
            indices_tmp.emplace_back(i);
        }
        submap_indices.emplace_back(indices_tmp);
    }

    std::cout << "\033[1;32mStart to run ERASOR2\033[0m" << std::endl;
    int cluster_id = 0;
    for (const auto &indices: submap_indices) {
        unique_ptr<ERASOR2> erasor2(new ERASOR2());
        for (const auto &idx : indices) {
            signal(SIGINT, erasor_utils::signal_callback_handler);
//            if (idx % 10 == 0) {
//                cout << "[DataLoader] " << idx << "th frame comes!\n";
//            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gt_label(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_label(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gt_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr noise(new pcl::PointCloud<pcl::PointXYZI>);

            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            // A scan contains label in `intensity`
            if (dataset_name == "SemanticKITTI") {
                loader->getGTLabeledScan(idx, *cloud_gt_label);
                loader->rejectNeighboringPoints(*cloud_gt_label, erasor2->robot_body_size_, *cloud_gt_filtered, *noise);
            }

            loader->getScanAndPose(idx, *cloud_est_label, pose);
//            cout << idx << "th pose: " << endl << pose << endl;
//            cout << idx << "th cloud size: " << cloud_est_label->size() << endl;
            loader->rejectNeighboringPoints(*cloud_est_label, erasor2->robot_body_size_, *cloud_est_filtered, *noise);
//            std::cout << "cloud_est_filtered size: " << cloud_est_filtered->size() << std::endl;

            if (dataset_name == "SemanticKITTI") {
                erasor2->setScanAndPose(pose, *cloud_gt_filtered, *cloud_est_filtered);
            } else {
                erasor2->setScanAndPose(pose, *cloud_est_filtered);
            }
        }
        cout << "[ERASOR2] Complete to set scans and poses\n";

        erasor2->setSubmap();
        erasor2->updateSteppableRegion();
        //    erasor2->dilateAndErode(); // not using
        erasor2->detectMovingObjects();
        erasor2->filterDynamicObjects(); // ? Original
        //erasor2->filterDynamicObjectsAndSaveLabel(dynamic_label_root, start_frame); // ? Modified for saving dynamic labels
        erasor2->saveDynamicLabels(dynamic_label_root, indices);
        // dynamic_label_root
        string map_path = params->abs_save_dir_ + "/" + params->sequence_ + "_" + to_string(cluster_id) + "_frame_" + to_string(start_frame) +
                      "_to_" + to_string(end_frame) + "_estimated.pcd";
//        erasor2->saveStaticMap(map_path);
//
//        erasor2->publishStaticMapResults();
        ++cluster_id;
    }
    return 0;
}