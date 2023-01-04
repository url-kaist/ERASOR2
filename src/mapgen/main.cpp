#include "mapgen/mapgen.hpp"
#include "dataloader/dataloader.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mapgen");
    ros::NodeHandle nh;

    std::cout << "Mapgen started" << std::endl;
    unique_ptr<Mapgen> mapgen(new Mapgen());
    std::cout << "Set Mapgen complete" << std::endl;

    std::unique_ptr<DataLoader> loader;
    string                      dataset_name = mapgen->dataset_name_;
    if (dataset_name == "SemanticKITTI") {
        loader = std::move(std::make_unique<SemanticKITTILoader>(mapgen->abs_data_dir_, mapgen->sequence_));
    }

    std::cout << "Set dataloader complete" << std::endl;
    std::cout << "From " << mapgen->start_frame_ << " to " << mapgen->end_frame_ << std::endl;
    std::cout << mapgen->robot_body_size_ << endl;

    int start_frame = mapgen->start_frame_;
    int end_frame   = mapgen->end_frame_;
    int accum_interval = mapgen->accum_interval_;

    int cnt = 0;
    for (int i = start_frame; i < end_frame + accum_interval; ++i) {
        signal(SIGINT, erasor_utils::signal_callback_handler);
        if (i % 10 == 0) {
            std::cout << "[MAPGEN] " << i << "th frame comes!" << std::endl;
        }

        if (++cnt / accum_interval >= 1) {
            cnt = 0;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr noise(new pcl::PointCloud<pcl::PointXYZI>);

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        if (dataset_name == "SemanticKITTI") {
            loader->getGTLabeledScan(i, *cloud_raw);
            pose = loader->poses_gt_[i];
        }else {
            loader->getScanAndPose(i, *cloud_raw, pose);
        }

        loader->rejectNeighboringPoints(*cloud_raw, mapgen->robot_body_size_, *cloud, *noise);
        mapgen->accumPointCloud(*cloud, pose);
    }

    string abs_save_dir = mapgen->abs_save_dir_;
    string interval     = to_string(mapgen->accum_interval_);
    string voxel_size   = erasor_utils::format(mapgen->voxel_size_, 2);
    replace(voxel_size.begin(), voxel_size.end(), '.', '_');

    string original_path =
                   abs_save_dir + "/" + mapgen->sequence_ + "_" + to_string(start_frame) +
                   "_to_" + to_string(end_frame) + "_w_interval_" + interval + "_voxel_" + voxel_size +
                   "_original.pcd";

    string map_path = abs_save_dir + "/" + mapgen->sequence_ + "_" + to_string(start_frame) +
                      "_to_" + to_string(end_frame) + "_w_interval_" + interval + "_voxel_" + voxel_size +
                      ".pcd";

    mapgen->saveAccumMap(original_path, map_path);

    return 0;
}
