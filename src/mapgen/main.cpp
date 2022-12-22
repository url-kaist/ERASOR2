#include "mapgen/mapgen.hpp"
#include "dataloader/dataloader.h"

ros::Publisher cloudPublisher;
ros::Publisher mapPublisher;
ros::Publisher pathPublisher;
nav_msgs::Path path;
//
//void callbackData(const node msg) {
//    signal(SIGINT, erasor_utils::signal_callback_handler);
//    static int cnt = 0;
//    if ((cnt % viz_interval) == 0){
//        std::cout << std::left << setw(print_width) << setfill(separator) << "[MAPGEN] " << msg.header.seq << "th frame comes!" << std::endl;
//    }
//
//    mapgenerator.accumPointCloud(msg, path);
//    if (msg.header.seq >= std::stoi(final_stamp)){
//        saveGlobalMap();
//    }
//
//    // Visualization
//    if ((cnt % viz_interval) == 0){
//        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCurr(new pcl::PointCloud<pcl::PointXYZI>());
//        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZI>());
//
//        mapgenerator.getPointClouds(cloudMap, cloudCurr);
//        cloudPublisher.publish(erasor_utils::cloud2msg(*cloudCurr));
//        mapPublisher.publish(erasor_utils::cloud2msg(*cloudMap));
//        pathPublisher.publish(path);
//    }
//    cnt++;
//}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mapgen");
    ros::NodeHandle nh;
    std::cout << "MAPGEN STARTED" << std::endl;

    unique_ptr<Mapgen> mapgen;
    mapgen.reset(new Mapgen());

    std::unique_ptr<DataLoader> loader;
    string dataset_name = mapgen->dataset_name_;
    if (dataset_name == "SemanticKITTI") {
        string cloud_dir = "";
        string cloud_format = "bin";
        string pose_path = "bin";
        loader = std::move(std::make_unique<SemanticKITTILoader>(cloud_dir, cloud_format, pose_path));
    }

    std::string save_path;
    for (int i = mapgen->init_idx_; i < mapgen->end_idx_; ++i) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        loader->loadScanAndPose(i, cloud, pose);
    }

//    std::string original_path =
//                        save_path + "/" + sequence + "_" + init_stamp +
//                        "_to_" + final_stamp + "_w_interval" + std::to_string(interval) + "_voxel_" + std::to_string(voxelsize) +
//                        "_original.pcd";
//
//    std::string map_path = save_path + "/" + sequence + "_" + init_stamp +
//                          "_to_" + final_stamp + "_w_interval" + std::to_string(interval) + "_voxel_" + std::to_string(voxelsize) +
//                          ".pcd";
//
//    mapgen->saveNaiveMap(original_path, map_path);

    return 0;
}
