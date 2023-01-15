#include "tools/erasor_utils.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "removert_update");
    std::cout<<"KiTTI MAPVIZ STARTED"<<std::endl;
    ros::NodeHandle nodeHandler;

    std::string raw_path;
    std::string removert_path;

    nodeHandler.param<std::string>("/raw_path", raw_path, "");
    nodeHandler.param<std::string>("/removert_path", removert_path, "");


    // load dense map
    std::cout<<"\033[1;32mLoading dense raw map...\033[0m"<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::load_pcd(raw_path, raw_cloud);

    std::cout<<"\033[1;32mLoading Removert static map...\033[0m"<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr removert(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::load_pcd(removert_path, removert);

    pcl::PointCloud<pcl::PointXYZI>::Ptr removert_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::voxelize_preserving_labels(removert, *removert_voxelized, 0.2); // All evaluation is set to 0.2
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
    std::cout << "\033[1;32mLoad complete \033[0m" << std::endl;

    Eigen::Matrix4f tf;
    // Only needed for removert!!!
    tf << 0,  0, 1, 0,
         -1,  0, 0, 0,
          0, -1, 0, 1.73,
          0,  0, 0, 1;
    pcl::transformPointCloud(*removert_voxelized, *transformed, tf);

    std::cout<<"Labeling removert pcd..."<<std::endl;
    vector<int> correspondences;
    erasor_utils::findCorrespondences(*transformed, *raw_cloud, correspondences);
    for (int i = 0; i < transformed->points.size(); ++i) {
        transformed->points[i].intensity = raw_cloud->points[correspondences[i]].intensity;
    }

    std::cout<<"Saving removert pcd..."<<std::endl;
    transformed->width = transformed->points.size();
    transformed->height = 1;
    std::string tmp = removert_path;
    tmp.erase(tmp.end()-4, tmp.end());
    std::string save_name = tmp + "_w_label.pcd";
    std::cout << "Save path: " << save_name << std::endl;
    pcl::io::savePCDFileASCII(save_name, *transformed);
    std::cout<<"\033[1;32mComplete to save!!!\033[0m"<<std::endl;

    return 0;
}
