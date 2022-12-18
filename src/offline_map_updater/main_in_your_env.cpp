//
// Created by shapelim on 21. 10. 18..
//

#include "tools/erasor_utils.hpp"
#include <boost/format.hpp>
#include <std_msgs/Int8.h>
#include <cstdlib>
#include <erasor/OfflineMapUpdater.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#define INCREMENT_GROUND_LIKELIHOOD 10
#define DEFINITE_GROUND 0
#define GROUND_LABEL -1
#define UNKNOWN -1

string DATA_DIR;
int INTERVAL, INIT_IDX;
float VOXEL_SIZE;
bool STOP_FOR_EACH_FRAME;
std::string filename = "/staticmap_via_erasor.pcd";

using PointType = pcl::PointXYZI;


vector<float> split_line(string input, char delimiter) {
    vector<float> answer;
    stringstream ss(input);
    string temp;

    while (getline(ss, temp, delimiter)) {
        answer.push_back(stof(temp));
    }
    return answer;
}

void load_all_poses(string txt, vector<Eigen::Matrix4f >& poses){
    // These poses are already w.r.t body frame!
    // Thus, tf4x4 by pose * corresponding cloud -> map cloud
    cout<<"Target path: "<< txt<<endl;
    poses.clear();
    poses.reserve(2000);
    std::ifstream in(txt);
    std::string line;

    int count = 0;
    while (std::getline(in, line)) {
        if (count == 0){
            count++;
            continue;
        }

        vector<float> pose = split_line(line, ',');

        Eigen::Translation3f ts(pose[2], pose[3], pose[4]);
        Eigen::Quaternionf q(pose[8], pose[5], pose[6], pose[7]);
        Eigen::Matrix4f tf4x4_cam = Eigen::Matrix4f::Identity(); // Crucial!
        tf4x4_cam.topLeftCorner<3, 3>(0, 0) = q.toRotationMatrix();
        tf4x4_cam.topRightCorner(3, 1) = ts.vector();

        Eigen::Matrix4f tf4x4_lidar = tf4x4_cam;
        poses.emplace_back(tf4x4_lidar);
        count++;
    }
    std::cout<<"Total "<<count<<" poses are loaded"<<std::endl;
}

void calcMinMax(const vector<pcl::PointCloud<PointType>>& pcs, float& min_x, float& min_y,
                            float& max_x, float& max_y) {
    min_x = numeric_limits<float>::max();
    max_x = numeric_limits<float>::lowest();
    min_y = numeric_limits<float>::max();
    max_y = numeric_limits<float>::lowest();

    PointType min_pt, max_pt;
    for (const auto& cloud : pcs) {
        pcl::getMinMax3D(cloud, min_pt, max_pt);

        min_x = min(min_pt.x, min_x);
        min_y = min(min_pt.y, min_y);
        max_x = max(max_pt.x, max_x);
        max_y = max(max_pt.y, max_y);
    }
}

nav_msgs::OccupancyGrid setOccupancyGridMap(const float min_x, const float min_y,
                                            const float max_x, const float max_y,
                                            const float occugrid_resolution) {

    const int width = static_cast<int>( (max_x - min_x) / occugrid_resolution) + 1;
    const int height = static_cast<int>( (max_y - min_y) / occugrid_resolution) + 1;
    nav_msgs::OccupancyGrid gridmap;
    gridmap.info.resolution = occugrid_resolution;
    geometry_msgs::Pose origin;
    origin.position.x = min_x;
    origin.position.y = min_y;
    origin.position.z = -1.6;
    origin.orientation.w = 1;
    gridmap.info.origin = origin;
    gridmap.info.width = width;
    gridmap.info.height = height;

    for(int i=0;i < width * height; i++) {
        gridmap.data.push_back(UNKNOWN);
    }
    return gridmap;
}

// pos_x and pos_y should not have remainder
void voi2xygrid(
        const pcl::PointCloud<PointType> &src, float pos_x, float pos_y, float pos_z,
        float max_height, float min_height, float range, float resolution,
        vector<pcl::PointCloud<PointType>> &xygrid,
        pcl::PointCloud<PointType> &complement, std::string format="occugrid") {
    const int width = static_cast<int>(2 * range / resolution);
    const int height = static_cast<int>(2 * range / resolution);
    xygrid.resize(width * height);

    for (auto& grid : xygrid) {
        grid.points.clear();
    }

    for (auto const &pt : src.points) {
        if ((pt.z < pos_z + max_height) && (pt.z > pos_z + min_height) &&
                (pt.x < pos_x + range) && (pt.x > pos_x - range) &&
                (pt.y < pos_y + range) && (pt.y > pos_y - range)) {
            // +: To make indices positive
            int w, h;
            if (format == "occugrid") {
                // Left-bottom is the origin
                w = static_cast<int>((pt.x - (pos_x - range) ) / resolution);
                h = static_cast<int>((pt.y - (pos_y - range) ) / resolution);
            } else if (format == "gridmap") { // Grid map from ETH Zurich
                // Right-upper side is the origin
                w = static_cast<int>((pos_x + range - pt.x) / resolution);
                h = static_cast<int>((pos_y + range - pt.y) / resolution);
            }
//            std::cout << xygrid.size() << " <-> " << w + width * h << std::endl;
            xygrid[w + width * h].points.emplace_back(pt);
        } else { complement.points.push_back(pt); }
    }
}

bool isLikelyToBeGround(const pcl::PointCloud<PointType> pc) {
    int num_ground_pts = 0;
    for (const auto& pt: pc) {
        if (pt.intensity == GROUND_LABEL) {
           ++num_ground_pts;
        }
    }
    if ( (num_ground_pts > 0.8 * pc.points.size())) {
        return true; // Free
    } else { return false; }
}

nav_msgs::OccupancyGrid setEgocentricOccupancyGridMap(float range,
                                            const float occugrid_resolution,
                                            const vector<pcl::PointCloud<PointType>> & xygrid) {

    const int width = static_cast<int>(2 * range / occugrid_resolution);
    const int height = static_cast<int>(2 * range / occugrid_resolution);
    nav_msgs::OccupancyGrid gridmap;
    gridmap.info.resolution = occugrid_resolution;
    geometry_msgs::Pose origin;
    origin.position.x = -range;
    origin.position.y = -range;
    origin.position.z = -1.6;
    origin.orientation.w = 1;
    gridmap.info.origin = origin;
    gridmap.info.width = width;
    gridmap.info.height = height;

    // Initialization
    for(int i = 0;i < width * height; i++) {
        gridmap.data.push_back(UNKNOWN);
    }
    for(int i = 0;i < width * height; i++) {
        if (!xygrid[i].points.empty() && isLikelyToBeGround(xygrid[i])) {
            gridmap.data.at(i) = DEFINITE_GROUND;
        }
    }

    return gridmap;
}

grid_map::GridMap setEgocentricGridMap(float range,
                                      const float grid_resolution,
                                      const vector<pcl::PointCloud<PointType>> & xygrid) {

    grid_map::GridMap gridmap({"elevation", "steppable"});
    gridmap.setFrameId("map");
    gridmap.setGeometry(grid_map::Length(2 * range, 2 * range), grid_resolution);
    gridmap["elevation"].setConstant(0);
    gridmap["steppable"].setConstant(UNKNOWN);

    const int width = static_cast<int>(2 * range / grid_resolution);
    const int height = static_cast<int>(2 * range / grid_resolution);

    grid_map::Index idx;
    for (int u = 0; u < width; ++u) {
        for (int v = 0; v < height; ++v) {
            int i = u + width * v;
            if (!xygrid[i].points.empty() && isLikelyToBeGround(xygrid[i])) {
                idx(0) = u;
                idx(1) = v;
                gridmap.at("steppable", idx) = DEFINITE_GROUND;
            }
        }
    }
    return gridmap;
}

// range / resolution 했을 때 짝수가 되게!
float occugrid_resolution = 0.4;
float range_of_interest = 10.0;
float egocentric_grid_resolution = 0.2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "erasor_in_your_env");
    ros::NodeHandle nh;
//    erasor::OfflineMapUpdater updater = erasor::OfflineMapUpdater();

    nh.param<string>("/data_dir", DATA_DIR, "/");
    nh.param<float>("/voxel_size", VOXEL_SIZE, 0.075);
    nh.param<int>("/init_idx", INIT_IDX, 0);
    nh.param<int>("/interval", INTERVAL, 2);
    nh.param<bool>("/stop_for_each_frame", STOP_FOR_EACH_FRAME, false);

    std::string staticmap_path = std::getenv("HOME") + filename;

    // Set ROS visualization publishers
    ros::Publisher NodePublisher = nh.advertise<erasor::node>("/node/combined/optimized", 100, true);
    ros::Publisher MapPublisher = nh.advertise<sensor_msgs::PointCloud2>("/erasor2/map", 100, true);
    ros::Publisher CloudWithLabelsPublisher = nh.advertise<sensor_msgs::PointCloud2>("/erasor2/curr_scan", 100, true);
    ros::Publisher GridPublisher = nh.advertise<nav_msgs::OccupancyGrid>("/erasor2/occugrid", 100, true);
    ros::Rate loop_rate(1);

    // Set target data
    cout << "\033[1;32mTarget directory:" << DATA_DIR << "\033[0m" << endl;
    string pose_path = DATA_DIR + "/poses_lidar2body.csv";
    string map_path = DATA_DIR + "/partial_global_map.pcd";
    string pcd_dir = DATA_DIR + "/pcds"; //
    string ground_labels_dir = DATA_DIR + "/ground_labels"; //
    string instance_labels_dir = DATA_DIR + "/instance_labels"; //
    // Load raw pointcloud

    vector<Eigen::Matrix4f> poses;
    load_all_poses(pose_path, poses);

    int N = poses.size();

    // Load all point clouds
    vector<pcl::PointCloud<PointType>> pcs;
    vector<Eigen::Matrix4f> poses_submap;
    Eigen::Matrix4f new_origin;
    bool is_initial = true;
    int END_IDX = 270;
    for (int i = INIT_IDX; i < END_IDX; ++i) {
        signal(SIGINT, erasor_utils::signal_callback_handler);
        cout<< "\033[1;32m"<< i << " th\033[0m" << endl;

        pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>);
        string pcd_name = (boost::format("%s/%06d.pcd") % pcd_dir % i).str();
        erasor_utils::load_pcd(pcd_name, src_cloud);
        std::vector<uint32_t> instance_labels;
        std::vector<uint32_t> ground_labels;
        string inst_label_name = (boost::format("%s/%06d.label") % instance_labels_dir % i).str();
        string ground_label_name = (boost::format("%s/%06d.label") % ground_labels_dir % i).str();
//        cout << "\033[1;34m" << src_cloud->points.size() << endl;
//        cout << "\033[1;33m" << inst_label_name << endl;
//        cout << "\033[1;33m" << ground_label_name << "\033[0m" << endl;
        erasor_utils::load_labels(inst_label_name, instance_labels);
        erasor_utils::load_labels(ground_label_name, ground_labels);

        uint32_t max_i = 0;
        for (int j = 0; j < src_cloud->points.size(); ++j) {
            // Follow Rhiney and Lucas's format.
            if (ground_labels[j] == 9) {
                src_cloud->points[j].intensity = GROUND_LABEL;
            } else { // For non-ground points
                uint32_t inst_label            = instance_labels[j] >> 16;
                src_cloud->points[j].intensity = inst_label;
//                std::cout << inst_label << ", ";
                max_i = max(max_i, inst_label);
            }
        }

        Eigen::Matrix4f transform = poses[i];
        pcl::PointCloud<PointType>::Ptr transformed(new pcl::PointCloud<PointType>);
        if (is_initial) {
            new_origin = transform;
            is_initial = false;
        }
        poses_submap.emplace_back(new_origin.inverse() * transform);
        pcl::transformPointCloud(*src_cloud, *transformed, poses_submap.back());
        pcs.push_back(*transformed);

        vector<pcl::PointCloud<PointType>> xygrid;
        pcl::PointCloud<PointType> complement;
        voi2xygrid(*src_cloud, 0.0, 0.0, 0.0, 1.3, -1.6,
                   range_of_interest, egocentric_grid_resolution,
                   xygrid, complement);
        nav_msgs::OccupancyGrid gridmap = setEgocentricOccupancyGridMap(range_of_interest, egocentric_grid_resolution, xygrid);
        // Viz
//        CloudWithLabelsPublisher.publish(erasor_utils::cloud2msg(*src_cloud));
//        GridPublisher.publish(gridmap);
//        ros::spinOnce();
//        loop_rate.sleep();
    }

    pcl::PointCloud<PointType>::Ptr map_partial(new pcl::PointCloud<PointType>);

    float min_x, min_y, max_x, max_y;
    calcMinMax(pcs, min_x, min_y, max_x, max_y);
    nav_msgs::OccupancyGrid gridmap_submap = setOccupancyGridMap(min_x, min_y,
                                                                 max_x, max_y,
                                                                 occugrid_resolution);
    std::cout << "\033[1;32m" << gridmap_submap.info.origin.position.x << ", ";
    std::cout << "\033[1;32m" << gridmap_submap.info.origin.position.y << ", ";
    std::cout << "\033[1;32m" << gridmap_submap.info.origin.position.z << "\033[0m" << std::endl;
    int num_data = pcs.size();
    for (int k = 0; k < num_data; ++k) {
        int w_pc = static_cast<int>((poses_submap[k](0, 3) - min_x) / occugrid_resolution);
        int h_pc = static_cast<int>((poses_submap[k](1, 3) - min_y) / occugrid_resolution);
        std::cout << w_pc << " ,,, " << h_pc << std::endl;

        const float pos_x_approx = w_pc * occugrid_resolution + min_x;
        const float pos_y_approx = h_pc * occugrid_resolution + min_y;
        const int width = static_cast<int>(2 * range_of_interest / occugrid_resolution);
        const int height = static_cast<int>(2 * range_of_interest / occugrid_resolution);

        vector<pcl::PointCloud<PointType>> xygrid;
        pcl::PointCloud<PointType> complement;
        *map_partial += pcs[k];
        std::cout << poses_submap[k] << std::endl;
        std::cout << pos_x_approx << " , " << pos_y_approx << std::endl;
        voi2xygrid(pcs[k], pos_x_approx, pos_y_approx, poses_submap[k](2, 3), 1.3, -1.6,
                   range_of_interest, occugrid_resolution,
                   xygrid, complement);
        int count = 0;
        for (int h = h_pc - height / 2; h < h_pc + height /2; ++h) {
            for (int w = w_pc - width / 2; w < w_pc + width / 2; ++w) {
                auto target_pc = xygrid[count];
                if (!target_pc.points.empty() && isLikelyToBeGround(target_pc)) {
                    if (gridmap_submap.data.at(w + gridmap_submap.info.width * h) == UNKNOWN) {
                        gridmap_submap.data.at(w + gridmap_submap.info.width * h) = 0;
                    }
                }
                count++;
            }
        }
        GridPublisher.publish(gridmap_submap);
        ros::spinOnce();
        loop_rate.sleep();
    }

//    int failure_flag = erasor_utils::load_pcd(map_path, map_partial);
    MapPublisher.publish(erasor_utils::cloud2msg(*map_partial));
    while (ros::ok()) {
        GridPublisher.publish(gridmap_submap);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}