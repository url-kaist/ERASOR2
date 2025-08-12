//
// Created by shapelim on 21. 10. 18..
//

#include <cstdlib>

#include <boost/format.hpp>
#include <std_msgs/Int8.h>

#include "tools/erasor_utils.hpp"
// #include <erasor/OfflineMapUpdater.h>
#include <algorithm>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define INCREMENT_GROUND_LIKELIHOOD 0.3
#define DECREMENT_GROUND_LIKELIHOOD 0.1
#define LOWEST_LIKELIHOOD 0.0
#define POTENTIAL_GROUND 0.5
#define POTENTIAL_NONGROUND 0.2
// Point-wise label
#define GROUND_LABEL -1
#define NOISE_LABEL -2
#define NOT_VOLUME_OF_INTEREST -3
#define NOT_INTEREST 0  // ground and noisy points in the estimated labels
// Gridmap characteristics
#define unknown_ground_likelihood_ -1
// Just for visualization
#define DIST_FROM_GROUND_TO_ORIGIN -1.8

using namespace std;

string DATA_DIR;
int INTERVAL, INIT_IDX, END_IDX;
float VOXEL_SIZE;
bool STOP_FOR_EACH_FRAME;
std::string filename = "/staticmap_via_erasor.pcd";

using PointType = pcl::PointXYZI;

struct GridMapInfo {
  float center_x;
  float center_y;
  float resolution;
  int width;
  int height;
  // Remainders of x_length / grid_resolution and
  // y_length / grid_resolution should be zeros, respectively.
  float x_length;
  float y_length;
};

visualization_msgs::Marker setVisualMarker(const float voxel_size,
                                           const float pos_x,
                                           const float pos_y) {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = "map";
  marker.header.stamp       = ros::Time();
  marker.ns                 = "my_namespace";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = pos_x;
  marker.pose.position.y    = pos_y;
  marker.pose.position.z    = DIST_FROM_GROUND_TO_ORIGIN;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = voxel_size;
  marker.scale.y            = voxel_size;
  marker.scale.z            = 0.1;
  marker.color.a            = 1.0;  // Don't forget to set the alpha!
  marker.color.r            = 0.0;
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;

  return marker;
}

vector<float> split_line(string input, char delimiter) {
  vector<float> answer;
  stringstream ss(input);
  string temp;

  while (getline(ss, temp, delimiter)) {
    answer.push_back(stof(temp));
  }
  return answer;
}

void load_all_poses(string txt, vector<Eigen::Matrix4f>& poses) {
  // These poses are already w.r.t body frame!
  // Thus, tf4x4 by pose * corresponding cloud -> map cloud
  cout << "Target path: " << txt << endl;
  poses.clear();
  poses.reserve(2000);
  std::ifstream in(txt);
  std::string line;

  int count = 0;
  while (std::getline(in, line)) {
    if (count == 0) {
      count++;
      continue;
    }

    vector<float> pose = split_line(line, ',');

    Eigen::Translation3f ts(pose[2], pose[3], pose[4]);
    Eigen::Quaternionf q(pose[8], pose[5], pose[6], pose[7]);
    Eigen::Matrix4f tf4x4_cam           = Eigen::Matrix4f::Identity();  // Crucial!
    tf4x4_cam.topLeftCorner<3, 3>(0, 0) = q.toRotationMatrix();
    tf4x4_cam.topRightCorner(3, 1)      = ts.vector();

    Eigen::Matrix4f tf4x4_lidar = tf4x4_cam;
    poses.emplace_back(tf4x4_lidar);
    count++;
  }
  std::cout << "Total " << count << " poses are loaded" << std::endl;
}

void assignLabels(const std::vector<uint32_t> ground_labels,
                  const std::vector<uint32_t> instance_labels,
                  const float min_z_voi,
                  const float max_z_voi,
                  pcl::PointCloud<PointType>& src_cloud,
                  uint32_t& max_instance) {
  max_instance = 0;
  for (int j = 0; j < src_cloud.points.size(); ++j) {
    // Follow Rhiney and Lucas's format.
    if (ground_labels[j] == 1) {
      src_cloud.points[j].intensity = GROUND_LABEL;
    } else {  // For non-ground points
      uint32_t inst_label           = instance_labels[j] >> 16;
      src_cloud.points[j].intensity = inst_label;
      //                std::cout << inst_label << ", ";
      max_instance = max(max_instance, inst_label);
    }
  }
  // Set volume of interest (voi)
  for (int j = 0; j < src_cloud.points.size(); ++j) {
    if (src_cloud.points[j].z < min_z_voi || src_cloud.points[j].z > max_z_voi) {
      src_cloud.points[j].intensity = NOT_VOLUME_OF_INTEREST;
    }
  }
}

nav_msgs::OccupancyGrid setOccupancyGridMap(const float min_x,
                                            const float min_y,
                                            const float max_x,
                                            const float max_y,
                                            const float occugrid_resolution) {
  const int width  = static_cast<int>((max_x - min_x) / occugrid_resolution) + 1;
  const int height = static_cast<int>((max_y - min_y) / occugrid_resolution) + 1;
  nav_msgs::OccupancyGrid gridmap;
  gridmap.info.resolution = occugrid_resolution;
  geometry_msgs::Pose origin;
  origin.position.x    = min_x;
  origin.position.y    = min_y;
  origin.position.z    = DIST_FROM_GROUND_TO_ORIGIN;
  origin.orientation.w = 1;
  gridmap.info.origin  = origin;
  gridmap.info.width   = width;
  gridmap.info.height  = height;

  for (int i = 0; i < width * height; i++) {
    gridmap.data.push_back(unknown_ground_likelihood_);
  }
  return gridmap;
}

GridMapInfo setGridMapParams(const float min_x,
                             const float min_y,
                             const float max_x,
                             const float max_y,
                             const float grid_resolution) {
  GridMapInfo grid_map_info;
  grid_map_info.center_x   = (min_x + max_x) / 2.0;
  grid_map_info.center_y   = (min_y + max_y) / 2.0;
  grid_map_info.resolution = grid_resolution;

  grid_map_info.width  = static_cast<int>(ceil((max_x - min_x) / grid_resolution));
  grid_map_info.height = static_cast<int>(ceil((max_x - min_x) / grid_resolution));

  // Remainders of x_length / grid_resolution and
  // y_length / grid_resolution should be zeros, respectively.
  grid_map_info.x_length = ceil((max_x - min_x) / grid_resolution) * grid_resolution;
  grid_map_info.y_length = ceil((max_y - min_y) / grid_resolution) * grid_resolution;
  return grid_map_info;
}

grid_map::GridMap setMapcentricGridMap(const GridMapInfo& grid_map_info) {
  grid_map::GridMap gridmap({"elevation", "steppable"});
  gridmap.setFrameId("map");
  gridmap.setGeometry(grid_map::Length(grid_map_info.x_length, grid_map_info.y_length),
                      grid_map_info.resolution);
  gridmap["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
  gridmap["steppable"].setConstant(unknown_ground_likelihood_);
  gridmap.setPosition(grid_map::Position(grid_map_info.center_x, grid_map_info.center_y));
  return gridmap;
}

// pos_x and pos_y should not have remainder
// void voi2xygrid(
//        const pcl::PointCloud<PointType> &src, float pos_x, float pos_y, float pos_z,
//        float max_height, float min_height, float range, float resolution,
//        vector<pcl::PointCloud<PointType>> &xygrid,
//        std::string format="gridmap") {
//    const int width = static_cast<int>(2 * range / resolution);
//    const int height = static_cast<int>(2 * range / resolution);
//    xygrid.resize(width * height);
//
//    for (auto& grid : xygrid) {
//        grid.points.clear();
//    }
//
//    for (auto const &pt : src.points) {
//        if ((pt.z < pos_z + max_height) && (pt.z > pos_z + min_height) &&
//                (pt.x < pos_x + range) && (pt.x > pos_x - range) &&
//                (pt.y < pos_y + range) && (pt.y > pos_y - range)) {
//            // +: To make indices positive
//            int w, h;
//            if (format == "occugrid") {
//                // Left-bottom is the origin
//                w = static_cast<int>((pt.x - (pos_x - range) ) / resolution);
//                h = static_cast<int>((pt.y - (pos_y - range) ) / resolution);
//            } else if (format == "gridmap") { // Grid map from ETH Zurich
//                // Right-upper side is the origin
//                w = static_cast<int>((pos_x + range - pt.x) / resolution);
//                h = static_cast<int>((pos_y + range - pt.y) / resolution);
//            } else { throw invalid_argument("Not implemented"); }
////            std::cout << xygrid.size() << " <-> " << w + width * h << std::endl;
//            xygrid[w + width * h].points.emplace_back(pt);
//    }
//}

void voi2xygrid(const pcl::PointCloud<PointType>& src,
                float pos_x,
                float pos_y,
                float pos_z,
                float max_height,
                float min_height,
                float range,
                float resolution,
                vector<pcl::PointCloud<PointType>>& xygrid,
                pcl::PointCloud<PointType>& complement,
                std::string format = "gridmap") {
  const int width  = static_cast<int>(2 * range / resolution);
  const int height = static_cast<int>(2 * range / resolution);
  xygrid.resize(width * height);

  for (auto& grid : xygrid) {
    grid.points.clear();
  }

  for (auto const& pt : src.points) {
    if ((pt.z < pos_z + max_height) && (pt.z > pos_z + min_height) && (pt.x < pos_x + range) &&
        (pt.x > pos_x - range) && (pt.y < pos_y + range) && (pt.y > pos_y - range)) {
      // +: To make indices positive
      int w, h;
      if (format == "occugrid") {
        // Left-bottom is the origin
        w = static_cast<int>((pt.x - (pos_x - range)) / resolution);
        h = static_cast<int>((pt.y - (pos_y - range)) / resolution);
      } else if (format == "gridmap") {  // Grid map from ETH Zurich
        // Right-upper side is the origin
        w = static_cast<int>((pos_x + range - pt.x) / resolution);
        h = static_cast<int>((pos_y + range - pt.y) / resolution);
      } else {
        throw invalid_argument("Not implemented");
      }
      //            std::cout << xygrid.size() << " <-> " << w + width * h << std::endl;
      xygrid[w + width * h].points.emplace_back(pt);
    } else {
      complement.points.push_back(pt);
    }
  }
}

bool isLikelyToBeGround(const pcl::PointCloud<PointType> pc, float ratio_num = 0.95) {
  int num_ground_pts = 0;
  for (const auto& pt : pc) {
    if (pt.intensity == GROUND_LABEL) {
      ++num_ground_pts;
    }
  }
  if ((num_ground_pts > ratio_num * pc.points.size())) {
    return true;  // Free
  } else {
    return false;
  }
}

bool isLikelyToBeSteppableRegion(const pcl::PointCloud<PointType>& curr_pc,
                                 const pcl::PointCloud<PointType>& map_pc,
                                 const float scan_ratio_threshold,
                                 const float th_bin_max_h,
                                 const bool verbose = false) {
  if (curr_pc.empty() || map_pc.empty()) {
    return false;
  }
  float curr_min_z, curr_max_z;
  float map_min_z, map_max_z;
  PointType min_pt, max_pt;

  pcl::getMinMax3D(curr_pc, min_pt, max_pt);
  curr_min_z = min_pt.z;
  curr_max_z = max_pt.z;

  pcl::getMinMax3D(map_pc, min_pt, max_pt);
  map_min_z = min_pt.z;
  map_max_z = max_pt.z;

  float lowest_z    = min(curr_min_z, map_min_z);
  float map_h_diff  = map_max_z - lowest_z;
  float curr_h_diff = curr_min_z - lowest_z;
  float scan_ratio  = min(map_h_diff / curr_h_diff, curr_h_diff / map_h_diff);

  //    std::cout << map_max_z << ", " << map_min_z << std::endl;
  //    std::cout << curr_max_z << ", " << curr_min_z << std::endl;
  //    std::cout << map_h_diff << ", " << curr_h_diff << " => " << scan_ratio << std::endl;
  //    std::cout << isLikelyToBeGround(curr_pc, 0.99) << std::endl;

  // Dynamic!
  if (scan_ratio < scan_ratio_threshold && isLikelyToBeGround(curr_pc, 0.99)) {  // find dynamic!
    if (map_max_z > th_bin_max_h) {  // To reduce false positives
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

nav_msgs::OccupancyGrid setEgocentricOccupancyGridMap(
    float range,
    const float occugrid_resolution,
    const vector<pcl::PointCloud<PointType>>& xygrid) {
  const int width  = static_cast<int>(2 * range / occugrid_resolution);
  const int height = static_cast<int>(2 * range / occugrid_resolution);
  nav_msgs::OccupancyGrid gridmap;
  gridmap.info.resolution = occugrid_resolution;
  geometry_msgs::Pose origin;
  origin.position.x    = -range;
  origin.position.y    = -range;
  origin.position.z    = -1.6;
  origin.orientation.w = 1;
  gridmap.info.origin  = origin;
  gridmap.info.width   = width;
  gridmap.info.height  = height;

  // Initialization
  for (int i = 0; i < width * height; i++) {
    gridmap.data.push_back(unknown_ground_likelihood_);
  }
  for (int i = 0; i < width * height; i++) {
    if (!xygrid[i].points.empty() && isLikelyToBeGround(xygrid[i])) {
      gridmap.data.at(i) = POTENTIAL_GROUND;
    }
  }

  return gridmap;
}

grid_map::GridMap setEgocentricGridMap(float range,
                                       const float grid_resolution,
                                       const vector<pcl::PointCloud<PointType>>& xygrid) {
  grid_map::GridMap gridmap({"elevation", "steppable"});
  gridmap.setFrameId("map");
  gridmap.setGeometry(grid_map::Length(2 * range, 2 * range), grid_resolution);
  gridmap["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
  gridmap["steppable"].setConstant(unknown_ground_likelihood_);

  const int width  = static_cast<int>(2 * range / grid_resolution);
  const int height = static_cast<int>(2 * range / grid_resolution);

  grid_map::Index idx;
  for (int u = 0; u < width; ++u) {
    for (int v = 0; v < height; ++v) {
      int i = u + width * v;
      if (!xygrid[i].points.empty() && isLikelyToBeGround(xygrid[i])) {
        idx(0)                       = u;
        idx(1)                       = v;
        gridmap.at("steppable", idx) = POTENTIAL_GROUND;
      }
    }
  }
  return gridmap;
}

void dilateAndErode(grid_map::GridMap& gridmap_submap) {
  // Noise filtering?
  cv::Mat img, img_eroded, img_dilated;
  const float min_coefficient = gridmap_submap.get("steppable").minCoeff();
  const float max_coefficient = gridmap_submap.get("steppable").maxCoeff();
  std::cout << min_coefficient << ", " << max_coefficient << std::endl;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
      gridmap_submap, "steppable", CV_8UC1, min_coefficient, max_coefficient, img);
  std::string save_dir = "/home/shapelim/Pictures/erasor2";
  cv::imwrite(save_dir + "/original.png", img);
  dilate(img, img_dilated, cv::Mat::ones(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 2);
  cv::imwrite(save_dir + "/dilation.png", img_dilated);
  erode(img_dilated, img_eroded, cv::Mat::ones(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 2);
  cv::imwrite(save_dir + "/erosion.png", img_eroded);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
      img_eroded, "steppable", gridmap_submap, min_coefficient, max_coefficient);
  const float min_coefficient_after = gridmap_submap.get("steppable").minCoeff();
  const float max_coefficient_after = gridmap_submap.get("steppable").maxCoeff();
  std::cout << min_coefficient_after << ", " << max_coefficient_after << std::endl;
}

// range / resolution 했을 때 짝수가 되게!
float grid_resolution;
float range_of_interest;
float egocentric_grid_resolution;
float ground_log_odds_thr;
float min_z_voi, max_z_voi;
float scan_ratio_threshold;
float th_bin_max_h;

bool verbose;

int main(int argc, char** argv) {
  ros::init(argc, argv, "erasor_in_your_env");
  ros::NodeHandle nh;
  //    erasor::OfflineMapUpdater updater = erasor::OfflineMapUpdater();

  nh.param<string>("/data_dir", DATA_DIR, "/");
  nh.param<float>("/voxel_size", VOXEL_SIZE, 0.075);
  nh.param<int>("/init_idx", INIT_IDX, 0);
  nh.param<int>("/end_idx", END_IDX, 0);
  nh.param<float>("/grid_resolution", grid_resolution, 0.4);
  nh.param<float>("/egocentric_grid_resolution", egocentric_grid_resolution, 0.2);
  nh.param<float>("/range_of_interest", range_of_interest, 10.0);
  nh.param<float>("/ground_log_odds_thr", ground_log_odds_thr, 0.5);
  nh.param<int>("/interval", INTERVAL, 2);
  nh.param<bool>("/stop_for_each_frame", STOP_FOR_EACH_FRAME, false);
  nh.param<bool>("/verbose", verbose, false);
  nh.param<float>("/erasor2/min_z_voi", min_z_voi, -1.6);
  nh.param<float>("/erasor2/max_z_voi", max_z_voi, 1.3);
  nh.param<float>("/erasor2/scan_ratio_threshold", scan_ratio_threshold, 0.3);
  nh.param<float>("/erasor2/th_bin_max_h", th_bin_max_h, 0.3);
  nh.param<float>("/erasor2/th_bin_max_h", th_bin_max_h, 0.3);

  std::string staticmap_path = std::getenv("HOME") + filename;

  // Set ROS visualization publishers
  ros::Publisher MapPublisher = nh.advertise<sensor_msgs::PointCloud2>("/erasor2/map", 100, true);
  ros::Publisher DynMapPublisher =
      nh.advertise<sensor_msgs::PointCloud2>("/erasor2/dyn_points_all", 100, true);
  ros::Publisher DynCurrScanPublisher =
      nh.advertise<sensor_msgs::PointCloud2>("/erasor2/dyn_points", 100, true);
  ros::Publisher CurrScanPublisher =
      nh.advertise<sensor_msgs::PointCloud2>("/erasor2/curr_scan", 100, true);
  ros::Publisher EgocentricGridPublisher =
      nh.advertise<grid_map_msgs::GridMap>("/erasor2/gridmap_egocentric", 100, true);
  ros::Publisher GridPublisher =
      nh.advertise<grid_map_msgs::GridMap>("/erasor2/gridmap", 100, true);
  ros::Publisher VizMarkerPublisher =
      nh.advertise<visualization_msgs::Marker>("/erasor2/target_grid_loc", 100, true);
  ros::Rate loop_rate(50);

  // Set target data
  cout << "\033[1;32mTarget directory:" << DATA_DIR << "\033[0m" << endl;
  string pose_path           = DATA_DIR + "/poses_lidar2body.csv";
  string map_path            = DATA_DIR + "/partial_global_map.pcd";
  string pcd_dir             = DATA_DIR + "/pcds";                            //
  string ground_labels_dir   = DATA_DIR + "/ground_labels_via_patchwork2";    //
  string instance_labels_dir = DATA_DIR + "/instance_labels_via_patchwork2";  //
  // Load raw pointcloud

  vector<Eigen::Matrix4f> poses;
  load_all_poses(pose_path, poses);

  int N = poses.size();

  // Load all point clouds
  vector<pcl::PointCloud<PointType>> pcs;
  vector<Eigen::Matrix4f> poses_submap;
  Eigen::Matrix4f new_origin;
  bool is_initial = true;
  int END_IDX     = 270;
  for (int i = INIT_IDX; i < END_IDX; ++i) {
    signal(SIGINT, erasor_utils::signal_callback_handler);
    cout << "\033[1;32m" << i << " th\033[0m" << endl;

    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>);
    string pcd_name = (boost::format("%s/%06d.pcd") % pcd_dir % i).str();
    erasor_utils::load_pcd(pcd_name, src_cloud);
    std::vector<uint32_t> instance_labels;
    std::vector<uint32_t> ground_labels;
    string inst_label_name   = (boost::format("%s/%06d.label") % instance_labels_dir % i).str();
    string ground_label_name = (boost::format("%s/%06d.label") % ground_labels_dir % i).str();

    erasor_utils::load_labels(inst_label_name, instance_labels);
    erasor_utils::load_labels(ground_label_name, ground_labels);

    // 1. Set labels
    uint32_t max_instance;
    assignLabels(ground_labels, instance_labels, min_z_voi, max_z_voi, *src_cloud, max_instance);

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
    voi2xygrid(*src_cloud,
               0.0,
               0.0,
               0.0,
               max_z_voi,
               min_z_voi,
               range_of_interest,
               egocentric_grid_resolution,
               xygrid,
               complement);
    grid_map::GridMap gridmap =
        setEgocentricGridMap(range_of_interest, egocentric_grid_resolution, xygrid);
    // Viz
    CurrScanPublisher.publish(erasor_utils::cloud2msg(*src_cloud));
    grid_map_msgs::GridMap grid_msg;
    grid_map::GridMapRosConverter::toMessage(gridmap, grid_msg);
    EgocentricGridPublisher.publish(grid_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  /*
   * Submap-based static map building
   */
  int num_data = pcs.size();
  pcl::PointCloud<PointType>::Ptr map_partial_src(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr map_partial(new pcl::PointCloud<PointType>);
  for (int k = 0; k < num_data; ++k) {
    *map_partial_src += pcs[k];
  }

  static pcl::VoxelGrid<PointType> voxel_filter;
  voxel_filter.setInputCloud(map_partial_src);
  voxel_filter.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
  voxel_filter.filter(*map_partial);

  float min_x, min_y, max_x, max_y;
  erasor_utils::calcMinMaxXY(pcs, min_x, min_y, max_x, max_y);
  GridMapInfo grid_map_info        = setGridMapParams(min_x, min_y, max_x, max_y, grid_resolution);
  grid_map::GridMap gridmap_submap = setMapcentricGridMap(grid_map_info);

  vector<vector<pcl::PointCloud<PointType>>> xygrids;
  vector<grid_map::Index> idxes_approx;
  xygrids.resize(num_data);
  idxes_approx.resize(num_data);
  for (int k = 0; k < num_data; ++k) {
    grid_map::Position position(poses_submap[k](0, 3), poses_submap[k](1, 3));
    gridmap_submap.getIndex(position, idxes_approx[k]);

    int w_pc = idxes_approx[k](0);
    int h_pc = idxes_approx[k](1);

    grid_map::Position pos_approx;
    pos_approx(0) =
        grid_map_info.x_length / 2 + grid_map_info.center_x - w_pc * grid_map_info.resolution;
    pos_approx(1) =
        grid_map_info.y_length / 2 + grid_map_info.center_y - h_pc * grid_map_info.resolution;
    //        std::cout << "[Before] " << poses_submap[k](0, 3) << ", " << poses_submap[k](1, 3) <<
    //        std::endl; std::cout << "[After] " << pos_approx(0) << ", " << pos_approx(1) <<
    //        std::endl;

    pcl::PointCloud<PointType> complement;
    //        std::cout << poses_submap[k] << std::endl;
    //        std::cout << pos_x_approx << " , " << pos_y_approx << std::endl;
    voi2xygrid(pcs[k],
               pos_approx(0),
               pos_approx(1),
               poses_submap[k](2, 3),
               max_z_voi,
               min_z_voi,
               range_of_interest,
               grid_resolution,
               xygrids[k],
               complement);
    vector<pcl::PointCloud<PointType>> map_grid;
    pcl::PointCloud<PointType>::Ptr dummy(new pcl::PointCloud<PointType>);
    voi2xygrid(*map_partial,
               pos_approx(0),
               pos_approx(1),
               poses_submap[k](2, 3),
               max_z_voi,
               min_z_voi,
               range_of_interest,
               grid_resolution,
               map_grid,
               *dummy);

    // Only square range is available
    const int width  = static_cast<int>(2 * range_of_interest / grid_resolution);
    const int height = static_cast<int>(2 * range_of_interest / grid_resolution);

    grid_map::Index idx;
    int count = 0;
    for (int h = h_pc - height / 2; h < h_pc + height / 2; ++h) {
      for (int w = w_pc - width / 2; w < w_pc + width / 2; ++w) {
        // x direction first
        if (xygrids[k][count].points.size() > 1) {
          idx(0) = w;
          idx(1) = h;
          //                    std::cout << "\033[1;34m(" << w << ", " << h << ")\033[0m" <<
          //                    std::endl;
          if (isLikelyToBeSteppableRegion(xygrids[k][count],
                                          map_grid[count],
                                          scan_ratio_threshold,
                                          th_bin_max_h,
                                          verbose)) {
            if (gridmap_submap.at("steppable", idx) == unknown_ground_likelihood_) {
              gridmap_submap.at("steppable", idx) = POTENTIAL_GROUND;
            } else {
              gridmap_submap.at("steppable", idx) += INCREMENT_GROUND_LIKELIHOOD;
            }
          }
        }
        ++count;
      }
    }
    // Visualization
    CurrScanPublisher.publish(erasor_utils::cloud2msg(pcs[k]));
    grid_map_msgs::GridMap grid_msg;
    grid_map::GridMapRosConverter::toMessage(gridmap_submap, grid_msg);
    GridPublisher.publish(grid_msg);
    ros::spinOnce();
    cin.ignore();
    //        loop_rate.sleep();
  }

  //    dilateAndErode(gridmap_submap);

  // Re-project ground likelihood to each scan
  vector<vector<float>> dyn_points_ids_set;
  dyn_points_ids_set.resize(num_data);
  for (int k = 0; k < num_data; ++k) {
    vector<float> dyn_cand_ids;
    const int width  = static_cast<int>(2 * range_of_interest / grid_resolution);
    const int height = static_cast<int>(2 * range_of_interest / grid_resolution);

    int w_pc = idxes_approx[k](0);
    int h_pc = idxes_approx[k](1);

    grid_map::Index idx;
    int count = 0;
    // Searching
    for (int h = h_pc - height / 2; h < h_pc + height / 2; ++h) {
      for (int w = w_pc - width / 2; w < w_pc + width / 2; ++w) {
        idx(0) = w;
        idx(1) = h;
        if (gridmap_submap.at("steppable", idx) > ground_log_odds_thr) {
          // Extract indices
          for (const auto pt : xygrids[k][count].points) {
            if ((pt.intensity != GROUND_LABEL) && (pt.intensity != NOT_INTEREST) &&
                std::find(dyn_cand_ids.begin(), dyn_cand_ids.end(), pt.intensity) ==
                    dyn_cand_ids.end()) {
              dyn_cand_ids.push_back(pt.intensity);
            }
          }
        }
        ++count;
      }
    }

    // Filtering
    auto& ids = dyn_points_ids_set[k];
    ids.clear();
    for (const int dyn_cand_id : dyn_cand_ids) {
      // For visualization
      pcl::PointCloud<PointType>::Ptr dynamic_points(new pcl::PointCloud<PointType>);
      for (const auto& pt : pcs[k]) {
        if (pt.intensity == dyn_cand_id) {
          dynamic_points->points.emplace_back(pt);
        }
      }
      float total_score = 0;
      for (const auto& dyn_pt : dynamic_points->points) {
        grid_map::Position p_tmp(dyn_pt.x, dyn_pt.y);
        grid_map::Index idx_tmp;
        gridmap_submap.getIndex(p_tmp, idx_tmp);
        total_score += gridmap_submap.at("steppable", idx_tmp);
      }
      if (total_score / dynamic_points->points.size() > ground_log_odds_thr) {
        ids.push_back(dyn_cand_id);
      }
    }
  }

  pcl::PointCloud<PointType>::Ptr dynamic_points(new pcl::PointCloud<PointType>);
  for (int k = 0; k < num_data; ++k) {
    pcl::PointCloud<PointType>::Ptr dynamic_points_each_scan(new pcl::PointCloud<PointType>);
    auto& each_pc = pcs[k];
    auto& ids     = dyn_points_ids_set[k];
    for (const auto& pt : each_pc) {
      if (std::find(ids.begin(), ids.end(), pt.intensity) != ids.end()) {
        if (pt.intensity == NOT_VOLUME_OF_INTEREST) {
          continue;
        }
        dynamic_points_each_scan->points.emplace_back(pt);
        dynamic_points->points.emplace_back(pt);
      }
    }
    CurrScanPublisher.publish(erasor_utils::cloud2msg(each_pc));
    DynCurrScanPublisher.publish(erasor_utils::cloud2msg(*dynamic_points_each_scan));
    CurrScanPublisher.publish(erasor_utils::cloud2msg(each_pc));
    DynCurrScanPublisher.publish(erasor_utils::cloud2msg(*dynamic_points_each_scan));
    if (STOP_FOR_EACH_FRAME) {
      std::cout << "Waiting for pressing a key" << std::endl;
      cin.ignore();
    }
  }

  // Final viz
  DynMapPublisher.publish(erasor_utils::cloud2msg(*dynamic_points));
  MapPublisher.publish(erasor_utils::cloud2msg(*map_partial));
  ros::Rate final_loop_rate(1);
  while (ros::ok()) {
    grid_map_msgs::GridMap grid_msg;
    grid_map::GridMapRosConverter::toMessage(gridmap_submap, grid_msg);
    GridPublisher.publish(grid_msg);
    ros::spinOnce();
    final_loop_rate.sleep();
  }

  return 0;
}
