#include "erasor2/erasor2.h"

using namespace std;

ERASOR2::ERASOR2() {
    initializePointClouds();
}

ERASOR2::~ERASOR2() {}

double ERASOR2::xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
    if (y >= 0) {
        return atan2(y, x); // 1, 2 quadrant
    } else {
        return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
    }
}

double ERASOR2::xy2radius(const double &x, const double &y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

void ERASOR2::initializePointClouds() {
    int num_pts_for_reserve = 2000000;
    map_dynamic_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    map_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    map_complement_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    static_map_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    static_map_voxelized_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    map_dynamic_->points.reserve(num_pts_for_reserve);
    map_accum_->points.reserve(num_pts_for_reserve);
    map_complement_->points.reserve(num_pts_for_reserve);
    static_map_accum_->points.reserve(num_pts_for_reserve);
    static_map_voxelized_->points.reserve(num_pts_for_reserve);
}

void ERASOR2::setScanAndPose(const Eigen::Matrix4f &pose_raw,
                             const pcl::PointCloud<pcl::PointXYZI> &cloud_est_label) {
    throw invalid_argument("Not implemented");
}

void ERASOR2::setScanAndPose(const Eigen::Matrix4f &pose_raw,
                             const pcl::PointCloud<pcl::PointXYZI> &cloud_gt_label,
                             const pcl::PointCloud<pcl::PointXYZI> &cloud_est_label) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_w_voi_label(new pcl::PointCloud<pcl::PointXYZI>);
    if (is_initial_) {
        new_origin_ = pose_raw;
        is_initial_ = false;
    }
    poses_submap_.emplace_back(new_origin_.inverse() * pose_raw);

    // NOTE: `tf_h_of_ground_to_be_zero_` is not necessary, but we follow the legacy of ERASOR 1.0
    if (dataset_name_ == "SemanticKITTI") {
        pcl::transformPointCloud(cloud_gt_label, *transformed, poses_submap_.back() * tf_h_of_ground_to_be_zero_);
        pcs_gt_transformed_.emplace_back(*transformed);
    }

    // NOTE: First, VoI is set in an egocentric viewpoint
    // This affects the quality of xygrid
    maskNonVoI(cloud_est_label, *cloud_est_w_voi_label, min_z_voi_, max_z_voi_);
    pcl::transformPointCloud(*cloud_est_w_voi_label, *transformed, poses_submap_.back() * tf_h_of_ground_to_be_zero_);
    pcs_transformed_.emplace_back(*transformed);

    if (viz_set_scan_and_pose_) {
        std::cout << "publish?" << std::endl;
        ros::Rate                               sleep_rate(30);
        vector<pcl::PointCloud<pcl::PointXYZI>> xygrid;
        pcl::PointCloud<pcl::PointXYZI>         complement;
        voi2xygrid(*cloud_est_w_voi_label, 0.0, 0.0, 0.0,
                   range_of_interest_, grid_resolution_,
                   xygrid, complement, "gridmap");
        std::cout << "publish0" << std::endl;
        grid_map::GridMap gridmap = setEgocentricGridMap(range_of_interest_,
                                                         grid_resolution_, xygrid);

        // Viz
        CurrCloudPublisher.publish(erasor_utils::cloud2msg(cloud_est_label));
        grid_map_msgs::GridMap grid_msg;
        grid_map::GridMapRosConverter::toMessage(gridmap, grid_msg);
        EgocentricGridPublisher.publish(grid_msg);
        ros::spinOnce();
        sleep_rate.sleep();
        if (stop_for_each_frame_) {
            std::cout << "[Set scan and pose] Waiting for pressing a key" << std::endl;
            cin.ignore();
        }
        std::cout << "publish2" << std::endl;
    }
}

void ERASOR2::setSubmap() {
    int                                  num_data = pcs_transformed_.size();
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_partial_src(new pcl::PointCloud<pcl::PointXYZI>);
    for (int                             k        = 0; k < num_data; ++k) {
        *map_partial_src += pcs_transformed_[k];
    }

    // Estimated labels are preserved!
    erasor_utils::voxelize_preserving_labels_by_nanoflann(map_partial_src, *map_accum_, map_voxel_size_);

    float min_x, min_y, max_x, max_y;
    erasor_utils::calcMinMaxXY(pcs_transformed_, min_x, min_y, max_x, max_y);

    num_data_ = num_data;

    grid_map_info_  = setGridMapParams(min_x, min_y, max_x, max_y, grid_resolution_);
    gridmap_submap_ = setMapcentricGridMap(grid_map_info_);

    resize();
}

void ERASOR2::resize() {
    if (num_data_ == 0) {
        throw invalid_argument("The parameter `num_data_` is not set yet!");
    }
    xygrids_.resize(num_data_);
    idxes_approx_.resize(num_data_);
    dyn_ids_set_.resize(num_data_);
    static_points_transformed_.resize(num_data_);
}

void ERASOR2::updateSteppableRegion() {
    for (int k = 0; k < num_data_; ++k) {
        std::cout << "[Update] " << k << " / " << num_data_ << std::endl;
        grid_map::Position pos_xy(poses_submap_[k](0, 3), poses_submap_[k](1, 3));
        gridmap_submap_.getIndex(pos_xy, idxes_approx_[k]);

        int w_pc = idxes_approx_[k](0);
        int h_pc = idxes_approx_[k](1);

        grid_map::Position pos_approx;
        pos_approx(0) = grid_map_info_.x_length / 2 + grid_map_info_.center_x - w_pc * grid_map_info_.resolution;
        pos_approx(1) = grid_map_info_.y_length / 2 + grid_map_info_.center_y - h_pc * grid_map_info_.resolution;
//        std::cout << "[Before] " << poses_submap[k](0, 3) << ", " << poses_submap[k](1, 3) << std::endl;
//        std::cout << "[After] " << pos_approx(0) << ", " << pos_approx(1) << std::endl;

        pcl::PointCloud<pcl::PointXYZI> complement;
//        std::cout << poses_submap[k] << std::endl;
//        std::cout << pos_x_approx << " , " << pos_y_approx << std::endl;
        voi2xygrid(pcs_transformed_[k], pos_approx(0), pos_approx(1), poses_submap_[k](2, 3),
                   range_of_interest_, grid_resolution_, xygrids_[k], complement);
        vector<pcl::PointCloud<pcl::PointXYZI>> map_grid;
        pcl::PointCloud<pcl::PointXYZI>::Ptr    dummy(new pcl::PointCloud<pcl::PointXYZI>);
        voi2xygrid(*map_accum_, pos_approx(0), pos_approx(1), poses_submap_[k](2, 3),
                   range_of_interest_, grid_resolution_, map_grid, *dummy);

        // Assume that range of interest is square
        grid_map::Index idx;
        int             count = 0;
        for (int        h     = h_pc - neighboring_height_ / 2; h < h_pc + neighboring_height_ / 2; ++h) {
            for (int w = w_pc - neighboring_width_ / 2; w < w_pc + neighboring_width_ / 2; ++w) {
                // x direction first
                if (xygrids_[k][count].points.size() > 1) {
                    idx(0) = w;
                    idx(1) = h;
//                    cout << "(" << w << ", " << h << ") - ";
                    if (isLikelyToBeSteppableRegion(xygrids_[k][count], map_grid[count],
                                                    scan_ratio_threshold_,
                                                    th_bin_max_h_, verbose_)) {
//                        cout << "Success! " << endl;
                        if (gridmap_submap_.at("steppable", idx) == UNKNOWN) {
                            gridmap_submap_.at("steppable", idx) = initial_ground_likelihood_;
                        } else {
                            gridmap_submap_.at("steppable", idx) += increment_ground_likelihood_;
                        }
                    } else {
//                        cout << "Fail! " << endl;
                    }
                }
                ++count;
            }
        }
//         Visualization
        if (viz_update_) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr curr_voi(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi(new pcl::PointCloud<pcl::PointXYZI>);
            xygrid2cloud(xygrids_[k], *curr_voi);
            xygrid2cloud(map_grid, *map_voi);

            CurrCloudPublisher.publish(erasor_utils::cloud2msg(pcs_transformed_[k]));
            CurrVoIPublisher.publish(erasor_utils::cloud2msg(*curr_voi));
            MapVoIPublisher.publish(erasor_utils::cloud2msg(*map_voi));

            grid_map_msgs::GridMap grid_msg;
            grid_map::GridMapRosConverter::toMessage(gridmap_submap_, grid_msg);
            GridPublisher.publish(grid_msg);
            ros::spinOnce();
            if (stop_for_each_frame_) {
                std::cout << "[Update] Waiting for pressing a key" << std::endl;
                cin.ignore();
            }
        }
    }
}

// Re-project ground likelihood to each scan
void ERASOR2::detectDynamicObjects() {
    for (int k = 0; k < num_data_; ++k) {
        vector<float> dyn_cand_ids;

        int w_pc = idxes_approx_[k](0);
        int h_pc = idxes_approx_[k](1);

        grid_map::Index idx;
        int             count = 0;
        // Detect dynamic objects' ids per scan
        for (int        h     = h_pc - neighboring_height_ / 2; h < h_pc + neighboring_height_ / 2; ++h) {
            for (int w = w_pc - neighboring_width_ / 2; w < w_pc + neighboring_width_ / 2; ++w) {
                idx(0) = w;
                idx(1) = h;
                if (gridmap_submap_.at("steppable", idx) > ground_likelihood_thr_) {
                    // Extract indices
                    for (const auto pt: xygrids_[k][count].points) {
                        if ((pt.intensity != GROUND_LABEL) && (pt.intensity != NOT_INTEREST) &&
                            std::find(dyn_cand_ids.begin(), dyn_cand_ids.end(), pt.intensity) == dyn_cand_ids.end()) {
                            dyn_cand_ids.push_back(pt.intensity);
                        }
                    }
                }
                ++count;
            }
        }

        // Filtering
        auto &ids = dyn_ids_set_[k];
        ids.clear();
        for (const int dyn_cand_id: dyn_cand_ids) {
            // For visualization
            pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_points(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto                      &pt: pcs_transformed_[k]) {
                if (pt.intensity == dyn_cand_id) {
                    dynamic_points->points.emplace_back(pt);
                }
            }
            float                                total_score = 0;
            for (const auto                      &dyn_pt: dynamic_points->points) {
                grid_map::Position p_tmp(dyn_pt.x, dyn_pt.y);
                grid_map::Index    idx_tmp;
                gridmap_submap_.getIndex(p_tmp, idx_tmp);
                total_score += gridmap_submap_.at("steppable", idx_tmp);
            }
            if (total_score / dynamic_points->points.size() > ground_likelihood_thr_) {
                ids.push_back(dyn_cand_id);
            }
        }
    }

    for (int k = 0; k < num_data_; ++k) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_points_each_scan(new pcl::PointCloud<pcl::PointXYZI>);
        auto                                 &each_pc = pcs_transformed_[k];
        auto                                 &ids     = dyn_ids_set_[k];

        static_points_transformed_[k].clear();
        vector<bool> static_mask;
        estimateStaticMask(each_pc, ids, static_mask);
        if (dataset_name_ == "SemanticKITTI") {
            // To preserve the SemanticKITTI labels
            discernStaticAndDynamicPoints(pcs_gt_transformed_[k], static_mask, static_points_transformed_[k],
                                          *dynamic_points_each_scan);
        } else {
            discernStaticAndDynamicPoints(each_pc, static_mask, static_points_transformed_[k],
                                          *dynamic_points_each_scan);
        }
        (*map_dynamic_) += (*dynamic_points_each_scan);

        if (viz_detect_) {
            cout << "Total " << ids.size() << "dyn. objects are detected!" << endl;
            CurrCloudPublisher.publish(erasor_utils::cloud2msg(each_pc));
            DynCurrCloudPublisher.publish(erasor_utils::cloud2msg(*dynamic_points_each_scan));
            if (stop_for_each_frame_) {
                std::cout << "[Reproject] Waiting for pressing a key" << std::endl;
                cin.ignore();
            }
        }
    }
}

void ERASOR2::estimateStaticMask(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                 const std::vector<float> &dyn_ids, std::vector<bool> &static_mask) {
    static_mask.resize(cloud.points.size());
    int             count = 0;
    for (const auto &pt: cloud) {
        if (std::find(dyn_ids.begin(), dyn_ids.end(), pt.intensity) != dyn_ids.end()) {
            if (pt.intensity == NOT_VOLUME_OF_INTEREST) {
                // ToDo Improve
                static_mask[count] = true;
            } else {
                static_mask[count] = false;
            }
        } else {
            static_mask[count] = true;
        }
        ++count;
    }
}

void ERASOR2::discernStaticAndDynamicPoints(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                            const std::vector<bool> &static_mask,
                                            pcl::PointCloud<pcl::PointXYZI> &static_points,
                                            pcl::PointCloud<pcl::PointXYZI> &dynamic_points) {
    if (cloud.size() != static_mask.size()) {
        throw invalid_argument("Something's wrong!");
    }

    static_points.clear();
    dynamic_points.clear();
    int             count = 0;
    for (const auto &pt: cloud) {
        if (static_mask[count]) {
            static_points.points.emplace_back(pt);
        } else {
            dynamic_points.points.emplace_back(pt);
        }
        ++count;
    }
}

void
ERASOR2::discernStaticAndDynamicPoints(const pcl::PointCloud<pcl::PointXYZI> &cloud, const std::vector<float> &dyn_ids,
                                       pcl::PointCloud<pcl::PointXYZI> &static_points,
                                       pcl::PointCloud<pcl::PointXYZI> &dynamic_points) {
    for (const auto &pt: cloud) {
        if (std::find(dyn_ids.begin(), dyn_ids.end(), pt.intensity) != dyn_ids.end()) {
            if (pt.intensity == NOT_VOLUME_OF_INTEREST) {
                // ToDo Improve
                static_points.points.emplace_back(pt);
            } else {
                dynamic_points.points.emplace_back(pt);
            }
        } else {
            static_points.points.emplace_back(pt);
        }
    }
}

void ERASOR2::publishStaticMapResults() {
    std::cout << "[ERASOR2] Publish results!" << std::endl;
    std::cout << "# of dynamic points: " << map_dynamic_->points.size() << std::endl;
    std::cout << "# of static points: " << static_map_voxelized_->points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_for_viz(new pcl::PointCloud<pcl::PointXYZI>);
    // Because ERASOR2 uses the local origin,
    pcl::transformPointCloud(*static_map_voxelized_, *static_map_for_viz, new_origin_.inverse());
    DynMapPublisher.publish(erasor_utils::cloud2msg(*map_dynamic_));
    MapCloudPublisher.publish(erasor_utils::cloud2msg(*static_map_for_viz));
    ros::Rate final_loop_rate(1);
    while (ros::ok()) {
        grid_map_msgs::GridMap grid_msg;
        grid_map::GridMapRosConverter::toMessage(gridmap_submap_, grid_msg);
        GridPublisher.publish(grid_msg);
        ros::spinOnce();
        final_loop_rate.sleep();
    }
}

void ERASOR2::saveStaticMap(const string &static_map_path) {
    std::cout << "[ERASOR2] On saving results..." << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world_frame(new pcl::PointCloud<pcl::PointXYZI>);
    for (int                             i = 0; i < num_data_; ++i) {
        const auto &cloud = static_points_transformed_[i];
        pcl::transformPointCloud(cloud, *cloud_world_frame, new_origin_);
        (*static_map_accum_) += (*cloud_world_frame);
    }
    erasor_utils::voxelize_preserving_labels_by_nanoflann(static_map_accum_, *static_map_voxelized_, map_voxel_size_);

    static_map_voxelized_->width  = static_map_voxelized_->points.size();
    static_map_voxelized_->height = 1;
    std::cout << "[Debug]: (" << static_map_voxelized_->width << ", " << static_map_voxelized_->height << ") => "
              << static_map_voxelized_->points.size()
              << std::endl;
    std::cout << "\033[1;32mSaving the map to pcd...\033[0m" << std::endl;
    pcl::io::savePCDFileASCII(static_map_path, *static_map_voxelized_);
    std::cout << "\033[1;32mComplete to save the map!:";
    std::cout << static_map_path << "\033[0m" << std::endl;

}

void ERASOR2::maskNonVoI(const pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &cloud_out,
                         const float min_z_voi, const float max_z_voi) {
    cloud_out.clear();
    cloud_out.reserve(src.points.size());
    for (auto pt: src.points) {
        if (pt.z < min_z_voi || pt.z > max_z_voi) {
            pt.intensity = NOT_VOLUME_OF_INTEREST;
        }
        cloud_out.points.emplace_back(pt);
    }
}

nav_msgs::OccupancyGrid ERASOR2::setOccupancyGridMap(const float min_x, const float min_y,
                                                     const float max_x, const float max_y,
                                                     const float occugrid_resolution) {

    const int               width  = static_cast<int>((max_x - min_x) / occugrid_resolution) + 1;
    const int               height = static_cast<int>((max_y - min_y) / occugrid_resolution) + 1;
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
        gridmap.data.push_back(UNKNOWN);
    }
    return gridmap;
}

GridMapInfo ERASOR2::setGridMapParams(const float min_x, const float min_y,
                                      const float max_x, const float max_y,
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

grid_map::GridMap ERASOR2::setMapcentricGridMap(const GridMapInfo &grid_map_info) {
    cout << grid_map_info.x_length << endl;
    cout << grid_map_info.y_length << endl;
    cout << grid_map_info.resolution << endl;
    grid_map::GridMap gridmap({"elevation", "steppable"});
    gridmap.setFrameId("map");
    gridmap.setGeometry(grid_map::Length(grid_map_info.x_length, grid_map_info.y_length),
                        grid_map_info.resolution);
    gridmap["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
    gridmap["steppable"].setConstant(UNKNOWN);
    gridmap.setPosition(grid_map::Position(grid_map_info.center_x, grid_map_info.center_y));
    return gridmap;
}

void ERASOR2::voi2xygrid(
        const pcl::PointCloud<pcl::PointXYZI> &src, float pos_x, float pos_y, float pos_z,
        float range, float resolution, vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid,
        pcl::PointCloud<pcl::PointXYZI> &complement, std::string format) {
    const int width  = static_cast<int>(2.0000001 * range / resolution);
    const int height = static_cast<int>(2.0000001 * range / resolution);
    xygrid.resize(width * height);

    for (auto &grid: xygrid) {
        grid.points.clear();
    }

    for (auto const &pt: src.points) {
        if ((pt.intensity != NOT_VOLUME_OF_INTEREST) &&
            (pt.x < pos_x + range) && (pt.x > pos_x - range) &&
            (pt.y < pos_y + range) && (pt.y > pos_y - range)) {
            // +: To make indices positive
            int w, h;
            if (format == "occugrid") {
                // Left-bottom is the origin
                w = static_cast<int>((pt.x - (pos_x - range)) / resolution);
                h = static_cast<int>((pt.y - (pos_y - range)) / resolution);
            } else if (format == "gridmap") { // Grid map from ETH Zurich
                // Right-upper side is the origin
                w = static_cast<int>((pos_x + range - pt.x) / resolution);
                h = static_cast<int>((pos_y + range - pt.y) / resolution);
            } else { throw invalid_argument("Not implemented"); }

            if (w + width * h > xygrid.size()) {
                std::cout << range << " ,,, " << resolution << std::endl;
                std::cout << "pt: (" << pt.x << ", " << pt.y << ") | ";
                std::cout << "position: (" << pos_x << " ,, " << pos_y << ")" << std::endl;
                std::cout << range << " => " << pos_x + range << " => " << pos_x + range - pt.x << std::endl;
                std::cout << ((pos_x + range - pt.x) / resolution) << std::endl;
                std::cout << range << " => " << pos_y + range << " => " << pos_y + range - pt.y << std::endl;
                std::cout << ((pos_y + range - pt.y) / resolution) << std::endl;
                std::cout << "Neighboring " << neighboring_width_ << ", " << neighboring_height_ << std::endl;
                string error_msg = (boost::format("Pixel overflow occurs! w: %d, h: %d, width: %d, height: %d") % w %
                                    h % width %
                                    height).str();
                std::cout << "\033[1;33m" << error_msg << "\033[0m" << std::endl;
                complement.points.push_back(pt);
                continue;
            }
            xygrid[w + width * h].points.emplace_back(pt);
        } else { complement.points.push_back(pt); }
    }
}

void
ERASOR2::xygrid2cloud(const vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid, pcl::PointCloud<pcl::PointXYZI> &cloud) {
    cloud.clear();
    for (const auto &partial_cloud: xygrid) {
        cloud += partial_cloud;
    }
}

bool ERASOR2::isLikelyToBeGround(const pcl::PointCloud<pcl::PointXYZI> pc, const float ratio_num,
                                 const int num_min_pts) {
    int             num_ground_pts = 0;
    for (const auto &pt: pc) {
        if (pt.intensity == GROUND_LABEL) {
            ++num_ground_pts;
        }
    }
    if ((num_ground_pts > ratio_num * pc.points.size()) && pc.points.size() > num_min_pts) {
        return true; // Free
    } else { return false; }
}

bool ERASOR2::isLikelyToBeSteppableRegion(const pcl::PointCloud<pcl::PointXYZI> &curr_pc,
                                          const pcl::PointCloud<pcl::PointXYZI> &map_pc,
                                          const float scan_ratio_threshold, const float th_bin_max_h,
                                          const bool verbose) {
    if (curr_pc.empty() || map_pc.empty()) { return false; }

    float curr_min_z, curr_max_z;
    float map_min_z, map_max_z;

//    cout << "FF?" << curr_pc.size() << endl;
    erasor_utils::calcMinMaxZWithoutGround(curr_pc, curr_min_z, curr_max_z);
//    cout << "FF0" << map_pc.size() << endl;
    erasor_utils::calcMinMaxZWithoutGround(map_pc, map_min_z, map_max_z);
//    cout << "FF!" << endl;

    float lowest_z    = min(curr_min_z, map_min_z);
    float map_h_diff  = map_max_z - lowest_z;
    float curr_h_diff = curr_max_z - lowest_z;
    float scan_ratio  = min(map_h_diff / curr_h_diff,
                            curr_h_diff / map_h_diff);

    // Dynamic!
    if (scan_ratio < scan_ratio_threshold && isLikelyToBeGround(curr_pc, 0.95)) { // find dynamic!
        if (map_h_diff > th_bin_max_h) { // To reduce false positives
            return true;
        } else { return false; }
    } else {
        return false;
    }
}

grid_map::GridMap ERASOR2::setEgocentricGridMap(float range,
                                                const float grid_resolution,
                                                const vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid) {

    grid_map::GridMap gridmap({"elevation", "steppable"});
    gridmap.setFrameId("map");
    gridmap.setGeometry(grid_map::Length(2 * range, 2 * range), grid_resolution);
    gridmap["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
    gridmap["steppable"].setConstant(UNKNOWN);

    const int width  = static_cast<int>(2.00000001 * range / grid_resolution);
    const int height = static_cast<int>(2.00000001 * range / grid_resolution);

    grid_map::Index idx;
    for (int        u = 0; u < width; ++u) {
        for (int v = 0; v < height; ++v) {
            int i = u + width * v;
            if (!xygrid[i].points.empty() && isLikelyToBeGround(xygrid[i])) {
                idx(0)                       = u;
                idx(1)                       = v;
                gridmap.at("steppable", idx) = initial_ground_likelihood_;
            }
        }
    }
    return gridmap;
}

//void ERASOR2::dilateAndErode(grid_map::GridMap& gridmap_submap) {
//    // Noise filtering?
//    cv::Mat img, img_eroded, img_dilated;
//    const float min_coefficient = gridmap_submap.get("steppable").minCoeff();
//    const float max_coefficient = gridmap_submap.get("steppable").maxCoeff();
//    std::cout << min_coefficient << ", " << max_coefficient << std::endl;
//    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(gridmap_submap, "steppable", CV_8UC1,
//                                                            min_coefficient, max_coefficient, img);
//    std::string save_dir = "/home/shapelim/Pictures/erasor2";
//    cv::imwrite(save_dir + "/original.png", img);
//    dilate(img, img_dilated, cv::Mat::ones(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 2);
//    cv::imwrite(save_dir + "/dilation.png", img_dilated);
//    erode(img_dilated, img_eroded, cv::Mat::ones(cv::Size(3,3),CV_8UC1), cv::Point(-1,-1),2);
//    cv::imwrite(save_dir + "/erosion.png", img_eroded);
//    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(img_eroded, "steppable",
//                                                                      gridmap_submap, min_coefficient, max_coefficient);
//    const float min_coefficient_after = gridmap_submap.get("steppable").minCoeff();
//    const float max_coefficient_after = gridmap_submap.get("steppable").maxCoeff();
//    std::cout << min_coefficient_after << ", " << max_coefficient_after << std::endl;
//}

