#include "erasor2/erasor2.h"

using namespace std;

ERASOR2::ERASOR2() {
    cout << "[ERASOR2] Increment gain: " << increment_gain_ << endl;
    cout << "[ERASOR2] Increment: " << increment_ << endl;

    initializePointClouds();
    // `area_per_grid_` is used to check over-segmentation
    area_per_grid_ = grid_resolution_ * grid_resolution_;
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

double ERASOR2::prob2logOdds(double prob) {
    // o -> -inf, 1 -> inf
    return 2 * atanh(2 * prob - 1);
}

double ERASOR2::logOdds2prob(double log_odds) {
    return (tanh(log_odds / 2)  + 1) / 2;
}

grid_map::Position ERASOR2::idx2position(const grid_map::Index& idx) {
    int w_pc = idx(0);
    int h_pc = idx(1);

    grid_map::Position pos;
    pos(0) = grid_map_info_.x_length / 2 + grid_map_info_.center_x - w_pc * grid_map_info_.resolution;
    pos(1) = grid_map_info_.y_length / 2 + grid_map_info_.center_y - h_pc * grid_map_info_.resolution;
    return pos;
}

bool ERASOR2::isEqual(const grid_map::Index& idx0, const grid_map::Index& idx1) {
    if (idx0(0) == idx1(0) && idx0(1) == idx1(1)) {
        return true;
    } else {
        return false;
    }
}

bool ERASOR2::isInsideTheDynamicInstances(const pcl::PointXYZI& query, const pcl::PointXYZI& target) {
    float dist = sqrt((query.x - target.x) * (query.x - target.x) + (query.y - target.y) * (query.y - target.y)
            + (query.z - target.z) * (query.z - target.z));
    float xy_dist = sqrt((query.x - target.x) * (query.x - target.x) + (query.y - target.y) * (query.y - target.y));
    if  (dist < dist_thr_gain_ * voxel_size_ || xy_dist < dist_thr_gain_ * voxel_size_) {
        return true;
    } else {
        return false;
    }
}

int ERASOR2::globalIdx2LocalIdx(const grid_map::Index& global_idx, const grid_map::Index& center_idx) {
    int w_diff = global_idx(0) - (center_idx(0) - neighboring_width_ / 2);
    int h_diff = global_idx(1) - (center_idx(1) - neighboring_height_ / 2);
    return w_diff + h_diff * neighboring_width_;
}

void ERASOR2::initializePointClouds() {
    int num_pts_for_reserve = 2000000;
    map_noise_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    map_dynamic_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    map_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    map_complement_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    static_map_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    static_map_voxelized_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    map_noise_->points.reserve(num_pts_for_reserve);
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_w_voi_label(new pcl::PointCloud<pcl::PointXYZI>);
    transformed->reserve(cloud_est_label.size());
    cloud_est_w_voi_label->reserve(cloud_est_label.size());

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

    // Parse VoI and Non-VoI. Because non-VoIs are not targets of static map building

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_voi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_non_voi(new pcl::PointCloud<pcl::PointXYZI>);
    pcs_transformed_.emplace_back(*transformed);

    float max_id = getMaxInstanceId(*transformed);
    max_ids_.push_back(max_id);

    if (viz_set_scan_and_pose_) {
        ros::Rate                               sleep_rate(30);
        vector<pcl::PointCloud<pcl::PointXYZI>> xygrid;
        pcl::PointCloud<pcl::PointXYZI>         complement;
        voi2xygrid(*cloud_est_w_voi_label, 0.0, 0.0, 0.0,
                   range_of_interest_, grid_resolution_,
                   xygrid, complement, "gridmap");
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
    }
}

void ERASOR2::setSubmap() {
    cout << "[ERASOR2] Setting submap..." << endl;
    int                                  num_data = pcs_transformed_.size();
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_partial_src(new pcl::PointCloud<pcl::PointXYZI>);
    for (int                             k        = 0; k < num_data; ++k) {
        *map_partial_src += pcs_transformed_[k];
    }

    cout << "[ERASOR2] Voxelizing submap..." << endl;
    // Estimated labels are preserved!
    erasor_utils::voxelize_preserving_labels_by_nanoflann(map_partial_src, *map_accum_, map_voxel_size_);

    cout << "[ERASOR2] Calculating  min-max x, y values given " << pcs_transformed_.size() << " scan-pose pairs..." << endl;
    float min_x, min_y, max_x, max_y;
    erasor_utils::calcMinMaxXY(pcs_transformed_, min_x, min_y, max_x, max_y);
    cout << "[ERASOR2] Min-max x: " << min_x << " <-> " << max_x << endl;
    cout << "[ERASOR2] Min-max y: " << min_y << " <-> " << max_y << endl;

    num_data_ = num_data;

    cout << "[ERASOR2] Setting map-centric gridmap..." << endl;
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
    rejected_objs_set_.resize(num_data_);
    accepted_objs_set_.resize(num_data_);
    noisy_points_transformed_.resize(num_data_);
    static_points_transformed_.resize(num_data_);
    dynamic_points_transformed_.resize(num_data_);
    potential_dynamic_points_transformed_.resize(num_data_);

    ids_instances_set_.resize(num_data_);
}

void ERASOR2::updateSteppableRegion() {
    for (int k                   = 0; k < num_data_; ++k) {
//        std::cout << "\r[Update] " << k << " / " << num_data_ << std::endl;
        gridmap_submap_["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
        grid_map::Position pos_xy(poses_submap_[k](0, 3), poses_submap_[k](1, 3));
        gridmap_submap_.getIndex(pos_xy, idxes_approx_[k]);

        int w_pc = idxes_approx_[k](0);
        int h_pc = idxes_approx_[k](1);
        grid_map::Position pos_approx = idx2position(idxes_approx_[k]);

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
                idx(0) = w;
                idx(1) = h;
//                    cout << "(" << w << ", " << h << ") - ";
                if (isLikelyToBeSteppableRegion(xygrids_[k][count], map_grid[count],
                                                scan_ratio_threshold_,
                                                min_z_diff_thr_, verbose_)) {
                    // For debugging
                    gridmap_submap_.at("elevation", idx) = 1.0;
//                    cout << "(" << w << ", " << h << "): " << scan_ratio_ << " | " << ratio_num_ << " // ";
//                    cout << xygrids_[k][count].size() << " <-> "
//                         << erasor_utils::getNumGroundPoints(xygrids_[k][count]) << endl;
                    gridmap_submap_.at("status", idx) = TEMPORARILY_OCCUPIED;
                    updateLogOdds(idx, increment_ * increment_gain_);
                } else if (isLikelyToBeGround(xygrids_[k][count])) {
                    if (gridmap_submap_.at("status", idx) == NOT_OBSERVED) {
                        gridmap_submap_.at("status", idx) = GROUND_EXISTS;
                    }
                    updateLogOdds(idx, increment_);
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

            logOddsGrid2probGrid();
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
    logOddsGrid2probGrid();
}

// Re-project ground likelihood to each scan
void ERASOR2::detectMovingObjects() {
    grid_map_msgs::GridMap grid_msg;
    grid_map::GridMapRosConverter::toMessage(gridmap_submap_, grid_msg);
    GridPublisher.publish(grid_msg);
    for (int k = 0; k < num_data_; ++k) {
        vector<float> dyn_cand_ids; // temp. variable
//        unordered_map<float, DynamicInstance> &ids_clusters  = ids_instances_set_[k];
        noisy_points_transformed_[k].reserve(100);

        int w_pc = idxes_approx_[k](0);
        int h_pc = idxes_approx_[k](1);

        grid_map::Index idx;
        int             count = 0;
        // Detect dynamic objects' ids per scan
        for (int        h     = h_pc - neighboring_height_ / 2; h < h_pc + neighboring_height_ / 2; ++h) {
            for (int w = w_pc - neighboring_width_ / 2; w < w_pc + neighboring_width_ / 2; ++w) {
                idx(0) = w;
                idx(1) = h;
                if (gridmap_submap_.at("log_odds", idx) > ground_log_odds_thr_) {
                    // Extract indices
                    // Note that `xygrids_[k][count]` does not contain non-VoI points
                    for (const auto &pt: xygrids_[k][count].points) {
                        if ((pt.intensity != GROUND_LABEL) && (pt.intensity != NOT_INTEREST) &&
                            std::find(dyn_cand_ids.begin(), dyn_cand_ids.end(), pt.intensity) == dyn_cand_ids.end()) {
                            dyn_cand_ids.push_back(pt.intensity);
//                        if ((pt.intensity != GROUND_LABEL) && (pt.intensity != NOT_INTEREST)) {
//                            if (ids_clusters.find(pt.intensity) != ids_clusters.end()) {
//                               ids_clusters[pt.intensity].cloud_.emplace_back(pt);
//                            } else {
//                                DynamicInstance dynamic_cluster;
//                                dynamic_cluster.cloud_.reserve(200);
//                                ids_clusters[pt.intensity] = dynamic_cluster;
//                                ids_clusters[pt.intensity].cloud_.emplace_back()
//
//                            }
//                            &&
//                            std::find(dyn_cand_ids.begin(), dyn_cand_ids.end(), pt.intensity) == dyn_cand_ids.end()) {

                        } else if (pt.intensity == NOT_INTEREST) { // && gridmap_submap_.at("status", idx) == TEMPORARILY_OCCUPIED) {
                            noisy_points_transformed_[k].points.emplace_back(pt);
                        }
                    }
                }
                ++count;
            }
        }

        // 2. Set Dynamic instance
        auto &ids_clusters  = ids_instances_set_[k];
        ids_clusters.clear();
        DynamicInstance dyn_cluster;
        dyn_cluster.cloud_.reserve(200); // heuristic
        for (const int dyn_cand_id: dyn_cand_ids) {
            ids_clusters[dyn_cand_id] = dyn_cluster;
        }

        for (const auto &pt: pcs_transformed_[k]) {
            if (ids_clusters.find(pt.intensity) != ids_clusters.end()) {
                ids_clusters[pt.intensity].cloud_.emplace_back(pt);
            }
        }
    }
}

void ERASOR2::filterDynamicObjects() {
    for (int k = 0; k < num_data_; ++ k) {
        auto &ids_clusters  = ids_instances_set_[k];
        auto &rejected_objs = rejected_objs_set_[k];
        auto &accepted_objs = accepted_objs_set_[k];
        float max_id_for_register = max_ids_[k] + 10.0; // 10.0 is just a margin.

        // For managing over-segmentation
        vector<OverSegmentedInstance> instances_to_be_updated;

        // Only available on C++ 17
        for (auto &[dyn_cand_id, dynamic_instance]: ids_clusters) {
            setDynamicInstance(dynamic_instance, poses_submap_[k](0, 3), poses_submap_[k](1, 3));
            float adaptive_thr = dynamic_instance.is_close_to_body_frame_? obj_score_hard_thr_: obj_score_soft_thr_;
            if (dynamic_instance.moving_obj_score_ > adaptive_thr) {
                dynamic_instance.is_dynamic_ = true;
                // For visualization
                accepted_objs.push_back({dynamic_instance.centroid_, dynamic_instance.moving_obj_score_});
            } else { // it means that most parts are not in the region of interests
                if (isOverSegmented(dynamic_instance)) {
                    // To debug over-clustering
                    cout << "\033[1;35mOver-segmentation detected: " << dynamic_instance.cloud_.size() << " => ";
                    DynamicInstance static_inst, partial_dynamic_inst;
                    parseOverSegmentation(dynamic_instance, static_inst, partial_dynamic_inst,
                                          poses_submap_[k](0, 3), poses_submap_[k](1, 3));
                    cout << static_inst.cloud_.size() << "<-> " << partial_dynamic_inst.cloud_.size() << "\033[0m" << endl;

                    if (viz_over_seg_) {
                        CurrCloudPublisher.publish(erasor_utils::cloud2msg(pcs_transformed_[k]));
                        DynCurrCloudPublisher.publish(erasor_utils::cloud2msg(dynamic_instance.cloud_));
                        NoiseCurrCloudPublisher.publish(erasor_utils::cloud2msg(partial_dynamic_inst.cloud_));
                        cin.ignore();
                    }

                    OverSegmentedInstance inst_to_be_updated;
                    inst_to_be_updated.original_id = dyn_cand_id;
                    inst_to_be_updated.new_id_for_stat_inst = ++max_id_for_register;
                    inst_to_be_updated.new_id_for_dyn_inst = ++max_id_for_register;
                    inst_to_be_updated.static_inst = static_inst;
                    inst_to_be_updated.dynamic_inst = partial_dynamic_inst;
                    instances_to_be_updated.emplace_back(inst_to_be_updated);

                    accepted_objs.push_back({partial_dynamic_inst.centroid_, partial_dynamic_inst.moving_obj_score_});
                    rejected_objs.push_back({static_inst.centroid_, static_inst.moving_obj_score_});
                } else {
                    dynamic_instance.is_dynamic_ = false;
                    rejected_objs.push_back({dynamic_instance.centroid_, dynamic_instance.moving_obj_score_});
                }
            }

//            if (k > 10 || k < 20) {  // seq. 00
//            if (k == 46 || k == 51 || k > 90) {  // seq. 07
//            if (k == 15 || k == 32 ||k == 85 || k == 86 || k == 87) { // seq 0.5
//            if (k == 15 || k == 32 ||k == 85 || k == 86 || k == 87) { // seq 0.5
//                CurrCloudPublisher.publish(erasor_utils::cloud2msg(pcs_transformed_[k]));
//                DynCurrCloudPublisher.publish(erasor_utils::cloud2msg(dynamic_instance.cloud_));
//
//                const float obj_x = dynamic_instance.centroid_(0);
//                const float obj_y = dynamic_instance.centroid_(1);
//                const float pos_x = poses_submap_[k](0, 3);
//                const float pos_y = poses_submap_[k](1, 3);
//                float dist = sqrt((obj_x - pos_x) * (obj_x - pos_x) + (obj_y - pos_y) * (obj_y - pos_y));
//                cout << "Dist: " << dist << endl;
//                cout << "Score: " << dynamic_instance.moving_obj_score_ << endl;
//                cout << "Is close? " << dynamic_instance.is_close_to_body_frame_ << endl;
//                cout << "Is dyn? " << dynamic_instance.is_dynamic_ << endl;
//                printClusterInfo(dynamic_instance);
//
//                cin.ignore();
//            }
        }

        updateNewParsedInstances(instances_to_be_updated, pcs_transformed_[k], ids_clusters);

        if (rejected_objs.size() + accepted_objs.size() != ids_clusters.size()) {
            throw invalid_argument("Some objects are lost!");
        }
    }

    // Naively assign the static points
    for (int k = 0; k < num_data_; ++k) {

        const auto &each_pc      = pcs_transformed_[k];
        const auto &ids_clusters = ids_instances_set_[k];

        static_points_transformed_[k].clear();
        vector<int> static_mask;
        estimateStaticMask(each_pc, ids_clusters, static_mask);
        // Noisy points are added by the following function:
        updateNoisyMask(each_pc, noisy_points_transformed_[k], static_mask);
        if (dataset_name_ == "SemanticKITTI") {
            // To preserve the original SemanticKITTI labels
            discernStaticAndDynamicPoints(pcs_gt_transformed_[k], static_mask, static_points_transformed_[k],
                                          dynamic_points_transformed_[k]);
        } else {
            discernStaticAndDynamicPoints(each_pc, static_mask, static_points_transformed_[k],
                                          dynamic_points_transformed_[k]);
        }
    }

    for (int k = 0; k < num_data_; ++k) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_static_points(new pcl::PointCloud<pcl::PointXYZI>);
        windowBasedVolumetricOutlierRemoval(k, window_size_, dist_thr_gain_, *filtered_static_points,
                                            potential_dynamic_points_transformed_[k]);
        // It shows rather poor performance
//        instanceAwareOutlierRemoval(k, window_size_, dist_thr_gain_, *filtered_static_points,
//                                            potential_dynamic_points_transformed_[k]);
        static_points_transformed_[k] = *filtered_static_points;

        (*map_noise_) += noisy_points_transformed_[k];
        (*map_dynamic_) += dynamic_points_transformed_[k] + potential_dynamic_points_transformed_[k];

        if (viz_detect_) {
            cout << pcs_transformed_[k].points.size() << " => " << static_points_transformed_[k].points.size() << " / ";
            cout << dynamic_points_transformed_[k].points.size() << " / " << noisy_points_transformed_[k].size() << " / ";
            cout << potential_dynamic_points_transformed_[k].points.size() << endl;
            CurrCloudPublisher.publish(erasor_utils::cloud2msg(pcs_transformed_[k]));
            DynCurrCloudPublisher.publish(erasor_utils::cloud2msg(dynamic_points_transformed_[k]));
//            RejectedDynCurrCloudPublisher.publish(erasor_utils::cloud2msg(*rejected_dynamic_objs));
            OutlierCurrCloudPublisher.publish(erasor_utils::cloud2msg(potential_dynamic_points_transformed_[k]));
            NoiseCurrCloudPublisher.publish(erasor_utils::cloud2msg(noisy_points_transformed_[k]));
            if (dataset_name_ == "SemanticKITTI") {
                pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                erasor_utils::parseStaticAndDynamic(static_points_transformed_[k], *dynamic_cloud, *static_cloud);
                StaticCloudPublisher.publish(erasor_utils::cloud2msg(*static_cloud));
                DynamicCloudPublisher.publish(erasor_utils::cloud2msg(*dynamic_cloud));
            }
            publishObjScores(RejectedMovingObjScorePublisher, rejected_objs_set_[k],
                             {1.0, 1.0, 1.0}, num_prev_rejected_objs_);
            publishObjScores(AcceptedMovingObjScorePublisher, accepted_objs_set_[k],
                             {0.0, 1.0, 0.0}, num_prev_accepted_objs_);

            grid_map_msgs::GridMap grid_msg;
            grid_map::GridMapRosConverter::toMessage(gridmap_submap_, grid_msg);
            GridPublisher.publish(grid_msg);

            visualizeHardThrRadius(poses_submap_[k]);

            if (stop_for_each_frame_) {
                std::cout << "[Detect] " << k << " th. Waiting for pressing a key" << std::endl;
                cin.ignore();
            }
        }
    }
}

void ERASOR2::estimateStaticMask(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                 const unordered_map<float, DynamicInstance> &ids_clusters, std::vector<int> &static_mask) {
    static_mask.resize(cloud.points.size());
    int             count = 0;
    for (const auto &pt: cloud) {
        auto iter = ids_clusters.find(pt.intensity);
        if (iter !=  ids_clusters.end() && iter->second.is_dynamic_) {
//        if (std::find(dyn_ids.begin(), dyn_ids.end(), pt.intensity) != dyn_ids.end()) {
            if (pt.intensity == NOT_VOLUME_OF_INTEREST) {
                // ToDo Improve
                static_mask[count] = IS_STATIC;
            } else {
                static_mask[count] = IS_DYNAMIC;
            }
        } else {
            static_mask[count] = IS_STATIC;
        }
        ++count;
    }
}

void ERASOR2::updateNoisyMask(const pcl::PointCloud<pcl::PointXYZI> &src_cloud,
                              const pcl::PointCloud<pcl::PointXYZI> &noisy_points,
                              std::vector<int> &static_mask) {
    vector<int> correspondences;
    erasor_utils::findCorrespondences(noisy_points, src_cloud, correspondences);
    for (const int correspondence: correspondences) {
        static_mask[correspondence] = IS_NOISE_YET_POTENTIAL_DYNAMIC;
    }
}

void ERASOR2::accumDynamicCloud(const int k, const int window_size,
                               pcl::PointCloud<pcl::PointXYZI> &cloud_accum, bool use_voxelization) {
    int lower_bound = k - window_size / 2;
    int upper_bound = k + (window_size + 1) / 2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr dyn_points_accum(new pcl::PointCloud<pcl::PointXYZI>);

    for (int i = max(0, lower_bound); i < min(num_data_, upper_bound); ++i) {
        (*dyn_points_accum) += dynamic_points_transformed_[i];
    }

    if (use_voxelization) {
        static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(dyn_points_accum);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(cloud_accum);
    } else {
        cloud_accum = *dyn_points_accum;
    }
}

void ERASOR2::accumInstanceWiseDynamicCloud(const int k, const int window_size,
                               pcl::PointCloud<pcl::PointXYZI> &cloud_accum, bool use_voxelization) {
    float dynamic_score_thr_ = 14.0;

    int lower_bound = k - window_size / 2;
    int upper_bound = k + (window_size + 1) / 2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr dyn_points_accum(new pcl::PointCloud<pcl::PointXYZI>);

    for (int i = max(0, lower_bound); i < min(num_data_, upper_bound); ++i) {
        const auto& noisy_points = noisy_points_transformed_[i];
        const auto& ids_clusters = ids_instances_set_[i];
        for (auto &[dyn_cand_id, dynamic_instance]: ids_clusters) {
            if (dynamic_instance.is_dynamic_ && dynamic_instance.moving_obj_score_ > dynamic_score_thr_) {
                *dyn_points_accum += dynamic_instance.cloud_;

                vector<int> target_idxes;
                erasor_utils::radiusSearch(dynamic_instance.cloud_, noisy_points,
                                          dist_thr_gain_ * voxel_size_, target_idxes);
                for (const int target_idx: target_idxes) {
                    dyn_points_accum->points.emplace_back(noisy_points[target_idx]);
                }
            }
        }
    }

    if (use_voxelization) {
        static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(dyn_points_accum);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(cloud_accum);
    } else {
        cloud_accum = *dyn_points_accum;
    }
}

void ERASOR2::instanceAwareOutlierRemoval(const int k, const int window_size,
                                           const float dist_thr_gain,
                                           pcl::PointCloud<pcl::PointXYZI> &filtered_static_points,
                                           pcl::PointCloud<pcl::PointXYZI> &potential_dynamic_points) {
    filtered_static_points.clear();
    potential_dynamic_points.clear();

    // 1. Set volumetric dynamic points
    pcl::PointCloud<pcl::PointXYZI>::Ptr dyn_points_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    accumDynamicCloud(k, window_size, *dyn_points_voxel);

    // 2. Set region of interest
    int lower_bound = k - window_size / 2;
    int upper_bound = k + (window_size + 1) / 2;
    vector<grid_map::Index> regions_of_interest;

    regions_of_interest.reserve(64); // heuristic
    for (int i = max(0, lower_bound); i < min(num_data_, upper_bound); ++i) {
        auto &ids_clusters  = ids_instances_set_[i];
        for (const auto &[dyn_cand_id, dynamic_cluster]: ids_clusters) {
            if (dynamic_cluster.is_dynamic_) {
                for (const auto &occupied_region: dynamic_cluster.occupied_map_idxes_) {
                    bool is_first = true;
                    for (const auto& roi: regions_of_interest) {
                        if (isEqual(roi, occupied_region)) {
                            // `occupied_region` is already in the `regions_of_interest`
                           is_first = false;
                           break;
                        }
                    }
                    if (is_first) {
                        regions_of_interest.emplace_back(occupied_region);
                    }
                }
            }
        }
    }
    cout << "A" << endl;

    // 3. Set the static points that potentially contain dynamic points
    int w_pc = idxes_approx_[k](0);
    int h_pc = idxes_approx_[k](1);
    grid_map::Position pos_approx = idx2position(idxes_approx_[k]);

    pcl::PointCloud<pcl::PointXYZI>::Ptr complement(new pcl::PointCloud<pcl::PointXYZI>);
    vector<pcl::PointCloud<pcl::PointXYZI>> xygrid;
    voi2xygrid(static_points_transformed_[k], pos_approx(0), pos_approx(1), poses_submap_[k](2, 3),
               range_of_interest_, grid_resolution_, xygrid, *complement);

    pcl::PointCloud<pcl::PointXYZI>::Ptr static_points_of_interest(new pcl::PointCloud<pcl::PointXYZI>);
    vector<bool> is_roi(xygrid.size(), false);
    cout << "B" << endl;
    // global_idx: idx w.r.t. sudmap's grid map
    for (const auto& global_idx: regions_of_interest) {
        if (global_idx(0) >= w_pc - neighboring_width_ / 2 && global_idx(0) < w_pc + neighboring_width_ / 2 &&
            global_idx(1) >= h_pc - neighboring_height_ / 2 && global_idx(1) < w_pc + neighboring_height_ / 2) {
            int voi_idx = globalIdx2LocalIdx(global_idx, idxes_approx_[k]);
            if (voi_idx > xygrid.size() || voi_idx < 0) { continue; }
//            cout << global_idx.transpose() << endl;
//            cout << (global_idx(1) < w_pc + neighboring_height_ / 2) << " | ";
//            cout << global_idx(1) << "  " <<  w_pc + neighboring_height_ / 2 << endl;
//            cout << idxes_approx_[k].transpose() << endl;
//            cout << voi_idx  << " < " << xygrid.size() << endl;
            (*static_points_of_interest) += xygrid[voi_idx];
            is_roi[voi_idx] = true;
        }
    }

    cout << "C" << endl;

    vector<int> correspondences;
    erasor_utils::findCorrespondences(*static_points_of_interest, *dyn_points_voxel, correspondences);

    cout << "D" << endl;
    // 4. Set filtered_static points and potential dynamic points
    filtered_static_points += (*complement);
    for (int j = 0; j < is_roi.size(); ++j) {
        if (!is_roi[j]) {
            filtered_static_points += xygrid[j];
        }
    }

    cout << "E" << endl;
    for (int j = 0; j < correspondences.size(); ++j) {
        const auto& query = static_points_of_interest->points[j];
        const auto& target = dyn_points_voxel->points[correspondences[j]];
        if (isInsideTheDynamicInstances(query, target)) {
            potential_dynamic_points.points.emplace_back(query);
        } else {
            filtered_static_points.emplace_back(query);
        }
    }
}

void ERASOR2::setDynamicInstance(DynamicInstance& dynamic_cluster, const float pos_x, const float pos_y) {
    dynamic_cluster.moving_obj_score_ = calcMovingClusterScore(dynamic_cluster.cloud_,
                                                                       dynamic_cluster.occupied_map_idxes_);

    pcl::compute3DCentroid(dynamic_cluster.cloud_, dynamic_cluster.centroid_);
    dynamic_cluster.is_close_to_body_frame_ = isCloseToSensorFrame(dynamic_cluster,
                                                                     pos_x,
                                                                     pos_y,
                                                                     hard_thr_radius_);

}

bool ERASOR2::isOverSegmented(const DynamicInstance& dynamic_cluster) {
    int num_unknown_grids = 0;
    int num_dynamic_grids = 0;
    for (const auto& idx: dynamic_cluster.occupied_map_idxes_) {
        if (gridmap_submap_.at("status", idx) == NOT_OBSERVED) {
            ++num_unknown_grids;
        }
        if (gridmap_submap_.at("status", idx) == TEMPORARILY_OCCUPIED &&
        logOdds2prob(gridmap_submap_.at("log_odds", idx)) > PROB_FOR_DEFINITELY_MOVING_OBJ) {
            ++num_dynamic_grids;
        }
    }

    int minimum_num_grid = static_cast<int>(minimum_area_thr_ / area_per_grid_);
    float  ratio_of_unknown_prior = static_cast<float>(num_unknown_grids) / dynamic_cluster.occupied_map_idxes_.size();
    if (dynamic_cluster.occupied_map_idxes_.size() > minimum_num_grid && ratio_of_unknown_prior > ratio_of_unknown_prior_
        && num_dynamic_grids > 0) {
        return true;
    } else {
        return false;
    }
}

void ERASOR2::parseOverSegmentation(const DynamicInstance& over_segmented, DynamicInstance& static_inst,
                                    DynamicInstance& partial_dynamic_inst, const float pos_x, const float pos_y) {
    static_inst.cloud_.clear();
    static_inst.cloud_.reserve(200);
    partial_dynamic_inst.cloud_.clear();
    partial_dynamic_inst.cloud_.reserve(200);


    for (const auto& pt: over_segmented.cloud_) {
        grid_map::Position p_tmp(pt.x, pt.y);
        grid_map::Index    idx_tmp;
        gridmap_submap_.getIndex(p_tmp, idx_tmp);
        if (logOdds2prob(gridmap_submap_.at("log_odds", idx_tmp)) > PROB_FOR_DEFINITELY_MOVING_OBJ) {
            partial_dynamic_inst.cloud_.emplace_back(pt);
        } else {
            static_inst.cloud_.emplace_back(pt);
        }
    }

    setDynamicInstance(static_inst, pos_x, pos_y);
    static_inst.is_dynamic_ = false;
    setDynamicInstance(partial_dynamic_inst, pos_x, pos_y);
    partial_dynamic_inst.is_dynamic_ = true;
}

void ERASOR2::updateNewParsedInstances(const vector<OverSegmentedInstance>& instances_to_be_updated,
                                       pcl::PointCloud<pcl::PointXYZI> &cloud,
                                       unordered_map<float, DynamicInstance>& ids_clusters) {
    /***
     * It has two rules:
     * a) update original point cloud
     *     Because the id is used when estimating `static_mask`,
     *     so the ids of the original point cloud are also updated
     * b) append new instances into ids_clusters
     */
    for (const auto& inst: instances_to_be_updated) {
        vector<int> corr_dyn, corr_stat;
        erasor_utils::findCorrespondences(inst.dynamic_inst.cloud_, cloud, corr_dyn);
        erasor_utils::findCorrespondences(inst.static_inst.cloud_, cloud, corr_stat);

        int N = inst.dynamic_inst.cloud_.size();
        int M = inst.static_inst.cloud_.size();

        // 1. Update ids in raw point cloud
        for (int n = 0; n < N; ++n) {
            cloud.points[corr_dyn[n]].intensity = inst.new_id_for_dyn_inst;
        }
        for (int m = 0; m < M; ++m) {
            cloud.points[corr_stat[m]].intensity = inst.new_id_for_stat_inst;
        }

        // 2. Remove old one;
        ids_clusters.erase(inst.original_id);

        // 3. Update ids in raw point cloud
        ids_clusters[inst.new_id_for_dyn_inst] = inst.dynamic_inst;
        ids_clusters[inst.new_id_for_stat_inst] = inst.static_inst;
    }
}

void ERASOR2::windowBasedVolumetricOutlierRemoval(const int k, const int window_size,
                                           const float dist_thr_gain,
                                           pcl::PointCloud<pcl::PointXYZI> &filtered_static_points,
                                           pcl::PointCloud<pcl::PointXYZI> &potential_dynamic_points) {
    // 1. Set volumetric dynamic points
    pcl::PointCloud<pcl::PointXYZI>::Ptr dyn_points_voxel(new pcl::PointCloud<pcl::PointXYZI>);
//    accumDynamicCloud(k, window_size, *dyn_points_voxel);
    accumInstanceWiseDynamicCloud(k, window_size, *dyn_points_voxel);

    // 2. Dynamic removal
    volumetricOutlierRemoval(static_points_transformed_[k],
                             *dyn_points_voxel,
                             dist_thr_gain,
                             filtered_static_points, potential_dynamic_points);

}

void ERASOR2::volumetricOutlierRemoval(const pcl::PointCloud<pcl::PointXYZI> &static_points,
                                       const pcl::PointCloud<pcl::PointXYZI> &dynamic_points,
                                       const float dist_thr_gain,
                                       pcl::PointCloud<pcl::PointXYZI> &filtered_static_points,
                                       pcl::PointCloud<pcl::PointXYZI> &potential_dynamic_points) {
    int N = static_points.size();
    vector<int> static_mask(N, IS_STATIC);

    vector<int> target_idxes;
    erasor_utils::radiusSearch(dynamic_points, static_points,
                               dist_thr_gain_ * voxel_size_, target_idxes);

    for (const int idx: target_idxes) {
        static_mask[idx] = IS_DYNAMIC;
    }

//    cout << "\033[1;32mTotal " << valid_outlier_idxes.size() << " points are filtered\033[0m" << endl;
    filtered_static_points.clear();
    filtered_static_points.reserve(N);
    for (int j = 0; j < N; ++j) {
        const auto& status = static_mask[j];
        const auto& pt = static_points[j];
        if (status == IS_STATIC) {
            filtered_static_points.points.emplace_back(pt);
        } else if (status == IS_DYNAMIC) {
            potential_dynamic_points.points.emplace_back(pt);
        } else { throw invalid_argument("A wrong mask status is set!"); }
    }
    cout << "\033[1;32mTotal " << potential_dynamic_points.size() << " points are filtered\033[0m" << endl;
}

void ERASOR2::discernStaticAndDynamicPoints(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                            const std::vector<int> &static_mask,
                                            pcl::PointCloud<pcl::PointXYZI> &static_points,
                                            pcl::PointCloud<pcl::PointXYZI> &dynamic_points) {
    if (cloud.size() != static_mask.size()) {
        throw invalid_argument("Something's wrong!");
    }

    static_points.clear();
    dynamic_points.clear();
    int             count = 0;
    for (const auto &pt: cloud) {
        if (static_mask[count] == IS_STATIC) {
            static_points.points.emplace_back(pt);
        } else if (static_mask[count] == IS_DYNAMIC) {
            // Note that noisy points are not included
            dynamic_points.points.emplace_back(pt);
        }

        ++count;
    }
}

//void
//ERASOR2::discernStaticAndDynamicPoints(const pcl::PointCloud<pcl::PointXYZI> &cloud, const std::vector<int> &dyn_ids,
//                                       pcl::PointCloud<pcl::PointXYZI> &static_points,
//                                       pcl::PointCloud<pcl::PointXYZI> &dynamic_points) {
//    for (const auto &pt: cloud) {
//        if (std::find(dyn_ids.begin(), dyn_ids.end(), pt.intensity) != dyn_ids.end()) {
//            if (pt.intensity == NOT_VOLUME_OF_INTEREST) {
//                // ToDo Improve
//                static_points.points.emplace_back(pt);
//            } else {
//                dynamic_points.points.emplace_back(pt);
//            }
//        } else {
//            static_points.points.emplace_back(pt);
//        }
//    }
//}

void ERASOR2::publishStaticMapResults() {
    std::cout << "[ERASOR2] Publish results!" << std::endl;
//    std::cout << "# of dynamic points: " << map_dynamic_->points.size() << std::endl;
    std::cout << "# of static points: " << static_map_voxelized_->points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_for_viz(new pcl::PointCloud<pcl::PointXYZI>);
    // Because ERASOR2 uses the local origin,
    pcl::transformPointCloud(*static_map_voxelized_, *static_map_for_viz, new_origin_.inverse());
    MapCloudPublisher.publish(erasor_utils::cloud2msg(*static_map_for_viz));
    DynMapPublisher.publish(erasor_utils::cloud2msg(*map_dynamic_));
    NoiseMapPublisher.publish(erasor_utils::cloud2msg(*map_noise_));
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
    int N = src.points.size();
    cloud_out.reserve(N);
    cloud_out = src;

#pragma omp parallel for num_threads(num_omp_cores_)
    for (int i = 0; i < N; ++i) {
        if (cloud_out.points[i].z < min_z_voi || cloud_out.points[i].z > max_z_voi) {
            cloud_out.points[i].intensity = NOT_VOLUME_OF_INTEREST;
        }
    }
}

float ERASOR2::getMaxInstanceId(const pcl::PointCloud<pcl::PointXYZI> &src) {
    float max_id = -1;
    for (const auto& pt: src.points) {
        max_id = max(max_id, pt.intensity);
    }
    return max_id;
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
    grid_map::GridMap gridmap({"elevation", "status", "prob", "log_odds", "erosion"});
    gridmap.setFrameId("map");
    gridmap.setGeometry(grid_map::Length(grid_map_info.x_length, grid_map_info.y_length),
                        grid_map_info.resolution);
    gridmap["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
    gridmap["status"].setConstant(NOT_OBSERVED);
    gridmap["prob"].setConstant(0);
    gridmap["log_odds"].setConstant(0);
    gridmap["erosion"].setConstant(0);
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


bool ERASOR2::isLikelyToBeGround(const pcl::PointCloud<pcl::PointXYZI> &pc, const float ratio_num,
                                 const int num_min_pts) {
    if (pc.empty()) { return false; }

    int num_ground_pts = erasor_utils::getNumGroundPoints(pc);

    ratio_num_ = static_cast<float>(num_ground_pts) / pc.points.size();
    if ((ratio_num_ > ratio_num) && pc.points.size() > num_min_pts) {
        return true; // Free
    } else { return false; }
}

bool ERASOR2::isLikelyToBeSteppableRegion(const pcl::PointCloud<pcl::PointXYZI> &curr_pc,
                                          const pcl::PointCloud<pcl::PointXYZI> &map_pc,
                                          const float scan_ratio_threshold, const float min_z_diff_thr,
                                          const bool verbose) {
    if (curr_pc.points.size() < minimum_num_pts_ || map_pc.points.size() < minimum_num_pts_) {
        return false;
    }

    float curr_min_z, curr_max_z;
    float map_min_z, map_max_z;

//    cout << "FF?" << curr_pc.size() << endl;
    erasor_utils::calcMinMaxZ(curr_pc, curr_min_z, curr_max_z);
//    cout << "FF0" << map_pc.size() << endl;
    erasor_utils::calcMinMaxZWithoutGround(map_pc, map_min_z, map_max_z);
//    cout << "FF!" << endl;
    float curr_mean_ground_z = erasor_utils::calcMeanZOfGround(curr_pc);
    float map_mean_ground_z  = erasor_utils::calcMeanZOfGround(map_pc);

    float lowest_z    = min(curr_mean_ground_z, map_mean_ground_z);
    float map_h_diff  = map_max_z - lowest_z;
    float curr_h_diff = curr_max_z - lowest_z;

    scan_ratio_ = min(map_h_diff / curr_h_diff,
                      curr_h_diff / map_h_diff);

    // To reduce false positives
    if (map_h_diff < min_z_diff_thr || curr_mean_ground_z - map_min_z > min_z_diff_thr * 1.5) {
        return false;
    }

    // Dynamic!
    if (scan_ratio_ < scan_ratio_threshold &&
        isLikelyToBeGround(curr_pc, ratio_num_pts_, minimum_num_pts_)) { // find dynamic!
        return true;
    } else {
        return false;
    }
}

void ERASOR2::updateLogOdds(const grid_map::Index &idx, const float increment, const int kernel_size) {
    gridmap_submap_.at("log_odds", idx) += increment;

    auto idx_for_neighboring = idx;
    int  w                   = idx(0);
    int  h                   = idx(1);

    if (kernel_size == 3) {
        // Refer to https://www.opencv-srf.com/2018/03/gaussian-blur.html
        vector<pair<float, float> > plus_minus_for_adjacent = {{1,  0},
                                                               {-1, 0},
                                                               {0,  1},
                                                               {0,  -1}};
        for (const auto             &sign: plus_minus_for_adjacent) {
            idx_for_neighboring(0) = w + sign.first;
            idx_for_neighboring(1) = h + sign.second;
            gridmap_submap_.at("log_odds", idx) += increment / 2.0;
        }

        vector<pair<float, float> > plus_minus_for_diagonal = {{1,  1},
                                                               {1,  -1},
                                                               {-1, 1},
                                                               {-1, -1}};
        for (const auto             &sign: plus_minus_for_diagonal) {
            idx_for_neighboring(0) = w + sign.first;
            idx_for_neighboring(1) = h + sign.second;
            gridmap_submap_.at("log_odds", idx) += increment / 4.0;
        }
    } else if (kernel_size == 1) {
        return;
    } else {
        throw invalid_argument("Not implemented");
    }
}

grid_map::GridMap ERASOR2::setEgocentricGridMap(float range,
                                                const float grid_resolution,
                                                const vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid) {

    grid_map::GridMap gridmap({"elevation", "log_odds"});
    gridmap.setFrameId("map");
    gridmap.setGeometry(grid_map::Length(2 * range, 2 * range), grid_resolution);
    gridmap["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
    gridmap["log_odds"].setConstant(NOT_OBSERVED);

    const int width  = static_cast<int>(2.00000001 * range / grid_resolution);
    const int height = static_cast<int>(2.00000001 * range / grid_resolution);

    grid_map::Index idx;
    for (int        u = 0; u < width; ++u) {
        for (int v = 0; v < height; ++v) {
            int i = u + width * v;
            if (isLikelyToBeGround(xygrid[i], ratio_num_pts_, minimum_num_pts_)) {
                idx(0)                       = u;
                idx(1)                       = v;
                gridmap.at("log_odds", idx) = GROUND_EXISTS;
            }
        }
    }
    return gridmap;
}

float ERASOR2::calcMovingClusterScore(const pcl::PointCloud<pcl::PointXYZI> &dynamic_cluster,
                                      vector<grid_map::Index>& occupied_map_idxes) {
    float           total_score = 0;
    for (const auto &dyn_pt: dynamic_cluster.points) {
        grid_map::Position p_tmp(dyn_pt.x, dyn_pt.y);
        grid_map::Index    idx_tmp;
        gridmap_submap_.getIndex(p_tmp, idx_tmp);
        if (gridmap_submap_.at("status", idx_tmp) == NOT_OBSERVED) {
            total_score += negative_log_odds_;
        } else {
            total_score += gridmap_submap_.at("log_odds", idx_tmp);
        }

        bool is_first = true;
        for (const auto& occupied_region: occupied_map_idxes) {
            // Means already idx_tmp is updated
            if (idx_tmp(0) == occupied_region(0) && idx_tmp(1) == occupied_region(1)) {
                is_first = false;
                break;
            }
        }
        if (is_first) {
            occupied_map_idxes.emplace_back(idx_tmp);
        }
    }

    return total_score / dynamic_cluster.points.size();
}

void ERASOR2::logOddsGrid2probGrid() {
    grid_map::Index idx;
    for (int h = 0; h < grid_map_info_.height; ++h) {
        for (int w = 0; w < grid_map_info_.width; ++w) {
            idx(0) = w;
            idx(1) = h;
            gridmap_submap_.at("prob", idx) = logOdds2prob(gridmap_submap_.at("log_odds", idx));
        }
    }
}

void ERASOR2::dilateAndErode(grid_map::GridMap &gridmap_submap) {
    // Noise filtering?
    cv::Mat img, img_eroded, img_dilated;
    gridmap_submap["erosion"] = gridmap_submap["log_odds"];
    const float min_coefficient = gridmap_submap.get("erosion").minCoeff();
    const float max_coefficient = gridmap_submap.get("erosion").maxCoeff();
    std::cout << min_coefficient << ", " << max_coefficient << std::endl;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(gridmap_submap, "erosion", CV_8UC1,
                                                            min_coefficient, max_coefficient, img);
    std::string save_dir = "/home/shapelim/Pictures/erasor2";
    cv::imwrite(save_dir + "/original.png", img);
    dilate(img, img_dilated, cv::Mat::ones(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 1);
    cv::imwrite(save_dir + "/dilation.png", img_dilated);
    erode(img_dilated, img_eroded, cv::Mat::ones(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 2);
    cv::imwrite(save_dir + "/erosion.png", img_eroded);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(img_eroded, "erosion",
                                                                      gridmap_submap, min_coefficient, max_coefficient);
    const float min_coefficient_after = gridmap_submap.get("erosion").minCoeff();
    const float max_coefficient_after = gridmap_submap.get("erosion").maxCoeff();
    std::cout << min_coefficient_after << ", " << max_coefficient_after << std::endl;
}

void ERASOR2::erodeGridMap(grid_map::GridMap &gridmap_submap) {
    // Noise filtering?
    cv::Mat     img, img_eroded, img_dilated;
    const float min_coefficient = gridmap_submap.get("log_odds").minCoeff();
    const float max_coefficient = gridmap_submap.get("log_odds").maxCoeff();
    std::cout << "\033[1;34m" << min_coefficient << ", " << max_coefficient << std::endl;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(gridmap_submap, "log_odds", CV_8UC1,
                                                            min_coefficient, max_coefficient, img);

    erode(img, img_eroded, cv::Mat::ones(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 2);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(img_eroded, "eroded",
                                                                      gridmap_submap, min_coefficient, max_coefficient);
    const float min_coefficient_after = gridmap_submap.get("erroded").minCoeff();
    const float max_coefficient_after = gridmap_submap.get("erroded").maxCoeff();
    std::cout << min_coefficient_after << ", " << max_coefficient_after << "\033[0m" << std::endl;
}

void
ERASOR2::publishObjScores(const ros::Publisher &publisher, const vector<pair<Eigen::Matrix<float, 4, 1>, float> > &objs,
                          const vector<float> color, int &num_prev_objs) {
    visualization_msgs::MarkerArray marker_arr;
    visualization_msgs::Marker      marker;
    marker.header.frame_id = "map";
    marker.header.stamp    = ros::Time::now();

    int num_curr_objs = objs.size();
    cout << "Total " << num_curr_objs << " objs do exist." << endl;
    for (int i = 0; i < num_curr_objs; ++i) {
        const auto &pair = objs[i];
        marker.id                 = i;
        marker.text               = (boost::format("%.02f") % pair.second).str();
        marker.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action             = visualization_msgs::Marker::ADD;
        marker.pose.position.x    = pair.first(0);
        marker.pose.position.y    = pair.first(1);
        marker.pose.position.z    = pair.first(2) + 1.0; // For airborne
        marker.pose.orientation.w = 1.0;
//                    marker.scale.x = 1;
//                    marker.scale.y = 0.1;
        marker.scale.z            = 1.25;
        marker.color.a            = 1.0; // Don't forget to set the alpha!
        marker.color.r            = color[0];
        marker.color.g            = color[1];
        marker.color.b            = color[2];
        marker_arr.markers.push_back(marker);
    }

    for (int i = num_curr_objs; i < num_prev_objs; ++i) {
        visualization_msgs::Marker markers_to_be_removed;
        markers_to_be_removed.id              = i;
        markers_to_be_removed.header.frame_id = "map";
        markers_to_be_removed.header.stamp    = ros::Time::now();
        markers_to_be_removed.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
        markers_to_be_removed.action          = visualization_msgs::Marker::DELETE;
        marker_arr.markers.push_back(markers_to_be_removed);
    }
    publisher.publish(marker_arr);

    num_prev_objs = num_curr_objs;
}

bool ERASOR2::isCloseToSensorFrame(const DynamicInstance& dynamic_cluster, const float pos_x, const float pos_y,
                              const float range_thr) {
    const float obj_x = dynamic_cluster.centroid_(0);
    const float obj_y = dynamic_cluster.centroid_(1);
    float dist_sqr = (obj_x - pos_x) * (obj_x - pos_x) + (obj_y - pos_y) * (obj_y - pos_y);
    float thr_sqr  = range_thr * range_thr;
    if (dist_sqr > thr_sqr) {
        return false;
    } else {
        return true;
    }
}

bool ERASOR2::isSizeSufficientlySmall(const pcl::PointCloud<pcl::PointXYZI> &dynamic_cluster,
                                 const float xy_size_thr) {
    pcl::PointXYZI min_pt, max_pt;
    pcl::getMinMax3D(dynamic_cluster, min_pt, max_pt);
    if (abs(min_pt.x - max_pt.x) < xy_size_thr &&  abs(min_pt.x - max_pt.x) < xy_size_thr) {
        return true;
    } else {
        return false;
    }
}

void ERASOR2::visualizeHardThrRadius(const Eigen::Matrix4f &pose) {
    static float               z_diff = max_z_voi_ - min_z_voi_;
    visualization_msgs::Marker marker;
    marker.header.frame_id    = "map";
    marker.header.stamp       = ros::Time::now();
    marker.id                 = 0;
    marker.type               = visualization_msgs::Marker::CYLINDER;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.position.x    = pose(0, 3);
    marker.pose.position.y    = pose(1, 3);
    marker.pose.position.z    = pose(2, 3);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = hard_thr_radius_ * 2;
    marker.scale.y            = hard_thr_radius_ * 2;
    marker.scale.z            = z_diff;
    marker.color.a            = 0.3; // Don't forget to set the alpha!
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;

    AdaptiveRangePublisher.publish(marker);
}

void ERASOR2::printClusterInfo(const DynamicInstance& dynamic_cluster) {
    vector<grid_map::Index> occupied_map_idxes_ = dynamic_cluster.occupied_map_idxes_;
    int min_x = numeric_limits<int>::max();
    int max_x = 0;
    int min_y = numeric_limits<int>::max();
    int max_y = 0;
    for (const auto& idx: occupied_map_idxes_) {
        min_x = min(min_x, idx(0));
        max_x = max(max_x, idx(0));
        min_y = min(min_y, idx(1));
        max_y = max(max_y, idx(1));
    }
    // x-axis: top -> down
    // y-axis: left -> right
    vector<vector<char>> map;
    vector<char> line(max_x - min_x + 1, ' ');
    for (int i = 0; i <= max_y - min_y; ++i) {
        map.push_back(line);
    }

    for (const auto& idx: occupied_map_idxes_) {
//        cout << idx.transpose();
        int x_new = idx(1) - min_y;
        int y_new = max_x - idx(0);
//        cout << " = > " << x_new << ", " << y_new << endl;
        if (gridmap_submap_.at("status", idx) == TEMPORARILY_OCCUPIED) {
            map[x_new][y_new] = 'O';
        } else if (gridmap_submap_.at("status", idx) == GROUND_EXISTS) {
            map[x_new][y_new] = 'H';
        } else if (gridmap_submap_.at("status", idx) == NOT_OBSERVED) {
            map[x_new][y_new] = 'X';

        }
        cout << idx.transpose() << " -> " << x_new << ", " << y_new << " (" <<gridmap_submap_.at("status", idx) << " | "
                   << gridmap_submap_.at("log_odds", idx) << ")" << endl;
    }

    for (auto const& line_to_viz: map) {
        for (int j = 0; j < line_to_viz.size(); ++j) {
            cout << line_to_viz[j];
        }
        cout << endl;
    }

}