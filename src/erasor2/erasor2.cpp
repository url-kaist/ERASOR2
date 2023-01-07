#include "erasor2/erasor2.h"

using namespace std;

ERASOR2::ERASOR2() {
    cout << "[ERASOR2] Unknown prior: " << unknown_prior_ << endl;
    cout << "[ERASOR2] Initial prior: " << initial_prior_ << endl;
    cout << "[ERASOR2] Informative prior: " << informative_prior_ << endl;
    cout << "[ERASOR2] Increment gain: " << increment_gain_ << endl;
    cout << "[ERASOR2] Increment: " << increment_ << endl;
    if (initial_prior_ > informative_prior_ || increment_gain_ < 1.0) {
        throw invalid_argument("Parameters are wrongly set!");
    }
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
        std::cout << "publish2" << std::endl;
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

    cout << "[ERASOR2] Calculating  min-max x, y values..." << endl;
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

    dynamic_ids_set_.resize(num_data_);
    dynamic_ids_clusters_set_.resize(num_data_);
}

void ERASOR2::updateSteppableRegion() {
    for (int k                   = 0; k < num_data_; ++k) {
        std::cout << "\r[Update] " << k << " / " << num_data_ << std::endl;
        gridmap_submap_["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
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
                idx(0) = w;
                idx(1) = h;
//                    cout << "(" << w << ", " << h << ") - ";
                if (isLikelyToBeSteppableRegion(xygrids_[k][count], map_grid[count],
                                                scan_ratio_threshold_,
                                                th_bin_max_h_, verbose_)) {
                    // For debugging
                    gridmap_submap_.at("elevation", idx) = 1.0;
                    cout << "(" << w << ", " << h << "): " << scan_ratio_ << " | " << ratio_num_ << " // ";
                    cout << xygrids_[k][count].size() << " <-> "
                         << erasor_utils::getNumGroundPoints(xygrids_[k][count]) << endl;
                    updatePrior(idx, informative_prior_);
                    updatePosterior(idx, increment_ * increment_gain_);
//                    } else if (isLikelyToBeGround(xygrids_[k][count])) {
//                        updatePrior(idx, initial_prior_);
//                        updatePosterior(idx, increment_);
//                    }
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
            gridmap_submap_["steppable"] = gridmap_submap_["prior"] + gridmap_submap_["posterior"];
            grid_map::GridMapRosConverter::toMessage(gridmap_submap_, grid_msg);
            GridPublisher.publish(grid_msg);
            ros::spinOnce();
            if (stop_for_each_frame_) {
                std::cout << "[Update] Waiting for pressing a key" << std::endl;
                cin.ignore();
            }
        }
    }
    gridmap_submap_["steppable"] = gridmap_submap_["prior"] + gridmap_submap_["posterior"];
}

// Re-project ground likelihood to each scan
void ERASOR2::detectDynamicObjects() {
    dilateAndErode(gridmap_submap_);
    grid_map_msgs::GridMap grid_msg;
    grid_map::GridMapRosConverter::toMessage(gridmap_submap_, grid_msg);
    GridPublisher.publish(grid_msg);
    for (int k = 0; k < num_data_; ++k) {
        vector<float> &dyn_cand_ids = dynamic_ids_set_[k]; // temp. variable
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
                if (gridmap_submap_.at("steppable", idx) > ground_likelihood_thr_) {
                    // Extract indices
                    // Note that `xygrids_[k][count]` does not contain non-VoI points
                    for (const auto &pt: xygrids_[k][count].points) {
                        if ((pt.intensity != GROUND_LABEL) && (pt.intensity != NOT_INTEREST) &&
                            std::find(dyn_cand_ids.begin(), dyn_cand_ids.end(), pt.intensity) == dyn_cand_ids.end()) {
                            dyn_cand_ids.push_back(pt.intensity);
                        } else if (pt.intensity == NOT_INTEREST) {
                            noisy_points_transformed_[k].points.emplace_back(pt);
                        }
                    }
                }
                ++count;
            }
        }
    }
}

void ERASOR2::filterDynamicObjects() {
    for (int k = 0; k < num_data_; ++ k) {

        // Filtering
        auto & dyn_cand_ids = dynamic_ids_set_[k];
        auto &ids_clusters  = dynamic_ids_clusters_set_[k];
        auto &rejected_objs = rejected_objs_set_[k];
        auto &accepted_objs = accepted_objs_set_[k];

        ids_clusters.clear();
        DynamicCluster dyn_cluster;
        dyn_cluster.cloud_.reserve(200); // heuristic
        for (const int dyn_cand_id: dyn_cand_ids) {
            ids_clusters[dyn_cand_id] = dyn_cluster;
        }

        for (const auto &pt: pcs_transformed_[k]) {
            if (ids_clusters.find(pt.intensity) != ids_clusters.end()) {
                ids_clusters[pt.intensity].cloud_.emplace_back(pt);
            }
        }

        // Only available on C++ 17
        for (auto &[dyn_cand_id, dynamic_cluster]: ids_clusters) {
            dynamic_cluster.moving_obj_score_ = calcMovingClusterScore(dynamic_cluster.cloud_,
                                                                       dynamic_cluster.occupied_regions_);
//            if (k == 46 || k == 51) {
//                DynCurrCloudPublisher.publish(erasor_utils::cloud2msg(*dynamic_obj));
//                cout << "==> " << moving_obj_score << endl;
//                cin.ignore();
//            }
            pcl::compute3DCentroid(dynamic_cluster.cloud_, dynamic_cluster.centroid_);
            if (dynamic_cluster.moving_obj_score_ >
                getAdaptiveThreshold(dynamic_cluster.centroid_(0), dynamic_cluster.centroid_(1),
                                     poses_submap_[k](0, 3), poses_submap_[k](1, 3),
                                     adaptive_range_)) {
                dynamic_cluster.is_dynamic_ = true;
                // For visualization
                accepted_objs.push_back({dynamic_cluster.centroid_, dynamic_cluster.moving_obj_score_});
            } else { // it means that most parts are not in the region of interests
                dynamic_cluster.is_dynamic_ = false;
//                (*rejected_dynamic_objs) += dynamic_cluster.cloud_;
                // For visualization
                rejected_objs.push_back({dynamic_cluster.centroid_, dynamic_cluster.moving_obj_score_});
            }
        }
    }

    // Finally assign the static points
    for (int k = 0; k < num_data_; ++k) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_points_each_scan(new pcl::PointCloud<pcl::PointXYZI>);

        const auto &each_pc = pcs_transformed_[k];
        const auto &ids_clusters= dynamic_ids_clusters_set_[k];

        static_points_transformed_[k].clear();
        vector<int> static_mask;
        estimateStaticMask(each_pc, ids_clusters, static_mask);
        // Noisy points are added by the following function:
        updateNoisyMask(each_pc, noisy_points_transformed_[k], static_mask);
        if (dataset_name_ == "SemanticKITTI") {
            // To preserve the original SemanticKITTI labels
            discernStaticAndDynamicPoints(pcs_gt_transformed_[k], static_mask, static_points_transformed_[k],
                                          *dynamic_points_each_scan);
        } else {
            discernStaticAndDynamicPoints(each_pc, static_mask, static_points_transformed_[k],
                                          *dynamic_points_each_scan);
        }

        (*map_noise_) += noisy_points_transformed_[k];
        (*map_dynamic_) += (*dynamic_points_each_scan);

        if (viz_detect_) {
            cout << each_pc.size() << " => " << static_points_transformed_[k].points.size() << " / ";
            cout << dynamic_points_each_scan->points.size() << " / " << noisy_points_transformed_[k].size() << endl;
            CurrCloudPublisher.publish(erasor_utils::cloud2msg(each_pc));
            DynCurrCloudPublisher.publish(erasor_utils::cloud2msg(*dynamic_points_each_scan));
//            RejectedDynCurrCloudPublisher.publish(erasor_utils::cloud2msg(*rejected_dynamic_objs));
            NoiseCurrCloudPublisher.publish(erasor_utils::cloud2msg(noisy_points_transformed_[k]));
            publishObjScores(RejectedMovingObjScorePublisher, rejected_objs_set_[k],
                             {1.0, 1.0, 1.0}, num_prev_rejected_objs_);
            publishObjScores(AcceptedMovingObjScorePublisher, accepted_objs_set_[k],
                             {0.0, 1.0, 0.0}, num_prev_accepted_objs_);

            grid_map_msgs::GridMap grid_msg;
            grid_map::GridMapRosConverter::toMessage(gridmap_submap_, grid_msg);
            GridPublisher.publish(grid_msg);

            visualizeAdaptiveRange(poses_submap_[k]);

            if (stop_for_each_frame_) {
                std::cout << "[Detect] Waiting for pressing a key" << std::endl;
                cin.ignore();
            }
        }
    }
}

void ERASOR2::estimateStaticMask(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                 const unordered_map<float, DynamicCluster> &ids_clusters, std::vector<int> &static_mask) {
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
    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
            PointCloud<num_t>, 3 /* dim */
    >;

    int               N = src_cloud.points.size();
    PointCloud<num_t> cloud;
    cloud.pts.resize(N);
    for (size_t i = 0; i < N; i++) {
        cloud.pts[i].x = src_cloud.points[i].x;
        cloud.pts[i].y = src_cloud.points[i].y;
        cloud.pts[i].z = src_cloud.points[i].z;
    }

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

    for (auto &query_pcl: noisy_points.points) {
        const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};
        {
            size_t                num_results = 1;
            std::vector<uint32_t> ret_index(num_results);
            std::vector<num_t>    out_dist_sqr(num_results);

            num_results = index.knnSearch(
                    &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

            ret_index.resize(num_results);
            out_dist_sqr.resize(num_results);
            static_mask[ret_index[0]] = IS_NOISE_YET_POTENTIAL_DYNAMIC;
        }
    }
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
        } else {
            // Noisy points can be included
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
    grid_map::GridMap gridmap({"elevation", "prior", "posterior", "steppable", "erosion"});
    gridmap.setFrameId("map");
    gridmap.setGeometry(grid_map::Length(grid_map_info.x_length, grid_map_info.y_length),
                        grid_map_info.resolution);
    gridmap["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
    gridmap["prior"].setConstant(unknown_prior_);
    gridmap["posterior"].setConstant(0);
    gridmap["steppable"].setConstant(0);
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
                                          const float scan_ratio_threshold, const float th_bin_max_h,
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
    if (map_h_diff < th_bin_max_h || curr_mean_ground_z - map_min_z > th_bin_max_h * 1.5) {
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

void ERASOR2::updatePrior(const grid_map::Index &idx, const float prior) {
    // Initialization
    if (gridmap_submap_.at("prior", idx) == unknown_prior_) {
        gridmap_submap_.at("prior", idx) = prior;
    }

    // If a given prior is more reliable, update
    if (gridmap_submap_.at("prior", idx) < prior) {
        gridmap_submap_.at("prior", idx) = prior;
    }
}

void ERASOR2::updatePosterior(const grid_map::Index &idx, const float increment, const int kernel_size) {
    gridmap_submap_.at("posterior", idx) += increment;

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
            gridmap_submap_.at("posterior", idx) += increment / 2.0;
        }

        vector<pair<float, float> > plus_minus_for_diagonal = {{1,  1},
                                                               {1,  -1},
                                                               {-1, 1},
                                                               {-1, -1}};
        for (const auto             &sign: plus_minus_for_diagonal) {
            idx_for_neighboring(0) = w + sign.first;
            idx_for_neighboring(1) = h + sign.second;
            gridmap_submap_.at("posterior", idx) += increment / 4.0;
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

    grid_map::GridMap gridmap({"elevation", "steppable"});
    gridmap.setFrameId("map");
    gridmap.setGeometry(grid_map::Length(2 * range, 2 * range), grid_resolution);
    gridmap["elevation"].setConstant(DIST_FROM_GROUND_TO_ORIGIN);
    gridmap["steppable"].setConstant(unknown_prior_);

    const int width  = static_cast<int>(2.00000001 * range / grid_resolution);
    const int height = static_cast<int>(2.00000001 * range / grid_resolution);

    grid_map::Index idx;
    for (int        u = 0; u < width; ++u) {
        for (int v = 0; v < height; ++v) {
            int i = u + width * v;
            if (!xygrid[i].points.empty() && isLikelyToBeGround(xygrid[i], ratio_num_pts_, minimum_num_pts_)) {
                idx(0)                       = u;
                idx(1)                       = v;
                gridmap.at("steppable", idx) = initial_prior_;
            }
        }
    }
    return gridmap;
}

float ERASOR2::calcMovingClusterScore(const pcl::PointCloud<pcl::PointXYZI> &dynamic_cluster,
                                      vector<grid_map::Index>& occupied_regions) {
    float           total_score = 0;
    for (const auto &dyn_pt: dynamic_cluster.points) {
        grid_map::Position p_tmp(dyn_pt.x, dyn_pt.y);
        grid_map::Index    idx_tmp;
        gridmap_submap_.getIndex(p_tmp, idx_tmp);
        total_score += gridmap_submap_.at("steppable", idx_tmp);

        bool is_first = true;
        for (const auto& occupied_region: occupied_regions) {
            // Means already idx_tmp is updated
            if (idx_tmp(0) == occupied_region(0) && idx_tmp(1) == occupied_region(1)) {
                is_first = false;
                break;
            }
        }
        if (is_first) {
            occupied_regions.emplace_back(idx_tmp);
        }
    }

    return total_score / dynamic_cluster.points.size();
}

void ERASOR2::dilateAndErode(grid_map::GridMap &gridmap_submap) {
    // Noise filtering?
    cv::Mat img, img_eroded, img_dilated;
    gridmap_submap["erosion"] = gridmap_submap["steppable"];
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
    const float min_coefficient = gridmap_submap.get("steppable").minCoeff();
    const float max_coefficient = gridmap_submap.get("steppable").maxCoeff();
    std::cout << "\033[1;34m" << min_coefficient << ", " << max_coefficient << std::endl;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(gridmap_submap, "steppable", CV_8UC1,
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

float ERASOR2::getAdaptiveThreshold(const float obj_x, const float obj_y,
                                    const float pos_x, const float pos_y, const float adaptive_range) {
    float dist_sqr = (obj_x - pos_x) * (obj_x - pos_x) + (obj_y - pos_y) * (obj_y - pos_y);
    float thr_sqr  = adaptive_range * adaptive_range;
    if (dist_sqr > thr_sqr) {
        return obj_score_soft_thr_;
    } else {
        return obj_score_hard_thr_;
    }
}

void ERASOR2::visualizeAdaptiveRange(const Eigen::Matrix4f &pose) {
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
    marker.scale.x            = adaptive_range_;
    marker.scale.y            = adaptive_range_;
    marker.scale.z            = z_diff;
    marker.color.a            = 0.3; // Don't forget to set the alpha!
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;

    AdaptiveRangePublisher.publish(marker);
}