#include "tools/erasor_utils.hpp"

std::vector<int> DYNAMIC_CLASSES = {251, 252, 253, 254, 255, 256, 257, 258, 259};

namespace erasor_utils {
    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose) {
        geometry_msgs::Pose geoPose;

        tf::Matrix3x3 m;
        m.setValue((double) pose(0, 0),
                   (double) pose(0, 1),
                   (double) pose(0, 2),
                   (double) pose(1, 0),
                   (double) pose(1, 1),
                   (double) pose(1, 2),
                   (double) pose(2, 0),
                   (double) pose(2, 1),
                   (double) pose(2, 2));

        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        geoPose.position.x = pose(0, 3);
        geoPose.position.y = pose(1, 3);
        geoPose.position.z = pose(2, 3);

        return geoPose;
    }

    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose) {
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        tf::Quaternion  q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
        tf::Matrix3x3   m(q);
        result(0, 0) = m[0][0];
        result(0, 1) = m[0][1];
        result(0, 2) = m[0][2];
        result(1, 0) = m[1][0];
        result(1, 1) = m[1][1];
        result(1, 2) = m[1][2];
        result(2, 0) = m[2][0];
        result(2, 1) = m[2][1];
        result(2, 2) = m[2][2];
        result(3, 3) = 1;

        result(0, 3) = geoPose.position.x;
        result(1, 3) = geoPose.position.y;
        result(2, 3) = geoPose.position.z;

        return result;
    }

    void parseStaticAndDynamic(
            const pcl::PointCloud<pcl::PointXYZI> &cloud, pcl::PointCloud<pcl::PointXYZI> &dynamic_points,
            pcl::PointCloud<pcl::PointXYZI> &static_points) {
        dynamic_points.points.clear();
        static_points.points.clear();

        for (const auto &pt: cloud.points) {
            uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
            uint32_t semantic_label = float2int & 0xFFFF;
            uint32_t inst_label     = float2int >> 16;

            if (find(DYNAMIC_CLASSES.begin(), DYNAMIC_CLASSES.end(),
                     semantic_label) == DYNAMIC_CLASSES.end()) {
                static_points.points.push_back(pt);
            } else {
                dynamic_points.emplace_back(pt);
            }
        }
    }

    string format(float f, int digits) {
        std::ostringstream ss;
        ss.precision(digits);
        ss << f;
        return ss.str();
    }

    bool load_labels(const std::string &label_name, std::vector<uint32_t> &labels) {
        std::ifstream label_input(label_name, std::ios::binary);
        if (!label_input.is_open()) {
            std::cerr << "Could not open the label!" << std::endl;
            return false;
        }
//        std::cout << "Load complete" << std::endl;
        label_input.seekg(0, std::ios::end);
        uint32_t num_points = label_input.tellg() / sizeof(uint32_t);
//        std::cout << num_points << std::endl;
        label_input.seekg(0, std::ios::beg);

        labels.resize(num_points);
        label_input.read((char *) &labels[0], num_points * sizeof(uint32_t));

        label_input.close();
        return true;
    }

    void findCorrespondences(const pcl::PointCloud<pcl::PointXYZI> &query_cloud,
                             const pcl::PointCloud<pcl::PointXYZI> &target_cloud,
                             vector<int>& correspondences) {

        PointCloud<num_t> cloud;
        pcl2nanoflann(target_cloud, cloud);

        my_kd_tree_t kdtree(3 /*dim*/, cloud, {10 /* max leaf */});

        correspondences.resize(query_cloud.size());
        int query_idx = 0;
        for (auto &query_pcl: query_cloud.points) {
            const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};
            size_t                num_results = 1;
            std::vector<uint32_t> ret_index(num_results);
            std::vector<num_t>    out_dist_sqr(num_results);

            num_results = kdtree.knnSearch(
                    &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

            ret_index.resize(num_results);
            out_dist_sqr.resize(num_results);

            correspondences[query_idx] = ret_index[0];
            ++query_idx;
        }
    }

    void findEmptyCorrespondences(const pcl::PointCloud<pcl::PointXYZI> &query_cloud,
                             const pcl::PointCloud<pcl::PointXYZI> &target_cloud,
                             vector<int>& correspondences, const float margin) {

        PointCloud<num_t> cloud;
        pcl2nanoflann(target_cloud, cloud);

        my_kd_tree_t kdtree(3 /*dim*/, cloud, {10 /* max leaf */});

        correspondences.reserve(query_cloud.size());
        int query_idx = 0;
        for (auto &query_pcl: query_cloud.points) {
            const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};
            size_t                num_results = 1;
            std::vector<uint32_t> ret_index(num_results);
            std::vector<num_t>    out_dist_sqr(num_results);

            num_results = kdtree.knnSearch(
                    &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

            ret_index.resize(num_results);
            out_dist_sqr.resize(num_results);

            if (out_dist_sqr[0] > margin) {
                correspondences.push_back(query_idx);
            }

            ++query_idx;
        }
    }

    void radiusSearch(const pcl::PointCloud<pcl::PointXYZI> &query_cloud, const pcl::PointCloud<pcl::PointXYZI> &target_cloud,
                     const float radius, vector<pair<int, vector<int>>>& correspondences) {
        PointCloud<num_t> cloud;
        erasor_utils::pcl2nanoflann(target_cloud, cloud);
        erasor_utils::my_kd_tree_t kdtree(3 /*dim*/, cloud, {10 /* max leaf */});

        int query_idx = 0;
        correspondences.clear();
        correspondences.resize(query_cloud.size());
        for (auto &query_pcl: query_cloud.points) {
            const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};
            std::vector<nanoflann::ResultItem<uint32_t, num_t>> ret_matches;

            int num_matched = kdtree.radiusSearch(
                    &query_pt[0], radius, ret_matches);
            vector<int> neighboring_idxes(num_matched);
            for (int i = 0; i < num_matched; ++i) {
                neighboring_idxes[i] = ret_matches[i].first;
            }
            pair<int, vector<int>> corr_pair = {query_idx, neighboring_idxes};
            correspondences[query_idx] = corr_pair;
            ++query_idx;
        }
    }

    void radiusSearch(const pcl::PointCloud<pcl::PointXYZI> &query_cloud, const pcl::PointCloud<pcl::PointXYZI> &target_cloud,
                     const float radius, vector<int>& target_idxes) {
        PointCloud<num_t> cloud;
        erasor_utils::pcl2nanoflann(target_cloud, cloud);
        erasor_utils::my_kd_tree_t kdtree(3 /*dim*/, cloud, {10 /* max leaf */});

        int N = target_cloud.size();
        vector<bool> is_neighboring(N, false);
        for (auto &query_pcl: query_cloud.points) {
            const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};
            std::vector<nanoflann::ResultItem<uint32_t, num_t>> ret_matches;

            int num_matched = kdtree.radiusSearch(
                    &query_pt[0], radius, ret_matches);

            for (int i = 0; i < num_matched; ++i) {
                is_neighboring[ret_matches[i].first] = true;
            }
        }

        target_idxes.clear();
        for (int j = 0; j < N; ++j) {
            if (is_neighboring[j]) {
                target_idxes.push_back(j);
            }
        }
    }

    void fillGTLabel(const pcl::PointCloud<pcl::PointXYZI> &gt_cloud, pcl::PointCloud<pcl::PointXYZI> &est_cloud,
                     const float margin) {
        vector<int> correspondences;
        findCorrespondences(est_cloud, gt_cloud, correspondences);
        for (int i = 0; i < correspondences.size(); ++i) {
            int correspondence = correspondences[i];
            est_cloud.points[i].intensity = gt_cloud.points[correspondence].intensity;
        }
    }

    void voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst,
                                    double leaf_size) {
        /**< IMPORTANT
         * Because PCL voxlizaiton just does average the intensity of point cloud,
         * so this function is to conduct voxelization followed by nearest points search to re-assign the label of each point */
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZI>);

        // 1. Voxelization
        static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(src);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*ptr_voxelized);

        // 2. Find nearest point to update intensity (index and id)
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(src);

        ptr_reassigned->points.reserve(ptr_voxelized->points.size());

        int K = 1;

        std::vector<int>   pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        // Set dst <- output
        for (const auto &pt: ptr_voxelized->points) {
            if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                auto updated = pt;
                // Update meaned intensity to original intensity
                updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
                ptr_reassigned->points.emplace_back(updated);
            }
        }
        dst = *ptr_reassigned;
    }

    void pcl2nanoflann(const pcl::PointCloud<pcl::PointXYZI>& src_cloud, PointCloud<num_t>& cloud) {
        int N = src_cloud.points.size();

        cloud.pts.resize(N);
        for (size_t i = 0; i < N; i++) {
            cloud.pts[i].x = src_cloud.points[i].x;
            cloud.pts[i].y = src_cloud.points[i].y;
            cloud.pts[i].z = src_cloud.points[i].z;
        }
    }

    void voxelize_preserving_labels_by_nanoflann(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
                                                 pcl::PointCloud<pcl::PointXYZI> &dst, const double leaf_size,
                                                 const int minimum_num_pts_per_voxel) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);

        // 1. Voxelization
        std::chrono::system_clock::time_point t_v_s = std::chrono::system_clock::now();
        static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(src);
        voxel_filter.setMinimumPointsNumberPerVoxel(minimum_num_pts_per_voxel);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*ptr_voxelized);
        cout << "\033[1;35m[Voxelization] " << src->points.size() << " => " << ptr_voxelized->points.size() << "\n";
        std::chrono::system_clock::time_point t_v_e = std::chrono::system_clock::now();
        // Does not work...
//        static HashVoxelGrid<pcl::PointXYZI> hash_voxel_filter;
//        hash_voxel_filter.setInputCloud(src);
//        hash_voxel_filter.setMinimumPointsNumberPerVoxel(minimum_num_pts_per_voxel);
//        hash_voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
//        hash_voxel_filter.filter(*ptr_voxelized);
//        std::chrono::system_clock::time_point t_h_e = std::chrono::system_clock::now();
//        cout << "Elapsed time in milliseconds: "
//             << chrono::duration_cast<chrono::milliseconds>(t_h_e - t_v_e).count()
//             << " ms" << endl;
//        cout << "\033[1;35m" << ptr_voxelized->points.size() << "\033[0m" << endl;

        // 2. Find nearest point to update intensity (index and id)
        vector<int> correspondences;
        findCorrespondences(*ptr_voxelized, *src, correspondences);

        for (int i = 0; i < correspondences.size(); ++i) {
            int correspondence = correspondences[i];
            ptr_voxelized->points[i].intensity = src->points[correspondence].intensity;
        }
        dst = *ptr_voxelized;
        std::chrono::system_clock::time_point t_f_e = std::chrono::system_clock::now();
        cout << "Elapsed time in milliseconds: "
             << chrono::duration_cast<chrono::milliseconds>(t_f_e - t_v_s).count()
             << " ms(" <<  chrono::duration_cast<chrono::milliseconds>(t_v_e - t_v_s).count() << " ms + ";
        cout << chrono::duration_cast<chrono::milliseconds>(t_f_e - t_v_e).count()<< " ms)\033[0m\n";
    }

    void count_stat_dyn(const pcl::PointCloud<pcl::PointXYZI> &cloudIn, int &num_static, int &num_dynamic) {
        int             tmp_static  = 0;
        int             tmp_dynamic = 0;
        for (const auto &pt: cloudIn.points) {
            uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
            uint32_t semantic_label = float2int & 0xFFFF;
            uint32_t inst_label     = float2int >> 16;
            bool     is_static      = true;
            for (int class_num: DYNAMIC_CLASSES) {
                if (semantic_label == class_num) { // 1. check it is in the moving object classes
                    is_static = false;
                }
            }
            if (is_static) {
                tmp_static++;
            } else {
                tmp_dynamic++;
            }

        }
        num_static  = tmp_static;
        num_dynamic = tmp_dynamic;
    }

    void signal_callback_handler(int signum) {
        cout << "Caught Ctrl + c " << endl;
        exit(signum);
    }

    void calcMinMaxXY(const vector<pcl::PointCloud<pcl::PointXYZI>> &pcs, float &min_x, float &min_y, float &max_x,
                      float &max_y) {
        min_x = numeric_limits<float>::max();
        max_x = numeric_limits<float>::lowest();
        min_y = numeric_limits<float>::max();
        max_y = numeric_limits<float>::lowest();

        pcl::PointXYZI  min_pt, max_pt;
        for (const auto &cloud: pcs) {
            pcl::getMinMax3D(cloud, min_pt, max_pt);
            min_x = min(min_pt.x, min_x);
            min_y = min(min_pt.y, min_y);
            max_x = max(max_pt.x, max_x);
            max_y = max(max_pt.y, max_y);
//            cout << min_x << " <-> " << max_x << endl;
//            cout << min_y << " <-> " << max_y << endl;
        }
    }

    void calcMinMaxZ(const pcl::PointCloud<pcl::PointXYZI> &pcs, float &min_z, float &max_z) {
        min_z       = numeric_limits<float>::max();
        max_z       = numeric_limits<float>::lowest();

        int num_pts = pcs.points.size();
        for (size_t i = 0; i < num_pts; i++) {
            const auto &pt = pcs.points.at(i);
            if (pt.intensity != NOT_VOLUME_OF_INTEREST) {
                min_z = min(min_z, pt.z);
                max_z = max(max_z, pt.z);
            }
        }
    }

    void calcMinMaxZWithoutGround(const pcl::PointCloud<pcl::PointXYZI> &pcs, float &min_z, float &max_z) {
        min_z       = numeric_limits<float>::max();
        max_z       = numeric_limits<float>::lowest();

        int num_pts = pcs.points.size();
        for (size_t i = 0; i < num_pts; i++) {
            const auto &pt = pcs.points.at(i);
            if (pt.intensity != GROUND_LABEL && pt.intensity != NOT_VOLUME_OF_INTEREST) {
                min_z = min(min_z, pt.z);
                max_z = max(max_z, pt.z);
            }
        }
    }

    float calcMeanZOfGround(const pcl::PointCloud<pcl::PointXYZI>& pcs) {
        float sum_z = 0;
        int num_pts = pcs.points.size();
        for (size_t i = 0; i < num_pts; i++) {
            const auto &pt = pcs.points.at(i);
            if (pt.intensity == GROUND_LABEL) {
                sum_z += pt.z;
            }
        }
        int num_ground_pts = getNumGroundPoints(pcs);
        return sum_z / num_ground_pts;
    }

    int getNumGroundPoints(const pcl::PointCloud<pcl::PointXYZI>& pc) {
        int             num_ground_pts = 0;
        for (const auto &pt: pc) {
            if (pt.intensity == GROUND_LABEL) {
                ++num_ground_pts;
            }
        }
        return num_ground_pts;
    }
}