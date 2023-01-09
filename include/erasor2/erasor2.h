#include "tools/erasor_utils.hpp"
#include "rosparam_server.hpp"

#define IS_DEFINITELY_MOVING_OBJ 0.999999 // <-> 14 in log-odds

using namespace std;

struct DynamicCluster {
    pcl::PointCloud<pcl::PointXYZI> cloud_;
    float moving_obj_score_;
    Eigen::Matrix<float, 4, 1> centroid_;
    bool is_close_to_body_frame_ = false;
    bool is_dynamic_;
    vector<grid_map::Index> occupied_map_idxes_;
};

struct GridMapInfo {
    float center_x;
    float center_y;
    float resolution;
    int   width;
    int   height;
    // Remainders of x_length / grid_resolution and
    // y_length / grid_resolution should be zeros, respectively.
    float x_length;
    float y_length;
};

class ERASOR2 : public RosParamServer {
public:
    ERASOR2();

    ~ERASOR2();

    bool                                    is_initial_ = true;
    Eigen::Matrix4f                         new_origin_ = Eigen::Matrix4f::Identity();
    vector<pcl::PointCloud<pcl::PointXYZI>> pcs_transformed_;
    vector<pcl::PointCloud<pcl::PointXYZI>> complements_transformed_;
    vector<pcl::PointCloud<pcl::PointXYZI>> pcs_gt_transformed_;
    vector<Eigen::Matrix4f>                 poses_submap_;

    // Outputs
    vector<pcl::PointCloud<pcl::PointXYZI>> noisy_points_transformed_;
    vector<pcl::PointCloud<pcl::PointXYZI>> static_points_transformed_;
    vector<pcl::PointCloud<pcl::PointXYZI>> dynamic_points_transformed_;
    vector<pcl::PointCloud<pcl::PointXYZI>> potential_dynamic_points_transformed_;

    vector<vector<pcl::PointCloud<pcl::PointXYZI>>>          xygrids_;
    vector<grid_map::Index>                                  idxes_approx_;
    vector<unordered_map<float, DynamicCluster>> dynamic_ids_clusters_set_;

    // For visualize the moving object score
    vector<vector<pair<Eigen::Matrix<float, 4, 1>, float> >> rejected_objs_set_;
    vector<vector<pair<Eigen::Matrix<float, 4, 1>, float> >> accepted_objs_set_;

    // To flush markers. These are used in `publishObjScores`
    int num_prev_accepted_objs_ = 0;
    int num_prev_rejected_objs_ = 0;

    int               num_data_ = 0;
    GridMapInfo       grid_map_info_;
    grid_map::GridMap gridmap_submap_;

    float scan_ratio_;
    float ratio_num_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_noise_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_dynamic_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_accum_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_complement_;

    // Final output
    pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_accum_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_voxelized_;

    void setPriors();

    void initializePointClouds();

    // Main functions
    void setScanAndPose(const Eigen::Matrix4f &pose_raw,
                        const pcl::PointCloud<pcl::PointXYZI> &cloud_est_label);

    void setScanAndPose(const Eigen::Matrix4f &pose_raw,
                        const pcl::PointCloud<pcl::PointXYZI> &cloud_gt_label,
                        const pcl::PointCloud<pcl::PointXYZI> &cloud_est_label);

    void setSubmap();

    void resize();

    void updateSteppableRegion();

    void detectMovingObjects();

    void filterDynamicObjects();

    void estimateStaticMask(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                            const unordered_map<float, DynamicCluster> &ids_clusters,
                            std::vector<int> &static_mask);

    void updateNoisyMask(const pcl::PointCloud<pcl::PointXYZI> &src_cloud,
                         const pcl::PointCloud<pcl::PointXYZI> &noisy_points,
                         std::vector<int> &static_mask);

    void updateNeighboringDynamicMask(const erasor_utils::my_kd_tree_t& kdtree,
                                        const pcl::PointCloud<pcl::PointXYZI> &query_points,
                                        std::vector<int> &static_mask);

    void setAccumDynamicPoints(const int k, const int window_size,
                               pcl::PointCloud<pcl::PointXYZI> &cloud_accum, bool use_voxelization=true);

    void instanceAwareOutlierRemoval(const int k, const int window_size,
                                           const float dist_thr_gain,
                                           pcl::PointCloud<pcl::PointXYZI> &filtered_static_points,
                                           pcl::PointCloud<pcl::PointXYZI> &potential_dynamic_points);

    void windowBasedVolumetricOutlierRemoval(const int k, const int window_size,
                                           const float dist_thr_gain,
                                           pcl::PointCloud<pcl::PointXYZI> &filtered_static_points,
                                           pcl::PointCloud<pcl::PointXYZI> &potential_dynamic_points);

    void volumetricOutlierRemoval(const pcl::PointCloud<pcl::PointXYZI> &static_points,
                                       const pcl::PointCloud<pcl::PointXYZI> &dynamic_points,
                                       const float dist_thr_gain,
                                       pcl::PointCloud<pcl::PointXYZI> &filtered_static_points,
                                       pcl::PointCloud<pcl::PointXYZI> &potential_dynamic_points);

    void
    discernStaticAndDynamicPoints(const pcl::PointCloud<pcl::PointXYZI> &cloud, const std::vector<int> &static_mask,
                                  pcl::PointCloud<pcl::PointXYZI> &static_points,
                                  pcl::PointCloud<pcl::PointXYZI> &dynamic_points);

//    void discernStaticAndDynamicPoints(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<int>& dyn_ids,
//                              pcl::PointCloud<pcl::PointXYZI>& static_points, pcl::PointCloud<pcl::PointXYZI>& dynamic_points);

    void saveStaticMap(const string &static_map_path);

    void publishStaticMapResults();

    void maskNonVoI(const pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &cloud_out,
                    const float min_z_voi, const float max_z_voi);

    GridMapInfo setGridMapParams(const float min_x, const float min_y,
                                 const float max_x, const float max_y,
                                 const float grid_resolution);

    void voi2xygrid(
            const pcl::PointCloud<pcl::PointXYZI> &src, float pos_x, float pos_y, float pos_z,
            float range, float resolution, vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid,
            pcl::PointCloud<pcl::PointXYZI> &complement, std::string format = "gridmap");

    void xygrid2cloud(const vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid,
                      pcl::PointCloud<pcl::PointXYZI> &cloud);


    bool isLikelyToBeGround(const pcl::PointCloud<pcl::PointXYZI>& pc, const float ratio_num = 0.95,
                            const int num_min_pts = 3);

    bool isLikelyToBeSteppableRegion(const pcl::PointCloud<pcl::PointXYZI> &curr_pc,
                                     const pcl::PointCloud<pcl::PointXYZI> &map_pc,
                                     const float scan_ratio_threshold, const float th_bin_max_h,
                                     const bool verbose = false);

    void updatePrior(const grid_map::Index& idx, const float prior);

    void updatePosterior(const grid_map::Index& idx, const float increment, const int kernel_size=3);

    grid_map::GridMap setMapcentricGridMap(const GridMapInfo &grid_map_info);

    grid_map::GridMap setEgocentricGridMap(float range,
                                           const float grid_resolution,
                                           const vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid);

    float calcMovingClusterScore(const pcl::PointCloud<pcl::PointXYZI> &dynamic_cluster,
                                      vector<grid_map::Index>& occupied_map_idxes);

    void dilateAndErode(grid_map::GridMap &gridmap_submap);

    void erodeGridMap(grid_map::GridMap &gridmap_submap);

    void publishObjScores(const ros::Publisher& publisher, const vector<pair<Eigen::Matrix<float, 4, 1>, float> >& objs,
                               const vector<float> color, int& num_prev_objs);

    bool isCloseToSensorFrame(const DynamicCluster& dynamic_cluster, const float pos_x, const float pos_y,
                              const float range_thr);

    bool isSizeSufficientlySmall(const pcl::PointCloud<pcl::PointXYZI> &dynamic_cluster,
                                 const float size_thr=30.0);

    void visualizeHardThrRadius(const Eigen::Matrix4f& pose);

private:
    double xy2theta(const double &x, const double &y);

    double xy2radius(const double &x, const double &y);

    double prob2logOdds(double prob);

    double logOdds2prob(double log_odds);

    grid_map::Position idx2position(const grid_map::Index& idx);

    int globalIdx2LocalIdx(const grid_map::Index& global_idx,
                           const grid_map::Index& center_idx);

    bool isEqual(const grid_map::Index& idx0, const grid_map::Index& idx1);

    bool isInsideTheDynamicClusters(const pcl::PointXYZI& query, const pcl::PointXYZI& target);

    void printClusterInfo(const DynamicCluster& dynamic_cluster);
};


