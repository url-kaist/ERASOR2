#include "tools/erasor_utils.hpp"
#include "rosparam_server.hpp"

using namespace std;

struct DynamicCluster {
    pcl::PointCloud<pcl::PointXYZI> cloud_;
    float moving_obj_score_;
    Eigen::Matrix<float, 4, 1> centroid_;
    bool is_dynamic_;
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

    vector<vector<pcl::PointCloud<pcl::PointXYZI>>>          xygrids_;
    vector<grid_map::Index>                                  idxes_approx_;
//    vector<vector<float>>                                    dyn_ids_set_;
    vector<vector<float>>                        dynamic_ids_set_;
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

    void detectDynamicObjects();

    void filterDynamicObjects();

    void estimateStaticMask(const pcl::PointCloud<pcl::PointXYZI> &cloud, const std::vector<float> &dyn_ids,
                            std::vector<int> &static_mask);

    void updateNoisyMask(const pcl::PointCloud<pcl::PointXYZI> &src_cloud,
                         const pcl::PointCloud<pcl::PointXYZI> &noisy_points,
                         std::vector<int> &static_mask);

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

    float calcMovingClusterScore(const pcl::PointCloud<pcl::PointXYZI>& dynamic_obj);

    void dilateAndErode(grid_map::GridMap &gridmap_submap);

    void erodeGridMap(grid_map::GridMap &gridmap_submap);

    void publishObjScores(const ros::Publisher& publisher, const vector<pair<Eigen::Matrix<float, 4, 1>, float> >& objs,
                               const vector<float> color, int& num_prev_objs);

    float getAdaptiveThreshold(const float obj_x, const float obj_y,
                             const float pos_x, const float pos_y, const float adaptive_range);

    void visualizeAdaptiveRange(const Eigen::Matrix4f& pose);

private:
    std::vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};

    double xy2theta(const double &x, const double &y);

    double xy2radius(const double &x, const double &y);

    geometry_msgs::PolygonStamped set_polygons(int r_idx, int theta_idx, int num_split = 3);
};


