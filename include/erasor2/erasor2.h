#include "tools/erasor_utils.hpp"
#include "rosparam_server.hpp"

#define INF 10000000000000.0
#define ENOUGH_NUM 8000

// COLORS:
// 0 -> BLUE
#define MAP_IS_HIGHER 0.5
#define CURR_IS_HIGHER 1.0
#define LITTLE_NUM 0.0       // For viz: blue - not activated
#define BLOCKED 0.8         // For viz

#define MERGE_BINS 0.25
#define NOT_ASSIGNED 0.0
// ground params


using namespace std;

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
    vector<pcl::PointCloud<pcl::PointXYZI>> pcs_gt_transformed_;
    vector<pcl::PointCloud<pcl::PointXYZI>> static_points_transformed_;
    vector<Eigen::Matrix4f>                 poses_submap_;

    vector<vector<pcl::PointCloud<pcl::PointXYZI>>> xygrids_;
    vector<grid_map::Index>                         idxes_approx_;

    vector<vector<float>> dyn_ids_set_;

    int               num_data_ = 0;
    GridMapInfo       grid_map_info_;
    grid_map::GridMap gridmap_submap_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_dynamic_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_accum_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_complement_;

    // Final output
    pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_accum_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_voxelized_;

    void initializePointClouds();

    // Main functions
    void setScanAndPose(const Eigen::Matrix4f &pose_raw,
                        const pcl::PointCloud<pcl::PointXYZI> &cloud_est_label);

    void setScanAndPose(const Eigen::Matrix4f& pose_raw,
                             const pcl::PointCloud<pcl::PointXYZI>& cloud_gt_label,
                             const pcl::PointCloud<pcl::PointXYZI>& cloud_est_label);
    void setSubmap();

    void resize();

    void updateSteppableRegion();

    void detectDynamicObjects();

    void estimateStaticMask(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<float>& dyn_ids,
                      std::vector<bool>& static_mask);

    void discernStaticAndDynamicPoints(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<bool>& static_mask,
                              pcl::PointCloud<pcl::PointXYZI>& static_points, pcl::PointCloud<pcl::PointXYZI>& dynamic_points);

    void discernStaticAndDynamicPoints(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<float>& dyn_ids,
                              pcl::PointCloud<pcl::PointXYZI>& static_points, pcl::PointCloud<pcl::PointXYZI>& dynamic_points);
    void saveStaticMap(const string& static_map_path);

    void publishStaticMapResults();

    void maskNonVoI(const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& cloud_out,
                const float min_z_voi, const float max_z_voi);

    nav_msgs::OccupancyGrid setOccupancyGridMap(const float min_x, const float min_y,
                                                const float max_x, const float max_y,
                                                const float occugrid_resolution);

    GridMapInfo setGridMapParams(const float min_x, const float min_y,
                                 const float max_x, const float max_y,
                                 const float grid_resolution);

    void voi2xygrid(
            const pcl::PointCloud<pcl::PointXYZI> &src, float pos_x, float pos_y, float pos_z,
            float range, float resolution, vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid,
            pcl::PointCloud<pcl::PointXYZI> &complement, std::string format = "gridmap");

    void xygrid2cloud(const vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid,
            pcl::PointCloud<pcl::PointXYZI> &cloud);

    bool isLikelyToBeGround(const pcl::PointCloud<pcl::PointXYZI> pc, const float ratio_num=0.95,
                                 const int num_min_pts=3);

    bool isLikelyToBeSteppableRegion(const pcl::PointCloud<pcl::PointXYZI> &curr_pc,
                                     const pcl::PointCloud<pcl::PointXYZI> &map_pc,
                                     const float scan_ratio_threshold, const float th_bin_max_h,
                                     const bool verbose = false);

    grid_map::GridMap setMapcentricGridMap(const GridMapInfo &grid_map_info);

    grid_map::GridMap setEgocentricGridMap(float range,
                                           const float grid_resolution,
                                           const vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid);

    void dilateAndErode(grid_map::GridMap &gridmap_submap);


private:
    std::vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};

    double xy2theta(const double &x, const double &y);

    double xy2radius(const double &x, const double &y);

    geometry_msgs::PolygonStamped set_polygons(int r_idx, int theta_idx, int num_split = 3);
};


