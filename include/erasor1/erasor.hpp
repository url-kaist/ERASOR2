#ifndef ERASOR1_ERASOR_HPP
#define ERASOR1_ERASOR_HPP

// Port of github.com/LimHyungTae/ERASOR's ERASOR class with ROS stripped.
// The algorithm (polar-grid pseudo-occupancy comparison + ground revert) is
// preserved bit-for-bit; only the ros::NodeHandle ctor parameter feed,
// the ros::Publisher members, and the jsk PolygonArray viz outputs were
// replaced with a plain config struct + our rerun adapter (which the
// orchestrator wires up).

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <vector>

namespace erasor1 {

constexpr double INF = 1.0e13;
constexpr double PI  = 3.1415926535;

// status codes used inside the R-POD bin grid
constexpr int EMPTY   = 0;
constexpr int MAP     = 1;
constexpr int PC_CURR = 2;

constexpr double MAP_IS_HIGHER  = 0.5;
constexpr double CURR_IS_HIGHER = 1.0;
constexpr double LITTLE_NUM     = 0.0;
constexpr double BLOCKED        = 0.8;
constexpr double MERGE_BINS     = 0.25;
constexpr double NOT_ASSIGNED   = 0.0;

struct Bin {
  double max_h;
  double min_h;
  double x;
  double y;
  double status;
  bool   is_occupied;
  pcl::PointCloud<pcl::PointXYZI> points;
};

using R_POD = std::vector<std::vector<Bin>>;  // [ring][sector]
using Ring  = std::vector<Bin>;

// All algorithm parameters previously fed by `nh.param("/erasor/...", ...)`.
struct Params {
  double max_range            = 60.0;
  int    num_rings            = 20;
  int    num_sectors          = 108;
  double max_h                = 1.5;
  double min_h                = -3.0;
  double th_bin_max_h         = 0.39;
  double scan_ratio_threshold = 0.22;
  int    num_lowest_pts       = 5;
  int    minimum_num_pts      = 4;
  double rejection_ratio      = 0.33;
  double gf_dist_thr          = 0.05;
  int    gf_iter              = 3;
  int    gf_num_lpr           = 10;
  double gf_th_seeds_height   = 0.5;
  double map_voxel_size       = 0.2;
};

class ERASOR {
 public:
  explicit ERASOR(const Params& params);
  ~ERASOR() = default;

  // Inputs: VoIs already cut + transformed into the body/egocentric frame.
  void set_inputs(const pcl::PointCloud<pcl::PointXYZI>& map_voi,
                  const pcl::PointCloud<pcl::PointXYZI>& query_voi);

  // The "v2 algorithm" from the original repo (with-ground-revert).
  void compare_vois_and_revert_ground(int frame);

  // Pull the filtered static map back out of the bin grid.
  void get_static_estimate(pcl::PointCloud<pcl::PointXYZI>& arranged,
                           pcl::PointCloud<pcl::PointXYZI>& complement);

  void get_outliers(pcl::PointCloud<pcl::PointXYZI>& map_rejected,
                    pcl::PointCloud<pcl::PointXYZI>& curr_rejected);

  // Reverted ground points (filled by compare_vois_and_revert_ground).
  pcl::PointCloud<pcl::PointXYZI> ground_viz;
  pcl::PointCloud<pcl::PointXYZI> debug_curr_rejected;
  pcl::PointCloud<pcl::PointXYZI> debug_map_rejected;
  pcl::PointCloud<pcl::PointXYZI> map_complement;

  R_POD r_pod_map;
  R_POD r_pod_curr;
  R_POD r_pod_selected;

  double get_max_range() const { return max_r; }

 private:
  // Polar-grid parameters
  double max_r;
  int    num_rings;
  int    num_sectors;
  int    num_lowest_pts;
  double ring_size;
  double sector_size;

  double min_h;
  double max_h;
  double scan_ratio_threshold;
  double th_bin_max_h;
  int    minimum_num_pts;
  double rejection_ratio;
  double map_voxel_size_;

  // Ground-extraction parameters
  double th_dist_;
  int    iter_groundfilter_;
  int    num_lprs_;
  double th_seeds_heights_;

  Eigen::MatrixXf normal_;
  double          th_dist_d_;
  double          d_;

  pcl::PointCloud<pcl::PointXYZI> piecewise_ground_, non_ground_;
  pcl::PointCloud<pcl::PointXYZI> ground_pc_, non_ground_pc_;

  // Polar helpers
  double xy2theta(double x, double y) const;
  double xy2radius(double x, double y) const;

  void init(R_POD& r_pod);
  void clear_bin(Bin& bin);
  void clear(pcl::PointCloud<pcl::PointXYZI>& pt_cloud);

  void pt2r_pod(const pcl::PointXYZI& pt, Bin& bin);
  void voi2r_pod(const pcl::PointCloud<pcl::PointXYZI>& src, R_POD& r_pod);
  void voi2r_pod(const pcl::PointCloud<pcl::PointXYZI>& src,
                 R_POD&                                  r_pod,
                 pcl::PointCloud<pcl::PointXYZI>&        complement);

  // Ground extraction
  void estimate_plane_(const pcl::PointCloud<pcl::PointXYZI>& ground);
  void extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZI>& p_sorted,
                              pcl::PointCloud<pcl::PointXYZI>&       init_seeds);
  void extract_ground(const pcl::PointCloud<pcl::PointXYZI>& src,
                      pcl::PointCloud<pcl::PointXYZI>&       dst,
                      pcl::PointCloud<pcl::PointXYZI>&       outliers);

  bool has_dynamic(Bin& bin);
  void merge_bins(const Bin& src1, const Bin& src2, Bin& dst);
  void r_pod2pc(const R_POD& sc, pcl::PointCloud<pcl::PointXYZI>& pc);
};

}  // namespace erasor1

#endif  // ERASOR1_ERASOR_HPP
