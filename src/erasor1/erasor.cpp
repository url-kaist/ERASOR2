// Port of github.com/LimHyungTae/ERASOR's src/offline_map_updater/src/erasor.cpp.
// ROS publisher / message-construction lines were stripped; pure algorithm
// logic is preserved verbatim so the polar-grid pseudo-occupancy comparison
// produces numerically identical results.

#include "erasor1/erasor.hpp"

#include "tools/erasor_utils.hpp"

#include <pcl/common/centroid.h>

#include <algorithm>
#include <cmath>

namespace erasor1 {

namespace {

constexpr int ENOUGH_NUM = 8000;

// SemanticKITTI dynamic-object class IDs (cars, persons, etc.).
constexpr int kDynamicClasses[] = {252, 253, 254, 255, 256, 257, 258, 259};

bool point_cmp(const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
  return a.z < b.z;
}

}  // namespace

ERASOR::ERASOR(const Params& p)
    : max_r(p.max_range),
      num_rings(p.num_rings),
      num_sectors(p.num_sectors),
      num_lowest_pts(p.num_lowest_pts),
      min_h(p.min_h),
      max_h(p.max_h),
      scan_ratio_threshold(p.scan_ratio_threshold),
      th_bin_max_h(p.th_bin_max_h),
      minimum_num_pts(p.minimum_num_pts),
      rejection_ratio(p.rejection_ratio),
      map_voxel_size_(p.map_voxel_size),
      th_dist_(p.gf_dist_thr),
      iter_groundfilter_(p.gf_iter),
      num_lprs_(p.gf_num_lpr),
      th_seeds_heights_(p.gf_th_seeds_height) {
  ring_size   = max_r / num_rings;
  sector_size = 2 * PI / num_sectors;

  init(r_pod_map);
  init(r_pod_curr);
  init(r_pod_selected);

  piecewise_ground_.reserve(130000);
  non_ground_.reserve(130000);
  ground_pc_.reserve(130000);
  non_ground_pc_.reserve(130000);

  std::cout << "-----\033[1;32mParams. of ERASOR (v1)\033[0m-----\n"
            << "  max range            : " << max_r << "\n"
            << "  num_rings            : " << num_rings << "\n"
            << "  num_sectors          : " << num_sectors << "\n"
            << "  min_h / max_h        : " << min_h << " / " << max_h << "\n"
            << "  scan_ratio_threshold : " << scan_ratio_threshold << "\n"
            << "  th_bin_max_h         : " << th_bin_max_h << "\n"
            << "  minimum_num_pts      : " << minimum_num_pts << "\n"
            << "-------------------------------\n";
}

double ERASOR::xy2theta(double x, double y) const {
  if (y >= 0) return std::atan2(y, x);
  return 2 * PI + std::atan2(y, x);
}

double ERASOR::xy2radius(double x, double y) const {
  return std::sqrt(x * x + y * y);
}

void ERASOR::clear(pcl::PointCloud<pcl::PointXYZI>& pt_cloud) {
  if (!pt_cloud.empty()) pt_cloud.clear();
}

void ERASOR::init(R_POD& r_pod) {
  if (!r_pod.empty()) r_pod.clear();
  Ring ring;
  Bin  bin = {-INF, INF, 0, 0, NOT_ASSIGNED, false};
  bin.points.reserve(ENOUGH_NUM);
  for (int i = 0; i < num_sectors; i++) ring.emplace_back(bin);
  for (int j = 0; j < num_rings; j++) r_pod.emplace_back(ring);
}

void ERASOR::clear_bin(Bin& bin) {
  bin.max_h       = -INF;
  bin.min_h       = INF;
  bin.x           = 0;
  bin.y           = 0;
  bin.is_occupied = false;
  bin.status      = NOT_ASSIGNED;
  if (!bin.points.empty()) bin.points.clear();
}

void ERASOR::set_inputs(const pcl::PointCloud<pcl::PointXYZI>& map_voi,
                        const pcl::PointCloud<pcl::PointXYZI>& query_voi) {
  clear(debug_curr_rejected);
  clear(debug_map_rejected);
  clear(map_complement);

  for (int theta = 0; theta < num_sectors; ++theta) {
    for (int r = 0; r < num_rings; ++r) {
      clear_bin(r_pod_map[r][theta]);
      clear_bin(r_pod_curr[r][theta]);
      clear_bin(r_pod_selected[r][theta]);
    }
  }
  voi2r_pod(query_voi, r_pod_curr);
  voi2r_pod(map_voi, r_pod_map, map_complement);
}

void ERASOR::pt2r_pod(const pcl::PointXYZI& pt, Bin& bin) {
  bin.is_occupied = true;
  bin.points.push_back(pt);
  if (pt.z >= bin.max_h) {
    bin.max_h = pt.z;
    bin.x     = pt.x;
    bin.y     = pt.y;
  }
  if (pt.z <= bin.min_h) bin.min_h = pt.z;
}

void ERASOR::voi2r_pod(const pcl::PointCloud<pcl::PointXYZI>& src, R_POD& r_pod) {
  for (const auto& pt : src.points) {
    if (pt.z < max_h && pt.z > min_h) {
      double r = xy2radius(pt.x, pt.y);
      if (r <= max_r) {
        double theta      = xy2theta(pt.x, pt.y);
        int    sector_idx = std::min(static_cast<int>(theta / sector_size), num_sectors - 1);
        int    ring_idx   = std::min(static_cast<int>(r / ring_size), num_rings - 1);
        pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));
      }
    }
  }
}

void ERASOR::voi2r_pod(const pcl::PointCloud<pcl::PointXYZI>& src,
                       R_POD&                                  r_pod,
                       pcl::PointCloud<pcl::PointXYZI>&        complement) {
  for (const auto& pt : src.points) {
    if (pt.z < max_h && pt.z > min_h) {
      double r = xy2radius(pt.x, pt.y);
      if (r <= max_r) {
        double theta      = xy2theta(pt.x, pt.y);
        int    sector_idx = std::min(static_cast<int>(theta / sector_size), num_sectors - 1);
        int    ring_idx   = std::min(static_cast<int>(r / ring_size), num_rings - 1);
        pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));
      } else {
        complement.points.push_back(pt);
      }
    } else {
      complement.points.push_back(pt);
    }
  }
}

void ERASOR::estimate_plane_(const pcl::PointCloud<pcl::PointXYZI>& ground) {
  Eigen::Matrix3f cov;
  Eigen::Vector4f pc_mean;
  pcl::computeMeanAndCovarianceMatrix(ground, cov, pc_mean);
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  normal_                    = svd.matrixU().col(2);
  Eigen::Vector3f seeds_mean = pc_mean.head<3>();
  d_                         = -(normal_.transpose() * seeds_mean)(0, 0);
  th_dist_d_                 = th_dist_ - d_;
}

void ERASOR::extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZI>& p_sorted,
                                    pcl::PointCloud<pcl::PointXYZI>&       init_seeds) {
  init_seeds.points.clear();
  pcl::PointCloud<pcl::PointXYZI> g_seeds_pc;

  double sum = 0;
  int    cnt = 0;
  for (size_t i = num_lowest_pts; i < p_sorted.points.size() && cnt < num_lprs_; i++) {
    sum += p_sorted.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;
  g_seeds_pc.clear();
  for (const auto& pt : p_sorted.points) {
    if (pt.z < lpr_height + th_seeds_heights_) g_seeds_pc.points.push_back(pt);
  }
  init_seeds = g_seeds_pc;
}

void ERASOR::extract_ground(const pcl::PointCloud<pcl::PointXYZI>& src,
                            pcl::PointCloud<pcl::PointXYZI>&       dst,
                            pcl::PointCloud<pcl::PointXYZI>&       outliers) {
  if (!dst.empty()) dst.clear();
  if (!outliers.empty()) outliers.clear();

  auto src_copy = src;
  std::sort(src_copy.points.begin(), src_copy.points.end(), point_cmp);
  auto it = src_copy.points.begin();
  for (size_t i = 0; i < src_copy.points.size(); i++) {
    if (src_copy.points[i].z < min_h)
      ++it;
    else
      break;
  }
  src_copy.points.erase(src_copy.points.begin(), it);

  if (!ground_pc_.empty()) ground_pc_.clear();
  if (!non_ground_pc_.empty()) non_ground_pc_.clear();

  extract_initial_seeds_(src_copy, ground_pc_);
  for (int i = 0; i < iter_groundfilter_; i++) {
    estimate_plane_(ground_pc_);
    ground_pc_.clear();

    Eigen::MatrixXf points(src.points.size(), 3);
    int             j = 0;
    for (const auto& p : src.points) points.row(j++) << p.x, p.y, p.z;
    Eigen::VectorXf result = points * normal_;
    for (int        r = 0; r < result.rows(); r++) {
      if (result[r] < th_dist_d_) {
        ground_pc_.points.push_back(src[r]);
      } else if (i == (iter_groundfilter_ - 1)) {
        non_ground_pc_.points.push_back(src[r]);
      }
    }
  }
  dst      = ground_pc_;
  outliers = non_ground_pc_;
}

void ERASOR::merge_bins(const Bin& src1, const Bin& src2, Bin& dst) {
  dst.max_h       = std::max(src1.max_h, src2.max_h);
  dst.min_h       = std::min(src1.min_h, src2.min_h);
  dst.is_occupied = true;
  dst.points.clear();
  for (const auto& pt : src1.points) dst.points.push_back(pt);
  for (const auto& pt : src2.points) dst.points.push_back(pt);
}

void ERASOR::r_pod2pc(const R_POD& sc, pcl::PointCloud<pcl::PointXYZI>& pc) {
  pc.points.clear();
  for (int theta = 0; theta < num_sectors; theta++) {
    for (int r = 0; r < num_rings; r++) {
      if (sc.at(r).at(theta).is_occupied) {
        for (const auto& pt : sc.at(r).at(theta).points) pc.points.push_back(pt);
      }
    }
  }
}

void ERASOR::get_outliers(pcl::PointCloud<pcl::PointXYZI>& map_rejected,
                          pcl::PointCloud<pcl::PointXYZI>& curr_rejected) {
  map_rejected  = debug_map_rejected;
  curr_rejected = debug_curr_rejected;
}

bool ERASOR::has_dynamic(Bin& bin) {
  for (const auto& pt : bin.points) {
    uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
    uint32_t semantic_label = float2int & 0xFFFF;
    for (int class_num : kDynamicClasses) {
      if (semantic_label == static_cast<uint32_t>(class_num)) return true;
    }
  }
  return false;
}

// Version 2 of the original algorithm: scan-ratio test + ground-revert via
// R-GPF on bins flagged as map-dominant. This is the path the v1 paper
// numbers were measured under.
void ERASOR::compare_vois_and_revert_ground(int /*frame*/) {
  ground_viz.points.clear();

  for (int theta = 0; theta < num_sectors; theta++) {
    for (int r = 0; r < num_rings; r++) {
      Bin& bin_curr = r_pod_curr[r][theta];
      Bin& bin_map  = r_pod_map[r][theta];

      if (static_cast<int>(bin_curr.points.size()) < minimum_num_pts) {
        r_pod_selected[r][theta] = bin_map;
        continue;
      }
      if (bin_curr.is_occupied && bin_map.is_occupied) {
        double map_h_diff  = bin_map.max_h - bin_map.min_h;
        double curr_h_diff = bin_curr.max_h - bin_curr.min_h;
        double scan_ratio  = std::min(map_h_diff / curr_h_diff, curr_h_diff / map_h_diff);

        if (scan_ratio < scan_ratio_threshold) {        // find dynamic!
          if (map_h_diff >= curr_h_diff) {              // map-occupied -> disappeared
            if (bin_map.max_h > th_bin_max_h) {
              r_pod_selected[r][theta] = bin_curr;
              // R-GPF on the rejected map bin: any ground points get reverted.
              if (!piecewise_ground_.empty()) piecewise_ground_.clear();
              if (!non_ground_.empty()) non_ground_.clear();
              extract_ground(bin_map.points, piecewise_ground_, non_ground_);
              r_pod_selected[r][theta].points += piecewise_ground_;
              ground_viz += piecewise_ground_;
              debug_map_rejected += non_ground_;
            } else {
              r_pod_selected[r][theta] = bin_map;
            }
          } else if (map_h_diff <= curr_h_diff) {       // map-free -> object appeared
            r_pod_selected[r][theta] = bin_map;
            if (bin_curr.max_h > th_bin_max_h) {
              debug_curr_rejected += bin_curr.points;
            }
          }
        } else {
          Bin bin_merged;
          merge_bins(bin_curr, bin_map, bin_merged);
          r_pod_selected[r][theta] = bin_merged;
        }
      } else if (bin_curr.is_occupied) {
        r_pod_selected[r][theta] = bin_curr;
      } else if (bin_map.is_occupied) {
        r_pod_selected[r][theta] = bin_map;
      }
    }
  }
}

// Lifted from upstream erasor.cpp:573-595 (ROS-free). Walks a small
// neighbourhood of `r_pod_selected` looking for any bin currently flagged
// CURR_IS_HIGHER (a dynamic object). Used by v3 to suppress merge_bins on
// otherwise-static cells whose dynamic neighbour might be polluting the
// scan-ratio test.
bool ERASOR::is_dynamic_obj_close(R_POD& r_pod_selected,
                                  int    r_target,
                                  int    theta_target,
                                  int    r_range,
                                  int    theta_range) {
  std::vector<int> theta_candidates;
  for (int j = theta_target - theta_range; j <= theta_target + theta_range; j++) {
    // Upstream uses `num_rings` here to wrap theta. Preserve that quirk
    // bit-for-bit so port numerically matches.
    if (j < 0) {
      theta_candidates.push_back(j + num_rings);
    } else if (j >= num_sectors) {
      theta_candidates.push_back(j - num_rings);
    } else {
      theta_candidates.push_back(j);
    }
  }
  for (int r = std::max(0, r_target - r_range);
       r <= std::min(r_target + r_range, num_rings - 1);
       r++) {
    for (const auto& theta : theta_candidates) {
      if (r == r_target && theta == theta_target) continue;
      if (r_pod_selected[r][theta].status == CURR_IS_HIGHER) return true;
    }
  }
  return false;
}

// Lifted from upstream erasor.cpp:438-571 (ROS-free; jsk PolygonArray and
// sensor_msgs publishes dropped; the per-bin voxelization swap from
// `voxelize_preserving_labels` to `voxelize_preserving_labels_by_nanoflann`
// is the *only* non-mechanical change -- PCL's int-indexed VoxelGrid
// integer-overflows at fine leaves over a 200m-extent map).
void ERASOR::compare_vois_and_revert_ground_w_block(int /*frame*/) {
  ground_viz.points.clear();

  // ----- Pass 1: tag bin status based on scan-ratio test --------------
  for (int theta = 0; theta < num_sectors; theta++) {
    for (int r = 0; r < num_rings; r++) {
      Bin& bin_curr = r_pod_curr[r][theta];
      Bin& bin_map  = r_pod_map[r][theta];

      if (bin_map.points.empty()) {
        r_pod_selected[r][theta].status = LITTLE_NUM;
        continue;
      }

      if (static_cast<int>(bin_curr.points.size()) < minimum_num_pts) {
        r_pod_selected[r][theta].status = LITTLE_NUM;
      } else {
        double map_h_diff  = bin_map.max_h - bin_map.min_h;
        double curr_h_diff = bin_curr.max_h - bin_curr.min_h;
        double scan_ratio  = std::min(map_h_diff / curr_h_diff, curr_h_diff / map_h_diff);
        if (bin_curr.is_occupied && bin_map.is_occupied) {
          if (scan_ratio < scan_ratio_threshold) {
            if (map_h_diff >= curr_h_diff) {
              r_pod_selected[r][theta].status = MAP_IS_HIGHER;
            } else if (map_h_diff <= curr_h_diff) {
              r_pod_selected[r][theta].status = CURR_IS_HIGHER;
            }
          } else {
            r_pod_selected[r][theta].status = MERGE_BINS;
          }
        } else if (bin_map.is_occupied) {
          r_pod_selected[r][theta].status = LITTLE_NUM;
        }
      }
    }
  }

  // ----- Pass 2: resolve each bin given the flagged statuses ----------
  for (int theta = 0; theta < num_sectors; theta++) {
    for (int r = 0; r < num_rings; r++) {
      Bin& bin_curr = r_pod_curr[r][theta];
      Bin& bin_map  = r_pod_map[r][theta];

      const double status = r_pod_selected[r][theta].status;
      if (status == LITTLE_NUM) {
        r_pod_selected[r][theta]        = bin_map;
        r_pod_selected[r][theta].status = LITTLE_NUM;
      } else if (status == MAP_IS_HIGHER) {
        // Fixed 0.5m height-diff gate (not th_bin_max_h) per upstream v3.
        if ((bin_map.max_h - bin_map.min_h) > 0.5) {
          r_pod_selected[r][theta]        = bin_curr;
          r_pod_selected[r][theta].status = MAP_IS_HIGHER;

          if (!piecewise_ground_.empty()) piecewise_ground_.clear();
          if (!non_ground_.empty()) non_ground_.clear();
          extract_ground(bin_map.points, piecewise_ground_, non_ground_);
          r_pod_selected[r][theta].points += piecewise_ground_;

          // Per-bin voxelize (the key v3 feature that prevents the
          // merge-bin doubling pathology v2 suffered from).
          pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
          *tmp = r_pod_selected[r][theta].points;
          erasor_utils::voxelize_preserving_labels_by_nanoflann(
              tmp, r_pod_selected[r][theta].points, map_voxel_size_);

          ground_viz += piecewise_ground_;
          debug_map_rejected += non_ground_;
        } else {
          r_pod_selected[r][theta]        = bin_map;
          r_pod_selected[r][theta].status = NOT_ASSIGNED;
        }
      } else if (status == CURR_IS_HIGHER) {
        r_pod_selected[r][theta]        = bin_map;
        r_pod_selected[r][theta].status = CURR_IS_HIGHER;
      } else if (status == MERGE_BINS) {
        if (is_dynamic_obj_close(r_pod_selected, r, theta, 1, 1)) {
          r_pod_selected[r][theta]        = bin_map;
          r_pod_selected[r][theta].status = BLOCKED;
        } else {
          r_pod_selected[r][theta]        = bin_map;
          r_pod_selected[r][theta].status = MERGE_BINS;
        }
      }
    }
  }
}

void ERASOR::get_static_estimate(pcl::PointCloud<pcl::PointXYZI>& arranged,
                                 pcl::PointCloud<pcl::PointXYZI>& complement) {
  r_pod2pc(r_pod_selected, arranged);
  arranged += ground_viz;
  complement = map_complement;
  // PCL width/height invariant so callers' downstream
  // transformPointCloud / save calls don't trip on width==0.
  arranged.width    = static_cast<std::uint32_t>(arranged.points.size());
  arranged.height   = arranged.points.empty() ? 0 : 1;
  complement.width  = static_cast<std::uint32_t>(complement.points.size());
  complement.height = complement.points.empty() ? 0 : 1;
}

}  // namespace erasor1
