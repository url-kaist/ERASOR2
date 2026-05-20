#include "erasor2/erasor2.h"

#include <opencv2/imgproc.hpp>

#include "erasor2/grid_map.hpp"

using namespace std;

ERASOR2::ERASOR2(const erasor2::Config &cfg) : RosParamServer(cfg) {
  const std::string rule(60, '-');
  cout << "\033[1;36m" << rule << "\n"
       << "[erasor2] configuration\n"
       << rule << "\033[0m\n"
       << "  sequence        : " << sequence_ << "\n"
       << "  increment       : " << increment_ << "\n"
       << "  increment gain  : " << increment_gain_ << "\n"
       << "  voxel size      : " << voxel_size_ << "\n";
  printExtrinsic();
  cout << "\033[1;36m" << rule << "\033[0m" << endl;

  if (increment_gain_ < 1.0) {
    throw invalid_argument("[ERASOR2] `increment_gain_` should be larger than 1.0!");
  }

  if (region_proposal_thr_ > 1.0 || region_proposal_thr_ < 0.0) {
    throw invalid_argument("[ERASOR2] `region_proposal_thr_` should be betwwen 0.0 to 1.0!");
  }

  if (negative_log_odds_ > 0) {
    throw invalid_argument("[ERASOR2] `negative_log_odds_` should be negative!");
  }

  initializePointClouds();
  // `area_per_grid_` is used to check over-segmentation
  area_per_grid_ = grid_resolution_ * grid_resolution_;
}

ERASOR2::~ERASOR2() {}

double ERASOR2::xy2theta(const double &x, const double &y) {  // 0 ~ 2 * PI
  if (y >= 0) {
    return atan2(y, x);  // 1, 2 quadrant
  } else {
    return 2 * M_PI + atan2(y, x);  // 3, 4 quadrant
  }
}

double ERASOR2::xy2radius(const double &x, const double &y) { return sqrt(pow(x, 2) + pow(y, 2)); }

double ERASOR2::prob2logOdds(double prob) {
  // o -> -inf, 1 -> inf
  return 2 * atanh(2 * prob - 1);
}

double ERASOR2::logOdds2prob(double log_odds) { return (tanh(log_odds / 2) + 1) / 2; }

erasor2::Position ERASOR2::idx2position(const erasor2::Index &idx) {
  int w_pc = idx(0);
  int h_pc = idx(1);

  erasor2::Position pos;
  pos(0) = grid_map_info_.x_length / 2 + grid_map_info_.center_x - w_pc * grid_map_info_.resolution;
  pos(1) = grid_map_info_.y_length / 2 + grid_map_info_.center_y - h_pc * grid_map_info_.resolution;
  return pos;
}

bool ERASOR2::isEqual(const erasor2::Index &idx0, const erasor2::Index &idx1) {
  if (idx0(0) == idx1(0) && idx0(1) == idx1(1)) {
    return true;
  } else {
    return false;
  }
}

bool ERASOR2::isInsideTheDynamicInstances(const pcl::PointXYZI &query,
                                          const pcl::PointXYZI &target) {
  float dist    = sqrt((query.x - target.x) * (query.x - target.x) +
                    (query.y - target.y) * (query.y - target.y) +
                    (query.z - target.z) * (query.z - target.z));
  float xy_dist = sqrt((query.x - target.x) * (query.x - target.x) +
                       (query.y - target.y) * (query.y - target.y));
  if (dist < dist_thr_gain_ * voxel_size_ || xy_dist < dist_thr_gain_ * voxel_size_) {
    return true;
  } else {
    return false;
  }
}

int ERASOR2::globalIdx2LocalIdx(const erasor2::Index &global_idx,
                                const erasor2::Index &center_idx) {
  int w_diff = global_idx(0) - (center_idx(0) - neighboring_width_ / 2);
  int h_diff = global_idx(1) - (center_idx(1) - neighboring_height_ / 2);
  return w_diff + h_diff * neighboring_width_;
}

void ERASOR2::initializePointClouds() {
  int num_pts_for_reserve = 5000000;
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
  // throw invalid_argument("Not implemented");
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_w_voi_label(new pcl::PointCloud<pcl::PointXYZI>);

  // 1. 입력 point 수 만큼 할당해주기
  transformed->reserve(cloud_est_label.size());
  cloud_est_w_voi_label->reserve(cloud_est_label.size());

  if (is_initial_) {
    new_origin_ = pose_raw;  // ** pose 의 첫 위치를 new_origin_ 으로 저장한다.
    is_initial_ = false;
  }
  poses_submap_.emplace_back(
      new_origin_.inverse() *
      pose_raw);  // ** new_origin_ 을 기준으로 삼아서 pose 를 저장한다. 그니까 오돔엣더리가 아니라,
                  // 첫번째 포즈를 서브맵의 기준으로 삼으시겠다는 거지.

  // NOTE: First, VoI is set in an egocentric viewpoint
  // This affects the quality of xygrid

  maskNonVoI(cloud_est_label, *cloud_est_w_voi_label, min_z_voi_, max_z_voi_);
  pcl::transformPointCloud(
      *cloud_est_w_voi_label, *transformed, poses_submap_.back() * tf_h_of_ground_to_be_zero_);
  // ** 여까지 하고 나면 transformed 는 VoI information 이 포함되었으며,
  // ** global 좌표계로 변환된 포인트 클라우드가 된다.

  // Parse VoI and Non-VoI. Because non-VoIs are not targets of static map building
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_voi(
      new pcl::PointCloud<pcl::PointXYZI>);  // ** 안 쓰는데 왜 존재하죠
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_non_voi(
      new pcl::PointCloud<pcl::PointXYZI>);  // ** 이것도 안 쓰는데 왜 존재하죠
  if (transformed->size() != cloud_est_label.size())
    throw runtime_error("# of points should be preserved!");
  pcs_transformed_.emplace_back(*transformed);
  // pcs_transformed_ 에 global 좌표계 포인트 쿨라우드가 들어가는데, accumulate_interval 마다
  // 들어가게 되니까
  //  실질적으로 관심 대상이 되는 스캔 데이터 보다는 적게 들어갈 듯

  float max_id = getMaxInstanceId(
      *transformed);  // VoI 도 붙어있는 포인트 클라우드에서 MaxInstanceId 를 구해준다.
  max_ids_.push_back(max_id);  // 아까 id 는 다 rearrange 해줬으니까, max_ids 에 현재 스캔에서의 max
                               // instance id 를 넣어주면 그것도 어떠한 의미가 있겠징!

  if (viz_set_scan_and_pose_) {
    vector<pcl::PointCloud<pcl::PointXYZI>> xygrid;
    pcl::PointCloud<pcl::PointXYZI> complement;
    voi2xygrid(*cloud_est_w_voi_label,
               0.0,
               0.0,
               0.0,
               range_of_interest_,
               grid_resolution_,
               xygrid,
               complement,
               "gridmap");
    erasor2::GridMap gridmap = setEgocentricGridMap(range_of_interest_, grid_resolution_, xygrid);
    auto parsed_cloud        = parseCurrCloud(cloud_est_label);

    erasor2::viz::setFrame(static_cast<int64_t>(pcs_transformed_.size()));
    CurrCloudPublisher.publish(cloud_est_label);
    NonGroundCurrCloudPublisher.publish(parsed_cloud.non_ground_);
    GroundCurrCloudPublisher.publish(parsed_cloud.ground_);
    NoiseCurrCloudPublisher.publish(parsed_cloud.noise_);
    EgocentricGridPublisher.publish(gridmap, "elevation");
    if (stop_for_each_frame_) {
      std::cout << "[Set scan and pose] Waiting for pressing a key" << std::endl;
      cin.ignore();
    }
  }
}

void ERASOR2::setScanAndPose(const Eigen::Matrix4f &pose_raw,
                             const pcl::PointCloud<pcl::PointXYZI> &cloud_gt_label,
                             const pcl::PointCloud<pcl::PointXYZI> &cloud_est_label) {
  // gt label 안에는 instance gt label 이 들어있고, est_label 안에는 초기 인스턴스 및 ground label
  // 이 들어있다.

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_w_voi_label(new pcl::PointCloud<pcl::PointXYZI>);

  // 1. 입력 point 수 만큼 할당해주기
  transformed->reserve(cloud_est_label.size());
  cloud_est_w_voi_label->reserve(cloud_est_label.size());

  if (is_initial_) {
    new_origin_ = pose_raw;  // ** pose 의 첫 위치를 new_origin_ 으로 저장한다.
    is_initial_ = false;
  }
  poses_submap_.emplace_back(
      new_origin_.inverse() *
      pose_raw);  // ** new_origin_ 을 기준으로 삼아서 pose 를 저장한다. 그니까 오돔엣더리가 아니라,
                  // 첫번째 포즈를 서브맵의 기준으로 삼으시겠다는 거지.

  // NOTE: `tf_h_of_ground_to_be_zero_` is not necessary, but we follow the legacy of ERASOR 1.0
  if (dataset_name_ == "SemanticKITTI") {
    pcl::transformPointCloud(
        cloud_gt_label,
        *transformed,
        poses_submap_.back() *
            tf_h_of_ground_to_be_zero_);  // tf_h_of_ground_to_be_zero_ 는 Identity matrix 인데 왜
                                          // 곱하는것인가묘
    pcs_gt_transformed_.emplace_back(*transformed);  // ** pcs_gt_transformed_ 에는 global 좌표계의
                                                     // 포인트 클라우드를 하나씩 넣어준다.
  }

  // NOTE: First, VoI is set in an egocentric viewpoint
  // This affects the quality of xygrid
  maskNonVoI(cloud_est_label, *cloud_est_w_voi_label, min_z_voi_, max_z_voi_);
  pcl::transformPointCloud(
      *cloud_est_w_voi_label, *transformed, poses_submap_.back() * tf_h_of_ground_to_be_zero_);
  // ** 여까지 하고 나면 transformed 는 VoI information 이 포함되었으며, global 좌표계로 변환된
  // 포인트 클라우드가 된다.

  // Parse VoI and Non-VoI. Because non-VoIs are not targets of static map building
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_voi(
      new pcl::PointCloud<pcl::PointXYZI>);  // ** 안 쓰는데 왜 존재하죠
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_non_voi(
      new pcl::PointCloud<pcl::PointXYZI>);  // ** 이것도 안 쓰는데 왜 존재하죠
  pcs_transformed_.emplace_back(*transformed);

  float max_id = getMaxInstanceId(
      *transformed);  // VoI 도 붙어있는 포인트 클라우드에서 MaxInstanceId 를 구해준다.
  max_ids_.push_back(max_id);  // 아까 id 는 다 rearrange 해줬으니까, max_ids 에 현재 스캔에서의 max
                               // instance id 를 넣어주면 그것도 어떠한 의미가 있겠징!

  if (viz_set_scan_and_pose_) {
    vector<pcl::PointCloud<pcl::PointXYZI>> xygrid;
    pcl::PointCloud<pcl::PointXYZI> complement;
    voi2xygrid(*cloud_est_w_voi_label,
               0.0,
               0.0,
               0.0,
               range_of_interest_,
               grid_resolution_,
               xygrid,
               complement,
               "gridmap");
    erasor2::GridMap gridmap = setEgocentricGridMap(range_of_interest_, grid_resolution_, xygrid);
    auto parsed_cloud        = parseCurrCloud(cloud_est_label);

    erasor2::viz::setFrame(static_cast<int64_t>(pcs_transformed_.size()));
    CurrCloudPublisher.publish(cloud_est_label);
    NonGroundCurrCloudPublisher.publish(parsed_cloud.non_ground_);
    GroundCurrCloudPublisher.publish(parsed_cloud.ground_);
    NoiseCurrCloudPublisher.publish(parsed_cloud.noise_);
    EgocentricGridPublisher.publish(gridmap, "elevation");
    if (stop_for_each_frame_) {
      std::cout << "[Set scan and pose] Waiting for pressing a key" << std::endl;
      cin.ignore();
    }
  }
}

void ERASOR2::setSubmap() {
  cout << "[ERASOR2] Setting submap..." << endl;
  int num_data = pcs_transformed_.size();  // acc interval 만큼 쌓인 실질적인 포인트 클라우드들.
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_partial_src(new pcl::PointCloud<pcl::PointXYZI>);
  for (int k = 0; k < num_data; ++k) {
    // 너무 많은 points -> voxelization함!
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> cloud_tmp;
    *ptr_transformed = pcs_transformed_[k];
    erasor_utils::voxelize_preserving_labels_by_nanoflann(
        ptr_transformed, cloud_tmp, voxel_size_, minimum_num_per_voxel_);
    *map_partial_src += cloud_tmp;  // ** voI + instance information 이 붙은 global 좌표계 스캔들을
                                    // 모두 쌓아서 map_partial_src 에 넣어준다.
  }

  cout << "[ERASOR2] Voxelizing submap..." << endl;
  // Estimated labels are preserved!
  erasor_utils::voxelize_preserving_labels_by_nanoflann(
      map_partial_src,
      *map_accum_,
      map_voxel_size_);  // 그렇게 해서 만들어진 전체 맵은 map_accum_ 에 넣어주고 ,여기서는 voxel
                         // size 를 0.4 로 설정했네.
  // map_partial_src 는 러프하게 쌓인 맵, map_accum_ 은 voxelized 된 맵이다.

  cout << "[ERASOR2] Calculating  min-max x, y values given " << pcs_transformed_.size()
       << " scan-pose pairs..." << endl;
  float min_x, min_y, max_x, max_y;
  erasor_utils::calcMinMaxXY(pcs_transformed_,
                             min_x,
                             min_y,
                             max_x,
                             max_y);  // voxelized 되지 않은 맵에서 min, max X, Y 를 구해주고,
  cout << "[ERASOR2] Min-max x: " << min_x << " <-> " << max_x << endl;
  cout << "[ERASOR2] Min-max y: " << min_y << " <-> " << max_y << endl;

  num_data_ = num_data;

  cout << "[ERASOR2] Setting map-centric gridmap..." << endl;
  grid_map_info_  = setGridMapParams(min_x, min_y, max_x, max_y, grid_resolution_);
  gridmap_submap_ = setMapcentricGridMap(grid_map_info_);

  resize();

  // setting approximated (w, h) and `xygrids_` for each scan
  // Note that sometimes all the (pose, scan) pairs are not used for updating steppable regions
  // But for detecting moving instances, all scans may be employed
  for (int k = 0; k < num_data_; ++k) {
    erasor2::Position pos_xy(poses_submap_[k](0, 3), poses_submap_[k](1, 3));
    gridmap_submap_.getIndex(pos_xy, idxes_approx_[k]);
    erasor2::Position pos_approx = idx2position(idxes_approx_[k]);

    pcl::PointCloud<pcl::PointXYZI> complement;
    //        std::cout << poses_submap[k] << std::endl;
    //        std::cout << pos_x_approx << " , " << pos_y_approx << std::endl;
    //        cout << "\033[1;32mOn pc voi2xygrid...\033[0m\n";
    voi2xygrid(pcs_transformed_[k],
               pos_approx(0),
               pos_approx(1),
               poses_submap_[k](2, 3),
               range_of_interest_,
               grid_resolution_,
               xygrids_[k],
               complement);
  }
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

ParsedCurrCloud ERASOR2::parseCurrCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  // Viz
  ParsedCurrCloud parsed_cloud;
  parsed_cloud.non_ground_.reserve(5000000);
  parsed_cloud.ground_.reserve(5000000);
  parsed_cloud.noise_.reserve(1000000);

  // If there exists some instances apart from the current frame, then some times max_ids_.back() <
  // getMaxInstanceId(cloud)
  int max_idx = max(max_ids_.back(), getMaxInstanceId(cloud));
  vector<float> tmp_idxes;
  for (const auto &pt : cloud) {
    if (pt.intensity == GROUND_LABEL) {
      parsed_cloud.ground_.emplace_back(pt);
    } else if (pt.intensity == NOT_INTEREST) {
      parsed_cloud.noise_.emplace_back(pt);
    } else if (pt.intensity != NOT_VOLUME_OF_INTEREST) {
      pcl::PointXYZRGB pt_new;
      pt_new.x = pt.x;
      pt_new.y = pt.y;
      pt_new.z = pt.z;
      tmp_idxes.push_back(pt.intensity);
      parsed_cloud.non_ground_.emplace_back(pt_new);
    }
  }
  std::cout << "[Debug] max_idx: " << max_idx << std::endl;
  std::cout << "[Debug] cloud.size(): " << cloud.size() << std::endl;
  std::cout << "[Debug] tmp_idxes.size(): " << tmp_idxes.size() << std::endl;

  std::cout << "[Debug] parsed_cloud.non_ground_.size(): " << parsed_cloud.non_ground_.size()
            << std::endl;
  std::cout << "[Debug] parsed_cloud.ground_.size(): " << parsed_cloud.ground_.size() << std::endl;
  std::cout << "[Debug] parsed_cloud.noise_.size(): " << parsed_cloud.noise_.size() << std::endl;

  colors.clear();
  for (int i = 0; i < max_idx; ++i) {  // ! 240112 modified max_idx + 1 -> max_idx + 2
    colors.emplace_back(erasor_utils::getRandomColor());
  }
  // std::cout << "[Debug] colors.size(): " << colors.size() << std::endl;
  int ccc = 0;
  for (auto &pt : parsed_cloud.non_ground_) {
    int idx_tmp = int(tmp_idxes[ccc]);
    // For defensive programming
    // ToDo. Fix this weird overflow issue
    while (idx_tmp >= colors.size()) {
      std::cout << "[Debug] \033[1;33mOverflow happens: idx_tmp: " << idx_tmp << " vs "
                << colors.size() << std::endl;
      colors.emplace_back(erasor_utils::getRandomColor());
    }
    pt.r = colors[idx_tmp][0];
    pt.g = colors[idx_tmp][1];
    pt.b = colors[idx_tmp][2];
    ++ccc;
  }
  // std::cout << "[Debug] colors2 .size(): " << colors.size() << std::endl;

  return parsed_cloud;
}

void ERASOR2::updateSteppableRegion() {
  for (int k = 0; k < num_data_; k += update_interval_) {
    cout << "\r[ERASOR2] Updating " << k + 1 << " / " << num_data_ << flush;
    gridmap_submap_["elevation"].setConstant(NOT_UPDATED);
    //        erasor2::Position pos_xy(poses_submap_[k](0, 3), poses_submap_[k](1, 3));//
    //        new_origin_ 을 기준으로 삼아서 pose 를 저장해놓은 것에서 저 떄의 로봇 x, y 좌표의 위치
    //        뽑아오기 gridmap_submap_.getIndex(pos_xy, idxes_approx_[k]); // 현재 로봇 위치를
    //        기준으로 gridmap 에서의 인덱스를 뽑아서 idxes_approx_ 안에 넣어준다.

    int w_pc                     = idxes_approx_[k](0);
    int h_pc                     = idxes_approx_[k](1);
    erasor2::Position pos_approx = idx2position(idxes_approx_[k]);

    pcl::PointCloud<pcl::PointXYZI> complement;
    //        std::cout << poses_submap[k] << std::endl;
    //        std::cout << pos_x_approx << " , " << pos_y_approx << std::endl;
    vector<pcl::PointCloud<pcl::PointXYZI>> map_grid;

    pcl::PointCloud<pcl::PointXYZI>::Ptr dummy(new pcl::PointCloud<pcl::PointXYZI>);
    voi2xygrid(*map_accum_,
               pos_approx(0),
               pos_approx(1),
               poses_submap_[k](2, 3),
               range_of_interest_,
               grid_resolution_,
               map_grid,
               *dummy);

    // Assume that range of interest is square
    erasor2::Index idx;
    int count = 0;
    for (int h = h_pc - neighboring_height_ / 2; h < h_pc + neighboring_height_ / 2;
         ++h) {  // 현재 로봇의 위치를 기점으로 약 10m 의 영역을 관측함.
      for (int w = w_pc - neighboring_width_ / 2; w < w_pc + neighboring_width_ / 2; ++w) {
        // x direction first
        idx(0) = w;
        idx(1) = h;
        //                cout << "\033[1;32m(" << w << ", " << h << ") - \033[0m";
        if (isLikelyToBeSteppableRegionbyBinaryDescriptor(
                xygrids_[k][count],
                map_grid[count],
                scan_ratio_threshold_,
                min_z_diff_thr_,
                verbose_)) {  // 오직 map, scan 으의 높이차이가 꽤 나면서도 scan 이 바닥으로 정확히
                              // 판명난 경우에만 True
          // For debugging
          gridmap_submap_.at("elevation", idx) = erasor_utils::calcMeanZOfGround(map_grid[count]);
          //                    cout << "(" << w << ", " << h << "): " << scan_ratio_ << " | " <<
          //                    ratio_num_ << " // "; cout << xygrids_[k][count].size() << " <-> "
          //                         << erasor_utils::getNumGroundPoints(xygrids_[k][count]) <<
          //                         endl;
          gridmap_submap_.at("status", idx) =
              TEMPORARILY_OCCUPIED;  // ** 아직 관측되지 않았을 때가 100.0 이었고, TEMPORARILY
                                     // OCCUPIED 일 때가 102.0 에 해당
          updateLogOdds(
              idx,
              increment_ * increment_gain_);  // gridmap 의 idx 부분에 increment_ * increment_gain_
                                              // 만큼을 더해줌. 만약애 TEMPORARILY_OCCUPIED 라면
                                              // log_odds 에는 0.15 * 2.0 만큼
        }

        else if (isLikelyToBeGround(
                     xygrids_[k][count])) {  // ** 그냥 map 자체가 ground 인 것 같아...! 라면.
          if (gridmap_submap_.at("status", idx) == NOT_OBSERVED) {
            gridmap_submap_.at("status", idx)    = GROUND_EXISTS;
            gridmap_submap_.at("elevation", idx) = erasor_utils::calcMeanZOfGround(map_grid[count]);
          }
          updateLogOdds(idx, increment_);  // 0.15 * 2.0 만큼 계속 더해지고
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

      publishPose(k);

      CurrCloudPublisher.publish(pcs_transformed_[k]);
      CurrVoIPublisher.publish(*curr_voi);
      MapVoIPublisher.publish(*map_voi);

      logOddsGrid2probGrid();
      GridPublisher.publish(gridmap_submap_, "prob");
      if (stop_for_each_frame_) {
        std::cout << "[Update] Waiting for pressing a key" << std::endl;
        cin.ignore();
      }
    }
  }
  logOddsGrid2probGrid();
  cout << "\n";
}

// Re-project ground likelihood to each scan
void ERASOR2::detectMovingObjects() {
  GridPublisher.publish(gridmap_submap_, "prob");
  for (int k = 0; k < num_data_; ++k) {
    cout << "\r[Detect] Detecting moving instances " << k + 1 << " / " << num_data_ << flush;
    vector<float> dyn_cand_ids;  // temp. variable
    //        unordered_map<float, DynamicInstance> &ids_clusters  = ids_instances_set_[k];
    noisy_points_transformed_[k].reserve(100);

    int w_pc = idxes_approx_[k](0);
    int h_pc = idxes_approx_[k](1);

    erasor2::Index idx;
    int count = 0;
    // Detect dynamic objects' ids per scan
    for (int h = h_pc - neighboring_height_ / 2; h < h_pc + neighboring_height_ / 2; ++h) {
      for (int w = w_pc - neighboring_width_ / 2; w < w_pc + neighboring_width_ / 2; ++w) {
        idx(0) = w;
        idx(1) = h;
        if (gridmap_submap_.at("prob", idx) > region_proposal_thr_) {
          // Extract indices
          // Note that `xygrids_[k][count]` does not contain non-VoI points
          for (const auto &pt : xygrids_[k][count].points) {
            if ((pt.intensity != GROUND_LABEL) && (pt.intensity != NOT_INTEREST) &&
                std::find(dyn_cand_ids.begin(), dyn_cand_ids.end(), pt.intensity) ==
                    dyn_cand_ids.end()) {
              dyn_cand_ids.push_back(pt.intensity);
            } else if (pt.intensity == NOT_INTEREST &&
                       gridmap_submap_.at("status", idx) == TEMPORARILY_OCCUPIED) {
              noisy_points_transformed_[k].points.emplace_back(pt);
            }
          }
        }
        ++count;
      }
    }
    //        cout << "\r[Detect] # of dyn. instances for the " << k+1 << "-th scan: " <<
    //        dyn_cand_ids.size() << flush;

    // 2. Set Dynamic instance
    auto &ids_clusters = ids_instances_set_[k];
    ids_clusters.clear();
    DynamicInstance dyn_cluster;
    dyn_cluster.cloud_.reserve(200);  // heuristic
    for (const int dyn_cand_id : dyn_cand_ids) {
      ids_clusters[dyn_cand_id] = dyn_cluster;
    }

    for (const auto &pt : pcs_transformed_[k]) {
      if (ids_clusters.find(pt.intensity) != ids_clusters.end()) {
        ids_clusters[pt.intensity].cloud_.emplace_back(pt);
      }
    }
  }
  cout << "\n";
}

void ERASOR2::saveDynamicLabels(const string &dynamic_label_root, const vector<size_t> &indices) {
  // Note that num_data_ is the indices of our interest + expanded frames
  // Thus, we only save the labels of our interest
  // i.e., the input of ERASOR2 was `frames` + `expanded frames`
  int save_data_size = indices.size();
  std::cout << "[ERASOR2] On saving label results..." << std::endl;
  for (int k = 0; k < save_data_size; ++k) {
    int frame_num = indices[k];
    cout << "\r[ERASOR2] Saving dynamic labels " << frame_num << " (" << k << " / "
         << save_data_size << ")" << flush;
    erasor_utils::save_dyn_label(dynamic_label_root,
                                 frame_num,
                                 pcs_transformed_[k],
                                 dynamic_points_transformed_[k],
                                 potential_dynamic_points_transformed_[k]);
  }
}

void ERASOR2::filterDynamicObjects() {
  for (int k = 0; k < num_data_; ++k) {
    auto &ids_clusters        = ids_instances_set_[k];
    auto &rejected_objs       = rejected_objs_set_[k];
    auto &accepted_objs       = accepted_objs_set_[k];
    float max_id_for_register = max_ids_[k] + 10.0;  // 10.0 is just a margin.

    // For managing over-segmentation
    vector<OverSegmentedInstance> instances_to_be_updated;

    // Only available on C++ 17
    for (auto &[dyn_cand_id, dynamic_instance] : ids_clusters) {
      setDynamicInstance(dynamic_instance, poses_submap_[k](0, 3), poses_submap_[k](1, 3));

      float adaptive_thr =
          dynamic_instance.is_close_to_body_frame_ ? obj_score_hard_thr_ : obj_score_soft_thr_;
      if (dynamic_instance.moving_obj_score_ > adaptive_thr) {
        dynamic_instance.is_dynamic_ = true;
        // For visualization
        accepted_objs.push_back({dynamic_instance.centroid_, dynamic_instance.moving_obj_score_});

      } else {  // it means that most parts are not in the region of interests
        if (isOverSegmented(dynamic_instance)) {
          // To debug over-clustering
          cout << "\033[1;35mOver-segmentation detected: " << dynamic_instance.cloud_.size()
               << " => ";
          DynamicInstance static_inst, partial_dynamic_inst;
          parseOverSegmentation(dynamic_instance,
                                static_inst,
                                partial_dynamic_inst,
                                poses_submap_[k](0, 3),
                                poses_submap_[k](1, 3));
          cout << static_inst.cloud_.size() << " <-> " << partial_dynamic_inst.cloud_.size()
               << "\033[0m" << endl;

          if (viz_over_seg_) {
            CurrCloudPublisher.publish(pcs_transformed_[k]);
            DynCurrCloudPublisher.publish(dynamic_instance.cloud_);
            NoiseCurrCloudPublisher.publish(partial_dynamic_inst.cloud_);
            cin.ignore();
          }

          OverSegmentedInstance inst_to_be_updated;
          inst_to_be_updated.original_id          = dyn_cand_id;
          inst_to_be_updated.new_id_for_stat_inst = ++max_id_for_register;
          inst_to_be_updated.new_id_for_dyn_inst  = ++max_id_for_register;
          inst_to_be_updated.static_inst          = static_inst;
          inst_to_be_updated.dynamic_inst         = partial_dynamic_inst;
          instances_to_be_updated.emplace_back(inst_to_be_updated);

          accepted_objs.push_back(
              {partial_dynamic_inst.centroid_, partial_dynamic_inst.moving_obj_score_});
          rejected_objs.push_back({static_inst.centroid_, static_inst.moving_obj_score_});
        } else {
          dynamic_instance.is_dynamic_ = false;
          rejected_objs.push_back({dynamic_instance.centroid_, dynamic_instance.moving_obj_score_});
        }
      }
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
      discernStaticAndDynamicPoints(pcs_gt_transformed_[k],
                                    static_mask,
                                    static_points_transformed_[k],
                                    dynamic_points_transformed_[k]);
    } else {
      discernStaticAndDynamicPoints(
          each_pc, static_mask, static_points_transformed_[k], dynamic_points_transformed_[k]);
    }
  }

  for (int k = 0; k < num_data_; ++k) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_static_points(
        new pcl::PointCloud<pcl::PointXYZI>);
    windowBasedVolumetricOutlierRemoval(k,
                                        window_size_,
                                        dist_thr_gain_,
                                        *filtered_static_points,
                                        potential_dynamic_points_transformed_[k]);
    // It shows rather poor performance
    //        instanceAwareOutlierRemoval(k, window_size_, dist_thr_gain_, *filtered_static_points,
    //                                            potential_dynamic_points_transformed_[k]);
    static_points_transformed_[k] = *filtered_static_points;

    (*map_noise_) += noisy_points_transformed_[k];
    (*map_dynamic_) += dynamic_points_transformed_[k] + potential_dynamic_points_transformed_[k];
    // erasor_utils::save_dyn_label( //? modified Test in 240114
    if (viz_detect_) {
      publishPose(k);

      pcl::PointCloud<pcl::PointXYZRGB> inst_colored;
      pcl::PointXYZRGB pt_colored;
      for (auto &[dyn_cand_id, dynamic_instance] : ids_instances_set_[k]) {
        while (colors.size() <= dyn_cand_id) {
          colors.emplace_back(erasor_utils::getRandomColor());
        }
        pt_colored.r = colors[dyn_cand_id][0];
        pt_colored.g = colors[dyn_cand_id][1];
        pt_colored.b = colors[dyn_cand_id][2];
        for (const auto &pt : dynamic_instance.cloud_) {
          pt_colored.x = pt.x;
          pt_colored.y = pt.y;
          pt_colored.z = pt.z;
          inst_colored.emplace_back(pt_colored);
        }
      }
      DynInstCurrCloudPublisher.publish(inst_colored);

      cout << pcs_transformed_[k].points.size() << " => "
           << static_points_transformed_[k].points.size() << " / ";
      cout << dynamic_points_transformed_[k].points.size() << " / "
           << noisy_points_transformed_[k].size() << " / ";
      cout << potential_dynamic_points_transformed_[k].points.size() << endl;

      CurrCloudPublisher.publish(pcs_transformed_[k]);
      DynCurrCloudPublisher.publish(dynamic_points_transformed_[k]);

      //            RejectedDynCurrCloudPublisher.publish(*rejected_dynamic_objs);
      OutlierCurrCloudPublisher.publish(potential_dynamic_points_transformed_[k]);
      NoiseCurrCloudPublisher.publish(noisy_points_transformed_[k]);

      if (dataset_name_ == "SemanticKITTI") {
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        erasor_utils::parseStaticAndDynamic(
            static_points_transformed_[k], *dynamic_cloud, *static_cloud);
        StaticCloudPublisher.publish(*static_cloud);
        DynamicCloudPublisher.publish(*dynamic_cloud);
      }
      RejectedMovingObjScorePublisher.publishScores(rejected_objs_set_[k], {1.0f, 1.0f, 1.0f});
      AcceptedMovingObjScorePublisher.publishScores(accepted_objs_set_[k], {0.0f, 1.0f, 0.0f});

      GridPublisher.publish(gridmap_submap_, "prob");

      visualizeHardThrRadius(poses_submap_[k]);

      if (stop_for_each_frame_) {
        std::cout << "[Detect] " << k << " th. Waiting for pressing a key" << std::endl;
        cin.ignore();
      }
    }
  }
}

void ERASOR2::estimateStaticMask(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                 const unordered_map<float, DynamicInstance> &ids_clusters,
                                 std::vector<int> &static_mask) {
  static_mask.resize(cloud.points.size());
  int count = 0;
  for (const auto &pt : cloud) {
    auto iter = ids_clusters.find(pt.intensity);
    if (iter != ids_clusters.end() && iter->second.is_dynamic_) {
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
  for (const int correspondence : correspondences) {
    static_mask[correspondence] = IS_NOISE_YET_POTENTIAL_DYNAMIC;
  }
}

void ERASOR2::accumDynamicCloud(const int k,
                                const int window_size,
                                pcl::PointCloud<pcl::PointXYZI> &cloud_accum,
                                bool use_voxelization) {
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

void ERASOR2::accumInstanceWiseDynamicCloud(const int k,
                                            const int window_size,
                                            pcl::PointCloud<pcl::PointXYZI> &cloud_accum,
                                            bool use_voxelization) {
  int lower_bound = k - window_size / 2;
  int upper_bound = k + (window_size + 1) / 2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr dyn_points_accum(new pcl::PointCloud<pcl::PointXYZI>);

  for (int i = max(0, lower_bound); i < min(num_data_, upper_bound); ++i) {
    const auto &noisy_points = noisy_points_transformed_[i];
    const auto &ids_clusters = ids_instances_set_[i];
    for (auto &[dyn_cand_id, dynamic_instance] : ids_clusters) {
      if (dynamic_instance.is_dynamic_ &&
          dynamic_instance.moving_obj_score_ > vor_cand_score_thr_) {
        *dyn_points_accum += dynamic_instance.cloud_;

        vector<int> target_idxes;
        erasor_utils::radiusSearch(
            dynamic_instance.cloud_, noisy_points, dist_thr_gain_ * voxel_size_, target_idxes);
        for (const int target_idx : target_idxes) {
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

// It shows rather poor performance
void ERASOR2::instanceAwareOutlierRemoval(
    const int k,
    const int window_size,
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
  vector<erasor2::Index> regions_of_interest;

  regions_of_interest.reserve(64);  // heuristic
  for (int i = max(0, lower_bound); i < min(num_data_, upper_bound); ++i) {
    auto &ids_clusters = ids_instances_set_[i];
    for (const auto &[dyn_cand_id, dynamic_cluster] : ids_clusters) {
      if (dynamic_cluster.is_dynamic_) {
        for (const auto &occupied_region : dynamic_cluster.occupied_map_idxes_) {
          bool is_first = true;
          for (const auto &roi : regions_of_interest) {
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
  int w_pc                     = idxes_approx_[k](0);
  int h_pc                     = idxes_approx_[k](1);
  erasor2::Position pos_approx = idx2position(idxes_approx_[k]);

  pcl::PointCloud<pcl::PointXYZI>::Ptr complement(new pcl::PointCloud<pcl::PointXYZI>);
  vector<pcl::PointCloud<pcl::PointXYZI>> xygrid;
  voi2xygrid(static_points_transformed_[k],
             pos_approx(0),
             pos_approx(1),
             poses_submap_[k](2, 3),
             range_of_interest_,
             grid_resolution_,
             xygrid,
             *complement);

  pcl::PointCloud<pcl::PointXYZI>::Ptr static_points_of_interest(
      new pcl::PointCloud<pcl::PointXYZI>);
  vector<bool> is_roi(xygrid.size(), false);
  cout << "B" << endl;
  // global_idx: idx w.r.t. sudmap's grid map
  for (const auto &global_idx : regions_of_interest) {
    if (global_idx(0) >= w_pc - neighboring_width_ / 2 &&
        global_idx(0) < w_pc + neighboring_width_ / 2 &&
        global_idx(1) >= h_pc - neighboring_height_ / 2 &&
        global_idx(1) < w_pc + neighboring_height_ / 2) {
      int voi_idx = globalIdx2LocalIdx(global_idx, idxes_approx_[k]);
      if (voi_idx > xygrid.size() || voi_idx < 0) {
        continue;
      }
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
    const auto &query  = static_points_of_interest->points[j];
    const auto &target = dyn_points_voxel->points[correspondences[j]];
    if (isInsideTheDynamicInstances(query, target)) {
      potential_dynamic_points.points.emplace_back(query);
    } else {
      filtered_static_points.emplace_back(query);
    }
  }
}

void ERASOR2::setDynamicInstance(DynamicInstance &dynamic_cluster,
                                 const float pos_x,
                                 const float pos_y) {
  setOccupiedMapIdxes(dynamic_cluster);
  setMovingInstanceScore(dynamic_cluster);
  pcl::compute3DCentroid(dynamic_cluster.cloud_, dynamic_cluster.centroid_);
  dynamic_cluster.is_close_to_body_frame_ =
      isCloseToBodyFrame(dynamic_cluster, pos_x, pos_y, hard_thr_radius_);
}

bool ERASOR2::isOverSegmented(const DynamicInstance &dynamic_cluster) {
  int num_unknown_grids = 0;
  int num_dynamic_grids = 0;
  for (const auto &idx : dynamic_cluster.occupied_map_idxes_) {
    if (gridmap_submap_.at("status", idx) == NOT_OBSERVED) {
      ++num_unknown_grids;
    }
    if (gridmap_submap_.at("status", idx) == TEMPORARILY_OCCUPIED &&
        gridmap_submap_.at("log_odds", idx) > obj_score_hard_thr_) {
      ++num_dynamic_grids;
    }
  }

  int minimum_num_grid = static_cast<int>(minimum_area_thr_ / area_per_grid_);
  float ratio_of_unknown_prior =
      static_cast<float>(num_unknown_grids) / dynamic_cluster.occupied_map_idxes_.size();
  if (dynamic_cluster.occupied_map_idxes_.size() > minimum_num_grid &&
      ratio_of_unknown_prior > ratio_of_unknown_prior_ && num_dynamic_grids > 0) {
    return true;
  } else {
    return false;
  }
}

void ERASOR2::parseOverSegmentation(const DynamicInstance &over_segmented,
                                    DynamicInstance &static_inst,
                                    DynamicInstance &partial_dynamic_inst,
                                    const float pos_x,
                                    const float pos_y) {
  static_inst.cloud_.clear();
  static_inst.cloud_.reserve(200);
  partial_dynamic_inst.cloud_.clear();
  partial_dynamic_inst.cloud_.reserve(200);

  for (const auto &pt : over_segmented.cloud_) {
    erasor2::Position p_tmp(pt.x, pt.y);
    erasor2::Index idx_tmp;
    gridmap_submap_.getIndex(p_tmp, idx_tmp);
    if ((gridmap_submap_.at("log_odds", idx_tmp)) > obj_score_hard_thr_) {
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

void ERASOR2::updateNewParsedInstances(const vector<OverSegmentedInstance> &instances_to_be_updated,
                                       pcl::PointCloud<pcl::PointXYZI> &cloud,
                                       unordered_map<float, DynamicInstance> &ids_clusters) {
  /***
   * It has two rules:
   * a) update original point cloud
   *     Because the id is used when estimating `static_mask`,
   *     so the ids of the original point cloud are also updated
   * b) append new instances into ids_clusters
   */
  for (const auto &inst : instances_to_be_updated) {
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
    ids_clusters[inst.new_id_for_dyn_inst]  = inst.dynamic_inst;
    ids_clusters[inst.new_id_for_stat_inst] = inst.static_inst;
  }
}

void ERASOR2::windowBasedVolumetricOutlierRemoval(
    const int k,
    const int window_size,
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
                           filtered_static_points,
                           potential_dynamic_points);
}

void ERASOR2::volumetricOutlierRemoval(const pcl::PointCloud<pcl::PointXYZI> &static_points,
                                       const pcl::PointCloud<pcl::PointXYZI> &dynamic_points,
                                       const float dist_thr_gain,
                                       pcl::PointCloud<pcl::PointXYZI> &filtered_static_points,
                                       pcl::PointCloud<pcl::PointXYZI> &potential_dynamic_points) {
  int N = static_points.size();
  vector<int> static_mask(N, IS_STATIC);

  vector<int> target_idxes;

  if (use_adaptive_voxel_size_) {
    vector<float> adaptive_radii;
    adaptive_radii.resize(dynamic_points.size());
    for (int i = 0; i < dynamic_points.size(); ++i) {
      const auto &pt = dynamic_points[i];
      erasor2::Position p_tmp(pt.x, pt.y);
      erasor2::Index idx_tmp;
      gridmap_submap_.getIndex(p_tmp, idx_tmp);
      bool is_in = idx_tmp(0) < grid_map_info_.width && idx_tmp(0) > -1 &&
                   idx_tmp(1) < grid_map_info_.height && idx_tmp(1) > -1;
      if (!is_in || gridmap_submap_.at("status", idx_tmp) == NOT_OBSERVED ||
          pt.z - gridmap_submap_.at("elevation", idx_tmp) < voxel_size_ * 0.5) {
        adaptive_radii[i] = voxel_size_;
      } else {
        adaptive_radii[i] = dist_thr_gain_ * voxel_size_;
      }
    }
    erasor_utils::radiusSearchWithAdaptiveRadii(
        dynamic_points, static_points, adaptive_radii, target_idxes);

  } else {
    erasor_utils::radiusSearch(
        dynamic_points, static_points, dist_thr_gain_ * voxel_size_, target_idxes);
  }

  for (const int idx : target_idxes) {
    static_mask[idx] = IS_DYNAMIC;
  }

  //    cout << "\033[1;32mTotal " << valid_outlier_idxes.size() << " points are filtered\033[0m" <<
  //    endl;
  filtered_static_points.clear();
  filtered_static_points.reserve(N);
  for (int j = 0; j < N; ++j) {
    const auto &status = static_mask[j];
    const auto &pt     = static_points[j];
    if (status == IS_STATIC) {
      filtered_static_points.points.emplace_back(pt);
    } else if (status == IS_DYNAMIC) {
      potential_dynamic_points.points.emplace_back(pt);
    } else {
      throw invalid_argument("A wrong mask status is set!");
    }
  }
  // Keep PCL width/height in sync with the points we just appended; PCL's
  // transformPointCloud computes `height = size / width` and will SIGFPE if
  // width is left at 0.
  filtered_static_points.width =
      static_cast<std::uint32_t>(filtered_static_points.points.size());
  filtered_static_points.height = filtered_static_points.points.empty() ? 0 : 1;
  potential_dynamic_points.width =
      static_cast<std::uint32_t>(potential_dynamic_points.points.size());
  potential_dynamic_points.height = potential_dynamic_points.points.empty() ? 0 : 1;
  cout << "\033[1;32mTotal " << potential_dynamic_points.size() << " points are filtered\033[0m"
       << endl;
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
  int count = 0;
  for (const auto &pt : cloud) {
    if (static_mask[count] == IS_STATIC) {
      static_points.points.emplace_back(pt);
    } else if (static_mask[count] == IS_DYNAMIC) {
      // Note that noisy points are not included
      dynamic_points.points.emplace_back(pt);
    }

    ++count;
  }
  // Keep width/height in sync with points so downstream PCL routines
  // (e.g. pcl::transformPointCloud computes height = size / width) do not
  // divide by zero.
  static_points.width   = static_cast<std::uint32_t>(static_points.points.size());
  static_points.height  = static_points.points.empty() ? 0 : 1;
  dynamic_points.width  = static_cast<std::uint32_t>(dynamic_points.points.size());
  dynamic_points.height = dynamic_points.points.empty() ? 0 : 1;
}

// void
// ERASOR2::discernStaticAndDynamicPoints(const pcl::PointCloud<pcl::PointXYZI> &cloud, const
// std::vector<int> &dyn_ids,
//                                        pcl::PointCloud<pcl::PointXYZI> &static_points,
//                                        pcl::PointCloud<pcl::PointXYZI> &dynamic_points) {
//     for (const auto &pt: cloud) {
//         if (std::find(dyn_ids.begin(), dyn_ids.end(), pt.intensity) != dyn_ids.end()) {
//             if (pt.intensity == NOT_VOLUME_OF_INTEREST) {
//                 // ToDo Improve
//                 static_points.points.emplace_back(pt);
//             } else {
//                 dynamic_points.points.emplace_back(pt);
//             }
//         } else {
//             static_points.points.emplace_back(pt);
//         }
//     }
// }

void ERASOR2::publishStaticMapResults() {
  std::cout << "[ERASOR2] Publish results!" << std::endl;
  //    std::cout << "# of dynamic points: " << map_dynamic_->points.size() << std::endl;
  std::cout << "# of static points: " << static_map_voxelized_->points.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_for_viz(new pcl::PointCloud<pcl::PointXYZI>);
  // Because ERASOR2 uses the local origin,
  pcl::transformPointCloud(*static_map_voxelized_, *static_map_for_viz, new_origin_.inverse());
  MapCloudPublisher.publish(*static_map_for_viz);
  DynMapPublisher.publish(*map_dynamic_);
  NoiseMapPublisher.publish(*map_noise_);
  GridPublisher.publish(gridmap_submap_, "prob");
  // Legacy code spin-looped here so RViz could keep redrawing the result.
  // Rerun retains the last log entry indefinitely, so a single publish is
  // enough — the user can still inspect the final map after the binary
  // exits.
}

void ERASOR2::saveStaticMap(const string &static_map_path) {
  std::cout << "[ERASOR2] On saving results..." << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world_frame(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < num_data_; ++i) {
    const auto &cloud = static_points_transformed_[i];
    // pcl::transformPointCloud divides by `width` internally; an
    // unpopulated slot has width=0 and triggers SIGFPE. Empty in, empty
    // out is a no-op for the accumulator, so just skip.
    if (cloud.empty()) continue;
    pcl::transformPointCloud(cloud, *cloud_world_frame, new_origin_);
    (*static_map_accum_) += (*cloud_world_frame);
  }
  erasor_utils::voxelize_preserving_labels_by_nanoflann(
      static_map_accum_, *static_map_voxelized_, map_voxel_size_);

  static_map_voxelized_->width  = static_map_voxelized_->points.size();
  static_map_voxelized_->height = 1;
  std::cout << "[Debug]: (" << static_map_voxelized_->width << ", " << static_map_voxelized_->height
            << ") => " << static_map_voxelized_->points.size() << std::endl;
  std::cout << "\033[1;32mSaving the map to pcd...\033[0m" << std::endl;
  pcl::io::savePCDFileASCII(static_map_path, *static_map_voxelized_);
  std::cout << "\033[1;32mComplete to save the map!:";
  std::cout << static_map_path << "\033[0m" << std::endl;
}

void ERASOR2::maskNonVoI(const pcl::PointCloud<pcl::PointXYZI> &src,
                         pcl::PointCloud<pcl::PointXYZI> &cloud_out,
                         const float min_z_voi,
                         const float max_z_voi) {
  cloud_out.clear();
  int N = src.points.size();
  cloud_out.reserve(N);
  cloud_out = src;

  unordered_map<float, int> num_per_instance;
  unordered_map<float, int> num_of_floating_instance;
  for (int i = 0; i < N; ++i) {
    // To reject reflected noises
    if (num_per_instance.find(src.points[i].intensity) == num_per_instance.end()) {
      num_per_instance[src.points[i].intensity] = 1;
    } else {
      num_per_instance[src.points[i].intensity] += 1;
    }
  }
  // #pragma omp parallel for num_threads(num_omp_cores_)
  for (int i = 0; i < N; ++i) {
    // To reject reflected noises
    if (cloud_out.points[i].z < min_z_voi) {
      cloud_out.points[i].intensity = NOT_VOLUME_OF_INTEREST;
    }

    // To reject hanging instances, e.g., leaves from crown of trees
    if (cloud_out.points[i].z > max_z_voi) {
      if (num_of_floating_instance.find(cloud_out.points[i].intensity) ==
          num_of_floating_instance.end()) {
        num_of_floating_instance[cloud_out.points[i].intensity] = 1;
      } else {
        num_of_floating_instance[cloud_out.points[i].intensity] += 1;
      }
      cloud_out.points[i].intensity = NOT_VOLUME_OF_INTEREST;
    }
  }
  vector<float> floating_instances;
  // If over 50 % are filtered, then it's floating
  for (const auto &pair : num_of_floating_instance) {
    if (static_cast<float>(pair.second) / static_cast<float>(num_per_instance[pair.first]) > 0.4) {
      //            std::cout << "\033[1;33mID" << pair.first << "will be ignored because it's a
      //            floating instance\033[0m" << std::endl;
      floating_instances.emplace_back(pair.first);
    }
  }

  for (int i = 0; i < N; ++i) {
    auto it = std::find(
        floating_instances.begin(), floating_instances.end(), cloud_out.points[i].intensity);
    if (it != floating_instances.end()) {
      cloud_out.points[i].intensity = NOT_VOLUME_OF_INTEREST;
    }
  }
}

float ERASOR2::getMaxInstanceId(const pcl::PointCloud<pcl::PointXYZI> &src) {
  float max_id = -1;
  for (const auto &pt : src.points) {
    max_id = max(max_id, pt.intensity);
  }
  return max_id;
}

GridMapInfo ERASOR2::setGridMapParams(const float min_x,
                                      const float min_y,
                                      const float max_x,
                                      const float max_y,
                                      const float grid_resolution) {
  GridMapInfo grid_map_info;
  grid_map_info.center_x   = (min_x + max_x) / 2.0;
  grid_map_info.center_y   = (min_y + max_y) / 2.0;
  grid_map_info.resolution = grid_resolution;

  grid_map_info.width  = static_cast<int>(ceil((max_x - min_x) / grid_resolution));  //
  grid_map_info.height = static_cast<int>(ceil((max_y - min_y) / grid_resolution));  //

  // Remainders of x_length / grid_resolution and
  // y_length / grid_resolution should be zeros, respectively.
  grid_map_info.x_length = ceil((max_x - min_x) / grid_resolution) * grid_resolution;
  grid_map_info.y_length = ceil((max_y - min_y) / grid_resolution) * grid_resolution;

  cout << "x length: " << grid_map_info.x_length << endl;
  cout << "y length: " << grid_map_info.y_length << endl;
  cout << "width: " << grid_map_info.width << endl;
  cout << "height: " << grid_map_info.height << endl;
  cout << "resolution: " << grid_map_info.resolution << endl;

  return grid_map_info;
}

erasor2::GridMap ERASOR2::setMapcentricGridMap(const GridMapInfo &grid_map_info) {
  erasor2::GridMap gridmap({"elevation", "status", "prob", "log_odds", "erosion"});
  gridmap.setFrameId("map");
  gridmap.setGeometry(erasor2::Length(grid_map_info.x_length, grid_map_info.y_length),
                      grid_map_info.resolution);
  gridmap["elevation"].setConstant(NOT_UPDATED);
  gridmap["status"].setConstant(NOT_OBSERVED);
  gridmap["prob"].setConstant(0);
  gridmap["log_odds"].setConstant(0);
  gridmap["erosion"].setConstant(0);
  gridmap.setPosition(erasor2::Position(grid_map_info.center_x, grid_map_info.center_y));
  return gridmap;
}

void ERASOR2::voi2xygrid(const pcl::PointCloud<pcl::PointXYZI> &src,
                         float pos_x,
                         float pos_y,
                         float pos_z,
                         float range,
                         float resolution,
                         vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid,
                         pcl::PointCloud<pcl::PointXYZI> &complement,
                         std::string format) {
  const int width  = static_cast<int>(2.0000001 * range / resolution);
  const int height = static_cast<int>(2.0000001 * range / resolution);
  xygrid.resize(width * height);

  for (auto &grid : xygrid) {
    grid.points.clear();
  }

  for (auto const &pt : src.points) {
    if ((pt.intensity != NOT_VOLUME_OF_INTEREST) && (pt.x < pos_x + range) &&
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

      if (w + width * h > xygrid.size() - 1) {
        std::cout << "range: " << range << " , res: " << resolution << std::endl;
        std::cout << "pt: (" << pt.x << ", " << pt.y << ") | ";
        std::cout << "position: (" << pos_x << " ,, " << pos_y << ")" << std::endl;
        std::cout << range << " => " << pos_x + range << " => " << pos_x + range - pt.x
                  << std::endl;
        std::cout << ((pos_x + range - pt.x) / resolution) << std::endl;
        std::cout << range << " => " << pos_y + range << " => " << pos_y + range - pt.y
                  << std::endl;
        std::cout << ((pos_y + range - pt.y) / resolution) << std::endl;
        std::cout << "Neighboring " << neighboring_width_ << ", " << neighboring_height_
                  << std::endl;
        string error_msg =
            (boost::format("Pixel overflow occurs! w: %d, h: %d, width: %d, height: %d") % w % h %
             width % height)
                .str();
        std::cout << "\033[1;33m" << error_msg << "\033[0m" << std::endl;
        complement.points.push_back(pt);
        continue;
      }
      xygrid[w + width * h].points.emplace_back(pt);
    } else {
      complement.points.push_back(pt);
    }
  }
  std::cout << "\n" << std::endl;
}

void ERASOR2::xygrid2cloud(const vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid,
                           pcl::PointCloud<pcl::PointXYZI> &cloud) {
  cloud.clear();
  for (const auto &partial_cloud : xygrid) {
    cloud += partial_cloud;
  }
}

bool ERASOR2::isLikelyToBeGround(const pcl::PointCloud<pcl::PointXYZI> &pc,
                                 const float ratio_num,
                                 const int num_min_pts) {
  if (pc.empty()) {
    return false;
  }

  int num_ground_pts = erasor_utils::getNumGroundPoints(pc);
  float min_z, max_z;
  erasor_utils::calcMinMaxZ(pc, min_z, max_z);
  ratio_num_ = static_cast<float>(num_ground_pts) / pc.points.size();
  if ((ratio_num_ > ratio_num) && pc.points.size() > num_min_pts &&
      (max_z - min_z < 2 * grid_resolution_)) {
    return true;  // Free
  } else {
    return false;
  }
}

// Original scan ratio test
bool ERASOR2::isLikelyToBeSteppableRegion(const pcl::PointCloud<pcl::PointXYZI> &curr_pc,
                                          const pcl::PointCloud<pcl::PointXYZI> &map_pc,
                                          const float scan_ratio_threshold,
                                          const float min_z_diff_thr,
                                          const bool verbose) {
  if (curr_pc.points.size() < minimum_num_pts_ || map_pc.points.size() < minimum_num_pts_) {
    return false;
  }

  float curr_min_z, curr_max_z;
  float map_min_z, map_max_z;

  //    cout << "FF?" << curr_pc.size() << endl;
  erasor_utils::calcMinMaxZ(
      curr_pc, curr_min_z, curr_max_z);  // ** 현쟈 스캔에서의 min, max z 값을 구해주고,
                                         //    cout << "FF0" << map_pc.size() << endl;
  erasor_utils::calcMinMaxZWithoutGround(
      map_pc, map_min_z, map_max_z);  // ** 맵에서는 ground를 제외한 min, max z 값을 구해준다.
                                      //    cout << "FF!" << endl;
  float curr_mean_ground_z = erasor_utils::calcMeanZOfGround(
      curr_pc);  // label 이 ground 로 분류된 것들의 평균적인 높이 값을 구해주는 함수.
  float map_mean_ground_z = erasor_utils::calcMeanZOfGround(map_pc);

  // ** 아래 부분이 원본 ERASOR 괴ㅏ 동일한 부분
  float lowest_z  = min(curr_mean_ground_z,
                       map_mean_ground_z);  // 스캔과 맵의 평균 ground 높이중에 더 낮은 것을 고르고
  float highest_z = max(curr_max_z, map_max_z);

  //    std::cout << "\033[1;33m" << "curr_mean_ground_z: " << curr_mean_ground_z << " | ";
  //    std::cout << "\033[1;33m" << "map_mean_ground_z: " << map_mean_ground_z << "\033[0m" <<
  //    std::endl; std::cout << "\033[1;33m" << "curr_max_z: " << curr_max_z << " | "; std::cout <<
  //    "\033[1;33m" << "map_max_z: " << map_max_z << "\033[0m" << std::endl; std::cout <<
  //    "\033[1;33m" << "lowest_z: " << lowest_z << " highest_z: " << highest_z << "\033[0m" <<
  //    std::endl;
  if (highest_z == numeric_limits<float>::lowest() || highest_z < lowest_z) {
    //        std::cout << "\033[1;33mThere are no points of our interest\033[0m" << std::endl;
    return false;
  }

  float map_h_diff  = map_max_z - lowest_z;
  float curr_h_diff = curr_max_z - lowest_z;

  scan_ratio_ = min(map_h_diff / curr_h_diff, curr_h_diff / map_h_diff);

  // To reduce false positives
  if (map_h_diff < min_z_diff_thr || curr_mean_ground_z - map_min_z > min_z_diff_thr * 1.5) {
    return false;
  }

  // Dynamic!
  if (scan_ratio_ < scan_ratio_threshold &&
      isLikelyToBeGround(curr_pc, ratio_num_pts_, minimum_num_pts_)) {  // find dynamic!
    return true;  // scan ratio 가 threshold 보다 낮고,
  } else {
    return false;
  }
}

bool ERASOR2::isLikelyToBeSteppableRegionbyBinaryDescriptor(
    const pcl::PointCloud<pcl::PointXYZI> &curr_pc,
    const pcl::PointCloud<pcl::PointXYZI> &map_pc,
    const float scan_ratio_threshold,
    const float min_z_diff_thr,
    const bool verbose) {
  if (curr_pc.points.size() < minimum_num_pts_ || map_pc.points.size() < minimum_num_pts_) {
    return false;
  }

  float curr_min_z, curr_max_z;
  float map_min_z, map_max_z;

  //    cout << "FF?" << curr_pc.size() << endl;
  erasor_utils::calcMinMaxZ(
      curr_pc, curr_min_z, curr_max_z);  // ** 현쟈 스캔에서의 min, max z 값을 구해주고,
                                         //    cout << "FF0" << map_pc.size() << endl;
  erasor_utils::calcMinMaxZWithoutGround(
      map_pc, map_min_z, map_max_z);  // ** 맵에서는 ground를 제외한 min, max z 값을 구해준다.
                                      //    cout << "FF!" << endl;
  float curr_mean_ground_z = erasor_utils::calcMeanZOfGround(
      curr_pc);  // label 이 ground 로 분류된 것들의 평균적인 높이 값을 구해주는 함수.
  float map_mean_ground_z = erasor_utils::calcMeanZOfGround(map_pc);

  // ** 아래 부분이 원본 ERASOR과 동일한 부분
  float lowest_z  = min(curr_mean_ground_z,
                       map_mean_ground_z);  // 스캔과 맵의 평균 ground 높이중에 더 낮은 것을 고르고
  float highest_z = max(curr_max_z, map_max_z);

  std::cout << "\033[1;33m"
            << "curr_mean_ground_z: " << curr_mean_ground_z << " | ";
  std::cout << "\033[1;33m"
            << "map_mean_ground_z: " << map_mean_ground_z << "\033[0m" << std::endl;
  std::cout << "\033[1;33m"
            << "curr_max_z: " << curr_max_z << " | ";
  std::cout << "\033[1;33m"
            << "map_max_z: " << map_max_z << "\033[0m" << std::endl;
  std::cout << "\033[1;33m"
            << "lowest_z: " << lowest_z << " highest_z: " << highest_z << "\033[0m" << std::endl;
  if (highest_z == numeric_limits<float>::lowest() || highest_z < lowest_z) {
    std::cout << "\033[1;33mThere are no points of our interest\033[0m" << std::endl;
    return false;
  }
  float binary_scan_ratio = 1.0;
  // The below condition means all the points of a map are from ground
  int binary_size = static_cast<int>(ceil((highest_z - lowest_z) / voxel_size_));

  vector<int> binary_descriptor_curr(binary_size, 0);
  vector<int> binary_descriptor_map(binary_size, 0);

  for (int i = 0; i < curr_pc.size(); i++) {
    int idx = static_cast<int>(floor((curr_pc[i].z - lowest_z) / voxel_size_));
    if (idx < binary_size && idx >= 0) {
      binary_descriptor_curr[idx] = 1;
    }
  }

  for (int i = 0; i < map_pc.size(); i++) {
    int idx = static_cast<int>(floor((map_pc[i].z - lowest_z) / voxel_size_));
    if (idx < binary_size && idx >= 0) {
      binary_descriptor_map[idx] = 1;
    }
  }

  int count                  = 0;
  int count_actual_occupancy = 0;
  int count_empty            = 0;
  for (int i = 0; i < binary_size; i++) {
    if (binary_descriptor_curr[i] == 1 && binary_descriptor_map[i] == 1) {
      ++count;
    }

    if (binary_descriptor_map[i] == 1) {
      ++count_actual_occupancy;
      count_empty = 0;
    } else {
      ++count_empty;
    }

    // This means that map has weird empty spaces ranging from `1.5`
    if (count_empty > static_cast<int>(min_len_empty_space_ / voxel_size_)) return false;
  }
  binary_scan_ratio = static_cast<float>(count) / static_cast<float>(count_actual_occupancy);

  float map_h_diff  = map_max_z - lowest_z;
  float curr_h_diff = curr_max_z - lowest_z;

  scan_ratio_ = min(map_h_diff / curr_h_diff, curr_h_diff / map_h_diff);

  // To reduce false positives
  if (map_h_diff < min_z_diff_thr || curr_mean_ground_z - map_min_z > min_z_diff_thr * 1.5) {
    return false;
  }

  // Dynamic!
  if (scan_ratio_ < scan_ratio_threshold && binary_scan_ratio < binary_scan_ratio_threshold_ &&
      isLikelyToBeGround(curr_pc, ratio_num_pts_, minimum_num_pts_)) {  // find dynamic!
    return true;  // scan ratio 가 threshold 보다 낮고,
  } else {
    return false;
  }
}

void ERASOR2::updateLogOdds(const erasor2::Index &idx,
                            const float increment,
                            const int kernel_size) {
  gridmap_submap_.at("log_odds", idx) += increment;  // 원래 0에서 시작함

  auto idx_for_neighboring = idx;
  int w                    = idx(0);
  int h                    = idx(1);

  if (kernel_size == 3) {  // ** 이거 일단 안함
    // Refer to https://www.opencv-srf.com/2018/03/gaussian-blur.html
    vector<pair<float, float>> plus_minus_for_adjacent = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto &sign : plus_minus_for_adjacent) {
      idx_for_neighboring(0) = w + sign.first;
      idx_for_neighboring(1) = h + sign.second;
      gridmap_submap_.at("log_odds", idx) += increment / 2.0;
    }

    vector<pair<float, float>> plus_minus_for_diagonal = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    for (const auto &sign : plus_minus_for_diagonal) {
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

erasor2::GridMap ERASOR2::setEgocentricGridMap(
    float range,
    const float grid_resolution,
    const vector<pcl::PointCloud<pcl::PointXYZI>> &xygrid) {
  erasor2::GridMap gridmap({"elevation", "log_odds"});
  gridmap.setFrameId("map");
  gridmap.setGeometry(erasor2::Length(2 * range, 2 * range), grid_resolution);
  gridmap["elevation"].setConstant(NOT_UPDATED);
  gridmap["log_odds"].setConstant(NOT_OBSERVED);

  const int width  = static_cast<int>(2.00000001 * range / grid_resolution);
  const int height = static_cast<int>(2.00000001 * range / grid_resolution);

  erasor2::Index idx;
  for (int u = 0; u < width; ++u) {
    for (int v = 0; v < height; ++v) {
      int i = u + width * v;
      if (isLikelyToBeGround(xygrid[i], ratio_num_pts_, minimum_num_pts_)) {
        idx(0)                      = u;
        idx(1)                      = v;
        gridmap.at("log_odds", idx) = GROUND_EXISTS;
      }
    }
  }
  return gridmap;
}

void ERASOR2::setOccupiedMapIdxes(DynamicInstance &dynamic_cluster) {
  auto &occupied_map_idxes = dynamic_cluster.occupied_map_idxes_;
  auto &log_odds           = dynamic_cluster.log_odds_for_each_point_;
  //    vector<erasor2::Index> occupied_map_idxes;
  occupied_map_idxes.reserve(10);  // heuristic
  log_odds.resize(dynamic_cluster.cloud_.size());
  for (int i = 0; i < dynamic_cluster.cloud_.size(); ++i) {
    const pcl::PointXYZI &dyn_pt = dynamic_cluster.cloud_[i];
    erasor2::Position p_tmp(dyn_pt.x, dyn_pt.y);
    erasor2::Index idx_tmp;
    gridmap_submap_.getIndex(p_tmp, idx_tmp);
    if (idx_tmp(0) > grid_map_info_.width || idx_tmp(0) < 0 || idx_tmp(1) > grid_map_info_.height ||
        idx_tmp(1) < 0) {
      cout << "\033[1;33m[ERASOR2] Index overflow occurs! Skipped\033[0m" << endl;
      continue;
    }

    // 1. Set points' log-odds
    if (gridmap_submap_.at("status", idx_tmp) == NOT_OBSERVED) {
      log_odds[i] = negative_log_odds_;
    } else {
      log_odds[i] = gridmap_submap_.at("log_odds", idx_tmp);
    }

    // 2. Find unique map indices
    if (occupied_map_idxes.empty()) {
      occupied_map_idxes.push_back(idx_tmp);
    } else {
      bool is_first = true;
      for (const erasor2::Index &occupied_region : occupied_map_idxes) {
        // Means already idx_tmp is updated
        if (isEqual(idx_tmp, occupied_region)) {
          is_first = false;
          break;
        }
      }
      if (is_first) {
        occupied_map_idxes.push_back(idx_tmp);
      }
    }
  }
}

void ERASOR2::setMovingInstanceScore(DynamicInstance &dynamic_cluster) {
  float total_score = 0;
  for (const auto &log_odds : dynamic_cluster.log_odds_for_each_point_) {
    total_score += log_odds;
  }
  dynamic_cluster.moving_obj_score_ = total_score / dynamic_cluster.log_odds_for_each_point_.size();
}

void ERASOR2::logOddsGrid2probGrid() {
  erasor2::Index idx;
  for (int h = 0; h < grid_map_info_.height; ++h) {
    for (int w = 0; w < grid_map_info_.width; ++w) {
      idx(0)                          = w;
      idx(1)                          = h;
      gridmap_submap_.at("prob", idx) = logOdds2prob(gridmap_submap_.at("log_odds", idx));
    }
  }
}

void ERASOR2::dilateAndErode(erasor2::GridMap &gridmap_submap) {
  // Noise filtering?
  cv::Mat img, img_eroded, img_dilated;
  gridmap_submap["erosion"]   = gridmap_submap["log_odds"];
  const float min_coefficient = gridmap_submap.get("erosion").minCoeff();
  const float max_coefficient = gridmap_submap.get("erosion").maxCoeff();
  std::cout << min_coefficient << ", " << max_coefficient << std::endl;
  erasor2::GridMapCvConverter::toImage<unsigned char, 1>(
      gridmap_submap, "erosion", CV_8UC1, min_coefficient, max_coefficient, img);
  std::string save_dir = "/home/shapelim/Pictures/erasor2";
  cv::imwrite(save_dir + "/original.png", img);
  cv::dilate(img, img_dilated, cv::Mat::ones(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 1);
  cv::imwrite(save_dir + "/dilation.png", img_dilated);
  cv::erode(img_dilated, img_eroded, cv::Mat::ones(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 2);
  cv::imwrite(save_dir + "/erosion.png", img_eroded);
  erasor2::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
      img_eroded, "erosion", gridmap_submap, min_coefficient, max_coefficient);
  const float min_coefficient_after = gridmap_submap.get("erosion").minCoeff();
  const float max_coefficient_after = gridmap_submap.get("erosion").maxCoeff();
  std::cout << min_coefficient_after << ", " << max_coefficient_after << std::endl;
}

void ERASOR2::erodeGridMap(erasor2::GridMap &gridmap_submap) {
  // Noise filtering?
  cv::Mat img, img_eroded, img_dilated;
  const float min_coefficient = gridmap_submap.get("log_odds").minCoeff();
  const float max_coefficient = gridmap_submap.get("log_odds").maxCoeff();
  std::cout << "\033[1;34m" << min_coefficient << ", " << max_coefficient << std::endl;
  erasor2::GridMapCvConverter::toImage<unsigned char, 1>(
      gridmap_submap, "log_odds", CV_8UC1, min_coefficient, max_coefficient, img);

  cv::erode(img, img_eroded, cv::Mat::ones(cv::Size(3, 3), CV_8UC1), cv::Point(-1, -1), 2);
  erasor2::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
      img_eroded, "eroded", gridmap_submap, min_coefficient, max_coefficient);
  const float min_coefficient_after = gridmap_submap.get("erroded").minCoeff();
  const float max_coefficient_after = gridmap_submap.get("erroded").maxCoeff();
  std::cout << min_coefficient_after << ", " << max_coefficient_after << "\033[0m" << std::endl;
}

// Legacy `publishObjScores` is no longer needed: the publisher itself now
// owns the rerun entity path and the score-formatting logic. Both call
// sites switched to `RejectedMovingObjScorePublisher.publishScores(...)`.
// Kept as a thin compatibility shim in case external code still references
// the symbol.
void ERASOR2::publishObjScores(erasor2::viz::TextArrayPublisher &publisher,
                               const vector<pair<Eigen::Matrix<float, 4, 1>, float>> &objs,
                               const vector<float> color,
                               int &num_prev_objs) {
  std::array<float, 3> col{color.size() > 0 ? color[0] : 1.0f,
                           color.size() > 1 ? color[1] : 1.0f,
                           color.size() > 2 ? color[2] : 1.0f};
  publisher.publishScores(objs, col);
  num_prev_objs = static_cast<int>(objs.size());
}

bool ERASOR2::isCloseToBodyFrame(const DynamicInstance &dynamic_cluster,
                                 const float pos_x,
                                 const float pos_y,
                                 const float range_thr) {
  const float obj_x = dynamic_cluster.centroid_(0);
  const float obj_y = dynamic_cluster.centroid_(1);
  float dist_sqr    = (obj_x - pos_x) * (obj_x - pos_x) + (obj_y - pos_y) * (obj_y - pos_y);
  float thr_sqr     = range_thr * range_thr;
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
  if (abs(min_pt.x - max_pt.x) < xy_size_thr && abs(min_pt.x - max_pt.x) < xy_size_thr) {
    return true;
  } else {
    return false;
  }
}

void ERASOR2::visualizeHardThrRadius(const Eigen::Matrix4f &pose) {
  // Legacy code drew a translucent cylinder marker; rerun has no native
  // cylinder, so we emit a circle line strip at the body height. The Z
  // extent (max_z_voi_ - min_z_voi_) is no longer represented; we only
  // care about the radius for visual sanity-checking.
  Eigen::Vector3f center(pose(0, 3), pose(1, 3), pose(2, 3));
  erasor2::viz::logCircle(AdaptiveRangePublisher.path(), center, hard_thr_radius_);
}

void ERASOR2::printClusterInfo(const DynamicInstance &dynamic_cluster) {
  vector<erasor2::Index> occupied_map_idxes_ = dynamic_cluster.occupied_map_idxes_;
  int min_x                                  = numeric_limits<int>::max();
  int max_x                                  = 0;
  int min_y                                  = numeric_limits<int>::max();
  int max_y                                  = 0;
  for (const auto &idx : occupied_map_idxes_) {
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

  for (const auto &idx : occupied_map_idxes_) {
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
    cout << idx.transpose() << " -> " << x_new << ", " << y_new << " ("
         << gridmap_submap_.at("status", idx) << " | " << gridmap_submap_.at("log_odds", idx) << ")"
         << endl;
  }

  for (auto const &line_to_viz : map) {
    for (int j = 0; j < line_to_viz.size(); ++j) {
      cout << line_to_viz[j];
    }
    cout << endl;
  }
}

void ERASOR2::publishPose(int k) {
  // tf2 broadcaster is gone; rerun's Transform3D under "world/body"
  // serves the same purpose (parents subsequent body-frame logs).
  erasor2::viz::setFrame(static_cast<int64_t>(k));
  PosePublisher.publish(poses_submap_[k]);
}
