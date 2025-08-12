//
// Created by Hyungtae Lim on 24. 1. 30.
//
// #include "../nanoflann/nanoflann.hpp"
// #include "../nanoflann/nanoflann_utils.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

#include <boost/format.hpp>

using num_t              = float;
using RadiusSearchOutput = nanoflann::ResultItem<uint32_t, num_t>;
using my_kd_tree_t =
    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
                                        PointCloud<num_t>,
                                        3>;

// Note that there should exist an already clustered pose
inline int fetchClosestClusteredIdx(const uint32_t i, const vector<bool>& clustered) {
  int N                           = clustered.size();
  bool find_nearest_previous_pose = false;
  bool find_nearest_next_pose     = false;
  int decremental_idx             = static_cast<int>(i) - 1 < 0 ? 0 : static_cast<int>(i) - 1;
  int incremental_idx = static_cast<int>(i) + 1 > N - 1 ? N - 1 : static_cast<int>(i) + 1;

  while (true) {
    if (clustered[decremental_idx]) {
      return decremental_idx;
    } else {
      decremental_idx = decremental_idx - 1 < 0 ? 0 : decremental_idx - 1;
    }

    if (clustered[incremental_idx]) {
      return incremental_idx;
    } else {
      incremental_idx = incremental_idx + 1 > N - 1 ? N - 1 : incremental_idx + 1;
    }
  }
}

inline void fetchDistantClusteredIdx(size_t& prev_idx,
                                     size_t& next_idx,
                                     const size_t i,
                                     const vector<Eigen::Vector3f>& poses,
                                     const float dist_threshold) {
  const int N            = poses.size();
  size_t decremental_idx = i - 1 < 1 ? 0 : i - 1;
  size_t incremental_idx = i + 1 > N - 2 ? N - 1 : i + 1;

  if (decremental_idx == 0) {
    prev_idx = 0;
  }
  if (incremental_idx == N - 1) {
    next_idx = N - 1;
  }

  while (decremental_idx > 0) {
    float dist = (poses[i] - poses[decremental_idx]).norm();
    if (dist > dist_threshold) {
      prev_idx = decremental_idx;
      break;
    } else {
      decremental_idx = decremental_idx - 1 < 1 ? 0 : decremental_idx - 2;
    }
  }

  while (incremental_idx < N - 1) {
    float dist = (poses[i] - poses[incremental_idx]).norm();
    if (dist > dist_threshold) {
      next_idx = incremental_idx;
      break;
    } else {
      incremental_idx = incremental_idx + 1 > N - 2 ? N - 1 : incremental_idx + 2;
    }
  }
}

inline vector<pair<uint32_t, uint32_t>> sortAndFindDiscontinuousFrameNum(
    vector<RadiusSearchOutput>& ret_matches) {
  sort(ret_matches.begin(), ret_matches.end(), [](const auto& a, const auto& b) {
    return a.first < b.first;
  });
  vector<pair<uint32_t, uint32_t>> discontinuous_indices;
  for (size_t j = 0; j < ret_matches.size() - 1; j++) {
    if (ret_matches[j].first + 1 != ret_matches[j + 1].first) {
      //            std::cout << "\033[1;33mDiscnotinuous indices detected:" << ret_matches[j].first
      //            << " vs " << ret_matches[j + 1].first << "\033[0m\n";
      discontinuous_indices.emplace_back(ret_matches[j].first, ret_matches[j + 1].first);
    }
  }
  return discontinuous_indices;
}

// To deal with some corner cases
inline void connectDiscontinuousTrajectory(vector<RadiusSearchOutput>& ret_matches,
                                           const vector<Eigen::Vector3f>& poses,
                                           const size_t i,
                                           const int sufficiently_small_frame_interval = 100) {
  vector<pair<uint32_t, uint32_t>> discontinuous_frame_nums =
      sortAndFindDiscontinuousFrameNum(ret_matches);

  for (const auto& discontinuous_frame_pair : discontinuous_frame_nums) {
    //        std::cout << discontinuous_frame_pair.first << " <---> " <<
    //        discontinuous_frame_pair.second << std::endl;
    int frame_interval = abs(static_cast<int>(discontinuous_frame_pair.first) -
                             static_cast<int>(discontinuous_frame_pair.second));
    // If these are sufficiently close:
    if (frame_interval < sufficiently_small_frame_interval) {
      for (uint32_t k = discontinuous_frame_pair.first + 1; k < discontinuous_frame_pair.second;
           k++) {
        float sqr_dist = (poses[k] - poses[i]).squaredNorm();
        ret_matches.emplace_back(RadiusSearchOutput(k, sqr_dist));
      }
    }
  }
}

inline vector<vector<RadiusSearchOutput>> separateLoopClusters(
    vector<RadiusSearchOutput>& ret_matches,
    const vector<Eigen::Vector3f>& poses,
    const size_t i) {
  connectDiscontinuousTrajectory(ret_matches, poses, i);
  const auto discontinuous_frame_pairs = sortAndFindDiscontinuousFrameNum(ret_matches);

  vector<uint32_t> indices_tmp;
  for (const auto& match : ret_matches) {
    indices_tmp.emplace_back(match.first);
  }

  vector<vector<RadiusSearchOutput>> separated_clusters;
  for (uint32_t j = 0; j < discontinuous_frame_pairs.size() + 1; j++) {
    vector<RadiusSearchOutput> clusters;
    uint32_t start_idx;
    if (j == 0) {
      start_idx = 0;
    } else {
      auto it = std::find(
          indices_tmp.begin(), indices_tmp.end(), discontinuous_frame_pairs[j - 1].second);
      if (it != indices_tmp.end()) {
        start_idx = std::distance(indices_tmp.begin(), it);
      } else {
        throw std::runtime_error("Value not found in the vector!");
      }
    }
    uint32_t end_idx;
    if (j != discontinuous_frame_pairs.size()) {
      auto it =
          std::find(indices_tmp.begin(), indices_tmp.end(), discontinuous_frame_pairs[j].first);
      if (it != indices_tmp.end()) {
        end_idx = std::distance(indices_tmp.begin(), it);
      } else {
        throw std::runtime_error("Value not found in the vector!");
      }
    } else {
      end_idx = indices_tmp.size() - 1;  // 마지막 인덱스
    }
    //        std:cout << "Checking indices: " << start_idx << " " << end_idx << std::endl;
    // `end_idx` should be included, i.e., <= is required
    for (int k = start_idx; k <= end_idx; ++k) {
      clusters.emplace_back(ret_matches[k]);
    }
    separated_clusters.emplace_back(clusters);
  }
  return separated_clusters;
}

inline void extractClusters(std::vector<bool>& clustered,
                            my_kd_tree_t& index,
                            vector<vector<RadiusSearchOutput>>& cluster_candidates,
                            const vector<Eigen::Vector3f>& poses,
                            const bool only_consider_crossroad,
                            const bool only_consider_revisit,
                            const bool separate_revisited_scenes,
                            const float angle_threshold,
                            const float dist_threshold,
                            const int min_num_neighbors,
                            const float clustering_radius,
                            const int min_frame_interval_for_revisit) {
  int N = clustered.size();
  vector<RadiusSearchOutput> ret_matches;

  for (uint32_t i = 1; i < N - 1; i += 2) {
    auto& curr_point = poses[i];

    if ((!clustered[i])) {
      // To prevent a cluster from being too close to the previous cluster,
      //  check the distance between the current point and the nearest previous/next clustered point
      int closest_idx = fetchClosestClusteredIdx(i, clustered);
      float dist      = (curr_point - poses[closest_idx]).norm();
      if (dist < 2.0 / 3.0 * clustering_radius) {
        continue;
      }

      const num_t query_pt[3] = {curr_point(0), curr_point(1), curr_point(2)};
      auto num_results =
          index.radiusSearch(&query_pt[0], clustering_radius * clustering_radius, ret_matches);

      bool has_revisit = false;
      uint32_t revisit_idx;
      for (const auto match : ret_matches) {
        int frame_diff = i - match.first;
        if (abs(frame_diff) > min_frame_interval_for_revisit) {
          has_revisit = true;
          revisit_idx = match.first;
          break;
        }
      }

      if (has_revisit && only_consider_revisit) {
        bool is_valid = true;
        if (only_consider_crossroad) {
          size_t prev_idx, next_idx;
          fetchDistantClusteredIdx(prev_idx, next_idx, i, poses, dist_threshold);
          auto& prev_point = poses[prev_idx];
          auto& curr_point = poses[i];

          fetchDistantClusteredIdx(prev_idx, next_idx, revisit_idx, poses, dist_threshold);
          auto& prev_revisit_point = poses[prev_idx];
          auto& revisit_point      = poses[revisit_idx];

          Eigen::Vector3f curr_dir    = curr_point - prev_point;
          Eigen::Vector3f revisit_dir = revisit_point - prev_revisit_point;

          if (curr_dir.norm() < 1e-1 || revisit_dir.norm() < 1e-1) continue;

          float angle =
              RAD2DEG(acos(curr_dir.dot(revisit_dir) / (curr_dir.norm() * revisit_dir.norm())));

          if (angle < 90.0 - angle_threshold || angle > 90.0 + angle_threshold) {
            is_valid = false;
          }
        }

        if (is_valid && ret_matches.size() > min_num_neighbors) {
          if (separate_revisited_scenes) {
            const auto ret_matches_sets = separateLoopClusters(ret_matches, poses, i);
            for (const auto& separated_ret_matches : ret_matches_sets) {
              cluster_candidates.emplace_back(separated_ret_matches);
              for (const auto& match : separated_ret_matches) {
                clustered[match.first] = true;
              }
            }
          } else {
            sort(ret_matches.begin(), ret_matches.end(), [](const auto& a, const auto& b) {
              return a.first < b.first;
            });
            cluster_candidates.emplace_back(ret_matches);
            for (const auto& match : ret_matches) {
              clustered[match.first] = true;
            }
          }
        }
      }

      if (!has_revisit && !only_consider_revisit) {
        if (ret_matches.size() > min_num_neighbors) {
          if (separate_revisited_scenes) {
            const auto ret_matches_sets = separateLoopClusters(ret_matches, poses, i);
            for (const auto& separated_ret_matches : ret_matches_sets) {
              cluster_candidates.emplace_back(separated_ret_matches);
              for (const auto& match : separated_ret_matches) {
                clustered[match.first] = true;
              }
            }
          } else {
            sort(ret_matches.begin(), ret_matches.end(), [](const auto& a, const auto& b) {
              return a.first < b.first;
            });
            cluster_candidates.emplace_back(ret_matches);
            for (const auto& match : ret_matches) {
              clustered[match.first] = true;
            }
          }
        }
      }
    }
  }
}

/*
 * min_num_neighbors: For each frame, a autonomous car moves about 0.3 m.
 *                    i.e., `min_num_neighbors` $\sim$ `clustering_radius` / 0.3
 */
inline vector<vector<size_t>> clusterTrajectoryXYZ(
    const vector<Eigen::Vector3f>& poses,
    const float angle_threshold                       = 10,
    const float dist_threshold_for_crossroad_checking = 3.0,
    const float clustering_radius                     = 50.0,
    const int min_num_neighbors                       = 200,
    const int min_frame_interval_for_revisit          = 500) {
  PointCloud<num_t> cloud;
  int N = poses.size();
  cloud.pts.resize(N);
  for (size_t n = 0; n < N; n++) {
    cloud.pts[n].x = poses[n](0);
    cloud.pts[n].y = poses[n](1);
    cloud.pts[n].z = poses[n](2);
  }

  my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

  std::vector<bool> clustered(N, false);
  // Step 1. Find crossroad by using angular difference and # of points
  vector<vector<RadiusSearchOutput>> trajectory_clusters;
  vector<RadiusSearchOutput> ret_matches;

  // Step 1-1. Find crossroad by using angular difference between the local trajectory
  size_t prev_idx;
  size_t next_idx;
  // 1 & N-1: To avoid exceptional cases
  // 2: To reduce redundant computation
  for (size_t i = 1; i < N - 1; i += 2) {
    fetchDistantClusteredIdx(prev_idx, next_idx, i, poses, dist_threshold_for_crossroad_checking);
    //        std::cout << prev_idx << " " << i << " " << next_idx << std::endl;
    auto& prev_point = poses[prev_idx];
    auto& curr_point = poses[i];
    auto& next_point = poses[next_idx];

    Eigen::Vector3f prev_dir = curr_point - prev_point;
    Eigen::Vector3f next_dir = next_point - curr_point;

    // To prevent the case that the car is stopped
    if (prev_dir.norm() < 1e-1 || next_dir.norm() < 1e-1) continue;

    float angle = RAD2DEG(acos(prev_dir.dot(next_dir) / (prev_dir.norm() * next_dir.norm())));

    if ((!clustered[i]) && (angle > angle_threshold && angle < 180.0 - angle_threshold)) {
      const num_t query_pt[3] = {curr_point(0), curr_point(1), curr_point(2)};

      auto num_results =
          index.radiusSearch(&query_pt[0], clustering_radius * clustering_radius, ret_matches);

      std::cout << prev_idx << " <-> " << i << " <-> " << next_idx << " | " << angle << " > "
                << angle_threshold << " | " << ret_matches.size() << std::endl;

      if (ret_matches.size() > min_num_neighbors) {
        connectDiscontinuousTrajectory(ret_matches, poses, i);
        trajectory_clusters.emplace_back(ret_matches);
        for (const auto& match : ret_matches) {
          clustered[match.first] = true;
        }
      }
    }
  }

  // Step 1-2 Check the crossroad scenes
  extractClusters(clustered,
                  index,
                  trajectory_clusters,
                  poses,
                  true,
                  true,
                  false,
                  45.0,
                  dist_threshold_for_crossroad_checking,
                  min_num_neighbors,
                  clustering_radius,
                  min_frame_interval_for_revisit);
  // Step 1-3 Check the revisited scenes
  extractClusters(clustered,
                  index,
                  trajectory_clusters,
                  poses,
                  false,
                  true,
                  false,
                  45.0,
                  dist_threshold_for_crossroad_checking,
                  min_num_neighbors,
                  clustering_radius,
                  min_frame_interval_for_revisit);
  /*
   * Step 1-4 For a trajectory visited only once
   * Thus, `min_num_neighbors` should be set to more lenient threshold
   */
  extractClusters(clustered,
                  index,
                  trajectory_clusters,
                  poses,
                  false,
                  false,
                  false,
                  45.0,
                  dist_threshold_for_crossroad_checking,
                  min_num_neighbors * 2 / 3,
                  clustering_radius,
                  min_frame_interval_for_revisit);
  extractClusters(clustered,
                  index,
                  trajectory_clusters,
                  poses,
                  false,
                  false,
                  false,
                  45.0,
                  dist_threshold_for_crossroad_checking,
                  min_num_neighbors * 2 / 3,
                  clustering_radius * 2.0 / 3.0,
                  min_frame_interval_for_revisit);

  const int not_assigned = -1;
  std::vector<int> cluster_ids(N, not_assigned);

  int cluster_id = 0;
  vector<vector<size_t>> cluster_indices;
  for (const auto& cluster : trajectory_clusters) {
    vector<size_t> indices_for_each_cluster;
    for (const auto& match : cluster) {
      if (cluster_ids[match.first] == not_assigned) {
        cluster_ids[match.first] = cluster_id;
        indices_for_each_cluster.push_back(match.first);
      }
    }
    cluster_indices.emplace_back(indices_for_each_cluster);

    ++cluster_id;
  }

  // For unclustered poses
  for (size_t i = 0; i < N; ++i) {
    if (cluster_ids[i] == not_assigned) {
      int closest_idx = fetchClosestClusteredIdx(i, clustered);
      cluster_indices[cluster_ids[closest_idx]].push_back(i);
    }
  }
  return cluster_indices;
}

inline vector<vector<size_t>> clusterTrajectoryXYZDistinguishingTemporalTrajectories(
    const vector<Eigen::Vector3f>& poses,
    const float angle_threshold                       = 10,
    const float dist_threshold_for_crossroad_checking = 3.0,
    const float clustering_radius                     = 50.0,
    const int min_num_neighbors                       = 200,
    const int min_frame_interval_for_revisit          = 500,
    const bool random_shuffle_for_better_viz          = false) {
  PointCloud<num_t> cloud;
  int N = poses.size();
  cloud.pts.resize(N);
  for (size_t n = 0; n < N; n++) {
    cloud.pts[n].x = poses[n](0);
    cloud.pts[n].y = poses[n](1);
    cloud.pts[n].z = poses[n](2);
  }

  my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

  std::vector<bool> clustered(N, false);
  // Step 1. Find crossroad by using angular difference and # of points
  vector<vector<RadiusSearchOutput>> trajectory_clusters;
  vector<RadiusSearchOutput> ret_matches;

  // Step 1-1. Find crossroad by using angular difference between the local trajectory
  size_t prev_idx;
  size_t next_idx;
  // 1 & N-1: To avoid exceptional cases
  // 2: To reduce redundant computation
  for (size_t i = 1; i < N - 1; i += 2) {
    fetchDistantClusteredIdx(prev_idx, next_idx, i, poses, dist_threshold_for_crossroad_checking);
    //        std::cout << prev_idx << " " << i << " " << next_idx << std::endl;
    auto& prev_point = poses[prev_idx];
    auto& curr_point = poses[i];
    auto& next_point = poses[next_idx];

    Eigen::Vector3f prev_dir = curr_point - prev_point;
    Eigen::Vector3f next_dir = next_point - curr_point;

    float angle = RAD2DEG(acos(prev_dir.dot(next_dir) / (prev_dir.norm() * next_dir.norm())));

    if ((!clustered[i]) && (angle > angle_threshold && angle < 180.0 - angle_threshold)) {
      const num_t query_pt[3] = {curr_point(0), curr_point(1), curr_point(2)};

      auto num_results =
          index.radiusSearch(&query_pt[0], clustering_radius * clustering_radius, ret_matches);

      //            std::cout << i << " | " << angle << " > " << angle_threshold << "->" <<
      //            ret_matches.size() << std::endl;
      if (ret_matches.size() > min_num_neighbors) {
        const auto ret_matches_sets = separateLoopClusters(ret_matches, poses, i);
        //                std::cout << "ret_matches_sets.size(): " << ret_matches_sets.size() <<
        //                std::endl;
        for (const auto& separated_ret_matches : ret_matches_sets) {
          trajectory_clusters.emplace_back(separated_ret_matches);
          for (const auto match : separated_ret_matches) {
            clustered[match.first] = true;
          }
        }
      }
    }
  }

  // Step 1-2 Check the crossroad scenes
  extractClusters(clustered,
                  index,
                  trajectory_clusters,
                  poses,
                  true,
                  true,
                  true,
                  45.0,
                  dist_threshold_for_crossroad_checking,
                  min_num_neighbors,
                  clustering_radius,
                  min_frame_interval_for_revisit);
  // Step 1-3 Check the revisited scenes
  extractClusters(clustered,
                  index,
                  trajectory_clusters,
                  poses,
                  false,
                  true,
                  true,
                  45.0,
                  dist_threshold_for_crossroad_checking,
                  min_num_neighbors,
                  clustering_radius,
                  min_frame_interval_for_revisit);
  /*
   * Step 1-4 For a trajectory visited only once
   * Thus, `min_num_neighbors` should be set to more lenient threshold
   */
  extractClusters(clustered,
                  index,
                  trajectory_clusters,
                  poses,
                  false,
                  false,
                  true,
                  45.0,
                  dist_threshold_for_crossroad_checking,
                  min_num_neighbors * 2 / 3,
                  clustering_radius,
                  min_frame_interval_for_revisit);
  extractClusters(clustered,
                  index,
                  trajectory_clusters,
                  poses,
                  false,
                  false,
                  true,
                  45.0,
                  dist_threshold_for_crossroad_checking,
                  min_num_neighbors * 2 / 3,
                  clustering_radius * 2.0 / 3.0,
                  min_frame_interval_for_revisit);

  const int not_assigned = -1;
  std::vector<int> cluster_ids(N, not_assigned);

  int cluster_id = 0;
  vector<vector<size_t>> cluster_indices;
  if (random_shuffle_for_better_viz) {
    std::random_device seed;
    std::mt19937 engine(seed());
    std::shuffle(trajectory_clusters.begin(), trajectory_clusters.end(), engine);
  }

  for (const auto& cluster : trajectory_clusters) {
    vector<size_t> indices_for_each_cluster;
    for (const auto& match : cluster) {
      if (cluster_ids[match.first] == not_assigned) {
        cluster_ids[match.first] = cluster_id;
        indices_for_each_cluster.push_back(match.first);
      }
    }
    cluster_indices.emplace_back(indices_for_each_cluster);

    ++cluster_id;
  }

  // For unclustered poses
  for (size_t i = 0; i < N; ++i) {
    if (cluster_ids[i] == not_assigned) {
      int closest_idx = fetchClosestClusteredIdx(i, clustered);
      cluster_indices[cluster_ids[closest_idx]].push_back(i);
    }
  }
  return cluster_indices;
}

template <typename T>
vector<vector<int>> getContinuousSegments(vector<T>& indices) {
  sort(indices.begin(), indices.end(), [](const auto& a, const auto& b) { return a < b; });

  vector<vector<int>> continuous_segments;
  vector<int> current_segment;
  int previous_idx = -2;  // 초기값은 연속성이 없음을 가정

  for (int idx : indices) {
    if (previous_idx + 1 != idx) {
      if (!current_segment.empty()) {
        continuous_segments.push_back(current_segment);
        current_segment.clear();
      }
    }
    current_segment.push_back(idx);
    previous_idx = idx;
  }
  // add last `current_segment`
  if (!current_segment.empty()) {
    continuous_segments.push_back(current_segment);
  }
  return continuous_segments;
}

template <typename T>
vector<size_t> getExpandedFrameNums(vector<T> indices, const int range, const int max_frame) {
  // To avoid not sorting the original indices
  sort(indices.begin(), indices.end(), [](const auto& a, const auto& b) { return a < b; });

  vector<size_t> edge_frame_nums;
  vector<vector<int>> continuous_segments;
  vector<int> current_segment;
  int previous_idx = -2;

  for (int i = indices[0] - range; i < indices[0]; ++i) {
    if (i < 0) {
      continue;
    }
    edge_frame_nums.push_back(i);
  }

  for (int idx : indices) {
    if (previous_idx + 1 != idx) {
      if (!current_segment.empty()) {
        continuous_segments.push_back(current_segment);
        current_segment.clear();

        for (int i = previous_idx - range; i < previous_idx; ++i) {
          if (i < 0) {
            continue;
          }
          edge_frame_nums.push_back(i);
        }

        for (int i = idx; i < idx + range; ++i) {
          if (i > max_frame - 1) {
            continue;
          }
          edge_frame_nums.push_back(i);
        }
      }
    }
    current_segment.push_back(idx);
    previous_idx = idx;
  }
  // add last `current_segment`
  if (!current_segment.empty()) {
    continuous_segments.push_back(current_segment);
  }

  for (int i = current_segment.back(); i < current_segment.back() + range; ++i) {
    if (i > max_frame - 1) {
      continue;
    }
    edge_frame_nums.push_back(i);
  }
  return edge_frame_nums;
}
