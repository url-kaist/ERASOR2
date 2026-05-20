//
// Created by shapelim on 21. 10. 18..
//

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

#include "erasor2/Config.hpp"
#include "erasor2/RerunLogger.hpp"
#include "erasor2/erasor2.h"

#include "dataloader/dataloader.h"
#include "dataprocessor/TrajectoryClustering.hpp"
#include "tools/erasor_utils.hpp"

using namespace std;

using PointType = pcl::PointXYZI;

vector<Eigen::Vector3f> getPoses(const DataLoader &loader,
                                 const int start_frame,
                                 const int end_frame,
                                 const int accum_interval) {
  // For fetching loops
  vector<Eigen::Vector3f> positions;
  vector<Eigen::Matrix4f> poses_gt;
  Eigen::Matrix4f pose_tmp = Eigen::Matrix4f::Identity();
  for (int i = start_frame; i < end_frame + accum_interval; ++i) {
    pose_tmp = loader.poses_gt_[i];
    poses_gt.emplace_back(pose_tmp);
  }
  for (auto const &pose : poses_gt) {
    Eigen::Vector3f position_vec(pose(0, 3), pose(1, 3), pose(2, 3));
    positions.emplace_back(position_vec);
  }
  return positions;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: run_erasor2 <config.yaml>\n";
    return 1;
  }
  const auto cfg = erasor2::Config::fromYaml(argv[1]);
  if (cfg.rerun_enabled) {
    erasor2::viz::init("erasor2", cfg.rerun_spawn, cfg.rerun_save_path);
  }

  std::cout << "ERASOR2 started" << std::endl;
  unique_ptr<RosParamServer> params(new RosParamServer(cfg));
  std::cout << "Set ERASOR2 complete" << std::endl;

  std::unique_ptr<DataLoader> loader;
  string dataset_name = params->dataset_name_;
  if (dataset_name == "SemanticKITTI") {
    loader = std::move(std::make_unique<SemanticKITTILoader>(
        params->abs_data_dir_, params->sequence_, params->instance_seg_method_));
  }

  else if (dataset_name == "HeLiPR") {
    loader = std::move(std::make_unique<HeLiPRLoader>(
        params->abs_data_dir_, params->sequence_, params->instance_seg_method_));
  }

  cout << "Set dataloader complete" << endl;
  cout << "From " << params->start_frame_ << " to " << params->end_frame_ << endl;
  cout << params->robot_body_size_ << endl;

  int start_frame = params->start_frame_;
  int end_frame   = params->end_frame_;
  int accum_interval =
      params->accum_interval_;  // 여기서는 accumulation 의 interval이 2로 설정되어 있기는 함.

  string dynamic_label_root = params->abs_save_dir_ + "/" + "mos";
  if (!std::filesystem::exists(params->abs_save_dir_)) {
    std::filesystem::create_directory(params->abs_save_dir_);
  }
  if (!std::filesystem::exists(dynamic_label_root)) {
    std::filesystem::create_directory(dynamic_label_root);
  }

  const auto pose_cloud = getPoses(*loader, start_frame, end_frame, accum_interval);

  vector<vector<size_t>> frames_clusters;
  if (params->run_traj_clustering_) {
    if (params->distinguish_temporal_trajectories_) {
      frames_clusters = clusterTrajectoryXYZDistinguishingTemporalTrajectories(pose_cloud);
    } else {
      frames_clusters = clusterTrajectoryXYZ(pose_cloud);
    }
    const auto &[clustered, unclustered] =
        erasor_utils::clusterIndices2PointCloud(pose_cloud, frames_clusters);
    if (!unclustered.empty()) {
      throw runtime_error("Some poses are not clustered!");
    }
    std::cout << "\033[1;32m" << frames_clusters.size() << " clusters are found\033[0m"
              << std::endl;

    // Log clustered/unclustered trajectories to rerun for paper viz.
    erasor2::viz::logCloud("trajectory/unclustered", unclustered);
    erasor2::viz::logCloud("trajectory/clustered", clustered);
  } else {
    frames_clusters.clear();
    vector<size_t> indices_tmp;
    for (size_t i = start_frame; i < end_frame + accum_interval; i += accum_interval) {
      indices_tmp.emplace_back(i);
    }
    frames_clusters.emplace_back(indices_tmp);
  }

  std::cout << "\033[1;32mStart to run ERASOR2\033[0m" << std::endl;
  int cluster_id = 0;
  unordered_map<size_t, Eigen::Matrix4f> corrected_poses;
  for (auto &indices : frames_clusters) {
    vector<size_t> expanded_frames = indices;
    if (params->expansion_range_ > 0) {
      const auto edge_frames_for_expansion = getExpandedFrameNums(indices, 20, end_frame);
      for (auto &frame : edge_frames_for_expansion) {
        if (std::find(expanded_frames.begin(), expanded_frames.end(), frame) ==
            expanded_frames.end()) {
          expanded_frames.emplace_back(frame);
        }
      }
    }

    if (params->run_traj_clustering_ && !params->distinguish_temporal_trajectories_ &&
        params->correct_poses_by_submap_matching_) {
      vector<size_t> frames_for_correction = expanded_frames;
      //            const auto continuous_segments = getContinuousSegments(indices);
      const auto continuous_segments = getContinuousSegments(frames_for_correction);
      std::cout << "# of continuous_segments: " << continuous_segments.size() << std::endl;

      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> submaps;

      // 각 세그먼트별로 submap 생성 및 저장
      for (const auto &segment : continuous_segments) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr submap(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr submap_voxelized(
            new pcl::PointCloud<pcl::PointXYZI>());

        static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        const float voxel_size = params->voxel_size_for_pose_correction_;

        for (int frame_idx : segment) {
          corrected_poses[frame_idx] = Eigen::Matrix4f::Identity();

          pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
          pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxelized(
              new pcl::PointCloud<pcl::PointXYZI>());
          pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
          Eigen::Matrix4f pose =
              Eigen::Matrix4f::Identity();  // 가정: loader->getScanAndPose 함수에서 pose를 가져옴
          loader->getScanAndPose(frame_idx, *cloud, pose);
          voxel_filter.setInputCloud(cloud);
          const float half_vs = voxel_size / 2.0f;  // Just heuristics
          voxel_filter.setLeafSize(half_vs, half_vs, half_vs);
          voxel_filter.filter(*cloud_voxelized);
          pcl::transformPointCloud(
              *cloud_voxelized, *transformed, pose * params->tf_h_of_ground_to_be_zero_);
          *submap += *transformed;
        }
        voxel_filter.setInputCloud(submap);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.filter(*submap_voxelized);
        submaps.push_back(submap_voxelized);
      }

      // 첫 번째 submap을 기준으로 G-ICP 실행
      pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
      gicp.setMaxCorrespondenceDistance(params->max_corr_dist_for_pose_correction_);
      gicp.setMaximumIterations(100);
      gicp.setTransformationEpsilon(1e-8);
      gicp.setEuclideanFitnessEpsilon(1);
      vector<Eigen::Matrix4f> corrected_transforms(submaps.size(), Eigen::Matrix4f::Identity());
      for (size_t i = 1; i < submaps.size(); ++i) {
        std::cout << "Aligning submap " << i << " to submap 0..." << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        gicp.setInputSource(submaps[i]);
        gicp.setInputTarget(submaps[0]);  // 첫 번째 submap을 대상으로 설정
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        gicp.align(*aligned, initial_guess);

        if (gicp.hasConverged()) {
          std::cout << "G-ICP has converged, score: " << gicp.getFitnessScore() << std::endl;
          // 결과 변환 행렬
          Eigen::Matrix4f transformation = gicp.getFinalTransformation();
          std::cout << "Transformation matrix: " << std::endl << transformation << std::endl;

          for (const auto &frame_idx : continuous_segments[i]) {
            corrected_poses[frame_idx] = transformation;
          }
        } else {
          std::cout << "G-ICP did not converge for segment " << i << std::endl;
        }
        //                ros::Publisher pub_src =
        //                nh.advertise<sensor_msgs::PointCloud2>("/map_registration/src",100, true);
        //                ros::Publisher pub_tgt =
        //                nh.advertise<sensor_msgs::PointCloud2>("/map_registration/tgt",100, true);
        //                ros::Publisher pub_est =
        //                nh.advertise<sensor_msgs::PointCloud2>("/map_registration/est",100, true);
        //                sensor_msgs::PointCloud2 output;
        //                pcl::toROSMsg(*submaps[i], output);
        //                output.header.frame_id = "map";
        //
        //                pub_src.publish(output);
        //                pcl::toROSMsg(*submaps[0], output);
        //                output.header.frame_id = "map";
        //                pub_tgt.publish(output);
        //
        //                pcl::toROSMsg(*aligned, output);
        //                output.header.frame_id = "map";
        //                pub_est.publish(output);
        //                std::cout << "Press Enter to continue...";
        //                cin.ignore();
      }
    }

    unique_ptr<ERASOR2> erasor2(new ERASOR2(cfg));
    for (const auto &idx : expanded_frames) {
      signal(SIGINT, erasor_utils::signal_callback_handler);
      //            if (idx % 10 == 0) {
      //                cout << "[DataLoader] " << idx << "th frame comes!\n";
      //            }

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gt_label(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_label(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gt_filtered(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_est_filtered(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr noise(new pcl::PointCloud<pcl::PointXYZI>);

      Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
      // A scan contains label in `intensity`
      if (dataset_name == "SemanticKITTI") {
        loader->getGTLabeledScan(idx, *cloud_gt_label);
        loader->rejectNeighboringPoints(
            *cloud_gt_label, erasor2->robot_body_size_, *cloud_gt_filtered, *noise);
      }

      loader->getScanAndPose(idx, *cloud_est_label, pose);
      // cout << idx << "th pose: " << endl << pose << endl;
      // cout << idx << "th cloud size: " << cloud_est_label->size() << endl;
      loader->rejectNeighboringPoints(
          *cloud_est_label, erasor2->robot_body_size_, *cloud_est_filtered, *noise);
      //            std::cout << "cloud_est_filtered size: " << cloud_est_filtered->size() <<
      //            std::endl;
      if (params->run_traj_clustering_ && !params->distinguish_temporal_trajectories_ &&
          params->correct_poses_by_submap_matching_) {
        pose = corrected_poses[idx] * pose;
      }
      if (dataset_name == "SemanticKITTI") {
        erasor2->setScanAndPose(pose, *cloud_gt_filtered, *cloud_est_filtered);
      } else {
        erasor2->setScanAndPose(pose, *cloud_est_filtered);
      }
    }
    cout << "[ERASOR2] Complete to set scans and poses\n";

    erasor2->setSubmap();
    erasor2->updateSteppableRegion();
    //    erasor2->dilateAndErode(); // not using
    erasor2->detectMovingObjects();
    erasor2->filterDynamicObjects();  // ? Original
    // erasor2->filterDynamicObjectsAndSaveLabel(dynamic_label_root, start_frame); // ? Modified for
    // saving dynamic labels
    erasor2->saveDynamicLabels(dynamic_label_root, indices);
    // dynamic_label_root

    if (params->save_map_) {
      string map_path = params->abs_save_dir_ + "/" + params->sequence_ + "_" +
                        to_string(cluster_id) + "_frame_" + to_string(start_frame) + "_to_" +
                        to_string(end_frame) + "_estimated.pcd";
      erasor2->saveStaticMap(map_path);
      erasor2->publishStaticMapResults();
    }

    ++cluster_id;
  }
  erasor2::viz::shutdown();
  return 0;
}
