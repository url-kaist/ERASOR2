#include <filesystem>

#include <pcl/registration/gicp.h>

#include "tools/erasor_utils.hpp"

void loadAndParse(const string &abs_pcd_path,
                  sensor_msgs::PointCloud2 &static_cloud_msg,
                  sensor_msgs::PointCloud2 &dynamic_cloud_msg) {
  if (abs_pcd_path == "") {
    std::cout << "\033[1;33mEmpty path is given\033[0m" << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  erasor_utils::load_pcd(abs_pcd_path, cloud);
  std::cout << "\033[1;32mLoad complete \033[0m" << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_static(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_dynamic(new pcl::PointCloud<pcl::PointXYZI>);
  erasor_utils::parseStaticAndDynamic(*cloud, *cloud_dynamic, *cloud_static);

  dynamic_cloud_msg = erasor_utils::cloud2msg(*cloud_dynamic);
  static_cloud_msg  = erasor_utils::cloud2msg(*cloud_static);
}

void getNegativeClouds(const pcl::PointCloud<pcl::PointXYZI> &est_cloud,
                       sensor_msgs::PointCloud2 &tn,
                       sensor_msgs::PointCloud2 &fn) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_static(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_dynamic(new pcl::PointCloud<pcl::PointXYZI>);
  erasor_utils::parseStaticAndDynamic(est_cloud, *cloud_dynamic, *cloud_static);

  tn = erasor_utils::cloud2msg(*cloud_static);
  fn = erasor_utils::cloud2msg(*cloud_dynamic);
}

// Both should be voxelized with same voxel size
void getPositiveClouds(const pcl::PointCloud<pcl::PointXYZI> &raw_cloud,
                       const pcl::PointCloud<pcl::PointXYZI> &est_cloud,
                       sensor_msgs::PointCloud2 &tp,
                       sensor_msgs::PointCloud2 &fp,
                       const float voxel_size = 0.2) {
  pcl::PointCloud<pcl::PointXYZI> true_positives, false_positives;
  true_positives.clear();
  false_positives.clear();
  true_positives.reserve(raw_cloud.size());
  false_positives.reserve(raw_cloud.size());

  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_static(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_dynamic(new pcl::PointCloud<pcl::PointXYZI>);
  erasor_utils::parseStaticAndDynamic(raw_cloud, *raw_dynamic, *raw_static);
  vector<int> TPs, FPs;
  erasor_utils::findEmptyCorrespondences(*raw_dynamic, est_cloud, TPs, voxel_size * voxel_size);
  erasor_utils::findEmptyCorrespondences(*raw_static, est_cloud, FPs, voxel_size * voxel_size);

  for (const int tp_idx : TPs) {
    true_positives.emplace_back(raw_dynamic->points[tp_idx]);
  }

  for (const int fp_idx : FPs) {
    false_positives.emplace_back(raw_static->points[fp_idx]);
  }

  tp = erasor_utils::cloud2msg(true_positives);
  fp = erasor_utils::cloud2msg(false_positives);
}

void getTPFN(const pcl::PointCloud<pcl::PointXYZI> &raw_cloud,
             const pcl::PointCloud<pcl::PointXYZI> &est_cloud,
             sensor_msgs::PointCloud2 &tp,
             sensor_msgs::PointCloud2 &fp,
             sensor_msgs::PointCloud2 &fn,
             sensor_msgs::PointCloud2 &tn,
             const float voxel_size = 0.2) {
  if (est_cloud.empty()) {
    return;
  }
  getNegativeClouds(est_cloud, tn, fn);
  getPositiveClouds(raw_cloud, est_cloud, tp, fp, voxel_size);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapviz");
  std::cout << "KiTTI MAPVIZ STARTED" << std::endl;
  ros::NodeHandle nodeHandler;

  std::string raw_path, octomap_path, pplremover_path, removert_path, auto_mos_path, erasor_path,
      erasor2_path;
  std::string four_d_mos_path, scan2map_path;

  nodeHandler.param<std::string>("/raw_path", raw_path, "");
  nodeHandler.param<std::string>("/octoMap_path", octomap_path, "");
  nodeHandler.param<std::string>("/pplremover_path", pplremover_path, "");
  nodeHandler.param<std::string>("/removert_path", removert_path, "");
  nodeHandler.param<std::string>("/auto_mos_path", auto_mos_path, "");
  nodeHandler.param<std::string>("/erasor_path", erasor_path, "");
  nodeHandler.param<std::string>("/erasor2_path", erasor2_path, "");
  nodeHandler.param<std::string>("/four_d_mos_path", four_d_mos_path, "");
  nodeHandler.param<std::string>("/scan2map_path", scan2map_path, "");

  std::string abs_path, seq;
  nodeHandler.param<std::string>("/abs_path", abs_path, "");
  nodeHandler.param<std::string>("/seq", seq, "");

  // raw_path = abs_path + "/gt/" + seq + "_voxel_0_2.pcd";
  // octomap_path = abs_path + "/estimate/" + seq + "_octomap.pcd";
  // pplremover_path = abs_path + "/estimate/" + seq + "_pplremover.pcd";
  // removert_path = [&]()-> string {
  //     if (seq == "19") return abs_path + "/estimate/" + seq + "_Removert.pcd";
  //     return abs_path + "/estimate/" + seq + "_Removert_rv1.pcd";
  // }();
  // // For Tracking 19!! No revert module
  // auto_mos_path = abs_path + "/estimate/" + seq + "_AutoMOS.pcd";
  // erasor_path = abs_path + "/estimate/" + seq + "_ERASOR.pcd";
  // erasor2_path = abs_path + "/estimate/" + seq + "_ERASOR2.pcd";
  // four_d_mos_path = abs_path + "/estimate/" + seq + "_4DMOS.pcd";

  ros::Publisher rawStatPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/raw/static", 100, true);
  ros::Publisher rawDynaPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/raw/dynamic", 100, true);

  ros::Publisher oTPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/TP", 100, true);
  ros::Publisher oFPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/FP", 100, true);
  ros::Publisher oFNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/FN", 100, true);
  ros::Publisher oTNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/TN", 100, true);

  ros::Publisher pTPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/TP", 100, true);
  ros::Publisher pFPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/FP", 100, true);
  ros::Publisher pFNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/FN", 100, true);
  ros::Publisher pTNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/TN", 100, true);

  ros::Publisher rTPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/TP", 100, true);
  ros::Publisher rFPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/FP", 100, true);
  ros::Publisher rFNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/FN", 100, true);
  ros::Publisher rTNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/TN", 100, true);

  ros::Publisher aTPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/auto_mos/TP", 100, true);
  ros::Publisher aFPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/auto_mos/FP", 100, true);
  ros::Publisher aFNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/auto_mos/FN", 100, true);
  ros::Publisher aTNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/auto_mos/TN", 100, true);

  ros::Publisher eTPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/TP", 100, true);
  ros::Publisher eFPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/FP", 100, true);
  ros::Publisher eFNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/FN", 100, true);
  ros::Publisher eTNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/TN", 100, true);

  ros::Publisher e2TPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/TP", 100, true);
  ros::Publisher e2FPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/FP", 100, true);
  ros::Publisher e2FNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/FN", 100, true);
  ros::Publisher e2TNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/TN", 100, true);

  ros::Publisher fTPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/4DMOS/TP", 100, true);
  ros::Publisher fFPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/4DMOS/FP", 100, true);
  ros::Publisher fFNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/4DMOS/FN", 100, true);
  ros::Publisher fTNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/4DMOS/TN", 100, true);

  ros::Publisher sTPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/scan2map/TP", 100, true);
  ros::Publisher sFPPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/scan2map/FP", 100, true);
  ros::Publisher sFNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/scan2map/FN", 100, true);
  ros::Publisher sTNPub =
      nodeHandler.advertise<sensor_msgs::PointCloud2>("/scan2map/TN", 100, true);

  ////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr est_octomap(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr est_pplremover(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr est_removert(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr est_auto_mos(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr est_erasor(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr est_erasor2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr est_4dmos(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr est_scan2map(new pcl::PointCloud<pcl::PointXYZI>);

  std::cout << "\033[1;32mLoading all clouds...\033[0m" << std::endl;
  if (std::filesystem::exists(raw_path)) {
    erasor_utils::load_pcd(raw_path, raw_cloud);
  } else {
    ROS_WARN_STREAM("File not found: " << raw_path);
  }

  if (std::filesystem::exists(octomap_path)) {
    erasor_utils::load_pcd(octomap_path, est_octomap);
  } else {
    ROS_WARN_STREAM("File not found: " << octomap_path);
  }

  if (std::filesystem::exists(pplremover_path)) {
    erasor_utils::load_pcd(pplremover_path, est_pplremover);
  } else {
    ROS_WARN_STREAM("File not found: " << pplremover_path);
  }

  if (std::filesystem::exists(removert_path)) {
    erasor_utils::load_pcd(removert_path, est_removert);
  } else {
    ROS_WARN_STREAM("File not found: " << removert_path);
  }

  if (std::filesystem::exists(auto_mos_path)) {
    erasor_utils::load_pcd(auto_mos_path, est_auto_mos);
  } else {
    ROS_WARN_STREAM("File not found: " << auto_mos_path);
  }

  if (std::filesystem::exists(erasor_path)) {
    erasor_utils::load_pcd(erasor_path, est_erasor);
  } else {
    ROS_WARN_STREAM("File not found: " << erasor_path);
  }

  if (std::filesystem::exists(erasor2_path)) {
    erasor_utils::load_pcd(erasor2_path, est_erasor2);
  } else {
    ROS_WARN_STREAM("File not found: " << erasor2_path);
  }

  if (std::filesystem::exists(four_d_mos_path)) {
    erasor_utils::load_pcd(four_d_mos_path, est_4dmos);
  } else {
    ROS_WARN_STREAM("File not found: " << four_d_mos_path);
  }

  std::cout << "\033[1;32mLoad complete \033[0m" << std::endl;

  // TN: remained static points
  // FN: remained dynamic points
  // TP: removed dynamic points
  // FP: removed static points
  sensor_msgs::PointCloud2 raw_tp, raw_fp, raw_fn, raw_tn;
  sensor_msgs::PointCloud2 o_tp, o_fp, o_fn, o_tn;
  sensor_msgs::PointCloud2 p_tp, p_fp, p_fn, p_tn;
  sensor_msgs::PointCloud2 r_tp, r_fp, r_fn, r_tn;
  sensor_msgs::PointCloud2 a_tp, a_fp, a_fn, a_tn;
  sensor_msgs::PointCloud2 e_tp, e_fp, e_fn, e_tn;
  sensor_msgs::PointCloud2 e2_tp, e2_fp, e2_fn, e2_tn;
  sensor_msgs::PointCloud2 f_tp, f_fp, f_fn, f_tn;
  sensor_msgs::PointCloud2 s_tp, s_fp, s_fn, s_tn;

  std::cout << "\033[1;32mOn parsing... \033[0m" << std::endl;
  getTPFN(*raw_cloud, *raw_cloud, raw_tp, raw_fp, raw_fn, raw_tn);
  getTPFN(*raw_cloud, *est_octomap, o_tp, o_fp, o_fn, o_tn);
  getTPFN(*raw_cloud, *est_pplremover, p_tp, p_fp, p_fn, p_tn);
  getTPFN(*raw_cloud, *est_removert, r_tp, r_fp, r_fn, r_tn);
  getTPFN(*raw_cloud, *est_auto_mos, a_tp, a_fp, a_fn, a_tn);
  getTPFN(*raw_cloud, *est_erasor, e_tp, e_fp, e_fn, e_tn);
  getTPFN(*raw_cloud, *est_erasor2, e2_tp, e2_fp, e2_fn, e2_tn);
  getTPFN(*raw_cloud, *est_4dmos, f_tp, f_fp, f_fn, f_tn);
  getTPFN(*raw_cloud, *est_scan2map, s_tp, s_fp, s_fn, s_tn);
  std::cout << "\033[1;32mParse complete!\033[0m" << std::endl;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped trans_msg;
  trans_msg.header.stamp            = ros::Time::now();
  trans_msg.transform.translation.x = 0.0;
  trans_msg.transform.translation.y = 0.0;
  trans_msg.transform.translation.z = 0.0;
  trans_msg.transform.rotation.x    = 0.0;
  trans_msg.transform.rotation.y    = 0.0;
  trans_msg.transform.rotation.z    = 0.0;
  trans_msg.transform.rotation.w    = 1.0;
  trans_msg.header.frame_id         = "world";
  trans_msg.child_frame_id          = "map";
  br.sendTransform(trans_msg);

  ros::Rate loop_rate(1);
  static int count = 0;
  for (int i = 0; i < 2; ++i) {
    rawStatPub.publish(raw_tn);
    rawDynaPub.publish(raw_fn);

    oTPPub.publish(o_tp);
    oFPPub.publish(o_fp);
    oFNPub.publish(o_fn);
    oTNPub.publish(o_tn);

    pTPPub.publish(p_tp);
    pFPPub.publish(p_fp);
    pFNPub.publish(p_fn);
    pTNPub.publish(p_tn);

    rTPPub.publish(r_tp);
    rFPPub.publish(r_fp);
    rFNPub.publish(r_fn);
    rTNPub.publish(r_tn);

    aTPPub.publish(a_tp);
    aFPPub.publish(a_fp);
    aFNPub.publish(a_fn);
    aTNPub.publish(a_tn);

    eTPPub.publish(e_tp);
    eFPPub.publish(e_fp);
    eFNPub.publish(e_fn);
    eTNPub.publish(e_tn);

    e2TPPub.publish(e2_tp);
    e2FPPub.publish(e2_fp);
    e2FNPub.publish(e2_fn);
    e2TNPub.publish(e2_tn);

    fTPPub.publish(f_tp);
    fFPPub.publish(f_fp);
    fFNPub.publish(f_fn);
    fTNPub.publish(f_tn);
    //
    //        sTPPub.publish(s_tp);
    //        sFPPub.publish(s_fp);
    //        sFNPub.publish(s_fn);
    //        sTNPub.publish(s_tn);
  }

  while (ros::ok()) {
    signal(SIGINT, erasor_utils::signal_callback_handler);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
