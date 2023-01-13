#include "tools/erasor_utils.hpp"

void loadAndParse(const string &abs_pcd_path, sensor_msgs::PointCloud2 &static_cloud_msg,
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

void getNegativeClouds(const pcl::PointCloud<pcl::PointXYZI>& est_cloud,
                   sensor_msgs::PointCloud2 &tn,
                  sensor_msgs::PointCloud2 &fn) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_static(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_dynamic(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::parseStaticAndDynamic(est_cloud, *cloud_dynamic, *cloud_static);

    tn  = erasor_utils::cloud2msg(*cloud_static);
    fn = erasor_utils::cloud2msg(*cloud_dynamic);
}

// Both should be voxelized with same voxel size
void getPositiveClouds(const pcl::PointCloud<pcl::PointXYZI>& raw_cloud,
                     const pcl::PointCloud<pcl::PointXYZI>& est_cloud,
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

    for (const int tp_idx: TPs) {
        true_positives.emplace_back(raw_dynamic->points[tp_idx]);
    }

    for (const int fp_idx: FPs) {
        false_positives.emplace_back(raw_static->points[fp_idx]);
    }

    tp = erasor_utils::cloud2msg(true_positives);
    fp  = erasor_utils::cloud2msg(false_positives);
}

void getTPFN(const pcl::PointCloud<pcl::PointXYZI>& raw_cloud,
                     const pcl::PointCloud<pcl::PointXYZI>& est_cloud,
                    sensor_msgs::PointCloud2 &tp, sensor_msgs::PointCloud2 &fp,
                     sensor_msgs::PointCloud2 &fn, sensor_msgs::PointCloud2 &tn,
                     const float voxel_size = 0.2) {
    getNegativeClouds(est_cloud, tn, fn);
    getPositiveClouds(raw_cloud, est_cloud, tp, fp, voxel_size);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mapviz");
    std::cout << "KiTTI MAPVIZ STARTED" << std::endl;
    ros::NodeHandle nodeHandler;

    std::string raw_path, octomap_path, ppl_path, removert_path, erasor_path, erasor2_path;
    std::string four_d_mos_path, scan2map_path;

    nodeHandler.param<std::string>("/raw_path", raw_path, "");
    nodeHandler.param<std::string>("/octoMap_path", octomap_path, "");
    nodeHandler.param<std::string>("/pplremover_path", ppl_path, "");
    nodeHandler.param<std::string>("/removert_path", removert_path, "");
    nodeHandler.param<std::string>("/erasor_path", erasor_path, "");
    nodeHandler.param<std::string>("/erasor2_path", erasor2_path, "");
    nodeHandler.param<std::string>("/four_d_mos_path", four_d_mos_path, "");
    nodeHandler.param<std::string>("/scan2map_path", scan2map_path, "");

    ros::Publisher msPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/static", 100, true);
    ros::Publisher mdPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/dynamic", 100, true);

    ros::Publisher osPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/static", 100, true);
    ros::Publisher odPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/dynamic", 100, true);

    ros::Publisher psPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/static", 100, true);
    ros::Publisher pdPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/dynamic", 100, true);

    ros::Publisher rTPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/TP", 100, true);
    ros::Publisher rFPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/FP", 100, true);
    ros::Publisher rFNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/FN", 100, true);
    ros::Publisher rTNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/TN", 100, true);

    ros::Publisher eTPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/TP", 100, true);
    ros::Publisher eFPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/FP", 100, true);
    ros::Publisher eFNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/FN", 100, true);
    ros::Publisher eTNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/TN", 100, true);

    ros::Publisher e2TPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/TP", 100, true);
    ros::Publisher e2FPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/FP", 100, true);
    ros::Publisher e2FNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/FN", 100, true);
    ros::Publisher e2TNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/TN", 100, true);

    ros::Publisher fTPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/4DMOS/TP", 100, true);
    ros::Publisher fFPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/4DMOS/FP", 100, true);
    ros::Publisher fFNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/4DMOS/FN", 100, true);
    ros::Publisher fTNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/4DMOS/TN", 100, true);

    ros::Publisher sTPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/scan2map/TP", 100, true);
    ros::Publisher sFPPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/scan2map/FP", 100, true);
    ros::Publisher sFNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/scan2map/FN", 100, true);
    ros::Publisher sTNPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/scan2map/TN", 100, true);

    ////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_removert(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_erasor(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_erasor2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_4dmos(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_scan2map(new pcl::PointCloud<pcl::PointXYZI>);

    std::cout << "\033[1;32mLoading all clouds...\033[0m" << std::endl;
    erasor_utils::load_pcd(raw_path, raw_cloud);
    erasor_utils::load_pcd(removert_path, est_removert);
    erasor_utils::load_pcd(erasor_path, est_erasor);
    erasor_utils::load_pcd(erasor2_path, est_erasor2);
    erasor_utils::load_pcd(four_d_mos_path, est_4dmos);
    erasor_utils::load_pcd(scan2map_path, est_scan2map);
    std::cout << "\033[1;32mLoad complete \033[0m" << std::endl;
    // TN: remained static points
    // FN: remained dynamic points
    // TP: removed dynamic points
    // FP: removed static points
    sensor_msgs::PointCloud2 r_tp, r_fp, r_fn, r_tn;
    sensor_msgs::PointCloud2 e_tp, e_fp, e_fn, e_tn;
    sensor_msgs::PointCloud2 e2_tp, e2_fp, e2_fn, e2_tn;
    sensor_msgs::PointCloud2 f_tp, f_fp, f_fn, f_tn;
    sensor_msgs::PointCloud2 s_tp, s_fp, s_fn, s_tn;

    getTPFN(*raw_cloud, *est_removert, r_tp, r_fp, r_fn, r_tn);
    getTPFN(*raw_cloud, *est_erasor, e_tp, e_fp, e_fn, e_tn);
    getTPFN(*raw_cloud, *est_erasor2, e2_tp, e2_fp, e2_fn, e2_tn);
    getTPFN(*raw_cloud, *est_4dmos, f_tp, f_fp, f_fn, f_tn);
    getTPFN(*raw_cloud, *est_scan2map, s_tp, s_fp, s_fn, s_tn);

    ros::Rate  loop_rate(1);
    static int count = 0;
    for (int i = 0; i < 2; ++i) {
//        esPub.publish(esmsg);
//        edPub.publish(edmsg);
//        rsPub.publish(rsmsg);
//        rdPub.publish(rdmsg);
//        psPub.publish(psmsg);
//        pdPub.publish(pdmsg);
//        osPub.publish(osmsg);
//        odPub.publish(odmsg);
        rTPPub.publish(r_tp);
        rFPPub.publish(r_fp);
        rFNPub.publish(r_fn);
        rTNPub.publish(r_tn);

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

        sTPPub.publish(s_tp);
        sFPPub.publish(s_fp);
        sFNPub.publish(s_fn);
        sTNPub.publish(s_tn);
    }

    while (ros::ok()) {
        signal(SIGINT, erasor_utils::signal_callback_handler);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
