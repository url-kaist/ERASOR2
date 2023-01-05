#include "tools/erasor_utils.hpp"

void loadAndParse(const string &abs_pcd_path, sensor_msgs::PointCloud2 &dynamic_cloud_msg,
                  sensor_msgs::PointCloud2 &static_cloud_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::load_pcd(abs_pcd_path, cloud);
    std::cout << "\033[1;32mLoad complete \033[0m" << std::endl;


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_static(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_dynamic(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::parse_dynamic_obj(*cloud, *cloud_dynamic, *cloud_static);

    dynamic_cloud_msg = erasor_utils::cloud2msg(*cloud_dynamic);
    static_cloud_msg  = erasor_utils::cloud2msg(*cloud_static);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mapviz");
    std::cout << "KiTTI MAPVIZ STARTED" << std::endl;
    ros::NodeHandle nodeHandler;

    std::string raw_path, octomap_path, ppl_path, removert_path, erasor_path, erasor2_path;

    nodeHandler.param<std::string>("/raw_path", raw_path, "/media/shapelim");
    nodeHandler.param<std::string>("/octoMap_path", octomap_path, "/media/shapelim");
    nodeHandler.param<std::string>("/pplremover_path", ppl_path, "/media/shapelim");
    nodeHandler.param<std::string>("/removert_path", removert_path, "/media/shapelim");
    nodeHandler.param<std::string>("/erasor_path", erasor_path, "/media/shapelim");
    nodeHandler.param<std::string>("/erasor2_path", erasor2_path, "/media/shapelim");

    ros::Publisher msPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/static", 100, true);
    ros::Publisher mdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/dynamic", 100, true);

    ros::Publisher osPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/static", 100, true);
    ros::Publisher odPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/dynamic", 100, true);

    ros::Publisher psPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/static", 100, true);
    ros::Publisher pdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/dynamic", 100, true);

    ros::Publisher rsPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/static", 100, true);
    ros::Publisher rdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/dynamic", 100, true);

    ros::Publisher esPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/static", 100, true);
    ros::Publisher edPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/dynamic", 100, true);

    ros::Publisher e2sPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/static", 100, true);
    ros::Publisher e2dPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor2/dynamic", 100, true);

    ////////////////////////////////////////////////////////////////////
    sensor_msgs::PointCloud2 esmsg, edmsg;
    sensor_msgs::PointCloud2 rsmsg, rdmsg;
    sensor_msgs::PointCloud2 psmsg, pdmsg;
    sensor_msgs::PointCloud2 osmsg, odmsg;

    sensor_msgs::PointCloud2 e2dmsg, e2smsg;

    loadAndParse(erasor2_path, e2dmsg, e2smsg);

    ros::Rate  loop_rate(1);
    static int count = 0;
    while (ros::ok()) {
        esPublisher.publish(esmsg);
        edPublisher.publish(edmsg);
        rsPublisher.publish(rsmsg);
        rdPublisher.publish(rdmsg);
        psPublisher.publish(psmsg);
        pdPublisher.publish(pdmsg);
        osPublisher.publish(osmsg);
        odPublisher.publish(odmsg);

        e2dPublisher.publish(e2dmsg);
        e2sPublisher.publish(e2smsg);

        if (++count % 2 == 0) std::cout << "On " << count << "th publish!" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
