#include "road_recognizer/road_recognizer.h"

RoadRecognizer::RoadRecognizer(void)
:local_nh("~")
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("LEAF_SIZE", LEAF_SIZE, {0.1});
    local_nh.param("OUTLIER_REMOVAL_K", OUTLIER_REMOVAL_K, {50});
    local_nh.param("OUTLIER_REMOVAL_THRESHOLD", OUTLIER_REMOVAL_THRESHOLD, {1.0});

    downsampled_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/downsampled", 1);
    filtered_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/filtered", 1);

    road_stored_cloud_sub = nh.subscribe("cloud/road/stored", 1, &RoadRecognizer::road_cloud_callback, this);

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "LEAF_SIZE: " << LEAF_SIZE << std::endl;
    std::cout << "OUTLIER_REMOVAL_K: " << OUTLIER_REMOVAL_K << std::endl;
    std::cout << "OUTLIER_REMOVAL_THRESHOLD: " << OUTLIER_REMOVAL_THRESHOLD << std::endl;
}

void RoadRecognizer::road_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "=== road recognizer ===" << std::endl;
    double start = ros::Time::now().toSec();
    CloudXYZINPtr road_cloud(new CloudXYZIN);
    pcl::fromROSMsg(*msg, *road_cloud);
    int cloud_size = road_cloud->points.size();
    std::cout << "road cloud size: " <<  cloud_size << std::endl;

    std::cout << "before cloud size: " << road_cloud->points.size() << std::endl;
    CloudXYZINPtr downsampled_cloud(new CloudXYZIN);
    pcl::VoxelGrid<PointXYZIN> vg;
    vg.setInputCloud(road_cloud);
    vg.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    vg.filter(*downsampled_cloud);
    std::cout << "after voxel grid cloud size: " << road_cloud->points.size() << std::endl;

    std::cout << "before cloud size: " << downsampled_cloud->points.size() << std::endl;
    CloudXYZINPtr filtered_cloud(new CloudXYZIN);
    pcl::StatisticalOutlierRemoval<PointXYZIN> sor;
    sor.setInputCloud(downsampled_cloud);
    sor.setMeanK(OUTLIER_REMOVAL_K);
    sor.setStddevMulThresh(OUTLIER_REMOVAL_THRESHOLD);
    sor.filter(*filtered_cloud);
    std::cout << "after statistical outlier removal cloud size: " << filtered_cloud->points.size() << std::endl;

    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(*downsampled_cloud, cloud1);
    downsampled_pub.publish(cloud1);

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(*filtered_cloud, cloud2);
    filtered_pub.publish(cloud2);

    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void RoadRecognizer::process(void)
{
    ros::spin();
}
