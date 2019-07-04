#include "road_recognizer/road_point_cloud_storer.h"

RoadPointCloudStorer::RoadPointCloudStorer(void)
:local_nh("~"), road_cloud_sub(nh, "cloud/road", 10), odom_sub(nh, "/odom/complement", 10)
, sync(sync_subs(10), road_cloud_sub, odom_sub)
{
    local_nh.param("HZ", HZ, {20});

    sync.registerCallback(boost::bind(&RoadPointCloudStorer::callback, this, _1, _2));

    road_cloud = CloudXYZINPtr(new CloudXYZIN);

    std::cout << "HZ: " << HZ << std::endl;
}

void RoadPointCloudStorer::callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud, const nav_msgs::OdometryConstPtr& msg_odom)
{
    std::cout << "=== road point cloud storer ===" << std::endl;
    double start = ros::Time::now().toSec();
    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void RoadPointCloudStorer::process(void)
{
    ros::spin();
}
