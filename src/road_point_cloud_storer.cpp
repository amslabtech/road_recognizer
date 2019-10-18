#include "road_recognizer/road_point_cloud_storer.h"

RoadPointCloudStorer::RoadPointCloudStorer(void)
:local_nh("~"), road_cloud_sub(nh, "cloud/road", 10), odom_sub(nh, "/odom/complement", 10)
, sync(sync_subs(10), road_cloud_sub, odom_sub)
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("STORE_NUM", STORE_NUM, {20});
    local_nh.param("POSITION_DIFFERENCE_THRESHOLD", POSITION_DIFFERENCE_THRESHOLD, {0.1});
    local_nh.param("YAW_DIFFERENCE_THRESHOLD", YAW_DIFFERENCE_THRESHOLD, {0.1});

    road_stored_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud/road/stored", 1);

    sync.registerCallback(boost::bind(&RoadPointCloudStorer::callback, this, _1, _2));

    road_cloud = CloudXYZINPtr(new CloudXYZIN);

    first_flag = true;

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "STORE_NUM: " << STORE_NUM << std::endl;
    std::cout << "POSITION_DIFFERENCE_THRESHOLD: " << POSITION_DIFFERENCE_THRESHOLD << std::endl;
    std::cout << "YAW_DIFFERENCE_THRESHOLD: " << YAW_DIFFERENCE_THRESHOLD << std::endl;
}

void RoadPointCloudStorer::callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud, const nav_msgs::OdometryConstPtr& msg_odom)
{
    std::cout << "=== road point cloud storer ===" << std::endl;
    double start = ros::Time::now().toSec();
    Eigen::Vector3d current_position(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z);
    double current_yaw = tf::getYaw(msg_odom->pose.pose.orientation);
    std::cout << "current_position: " << current_position.transpose() << std::endl;
    std::cout << "current_yaw: " << current_yaw << std::endl;
    static Eigen::Vector3d last_position;
    static double last_yaw;
    static Eigen::Vector3d last_add_position;
    static double last_add_yaw;
    std::cout << "last_position: " << last_position.transpose() << std::endl;
    std::cout << "last_yaw: " << last_yaw << std::endl;
    CloudXYZINPtr temp_cloud(new CloudXYZIN);
    pcl::fromROSMsg(*msg_cloud, *temp_cloud);
    int cloud_size = temp_cloud->points.size();
    std::cout << "new cloud size: " <<  cloud_size << std::endl;

    if(!first_flag){
        double d_yaw = current_yaw - last_yaw;
        d_yaw = atan2(sin(d_yaw), cos(d_yaw));
        /* d_yaw = - 2.0 *atan2(sin(d_yaw), cos(d_yaw)); */
        Eigen::Matrix3d r;
        r = Eigen::AngleAxisd(-d_yaw, Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d last_yaw_rotation;
        last_yaw_rotation = Eigen::AngleAxisd(-last_yaw, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d _current_position = last_yaw_rotation * current_position;
        Eigen::Vector3d _last_position = last_yaw_rotation * last_position;
        Eigen::Translation<double, 3> t(_last_position - _current_position);

        affine_transform = t * r;
        std::cout << "affine transformation: \n" << affine_transform.translation() << "\n" << affine_transform.rotation().eulerAngles(0,1,2) << std::endl;

        pcl::transformPointCloud(*road_cloud, *road_cloud, affine_transform);
        if(((current_position - last_add_position).norm() > POSITION_DIFFERENCE_THRESHOLD) || (fabs(current_yaw - last_add_yaw) > YAW_DIFFERENCE_THRESHOLD)){
            // store pointcloud only when robot moves
            *road_cloud += *temp_cloud;
            cloud_size_list.push_back(cloud_size);
            if(cloud_size_list.size() > STORE_NUM){
                road_cloud->points.erase(road_cloud->points.begin(), road_cloud->points.begin() + *(cloud_size_list.begin()));
                cloud_size_list.pop_front();
            }
            last_add_position = current_position;
            last_add_yaw = current_yaw;
        }
        std::cout << "stored cloud size: " <<  road_cloud->points.size() << std::endl;
        std::cout << "stored clouds num: " <<  cloud_size_list.size() << std::endl;
        for(const auto& s : cloud_size_list){
            std::cout << s << ", ";
        }
        std::cout << std::endl;
    }else{
        *road_cloud = *temp_cloud;
        cloud_size_list.push_back(cloud_size);
        first_flag = false;
        road_cloud->header = temp_cloud->header;
        last_position = current_position;
        last_add_position = current_position;
        last_yaw = current_yaw;
        last_add_yaw = current_yaw;
    }
    road_cloud->header = temp_cloud->header;
    last_position = current_position;
    last_yaw = current_yaw;

    // publish
    sensor_msgs::PointCloud2 publish_cloud;
    std::cout << "height: " << road_cloud->height << ", " << "width: " << road_cloud->width << std::endl;
    road_cloud->width = road_cloud->points.size();
    pcl::toROSMsg(*road_cloud, publish_cloud);
    road_stored_cloud_pub.publish(publish_cloud);

    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void RoadPointCloudStorer::process(void)
{
    ros::spin();
}
