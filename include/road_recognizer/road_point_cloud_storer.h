#ifndef __ROAD_POINT_CLOUD_STORER
#define __ROAD_POINT_CLOUD_STORER

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

// OMP
#include <omp.h>


class RoadPointCloudStorer
{
public:
    typedef pcl::PointXYZINormal PointXYZIN;
    typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
    typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;
    typedef pcl::PointXYZI PointXYZI;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;

    RoadPointCloudStorer(void);

    void process(void);
    void callback(const sensor_msgs::PointCloud2ConstPtr&, const nav_msgs::OdometryConstPtr&);

private:
    double HZ;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync_subs;
    message_filters::Subscriber<sensor_msgs::PointCloud2> road_cloud_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Synchronizer<sync_subs> sync;

    CloudXYZINPtr road_cloud;
};

#endif// __ROAD_POINT_CLOUD_STORER
