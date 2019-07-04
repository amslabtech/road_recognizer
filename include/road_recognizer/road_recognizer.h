#ifndef __ROAD_RECOGNIZER_H
#define __ROAD_RECOGNIZER_H

#include <ros/ros.h>

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

// RANSAC
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// OMP
#include <omp.h>


class RoadRecognizer
{
public:
    typedef pcl::PointXYZINormal PointXYZIN;
    typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
    typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;
    typedef pcl::PointXYZI PointXYZI;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;
    typedef pcl::PointXYZ PointXYZ;
    typedef pcl::PointCloud<PointXYZ> CloudXYZ;
    typedef pcl::PointCloud<PointXYZ>::Ptr CloudXYZPtr;
    typedef pcl::PointNormal PointN;
    typedef pcl::PointCloud<PointN> CloudN;
    typedef pcl::PointCloud<PointN>::Ptr CloudNPtr;

    RoadRecognizer(void);

    void process(void);
    void obstacles_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void ground_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void publish_clouds(void);

private:
    double HZ;
    double NORMAL_ESTIMATION_RADIUS;
    double LEAF_SIZE;
    int OUTLIER_REMOVAL_K;
    double OUTLIER_REMOVAL_THRESHOLD;
    double CURVATURE_THRESHOLD;
    double INTENSITY_UPPER_THRESHOLD;
    double INTENSITY_LOWER_THRESHOLD;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher curvature_cloud_pub;
    ros::Publisher downsampled_cloud_pub;
    ros::Publisher intensity_cloud_pub;
    ros::Publisher road_cloud_pub;
    ros::Subscriber obstacles_sub;
    ros::Subscriber ground_sub;

    CloudXYZIPtr obstacles_cloud;
    CloudXYZIPtr ground_cloud;
    CloudXYZINPtr curvature_cloud;
    CloudXYZINPtr intensity_cloud;
    CloudXYZINPtr road_cloud;
    bool obstacles_cloud_updated;
    bool ground_cloud_updated;
};

#endif// __ROAD_RECOGNIZER_H
