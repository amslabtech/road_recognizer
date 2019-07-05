#ifndef __ROAD_RECOGNIZER
#define __ROAD_RECOGNIZER

#include <random>
#include <functional>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>

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
#include <pcl/visualization/cloud_viewer.h>

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

    RoadRecognizer(void);

    void process(void);
    void road_cloud_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void visualize_cloud(void);

private:
    double HZ;
    double LEAF_SIZE;
    int OUTLIER_REMOVAL_K;
    double OUTLIER_REMOVAL_THRESHOLD;
    int MAX_RANDOM_SAMPLE_SIZE;
    double RANDOM_SAMPLE_RATIO;
    bool ENABLE_VISUALIZATION;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher downsampled_pub;
    ros::Publisher filtered_pub;
    ros::Publisher beam_cloud_pub;
    ros::Subscriber road_stored_cloud_sub;

    CloudXYZINPtr filtered_cloud;

    pcl::visualization::PCLVisualizer viewer{"viewer"};
};

#endif// __ROAD_RECOGNIZER
