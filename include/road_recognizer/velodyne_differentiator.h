#ifndef __VELODYNE_DIFFERENTIATOR_H
#define __VELODYNE_DIFFERENTIATOR_H

#include <ros/ros.h>

#include <random>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Eigen
#include <Eigen/Dense>

// OMP
#include <omp.h>

class GridState
{
public:
    GridState(void);
    double min_z;
    double max_z;
    double avg_z;
    int hit_count;

    void set_height(double);
private:
};

class VelodyneDifferentiator
{
public:
    VelodyneDifferentiator(void);

    void process(void);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr&);
    size_t get_ring_index_from_firing_order(size_t);
    void apply_grid_states_to_image(const std::vector<std::vector<GridState> >&, cv::Mat&);

private:
    int LAYER_NUM;
    double DISTANCE_THRESHOLD;
    double SECOND_DIFFERENTIAL_THRESHOLD;
    double VELODYNE_HEIGHT;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher derivative_cloud_pub;
    ros::Publisher second_derivative_cloud_pub;
    ros::Publisher edge_cloud_pub;
    ros::Subscriber velodyne_sub;
    ros::Subscriber odom_sub;

    double dx, dy, dyaw;
    bool first_odom_flag;
};

#endif// __VELODYNE_DIFFERENTIATOR_H
