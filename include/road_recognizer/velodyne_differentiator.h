#ifndef __VELODYNE_DIFFERENTIATOR_H
#define __VELODYNE_DIFFERENTIATOR_H

#include <ros/ros.h>

#include <random>

#include <ros/ros.h>

// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// Eigen
#include <Eigen/Dense>

// OMP
#include <omp.h>

class VelodyneDifferentiator
{
public:
    VelodyneDifferentiator(void);

    void process(void);
    void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr&);
    size_t get_ring_index_from_firing_order(size_t);

private:
    int LAYER_NUM;
    double DISTANCE_THRESHOLD;
    double SECOND_DIFFERENTIAL_THRESHOLD;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher derivative_cloud_pub;
    ros::Publisher second_derivative_cloud_pub;
    ros::Publisher edge_cloud_pub;
    ros::Subscriber velodyne_sub;
};

#endif// __VELODYNE_DIFFERENTIATOR_H
