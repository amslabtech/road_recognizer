/**
 * @file road_shape_estimator.h
 * @author amsl
 */
#ifndef __ROAD_SHAPE_ESTIMATOR_H
#define __ROAD_SHAPE_ESTIMATOR_H

#include <random>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace road_recognizer
{

// ref: https://arxiv.org/pdf/1411.7113.pdf
class RoadShapeEstimator
{
public:
    typedef pcl::PointXYZINormal PointT;
    RoadShapeEstimator(void);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void process(void);
    /**
     * @brief Divide cloud into segments using beam model 
     */
    std::vector<std::vector<Eigen::Vector2d>> divide_cloud_into_segments(const pcl::PointCloud<PointT>::Ptr cloud_ptr);
    /** 
     * @brief Compute spline fitting using RANSAC 
     */
    void fit_ransac_spline(const std::vector<Eigen::Vector2d>& segment);
    std::vector<unsigned int> get_random_sample(const std::vector<Eigen::Vector2d>& segment);
    void fit_spline(const std::vector<Eigen::Vector2d>& segment, const std::vector<unsigned int>& indices);

protected:
    /// Threshold of error 
    double convergence_threshold_;
    /// Max iteration num
    int max_iteration_;
    /// Min number of samples for fitting 
    int sample_num_;
    int fitting_decision_data_num_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Subscriber cloud_sub_;
    std::random_device rnd_;
    std::mt19937 mt_;
};

}

#endif// __ROAD_SHAPE_ESTIMATOR_H