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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "road_recognizer/peak_detector.h"

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
    Eigen::MatrixXd fit_ransac_spline(const std::vector<Eigen::Vector2d>& segment);
    std::vector<unsigned int> get_random_sample(const std::vector<Eigen::Vector2d>& segment);
    /**
     * @brief Compute control points of spline curve
     */
    Eigen::MatrixXd fit_spline(const std::vector<Eigen::Vector2d>& segment, const std::vector<unsigned int>& indices);
    void rasterize(const pcl::PointCloud<PointT>::Ptr cloud_ptr);
    /**
     * @brief Compute fitting score of given spline using rasterized image
     */
    double compute_score(const Eigen::MatrixXd& control_points);
    /**
     * @brief Returns {x^3, x^2, x, 1}
     */
    Eigen::Vector4d get_cubic(double x);
    /**
     * @brief Publish markers for visualizing curves
     */
    void publish_marker(const std::vector<Eigen::MatrixXd>& control_points_list, const std_msgs::Header& header);
    /**
     * @brief Compute beam model from origin in given cloud
     */
    std::vector<double> get_beam_from_cloud(pcl::PointCloud<PointT>::Ptr cloud_ptr, double origin_x, double origin_y);

protected:
    struct GridParams
    {
        float min_x;
        float min_y;
        float max_x;
        float max_y;
        double width;
        double height;
        unsigned int grid_width;
        unsigned int grid_height;
    };
    /// Threshold of error 
    double convergence_threshold_;
    /// Max iteration num
    unsigned int max_iteration_;
    /// Min number of samples for fitting 
    unsigned int sample_num_;
    unsigned int fitting_decision_data_num_;
    double cells_per_meter_;
    unsigned int beam_num_;
    double max_beam_range_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher curves_pub_;
    ros::Subscriber cloud_sub_;
    std::random_device rnd_;
    std::mt19937 mt_;
    Eigen::Matrix4d m_mat_;
    std::vector<std::vector<bool>> rasterized_image;
    GridParams grid_params_;
};

}

#endif// __ROAD_SHAPE_ESTIMATOR_H