/**
 * @file road_shape_estimator.h
 * @author amsl
 */

#include "road_recognizer/road_shape_estimator.h"

namespace road_recognizer
{

RoadShapeEstimator::RoadShapeEstimator(void)
: local_nh_("~")
, mt_(rnd_())
{
    local_nh_.param<double>("convergence_threshold", convergence_threshold_, 0.01);
    local_nh_.param<int>("max_iteration", max_iteration_, 100);
    local_nh_.param<int>("sample_num", sample_num_, 4);
    local_nh_.param<int>("fitting_decision_data_num", fitting_decision_data_num_, 10);

    cloud_sub_ = nh_.subscribe("cloud", 1, &RoadShapeEstimator::cloud_callback, this);

    m_mat_ << -1,  3, -3, 1,
               3, -6,  3, 0,
              -3,  3,  0, 0,
               1,  0,  0, 0;
}

void RoadShapeEstimator::process(void)
{
    ros::spin();
}

void RoadShapeEstimator::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud_ptr);

    std::vector<std::vector<Eigen::Vector2d>> segments = divide_cloud_into_segments(cloud_ptr);

    // RANSAC 
    for(const auto& segment : segments){
        fit_ransac_spline(segment);
    }
}

std::vector<std::vector<Eigen::Vector2d>> RoadShapeEstimator::divide_cloud_into_segments(const pcl::PointCloud<PointT>::Ptr cloud_ptr)
{
    std::vector<std::vector<Eigen::Vector2d>> segments;
    segments.resize(1);
    for(unsigned int i=0;i<cloud_ptr->points.size();i++){
        segments[0].push_back(Eigen::Vector2d(cloud_ptr->points[i].x, cloud_ptr->points[i].y));
    }
    return segments;
}

void RoadShapeEstimator::fit_ransac_spline(const std::vector<Eigen::Vector2d>& segment)
{
    std::vector<unsigned int> sample_indices = get_random_sample(segment);
    Eigen::MatrixXd control_points = fit_spline(segment, sample_indices);
}

std::vector<unsigned int> RoadShapeEstimator::get_random_sample(const std::vector<Eigen::Vector2d>& segment)
{
    // TODO: unique random sampling
    std::uniform_int_distribution<> dist(0, segment.size() - 1);
    std::vector<unsigned int> sample_indices;
    sample_indices.reserve(sample_num_);
    while(static_cast<int>(sample_indices.size()) < sample_num_){
        sample_indices.emplace_back(dist(mt_));
    }
    return sample_indices;
}

Eigen::MatrixXd RoadShapeEstimator::fit_spline(const std::vector<Eigen::Vector2d>& segment, const std::vector<unsigned int>& indices)
{
    const unsigned int num = indices.size() - 1;
    std::vector<double> d(num, 0);
    for(unsigned int i=0;i<num;++i){
        d[i] = (segment[indices[i+1]] - segment[indices[i]]).norm();
    }
    std::vector<double> cumulative_sum_of_d(num, 0);
    for(unsigned int i=0;i<num;++i){
        cumulative_sum_of_d[i] += d[i];
    }
    // t \in [0, 1]
    std::vector<double> t(num, 0);
    for(unsigned int i=0;i<num;++i){
        t[i] = d[i] / cumulative_sum_of_d[i];
    }
    Eigen::MatrixXd t_mat = Eigen::MatrixXd::Ones(num, 4);
    for(unsigned int i=0;i<num;++i){
        for(unsigned int j=1;j<4;++j){
            t_mat(i, 3-j) = t_mat(i, 4-j) * t[i]; 
        }
    }
    Eigen::MatrixXd q_mat = Eigen::MatrixXd::Zero(num, 2);
    for(unsigned int i=0;i<num;++i){
        q_mat.row(i) = segment[indices[i]].transpose();
    }

    // four control points
    Eigen::MatrixXd p_mat = Eigen::MatrixXd::Zero(4, 2);
    p_mat = (t_mat * m_mat_).completeOrthogonalDecomposition().pseudoInverse() * q_mat;
    return p_mat;
}

}