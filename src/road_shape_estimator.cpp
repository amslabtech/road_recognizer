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
    cloud_sub_ = nh_.subscribe("cloud", 1, &RoadShapeEstimator::cloud_callback, this);
}

void RoadShapeEstimator::process(void)
{
    ros::spin();
}

void RoadShapeEstimator::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud_ptr);

    std::vector<pcl::PointIndices> segments = divide_cloud_into_segments(cloud_ptr);

    // RANSAC 
    for(const auto& indices : segments){
        fit_ransac_spline(cloud_ptr, indices);
    }
}

std::vector<pcl::PointIndices> RoadShapeEstimator::divide_cloud_into_segments(const pcl::PointCloud<PointT>::Ptr cloud_ptr)
{
    std::vector<pcl::PointIndices> segments;
    segments.resize(1);
    for(unsigned int i=0;i<cloud_ptr->points.size();i++){
        segments[0].indices.push_back(i);
    }
    return segments;
}

void RoadShapeEstimator::fit_ransac_spline(const pcl::PointCloud<PointT>::Ptr cloud_ptr, const pcl::PointIndices& indices)
{
    pcl::PointIndices sample_indices = get_random_sample(indices);
    fit_spline(cloud_ptr, sample_indices);
}

pcl::PointIndices RoadShapeEstimator::get_random_sample(const pcl::PointIndices& indices)
{
    std::uniform_int_distribution<> dist(0, indices.indices.size());
    pcl::PointIndices sample_indices;
    sample_indices = indices;
    return sample_indices;
}

void RoadShapeEstimator::fit_spline(const pcl::PointCloud<PointT>::Ptr cloud_ptr, const pcl::PointIndices& indices)
{

}

}