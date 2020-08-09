/**
 * @file road_boudnary_detector.cpp
 * @author amsl
 */
#include "road_recognizer/road_boundary_detector.h"

namespace road_recognizer
{

RoadBoundaryDetector::RoadBoundaryDetector(void)
: local_nh_("~")
{
    cloud_sub_ = nh_.subscribe("velodyne_points", 1, &RoadBoundaryDetector::cloud_callback, this);

    int layer_num;
    local_nh_.param<int>("layer_num", layer_num, 32);
    layer_num_ = static_cast<unsigned int>(layer_num);
    local_nh_.param<double>("lidar_height", lidar_height_, 1.2);
    // HDL-32E gets 2160 points per layer at 600rpm(10Hz)
    local_nh_.param<double>("horizontal_resolution", horizontal_resolution_, 2.0 * M_PI / 2160.0);
    int lambda;
    local_nh_.param<int>("lambda", lambda, 2);
    lambda_ = static_cast<unsigned int>(lambda);
    local_nh_.param<double>("vertical_scan_angle_begin", vertical_scan_angle_begin_, -30.67 * 2.0 * M_PI / 180.0);
    local_nh_.param<double>("vertical_scan_angle_end", vertical_scan_angle_end_, 10.67 * 2.0 * M_PI / 180.0);

    num_sectors_ = std::floor(2.0 * M_PI / (lambda_ * horizontal_resolution_));
    num_bins_ = std::floor((vertical_scan_angle_end_ - vertical_scan_angle_begin_) / (lambda_ * horizontal_resolution_));
}

void RoadBoundaryDetector::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud_ptr);

    const unsigned int size = cloud_ptr->points.size();
    const double lambda_h_res = lambda_ * horizontal_resolution_;

    // contains point indices
    std::vector<std::vector<std::vector<unsigned int>>> polar_grid(num_sectors_, std::vector<std::vector<unsigned int>>(num_bins_));

    for(unsigned int i=0;i<size;++i){
        Eigen::Vector3d point(cloud_ptr->points[i].x, cloud_ptr->points[i].y, cloud_ptr->points[i].z);
        if(point.norm() < 0.1){
            // invalid scan
            continue;
        }

        const unsigned int ring_index = get_ring_index_from_firing_order(i % layer_num_);
        //
        // limit ring_index 
        //
        const double horizontal_distance = point.segment(0, 2).norm();
        const unsigned int sector_index = (acos(point(0) / horizontal_distance) + 2.0 + M_PI * unit_step_function(-point(1))) / lambda_h_res;
        //
        // search true bin_index
        //
        const unsigned int bin_index = lidar_height_ * tan(vertical_scan_angle_begin_ + ring_index * lambda_h_res);
        polar_grid[sector_index][bin_index].push_back(i);
    }

    auto is_lower = [&](unsigned int i0, unsigned int i1) -> bool
    {
        return cloud_ptr->points[i0].z < cloud_ptr->points[i1].z;
    };
    for(unsigned int i=0;i<num_sectors_;++i){
        for(unsigned int j=0;j<num_bins_;++j){
            if(polar_grid[i][j].empty()){
                continue;
            }
            std::sort(polar_grid[i][j].begin(), polar_grid[i][j].end(), is_lower);
        }
    }
}

void RoadBoundaryDetector::process(void)
{
    ros::spin();
}

unsigned int RoadBoundaryDetector::get_ring_index_from_firing_order(unsigned int order)
{
    if(order % 2){
        return order / 2 + layer_num_ / 2;
    }else{
        return order / 2;
    }
}

unsigned int RoadBoundaryDetector::unit_step_function(double value)
{
    if(value < 0){
        return 0;
    }
    return 1;
}

}