/**
 * @file road_boundary_detector.h
 * @author amsl 
 */
#ifndef __ROAD_BOUNDARY_DETECTOR_H 
#define __ROAD_BOUNDARY_DETECTOR_H 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace road_recognizer
{
// ref: https://ieeexplore.ieee.org/document/8654619 
/**
 * @brief Detect road boundary from velodyne pointcloud
 */
class RoadBoundaryDetector
{
public:
    typedef pcl::PointXYZI PointT;
    RoadBoundaryDetector(void);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void process(void);
    unsigned int get_ring_index_from_firing_order(unsigned int order);
    unsigned int unit_step_function(double value);
    double compute_distance(double x0, double y0, double x1, double y1);
    unsigned int get_bin_index(double x, double y);
    void publish_cloud(const pcl::PointCloud<PointT>::Ptr cloud_ptr, const std::vector<unsigned int>& ground_point_indices, const std::vector<unsigned int>& obstacle_point_indices);

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher obstacle_cloud_pub_; 
    ros::Publisher ground_cloud_pub_; 
    ros::Subscriber cloud_sub_; 

    unsigned int layer_num_;
    double lidar_height_;
    double horizontal_resolution_;
    double vertical_scan_angle_begin_;
    double vertical_scan_angle_end_;
    unsigned int vertical_scan_num_;
    unsigned int lambda_;
    unsigned int num_sectors_;
    // k
    unsigned int num_bins_;
    // c_{th}
    double bottom_threshold_;

    // effective mesurement ranges (horizontal)
    std::vector<double> b_;
    // \vartheta
    std::vector<double> vertical_angles_;
    // vertical angle resolution of LiDAR
    double vertical_resolution_;
    // \Delta \vartheta
    double delta_v_res_;
    // v_{th}
    std::vector<double> height_diff_threshold_;
};

}
#endif// __ROAD_BOUNDARY_DETECTOR_H