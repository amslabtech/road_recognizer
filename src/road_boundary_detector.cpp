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
    ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud/ground", 1);
    obstacle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud/obstacle", 1);
    boundary_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud/road_boundary", 1);
    cloud_sub_ = nh_.subscribe("velodyne_points", 1, &RoadBoundaryDetector::cloud_callback, this);

    int layer_num;
    local_nh_.param<int>("layer_num", layer_num, 32);
    layer_num_ = static_cast<unsigned int>(layer_num);
    local_nh_.param<double>("lidar_height", lidar_height_, 1.3);
    // HDL-32E gets 2160 points per layer at 600rpm(10Hz)
    local_nh_.param<double>("horizontal_resolution", horizontal_resolution_, 2.0 * M_PI / 2160.0);
    int lambda;
    local_nh_.param<int>("lambda", lambda, 2);
    lambda_ = static_cast<unsigned int>(lambda);
    // from -z axis 
    local_nh_.param<double>("vertical_scan_angle_begin", vertical_scan_angle_begin_, -30.67 * M_PI / 180.0 + M_PI * 0.5);
    double vertical_scan_angle_end_lidar;
    local_nh_.param<double>("vertical_scan_angle_end_lidar", vertical_scan_angle_end_lidar, 10.67 * M_PI / 180.0 + M_PI * 0.5);
    int vertical_scan_num;
    local_nh_.param<int>("vertical_scan_num", vertical_scan_num, 22);
    vertical_scan_num_ = static_cast<unsigned int>(vertical_scan_num);
    local_nh_.param<double>("bottom_threshold", bottom_threshold_, 0.05);

    ROS_INFO_STREAM("=== road_boundary_detector ===");
    ROS_INFO_STREAM("layer_num: " << layer_num_);
    ROS_INFO_STREAM("lidar_height: " << lidar_height_);
    ROS_INFO_STREAM("horizontal_resolution: " << horizontal_resolution_ << "[rad]");
    ROS_INFO_STREAM("lambda: " << lambda_);
    ROS_INFO_STREAM("vertical_scan_angle_begin: " << vertical_scan_angle_begin_ << "[rad]");
    ROS_INFO_STREAM("vertical_scan_angle_end_lidar: " << vertical_scan_angle_end_lidar << "[rad]");
    ROS_INFO_STREAM("vertical_scan_num: " << vertical_scan_num_);
    ROS_INFO_STREAM("bottom_threshold: " << bottom_threshold_ << "[m]");

    vertical_resolution_ = (vertical_scan_angle_end_lidar - vertical_scan_angle_begin_) / static_cast<double>(layer_num_ - 1);
    // only downward laser
    vertical_scan_angle_end_ = vertical_scan_angle_begin_ + vertical_resolution_ * vertical_scan_num_;
    ROS_INFO_STREAM("vertical_resolution: " << vertical_resolution_ << "[rad]");
    ROS_INFO_STREAM("vertical_scan_angle_end: " << vertical_scan_angle_end_ << "[rad]");

    num_sectors_ = std::floor(2.0 * M_PI / (lambda_ * horizontal_resolution_));
    delta_v_res_ = lambda_ * vertical_resolution_;
    num_bins_ = std::floor((vertical_scan_angle_end_ - vertical_scan_angle_begin_) / delta_v_res_);
    ROS_INFO_STREAM("num_sectors: " << num_sectors_);
    ROS_INFO_STREAM("delta_v_res: " << delta_v_res_ << "[rad]");
    ROS_INFO_STREAM("num_bins: " << num_bins_);

    b_.resize(num_bins_, 0.0);
    vertical_angles_.resize(num_bins_, 0.0);
    for(unsigned int i=0;i<num_bins_;++i){
        b_[i] = lidar_height_ * tan(vertical_scan_angle_begin_ + delta_v_res_ * i);
        vertical_angles_[i] = atan2(b_[i], lidar_height_);
        // ROS_INFO_STREAM("b[" << i << "]: " << b_[i] << "[m]");
    }

    height_diff_threshold_.resize(num_bins_-1, 0.0);
    for(unsigned int i=0;i<num_bins_-1;++i){
        height_diff_threshold_[i] = (b_[i+1] - b_[i]) * tan(M_PI * 0.5 - (vertical_scan_angle_begin_ + delta_v_res_ * i));
    }
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

        // const unsigned int ring_index = get_ring_index_from_firing_order(i % layer_num_);
        //
        // limit ring_index 
        //
        const double horizontal_distance = point.segment(0, 2).norm();
        const unsigned int sector_index = (acos(point(0) / horizontal_distance) + M_PI * unit_step_function(-point(1))) / lambda_h_res;
        const unsigned int bin_index = get_bin_index(point(0), point(1));
        polar_grid[sector_index][bin_index].push_back(i);
    }

    // extract ground points
    std::vector<unsigned int> ground_point_indices;
    std::vector<unsigned int> obstacle_point_indices;
    ground_point_indices.reserve(size);
    obstacle_point_indices.reserve(size);
    auto is_lower = [&](unsigned int i0, unsigned int i1) -> bool
    {
        return cloud_ptr->points[i0].z < cloud_ptr->points[i1].z;
    };
    for(unsigned int i=0;i<num_sectors_;++i){
        for(unsigned int j=0;j<num_bins_-1;++j){
            const unsigned int n = polar_grid[i][j].size();
            if(n == 0){
                continue;
            }
            std::sort(polar_grid[i][j].begin(), polar_grid[i][j].end(), is_lower);
            if(cloud_ptr->points[polar_grid[i][j][0]].z + lidar_height_ > bottom_threshold_){
                // obviously obstacle points in this grid
                obstacle_point_indices.emplace_back(polar_grid[i][j][0]);
                continue;
            }
            unsigned int k = 0;
            for(;k<n-1;++k){
                if(cloud_ptr->points[polar_grid[i][j][k+1]].z - cloud_ptr->points[polar_grid[i][j][k]].z > height_diff_threshold_[j]){
                    break;
                }
            }
            if(cloud_ptr->points[polar_grid[i][j][k]].z + lidar_height_ < bottom_threshold_){
                unsigned int l = 0;
                for(;l<=k;++l){
                    ground_point_indices.emplace_back(polar_grid[i][j][l]);
                }
                for(;l<n;++l){
                    obstacle_point_indices.emplace_back(polar_grid[i][j][l]);
                }
            }else{
                for(unsigned int l=0;l<n;++l){
                    if(cloud_ptr->points[polar_grid[i][j][l]].z + lidar_height_ < bottom_threshold_){
                        // ground_point_indices.emplace_back(polar_grid[i][j][l]);
                    }else{
                        obstacle_point_indices.emplace_back(polar_grid[i][j][l]);
                    }
                }
            }
        }
    }
    publish_cloud(cloud_ptr, ground_point_indices, obstacle_point_indices);
    detect_road_boundary(cloud_ptr, ground_point_indices);
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

double RoadBoundaryDetector::compute_distance(double x0, double y0, double x1, double y1)
{
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    return sqrt(dx * dx + dy * dy);
}

unsigned int RoadBoundaryDetector::get_bin_index(double x, double y)
{
    const double d = compute_distance(0, 0, x, y);
    unsigned int index = 0;
    for(;index<num_bins_-1;++index){
        if(b_[index] < d && d <= b_[index+1]){
            break;
        }
    }
    return index;
}

void RoadBoundaryDetector::publish_cloud(const pcl::PointCloud<PointT>::Ptr cloud_ptr, const std::vector<unsigned int>& ground_point_indices, const std::vector<unsigned int>& obstacle_point_indices)
{
    pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>);
    ground_cloud->header = cloud_ptr->header;
    obstacle_cloud->header = cloud_ptr->header;
    const unsigned int gp_size = ground_point_indices.size();
    const unsigned int op_size = obstacle_point_indices.size();
    ground_cloud->points.reserve(gp_size);
    obstacle_cloud->points.reserve(op_size);
    for(unsigned int i=0;i<gp_size;++i){
        ground_cloud->points.emplace_back(cloud_ptr->points[ground_point_indices[i]]);
    }
    for(unsigned int i=0;i<op_size;++i){
        obstacle_cloud->points.emplace_back(cloud_ptr->points[obstacle_point_indices[i]]);
    }
    ground_cloud_pub_.publish(*ground_cloud);
    obstacle_cloud_pub_.publish(*obstacle_cloud);
}

void RoadBoundaryDetector::detect_road_boundary(const pcl::PointCloud<PointT>::Ptr cloud_ptr, const std::vector<unsigned int>& ground_point_indices)
{
    // distribute points by ring index
    // low index -> right
    const double alpha_th = 135 * M_PI / 180.0;
    pcl::PointCloud<PointT>::Ptr boundary_cloud(new pcl::PointCloud<PointT>);
    boundary_cloud->header = cloud_ptr->header;
    boundary_cloud->points.reserve(ground_point_indices.size());

    std::vector<std::vector<unsigned int>> rings(vertical_scan_num_);
    for(auto& r : rings){
        r.reserve(2.0  * M_PI / horizontal_resolution_);
    }
    for(const auto& i : ground_point_indices){
        const unsigned int ring_index = get_ring_index_from_firing_order(i % layer_num_);
        if(0 <= ring_index && ring_index < vertical_scan_num_){
            rings[ring_index].emplace_back(i);
        }
    }
    auto compare = [](const unsigned int& lhs, const unsigned int& rhs)-> bool{return lhs < rhs;};
    for(auto& r : rings){
        const unsigned int num = r.size();
        if(num < 3){
            continue;
        }
        std::sort(r.begin(), r.end(), compare);
        const unsigned int j = 2;
        for(unsigned int i=j;i<num-j;++i){
            const Eigen::Vector3d p_0 = get_vector_from_point(cloud_ptr->points[r[i-j]]);
            const Eigen::Vector3d p_1 = get_vector_from_point(cloud_ptr->points[r[i]]);
            const Eigen::Vector3d p_2 = get_vector_from_point(cloud_ptr->points[r[i+j]]);
            const Eigen::Vector3d p_10 = p_0 - p_1;
            const Eigen::Vector3d p_12 = p_2 - p_1;
            const double alpha = acos(p_10.dot(p_12) / (p_10.norm() * p_12.norm()));
            if(alpha < alpha_th){
                boundary_cloud->points.emplace_back(cloud_ptr->points[r[i]]);
            }
        }
    }
    boundary_cloud_pub_.publish(*boundary_cloud);
}

Eigen::Vector3d RoadBoundaryDetector::get_vector_from_point(const PointT& p)
{
    Eigen::Vector3d v(p.x, p.y, p.z);
    return v;
}

}