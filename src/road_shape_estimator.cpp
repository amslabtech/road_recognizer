/**
 * @file road_shape_estimator.cpp
 * @author amsl
 */

#include "road_recognizer/road_shape_estimator.h"

namespace road_recognizer
{

RoadShapeEstimator::RoadShapeEstimator(void)
: local_nh_("~")
, mt_(rnd_())
{
    int max_iteration, sample_num, beam_num;
    local_nh_.param<double>("convergence_threshold", convergence_threshold_, 0.01);
    local_nh_.param<int>("max_iteration", max_iteration, 1000);
    max_iteration_ = max_iteration;
    local_nh_.param<int>("sample_num", sample_num, 4);
    sample_num_ = sample_num;
    local_nh_.param<double>("resolution", cells_per_meter_, 5.0);
    local_nh_.param<int>("beam_num", beam_num, 120);
    beam_num_ = beam_num;
    local_nh_.param<double>("max_beam_range", max_beam_range_, 20.0);
    local_nh_.param<double>("min_segment_length", min_segment_length_, 5.0);

    curves_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("viz/curves", 1);
    beam_pub_ = local_nh_.advertise<visualization_msgs::Marker>("viz/beam", 1);
    cloud_sub_ = nh_.subscribe("cloud", 1, &RoadShapeEstimator::cloud_callback, this);

    m_mat_ << -1,  3, -3, 1,
               3, -6,  3, 0,
              -3,  3,  0, 0,
               1,  0,  0, 0;
    
    last_curves_num_ = 0;
    
    std::cout << "convergence_threshold: " << convergence_threshold_ << std::endl;
    std::cout << "max_iteration: " << max_iteration_ << std::endl;
    std::cout << "sample_num: " << sample_num << std::endl;
    std::cout << "cells_per_meter: " << cells_per_meter_ << std::endl;
    std::cout << "beam_num: " << beam_num_ << std::endl;
    std::cout << "max_beam_range: " << max_beam_range_ << std::endl;
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
    if(segments.size() == 0){
        std::cout << "segment is empty, following process is skipped " << std::endl;
        return;
    }
    rasterize(segments);

    std::vector<Eigen::MatrixXd> control_points_list;
    // RANSAC 
    for(const auto& segment : segments){
        if(segment.size() == 0){
            continue;
        }
        control_points_list.push_back(fit_ransac_spline(segment));
    }
    publish_curve_marker(control_points_list, msg->header);
}

std::vector<std::vector<Eigen::Vector2d>> RoadShapeEstimator::divide_cloud_into_segments(const pcl::PointCloud<PointT>::Ptr cloud_ptr)
{
    std::vector<std::vector<Eigen::Vector2d>> segments;

    const std::vector<double> beam_list = get_beam_from_cloud(cloud_ptr, 0, 0);
    PeakDetector peak_detector;
    const std::vector<Peak> peak_list = peak_detector.detect_peaks(beam_list);
    publish_beam_marker(beam_list, peak_list, pcl_conversions::fromPCL(cloud_ptr->header));
    if(peak_list.size() < 2){
        std::cout << "num of peaks must be >= 2 to divide cloud" << std::endl;
        return segments;
    }
    auto get_peak_direction = [&](const Peak& peak)
    {
        const double direction = peak.index_ * 2 * M_PI / static_cast<double>(beam_num_) - M_PI;
        return direction;
    };
    const unsigned int peak_num = peak_list.size();
    segments.resize(peak_num);

    const double d_theta = 2 * M_PI / static_cast<double>(beam_num_);
    for(unsigned int i=0;i<beam_num_;++i){
        if(beam_list[i] >= max_beam_range_){
            continue;
        }
        const double direction = i * d_theta - M_PI;
        // check if the direction is between two peaks
        for(unsigned int j=0;j<peak_num;++j){
            const double direction_j = get_peak_direction(peak_list[j]);
            if(j > 0){
                const unsigned int k = j - 1; 
                const double direction_k = get_peak_direction(peak_list[k]);
                if(direction_k <= direction && direction < direction_j){
                    Eigen::Vector2d v;
                    v << beam_list[i] * cos(direction), beam_list[i] * sin(direction);
                    segments[j].push_back(v);
                    break;
                }
            }else{
                // if j == 0
                const unsigned int k = peak_num - 1; 
                const double direction_k = get_peak_direction(peak_list[k]);
                if(direction < direction_j || direction_k <= direction){
                    Eigen::Vector2d v;
                    v << beam_list[i] * cos(direction), beam_list[i] * sin(direction);
                    segments[j].push_back(v);
                    break;
                }
            }
        }
    }
    for(auto it=segments.begin();it!=segments.end();){
        if(compute_segment_length(*it) > min_segment_length_){
            ++it;
        }else{
            it = segments.erase(it);
        }
    }
    return segments;
}

Eigen::MatrixXd RoadShapeEstimator::fit_ransac_spline(const std::vector<Eigen::Vector2d>& segment)
{
    double best_score = 0.0;
    const double score_threshold = segment.size() * 0.5;
    Eigen::MatrixXd best_control_points = Eigen::MatrixXd::Zero(4, 2);

    for(unsigned int i=0;i<max_iteration_;++i){
        std::vector<unsigned int> sample_indices = get_random_sample(segment);
        Eigen::MatrixXd control_points = fit_spline(segment, sample_indices);
        const double score = compute_score(control_points);
        if(score > best_score){
            best_score = score;
            best_control_points = control_points;
        }
        if(best_score >= score_threshold){
            break;
    }
    }
    return best_control_points;
}

std::vector<unsigned int> RoadShapeEstimator::get_random_sample(const std::vector<Eigen::Vector2d>& segment)
{
    // TODO: unique random sampling
    std::uniform_int_distribution<> dist(1, segment.size() - 2);
    std::vector<unsigned int> sample_indices;
    sample_indices.reserve(sample_num_);
    sample_indices.emplace_back(0);
    while(sample_indices.size() < sample_num_ - 1){
        sample_indices.emplace_back(dist(mt_));
    }
    sample_indices.emplace_back(segment.size() - 1);
    std::sort(sample_indices.begin(), sample_indices.end());
    return sample_indices;
}

Eigen::MatrixXd RoadShapeEstimator::fit_spline(const std::vector<Eigen::Vector2d>& segment, const std::vector<unsigned int>& indices)
{
    const unsigned int num = indices.size();
    std::vector<double> d(num, 0);
    for(unsigned int i=1;i<num;++i){
        d[i] = (segment[indices[i]] - segment[indices[i-1]]).norm();
    }
    std::vector<double> cumulative_sum_of_d(num, 0);
    cumulative_sum_of_d[0] = d[0];
    for(unsigned int i=1;i<num;++i){
        cumulative_sum_of_d[i] = cumulative_sum_of_d[i-1] + d[i];
    }
    // t \in [0, 1]
    std::vector<double> t(num, 0);
    for(unsigned int i=0;i<num;++i){
        t[i] = cumulative_sum_of_d[i] / cumulative_sum_of_d[num-1];
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
    const Eigen::MatrixXd tm_mat = t_mat * m_mat_;
    const Eigen::MatrixXd p_mat = tm_mat.fullPivLu().solve(q_mat);
    return p_mat;
}

void RoadShapeEstimator::rasterize(const std::vector<std::vector<Eigen::Vector2d>>& segments)
{
    grid_params_.min_x = segments[0][0](0);
    grid_params_.max_x = segments[0][0](0);
    grid_params_.min_y = segments[0][0](1);
    grid_params_.max_y = segments[0][0](1);
    for(const auto& segment : segments){
        for(const auto& p : segment){
            grid_params_.min_x = std::min(static_cast<double>(grid_params_.min_x), p(0));
            grid_params_.max_x = std::max(static_cast<double>(grid_params_.max_x), p(0));
            grid_params_.min_y = std::min(static_cast<double>(grid_params_.min_y), p(1));
            grid_params_.max_y = std::max(static_cast<double>(grid_params_.max_y), p(1));
        }
    }
    grid_params_.grid_height = std::ceil((grid_params_.max_x - grid_params_.min_x) * cells_per_meter_);
    grid_params_.grid_width = std::ceil((grid_params_.max_y - grid_params_.min_y) * cells_per_meter_);
    rasterized_image = std::vector<std::vector<bool>>(grid_params_.grid_height, std::vector<bool>(grid_params_.grid_width, 0));

    for(const auto& segment : segments){
        for(const auto& p : segment){
            const unsigned int x_index = (p(0) - grid_params_.min_x) * cells_per_meter_;
            const unsigned int y_index = (p(1) - grid_params_.min_y) * cells_per_meter_;
        rasterized_image[x_index][y_index] = 1;
    }
}
}

double RoadShapeEstimator::compute_score(const Eigen::MatrixXd& control_points)
{
    // TODO: Vary num with the length of the curve
    const unsigned int num = 10;
    // t \in [0, 1]
    std::vector<double> t(num, 0);
    for(unsigned int i=1;i<num;++i){
        t[i] = t[i-1] + 0.1;
    }
    double score = 0;
    for(unsigned int i=0;i<num;++i){
        Eigen::Vector4d t_vec = get_cubic(t[i]);
        Eigen::Vector2d v = t_vec.transpose() * m_mat_ * control_points; 
        const unsigned int x_index = (v(0) - grid_params_.min_x) * cells_per_meter_;
        const unsigned int y_index = (v(1) - grid_params_.min_y) * cells_per_meter_;
        if(x_index >= grid_params_.grid_height){
            return 0;
        }else if(y_index >= grid_params_.grid_width){
            return 0;
        }
        if(rasterized_image[x_index][y_index] > 0){
            score += 1.0;
        }
    }
    return score;
}

Eigen::Vector4d RoadShapeEstimator::get_cubic(double x)
{
    Eigen::Vector4d v;
    v << x * x * x, x * x, x, 1;
    return v;
}

void RoadShapeEstimator::publish_curve_marker(const std::vector<Eigen::MatrixXd>& control_points_list, const std_msgs::Header& header)
{
    visualization_msgs::MarkerArray curves;
    unsigned int i = 0;
    for(;i<control_points_list.size();++i){
        const Eigen::MatrixXd& cp = control_points_list[i];
        visualization_msgs::Marker m;    
        m.header = header;
        m.action = visualization_msgs::Marker::ADD;
        m.ns = ros::this_node::getName();
        m.id = i;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.frame_locked = true;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.1;
        m.lifetime = ros::Duration();
        m.points.reserve(11);
        m.colors.reserve(11);

        const Eigen::MatrixXd tmp_mat = m_mat_ * cp;
        for(double t=0.0;t<=1.0;t+=0.1){
            const Eigen::Vector4d t_vec = get_cubic(t);
            const Eigen::Vector2d point = (t_vec.transpose() * tmp_mat).transpose();
            geometry_msgs::Point p;
            p.x = point(0); 
            p.y = point(1); 
            m.points.emplace_back(p);
            std_msgs::ColorRGBA c;
            c.r = 1.0;
            c.a = 0.8;
            m.colors.emplace_back(c);
        }

        curves.markers.push_back(m);
    }
    const unsigned curves_num = curves.markers.size();
    for(;i<last_curves_num_;++i){
        visualization_msgs::Marker m;    
        m.header = header;
        m.action = visualization_msgs::Marker::DELETE;
        m.ns = ros::this_node::getName();
        m.id = i;
        curves.markers.push_back(m);
    }
    curves_pub_.publish(curves);
    last_curves_num_ = curves_num;
}

std::vector<double> RoadShapeEstimator::get_beam_from_cloud(pcl::PointCloud<PointT>::Ptr cloud_ptr, double origin_x, double origin_y)
{
    const double d_theta = 2 * M_PI / static_cast<double>(beam_num_);
    std::vector<double> beam_list(beam_num_, max_beam_range_);
    auto compute_distance = [](double x0, double y0, double x1, double y1)
    {
        return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    };
    for(const auto& p : cloud_ptr->points){
        const double distance = compute_distance(p.x, p.y, origin_x, origin_y);
        const double direction = atan2(p.y - origin_y, p.x - origin_x) + M_PI;
        const unsigned int index = std::floor(direction / d_theta);
        beam_list[index] = std::min(beam_list[index], distance);
    }
    return beam_list;
}

void RoadShapeEstimator::publish_beam_marker(const std::vector<double>& beam_list, const std::vector<Peak>& peak_list, const std_msgs::Header& header)
{
    visualization_msgs::Marker beam_marker;    
    beam_marker.header = header;
    beam_marker.action = visualization_msgs::Marker::ADD;
    beam_marker.ns = ros::this_node::getName();
    beam_marker.id = 0;
    beam_marker.type = visualization_msgs::Marker::LINE_LIST;
    beam_marker.frame_locked = true;
    beam_marker.pose.orientation.w = 1.0;
    beam_marker.scale.x = 0.1;
    beam_marker.lifetime = ros::Duration();
    const unsigned int size = beam_list.size();
    beam_marker.points.reserve(size * 2);
    beam_marker.colors.reserve(size * 2);

    auto is_peak = [&](unsigned int index)
    {
        for(const auto& peak : peak_list){
            if(peak.index_ == static_cast<int>(index)){
                return true;
            }
        }
        return false;
    };

    const double d_theta = 2 * M_PI / static_cast<double>(beam_num_);
    for(unsigned int i=0;i<size;++i){
        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = 0.0;
        beam_marker.points.emplace_back(p);
        const double theta = i * d_theta - M_PI;
        p.x = beam_list[i] * cos(theta);
        p.y = beam_list[i] * sin(theta);
        beam_marker.points.emplace_back(p);
        std_msgs::ColorRGBA c;
        if(!is_peak(i)){
            c.g = 1.0;
        }else{
            c.r = 1.0;
        }
        c.a = 0.8;
        beam_marker.colors.emplace_back(c);
        beam_marker.colors.emplace_back(c);
    }

    beam_pub_.publish(beam_marker);
}

double RoadShapeEstimator::compute_segment_length(const std::vector<Eigen::Vector2d>& segment)
{
    double length = 0.0;
    const unsigned int size = segment.size();
    for(unsigned int i=1;i<size;++i){
        length += (segment[i] - segment[i - 1]).norm();
    }
    return length;
}

}