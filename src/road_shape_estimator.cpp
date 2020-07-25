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
    int max_iteration, sample_num, fitting_decision_data_num, beam_num;
    local_nh_.param<double>("convergence_threshold", convergence_threshold_, 0.01);
    local_nh_.param<int>("max_iteration", max_iteration, 100);
    max_iteration_ = max_iteration;
    local_nh_.param<int>("sample_num", sample_num, 4);
    sample_num_ = sample_num;
    local_nh_.param<int>("fitting_decision_data_num", fitting_decision_data_num, 10);
    fitting_decision_data_num_ = fitting_decision_data_num;
    local_nh_.param<double>("resolution", cells_per_meter_, 5.0);
    local_nh_.param<int>("beam_num", beam_num, 120);
    beam_num_ = beam_num;
    local_nh_.param<double>("max_beam_range", max_beam_range_, 10.0);

    curves_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("viz/curves", 1);
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

    rasterize(cloud_ptr);

    std::vector<std::vector<Eigen::Vector2d>> segments = divide_cloud_into_segments(cloud_ptr);

    std::vector<Eigen::MatrixXd> control_points_list;
    // RANSAC 
    for(const auto& segment : segments){
        control_points_list.push_back(fit_ransac_spline(segment));
    }

    publish_marker(control_points_list, msg->header);
}

std::vector<std::vector<Eigen::Vector2d>> RoadShapeEstimator::divide_cloud_into_segments(const pcl::PointCloud<PointT>::Ptr cloud_ptr)
{
    std::vector<std::vector<Eigen::Vector2d>> segments;

    const std::vector<double> beam_list = get_beam_from_cloud(cloud_ptr, 0, 0);
    PeakDetector peak_detector;
    const std::vector<Peak> peak_list = peak_detector.detect_peaks(beam_list);
    if(peak_list.size() < 2){
        std::cout << "num of peaks must be >= 2 to diveide cloud" << std::endl;
        return segments;
    }
    const unsigned int peak_num = peak_list.size();
    segments.resize(peak_num);

    const double d_theta = 2 * M_PI / static_cast<double>(beam_num_);
    for(unsigned int i=0;i<beam_num_;++i){
        const double direction = i * d_theta - M_PI;
        for(unsigned int j=0;j<peak_num;++j){
            if(j > 0){
                const unsigned int k = j - 1; 
                if(peak_list[j].angle_ <= direction && direction < peak_list[k].angle_){
                    Eigen::Vector2d v;
                    v << beam_list[i] * cos(direction), beam_list[i] * sin(direction);
                    segments[j].push_back(v);
                }
            }else{
                // if j == 0
                const unsigned int k = peak_num - 1; 
                if(direction < peak_list[j].angle_ || peak_list[k].angle_ <= direction){
                    Eigen::Vector2d v;
                    v << beam_list[i] * cos(direction), beam_list[i] * sin(direction);
                    segments[j].push_back(v);
                }
            }
        }
    }
    return segments;
}

Eigen::MatrixXd RoadShapeEstimator::fit_ransac_spline(const std::vector<Eigen::Vector2d>& segment)
{
    double best_score = 0.0;
    Eigen::MatrixXd best_control_points = Eigen::MatrixXd::Zero(4, 2);

    for(unsigned int i=0;i<max_iteration_;++i){
        std::vector<unsigned int> sample_indices = get_random_sample(segment);
        Eigen::MatrixXd control_points = fit_spline(segment, sample_indices);
        double score = compute_score(control_points);
        if(score > best_score){
            best_score = score;
            best_control_points = control_points;
        }
    }
    return best_control_points;
}

std::vector<unsigned int> RoadShapeEstimator::get_random_sample(const std::vector<Eigen::Vector2d>& segment)
{
    // TODO: unique random sampling
    std::uniform_int_distribution<> dist(0, segment.size() - 1);
    std::vector<unsigned int> sample_indices;
    sample_indices.reserve(sample_num_);
    while(sample_indices.size() < sample_num_){
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

void RoadShapeEstimator::rasterize(const pcl::PointCloud<PointT>::Ptr cloud_ptr)
{
    grid_params_.min_x = cloud_ptr->points[0].x;
    grid_params_.max_x = cloud_ptr->points[0].x;
    grid_params_.min_y = cloud_ptr->points[0].y;
    grid_params_.max_y = cloud_ptr->points[0].y;
    for(const auto& p : cloud_ptr->points){
        grid_params_.min_x = std::min(grid_params_.min_x, p.x);
        grid_params_.max_x = std::max(grid_params_.max_x, p.x);
        grid_params_.min_y = std::min(grid_params_.min_y, p.y);
        grid_params_.max_y = std::max(grid_params_.max_y, p.y);
    }
    grid_params_.grid_height = (grid_params_.max_x - grid_params_.min_x) * cells_per_meter_;
    grid_params_.grid_width = (grid_params_.max_y - grid_params_.min_y) * cells_per_meter_;
    rasterized_image = std::vector<std::vector<bool>>(grid_params_.grid_height, std::vector<bool>(grid_params_.grid_width, 0));

    for(const auto& p : cloud_ptr->points){
        const unsigned int x_index = (p.x - grid_params_.min_x) * cells_per_meter_;
        const unsigned int y_index = (p.y - grid_params_.min_y) * cells_per_meter_;
        rasterized_image[x_index][y_index] = 1;
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

void RoadShapeEstimator::publish_marker(const std::vector<Eigen::MatrixXd>& control_points_list, const std_msgs::Header& header)
{
    visualization_msgs::MarkerArray curves;
    for(unsigned int i=0;i<control_points_list.size();++i){
        const Eigen::MatrixXd& cp = control_points_list[i];
        visualization_msgs::Marker m;    
        m.header = header;
        m.action = visualization_msgs::Marker::ADD;
        m.ns = ros::this_node::getName();
        m.id = i;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.frame_locked = true;
        m.points.reserve(11);

        const Eigen::MatrixXd tmp_mat = m_mat_ * cp;
        for(double t=0.0;t<=1.0;t+=0.1){
            const Eigen::Vector4d t_vec = get_cubic(t);
            const Eigen::Vector2d point = (t_vec.transpose() * tmp_mat).transpose();
            geometry_msgs::Point p;
            p.x = point(0); 
            p.y = point(1); 
            m.points.emplace_back(p);
        }

        curves.markers.push_back(m);
    }
    curves_pub_.publish(curves);
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

}