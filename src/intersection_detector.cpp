/**
 * @file intersection_detector.cpp
 * @author amsl
 */
#include "road_recognizer/intersection_detector.h"

namespace road_recognizer{
IntersectionDetector::IntersectionDetector(void)
:local_nh_("~")
{
    beam_sub_ = nh_.subscribe("beam_array", 1, &IntersectionDetector::beam_callback, this);
    beam_pub_ = local_nh_.advertise<visualization_msgs::Marker>("beam", 1);
    intersection_directions_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/intersection_directions", 1);

    local_nh_.param("EPSILON1", EPSILON1_, {0.25});
    local_nh_.param("EPSILON2_DIV", EPSILON2_DIV_, {8});
    local_nh_.param("EPSILON3", EPSILON3_, {0.95});
    local_nh_.param("MIN_RANGE", MIN_RANGE_, {10.0});
    local_nh_.param("MIN_WIDTH", MIN_WIDTH_, {0.8});
    local_nh_.param("PEAK_ONLY", PEAK_ONLY_, {false});

    std::cout << "=== intersection_detector ===" << std::endl;
    std::cout << "EPSILON1: " << EPSILON1_ << std::endl;
    std::cout << "EPSILON2_DIV: " << EPSILON2_DIV_ << std::endl;
    std::cout << "EPSILON3: " << EPSILON3_ << std::endl;
    std::cout << "MIN_RANGE: " << MIN_RANGE_ << std::endl;
    std::cout << "MIN_WIDTH: " << MIN_WIDTH_ << std::endl;
    std::cout << "PEAK_ONLY: " << PEAK_ONLY_ << std::endl;
}

void IntersectionDetector::beam_callback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    std::cout << "--- beam_callback ---" << std::endl;
    double start_time = ros::Time::now().toSec();
    std::vector<double> beam_ranges = msg->data;
    const int N = beam_ranges.size();
    std::cout << "number of beam: " << N << std::endl;
    if(N > 0){
        BeamModel beam_model;
        beam_model.set_parameters(EPSILON1_, EPSILON2_DIV_, EPSILON3_, MIN_RANGE_, MIN_WIDTH_);
        std::vector<Peak> peak_list = beam_model.detect_peaks(beam_ranges);
        visualize_beam(beam_ranges, peak_list);

        std_msgs::Float64MultiArray directions;
        for(auto it=peak_list.begin();it!=peak_list.end();++it){
            directions.data.push_back(it->index_ * 2 * M_PI / (double)N - M_PI);
        }
        intersection_directions_pub_.publish(directions);
    }else{
        std::cout << "\033[31mbeam_ranges is empty\033[0m" << std::endl;
    }
    std::cout << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
}

void IntersectionDetector::visualize_beam(const std::vector<double>& beam_ranges, const std::vector<Peak>& peak_list)
{
    const int N = beam_ranges.size();
    visualization_msgs::Marker beam_marker;
    beam_marker.header.frame_id = "base_link";
    beam_marker.header.stamp = ros::Time::now();
    beam_marker.ns = "intersection_detector";
    beam_marker.id = 0;
    beam_marker.type = visualization_msgs::Marker::LINE_LIST;
    beam_marker.action = visualization_msgs::Marker::ADD;
    beam_marker.lifetime = ros::Duration(0);
    beam_marker.pose.orientation.w = 1.0;
    beam_marker.points.reserve(N);
    beam_marker.colors.reserve(N);
    beam_marker.scale.x = 0.1;
    for(int i=0;i<N;i++){
        bool is_not_peak = std::find(peak_list.begin(), peak_list.end(), i) == peak_list.end();
        if(is_not_peak){
            if(!PEAK_ONLY_){
                geometry_msgs::Point p;
                p.x = 0.0;
                p.y = 0.0;
                p.z = 0.0;
                beam_marker.points.push_back(p);
                double angle = i / (double)N * 2.0 * M_PI  - M_PI;
                p.x = beam_ranges[i] * cos(angle);
                p.y = beam_ranges[i] * sin(angle);
                beam_marker.points.push_back(p);
                std_msgs::ColorRGBA c;
                c.r = 0;
                c.g = 1;
                c.b = 0;
                c.a = 0.8;
                beam_marker.colors.push_back(c);
                beam_marker.colors.push_back(c);
            }
        }else{
            geometry_msgs::Point p;
            p.x = 0.0;
            p.y = 0.0;
            p.z = 0.0;
            beam_marker.points.push_back(p);
            double angle = i / (double)N * 2.0 * M_PI  - M_PI;
            p.x = beam_ranges[i] * cos(angle);
            p.y = beam_ranges[i] * sin(angle);
            beam_marker.points.push_back(p);
            std_msgs::ColorRGBA c;
            c.r = 1;
            c.g = 0;
            c.b = 0;
            c.a = 0.8;
            beam_marker.colors.push_back(c);
            beam_marker.colors.push_back(c);
        }
    }
    beam_pub_.publish(beam_marker);
}

void IntersectionDetector::process(void)
{
    ros::spin();
}
}