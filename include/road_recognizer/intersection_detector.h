#ifndef __INTERSECTION_DETECTOR_H
#define __INTERSECTION_DETECTOR_H

// reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4378557
// reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6005622
//
#include <algorithm>
#include <numeric>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include "fstream"

class Peak
{
public:
    Peak(void);
    Peak(int);

    bool operator==(const Peak&) const;
    bool operator!=(const Peak&) const;

    int index;
    double width;
    double angle;
    double angle_diff;
    double angle_width;

private:
};

class IntersectionDetector
{
public:
    IntersectionDetector(void);

    void beam_callback(const std_msgs::Float64MultiArrayConstPtr&);
    void search_peaks(const std::vector<double>&, double, std::vector<Peak>&);
    void set_peak_attribute(const std::vector<double>&, std::vector<Peak>&);
    void remove_square_peaks(const std::vector<double>&, std::vector<int>&, std::vector<Peak>&, const double&);
    void visualize_beam(const std::vector<double>&, const std::vector<Peak>&);
    void clean_peaks(std::vector<int>&, std::vector<Peak>&);
    void cout_peak_candidates(const std::vector<double>&, std::vector<Peak>&);
    void process(void);

private:
    double EPSILON1;
    double EPSILON2_DIV;
    double EPSILON3;
    double MIN_RANGE;
    double MIN_WIDTH;
    double MAX_WIDTH;
    double MAX_ANGLE_WIDTH;
	double SQUARE_RANGE_RATE;
	std::string OUTPUT_FILE_NAME;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Subscriber beam_sub;
    ros::Publisher beam_pub;
    // ros::Publisher intersection_flag_pub;
    ros::Publisher intersection_directions_pub;
};

#endif// __INTERSECTION_DETECTOR_H
