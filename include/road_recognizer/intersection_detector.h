#ifndef __INTERSECTION_DETECTOR_H
#define __INTERSECTION_DETECTOR_H

// reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4378557
// reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6005622
//
#include <algorithm>
#include <numeric>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>

class IntersectionDetector
{
public:
    IntersectionDetector(void);

    void beam_callback(const std_msgs::Float64MultiArrayConstPtr&);
    void search_peaks(const std::vector<double>&, double, std::vector<int>&);
    void visualize_beam(const std::vector<double>&, const std::vector<int>&);
    void process(void);

private:
    double EPSILON1;
    double EPSILON2_DIV;
    double EPSILON3;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Subscriber beam_sub;
    ros::Publisher beam_pub;
};

#endif// __INTERSECTION_DETECTOR_H
