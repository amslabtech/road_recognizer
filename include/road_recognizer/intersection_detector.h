/**
 * @file intersection_detector.h
 * @author amsl
 */
#ifndef __ROAD_RECOGNIZER_INTERSECTION_DETECTOR_H
#define __ROAD_RECOGNIZER_INTERSECTION_DETECTOR_H

// reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4378557
// reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6005622

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include "road_recognizer/beam_model.h"

namespace road_recognizer
{
class IntersectionDetector
{
public:
    IntersectionDetector(void);

    void beam_callback(const std_msgs::Float64MultiArrayConstPtr&);
    void visualize_beam(const std::vector<double>&, const std::vector<Peak>&);
    void process(void);

private:
    double EPSILON1_;
    double EPSILON2_DIV_;
    double EPSILON3_;
    double MIN_RANGE_;
    double MIN_WIDTH_;
    bool PEAK_ONLY_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Subscriber beam_sub_;
    ros::Publisher beam_pub_;
    ros::Publisher intersection_directions_pub_;
};
}
#endif// __ROAD_RECOGNIZER_INTERSECTION_DETECTOR_H