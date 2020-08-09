/**
 * @file road_boundary_detector_node.cpp
 * @author amsl
 */

#include "road_recognizer/road_boundary_detector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_boundary_detector");
    road_recognizer::RoadBoundaryDetector rbd;
    rbd.process();
    return 0;
}