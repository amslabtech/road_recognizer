/**
 * @file road_shape_estimator_node.cpp
 * @author amsl
 */

#include "road_recognizer/road_shape_estimator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_shape_estimator.h");
    road_recognizer::RoadShapeEstimator rse;
    rse.process();
    return 0;
}