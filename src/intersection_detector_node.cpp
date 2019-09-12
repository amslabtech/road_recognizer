#include "road_recognizer/intersection_detector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "intersection_detector");
    IntersectionDetector id;
    id.process();
    return 0;
}
