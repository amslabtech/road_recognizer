#include "road_recognizer/road_recognizer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_recognizer");
    RoadRecognizer road_recognizer;
    road_recognizer.process();
    return 0;
};
