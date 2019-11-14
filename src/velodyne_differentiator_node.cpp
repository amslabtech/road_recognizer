#include "road_recognizer/velodyne_differentiator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_differentiator");
    VelodyneDifferentiator vd;
    vd.process();
    return 0;
}
