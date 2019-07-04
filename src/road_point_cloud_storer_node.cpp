#include "road_recognizer/road_point_cloud_storer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_point_cloud_storer");
    RoadPointCloudStorer road_point_cloud_storer;
    road_point_cloud_storer.process();
    return 0;
};
