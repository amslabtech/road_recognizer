#include "road_recognizer/road_cloud_publisher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_cloud_publisher");
    RoadCloudPublisher road_cloud_publisher;
    road_cloud_publisher.process();
    return 0;
};
