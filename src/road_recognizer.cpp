#include "road_recognizer/road_recognizer.h"

RoadRecognizer::RoadRecognizer(void)
{
    obstacles_sub = nh.subscribe("/velodyne_obstacles", 1, &RoadRecognizer::obstacles_callback, this);
    ground_sub = nh.subscribe("/velodyne_clear", 1, &RoadRecognizer::ground_callback, this);
}

void RoadRecognizer::obstacles_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}

void RoadRecognizer::ground_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}

void RoadRecognizer::process(void)
{
    ros::spin();
}
