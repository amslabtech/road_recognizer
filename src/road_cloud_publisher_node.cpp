#include "road_recognizer/road_cloud_publisher.h"
#include <dynamic_reconfigure/server.h>
#include "road_recognizer/change_intensityConfig.h"

void callback(road_recognizer::change_intensityConfig &config, uint32_t level) {
    // ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
    //         config.int_param, config.double_param, 
    //         config.str_param.c_str(), 
    //         config.bool_param?"True":"False", 
    //         config.size);
    ROS_WARN("Rconfig : %d", config.grass_intensity_upper);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_cloud_publisher");

    dynamic_reconfigure::Server<road_recognizer::change_intensityConfig> server;
    dynamic_reconfigure::Server<road_recognizer::change_intensityConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    RoadCloudPublisher road_cloud_publisher;
    road_cloud_publisher.process();
    return 0;
};
