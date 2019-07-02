#ifndef __ROAD_RECOGNIZER_H
#define __ROAD_RECOGNIZER_H

#include <ros/ros.h>

// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>

// RANSAC
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class RoadRecognizer
{
public:
    RoadRecognizer(void);

    void process(void);
    void obstacles_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void ground_callback(const sensor_msgs::PointCloud2ConstPtr&);

private:
    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Subscriber obstacles_sub;
    ros::Subscriber ground_sub;
};

#endif// __ROAD_RECOGNIZER_H
