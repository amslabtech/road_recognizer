#ifndef __XMEANS_CLUSTERING_H
#define __XMEANS_CLUSTERING_H

#include <iostream>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


class XmeansClustering
{
public:
	typedef pcl::PointXYZI PointI;
	typedef pcl::PointCloud<PointI> CloudI;
	typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;
	typedef pcl::PointXYZINormal PointINormal;
	typedef pcl::PointCloud<PointINormal> CloudINormal;
	typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;

	XmeansClustering(void);

	void initialize(void);
	void intensity2z(CloudIPtr);
	void height_maped_intensity2z_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);

private:
	bool height_maped_intensity2z_pc_callback_flag;
	bool return_flag;
	double intensity2z_height_rate;

	struct Class{
		CloudIPtr cloud;
		std::vector<int> intensity_class;
	};

	ros::NodeHandle n;
	ros::Publisher intensity2z_pub;
	ros::Subscriber height_maped_intensity2z_sub;

	sensor_msgs::PointCloud2 height_maped_intensity2z_pc;
	sensor_msgs::PointCloud2 intensity2z_pub_pc;

	CloudINormalPtr tmp_pc {new CloudINormal};
	CloudINormalPtr intensity2z_pc {new CloudINormal};
	CloudINormalPtr xmeans_road_pc {new CloudINormal};
};


#endif// __XMEANS_CLUSTERING
