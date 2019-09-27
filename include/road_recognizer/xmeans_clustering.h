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

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"


class XmeansClustering
{
public:
	typedef pcl::PointXYZ Point;
	typedef pcl::PointCloud<Point> Cloud;
	typedef pcl::PointCloud<Point>::Ptr CloudPtr;
	typedef pcl::PointXYZI PointI;
	typedef pcl::PointCloud<PointI> CloudI;
	typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;
	typedef pcl::PointXYZINormal PointINormal;
	typedef pcl::PointCloud<PointINormal> CloudINormal;
	typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;

	XmeansClustering(bool, int, int, double, double, double);

	CloudINormalPtr execution(CloudIPtr);
	void initialization(void);
	void grid_partition(CloudIPtr not_partitioned_pc);
	int randomization(bool, int);
	void partitional_optimization(int, int);
	void xmeans_clustering(int);
	float density_function(int);
	float my_pow(float);

private:
	bool kmeans;
	bool kmeans_pp;
	bool CLUSTERING_METHOD_;
	int WIDTH_DIVISION_NUM_;
	int HEIGHT_DIVISION_NUM_;
	int MAX_WIDTH_;
	int MAX_HEIGHT_;
	double dX;
	double dY;
	double EPS_;
	struct grid{
		std::vector<std::vector<int> > affiliation;
		std::vector<std::vector<float> > intensity_average;
		std::vector<std::vector<float> > intensity_std_deviation;
		std::vector<std::vector<CloudIPtr> > point_cloud;
	};
	struct grid cells;

	std::vector<CloudPtr> grid_points;
	std::vector<std::vector<int> > point_counter;
	std::vector<std::vector<float> > intensity_sum;
	std::vector<std::vector<float> > sum_diff_pow;

	CloudINormalPtr xmeans_pc {new CloudINormal};
};


#endif// __XMEANS_CLUSTERING
