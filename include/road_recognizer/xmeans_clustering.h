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
#include <pcl/search/kdtree.h>
#include <pcl/common/pca.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"
#include "Eigen/Eigenvalues"
#include "Eigen/Geometry"


class XmeansClustering
{
public:
	typedef pcl::PointXYZI PointI;
	typedef pcl::PointCloud<PointI> CloudI;
	typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;
	typedef pcl::PointXYZINormal PointINormal;
	typedef pcl::PointCloud<PointINormal> CloudINormal;
	typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;

	XmeansClustering(bool, int, int, int, double, double, double);

	CloudINormalPtr execution(CloudIPtr);
	void initialization(void);
	void grid_partition(CloudIPtr not_partitioned_pc);
	CloudIPtr partitional_optimization(CloudIPtr, int);
	void xmeans_clustering(void);
	CloudINormalPtr points_extraction(void);
	CloudIPtr virtual_class_partition(CloudIPtr, int);
	float density_function(CloudIPtr, Eigen::Vector3f, Eigen::Vector3f);
	Eigen::Matrix3f covariance_matrix(int, CloudIPtr);
	float bic_calculation(bool, CloudIPtr);
	int randomization(int);
	float std_normal_distribution_integral(float);
	float my_pow(float);

private:
	bool kmeans;
	bool kmeans_pp;
	bool CLUSTERING_METHOD_;
	int WIDTH_DIVISION_NUM_;
	int HEIGHT_DIVISION_NUM_;
	int MAX_WIDTH_;
	int MAX_HEIGHT_;
	int N_;
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

	std::vector<std::vector<int> > point_counter;
	std::vector<std::vector<float> > intensity_sum;
	std::vector<std::vector<float> > sum_diff_pow;
};


#endif// __XMEANS_CLUSTERING
