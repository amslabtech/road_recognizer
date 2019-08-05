#ifndef __MAKE_IMAGE_H
#define __MAKE_IMAGE_H

#include <ros/ros.h>
#include <cstdlib> 
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
using std::vector;


class MakeImage
{
	private:
	public:
		pcl::PointCloud<pcl::PointXYZI>::Ptr rmground {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZI>::Ptr grass {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZI>::Ptr grass_points {new pcl::PointCloud<pcl::PointXYZI>};
		double w;	//x[m]
		double h;	//y[m]
		double resolution;	//[m]
		double resolution_rec;	
		int image_w;
		int image_h;
		bool houghline_flag;
		vector<vector<int>> precast;
		sensor_msgs::ImagePtr image_ros;
		sensor_msgs::ImagePtr image_ros2;
		sensor_msgs::PointCloud2 grass_pc2;

		MakeImage();
		void setparam(const double, const double, const double);
		void ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr&);
		void pcl_generater(pcl::PointCloud<pcl::PointXYZI>::Ptr&);
		void make_image(void);
		void generate_pcl(const cv::Mat&);
		void Precast(const int, const int, const int);
		void BEAM(const int, const int, const cv::Mat&, cv::Mat&);
		void AddPointData(const pcl::PointCloud<pcl::PointXYZI>::Ptr&, cv::Mat&, int&);
		void AddPointData_obs(const pcl::PointCloud<pcl::PointXYZI>::Ptr&, cv::Mat&);
		void normalize(cv::Mat& image,const int);
		void amp(cv::Mat& image,const double);
		int MeterpointToPixel_x(const double);
		int MeterpointToPixel_y(const double);
		double PicelyToMeterpoint_x(const int);
		double PicelxToMeterpoint_y(const int);
		void HoughLineP(cv::Mat& image,cv::Mat&);
};
#endif //__MAKE_IMAGE_H
