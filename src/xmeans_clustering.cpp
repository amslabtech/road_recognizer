#include <iostream>

#include <random>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <boost/thread.hpp>
#include <boost/multi_array.hpp>


typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

class XmeansCluster
{	
	private:
		bool pc_callback_flag = false;
		bool odom_callback_flag = false;
		bool tf_listen_flag = false;
		bool first_flag = false;

		int border_scan_num = 100;
		int count_scan_num = 1;

		const float Hz = 100.0;
		const float EPS = 0.1;

		struct PointCloudClassify{
			std::vector<int> cls;
			CloudIPtr pc_ptr_ {new CloudI};
		};
		
		ros::Subscriber pc_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher grass_pc_publisher;

		nav_msgs::Odometry odom;
		
		tf::TransformListener listener;
		tf::StampedTransform transform;

		// PointCloud
		CloudIPtr transformed_input_pc_ {new CloudI};
		CloudIPtr stored_pc_ {new CloudI};
		struct PointCloudClassify classified_pc_;

	public:
		XmeansCluster(void);

		void pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
		void execution(void);
		void two_means_cluster(CloudIPtr);
		
};





int main(int argc, char** argv)
{
	ros::init(argc, argv, "xmeans_clustering");

	XmeansCluster x_means_clustering;
	x_means_clustering.execution();

	return 0;
}


XmeansCluster::XmeansCluster(void)
{
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	pc_subscriber = n.subscribe("/velodyne_points", 10, &XmeansCluster::pc_callback, this);
    odom_subscriber = n.subscribe("/odom", 10, &XmeansCluster::odom_callback, this);
	
	grass_pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/grass_pc", 10);
}


void XmeansCluster::execution(void)
{
	ros::spin();
}

void XmeansCluster::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	odom = *msg;
	odom_callback_flag = true;
}


void XmeansCluster::pc_callback(const sensor_msgs::PointCloud2ConstPtr &input_pc)
{
	pc_callback_flag = true;

	sensor_msgs::PointCloud2 transformed_input_pc;
	CloudIPtr pre_input_pc_ {new CloudI};
	
	try{
		listener.lookupTransform("/odom","/velodyne", ros::Time(0), transform);
		tf_listen_flag = true;
	}   
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	if(pc_callback_flag && odom_callback_flag && tf_listen_flag){
		if(first_flag){
			size_t pre_input_pc_size = pre_input_pc_->points.size();

			pcl_ros::transformPointCloud("/odom", *input_pc, transformed_input_pc, listener);
			pcl::fromROSMsg(transformed_input_pc, *transformed_input_pc_);
			*stored_pc_ += *transformed_input_pc_;

			if(count_scan_num < border_scan_num){
				count_scan_num++;
			}
		}


	
		pre_input_pc_ = transformed_input_pc_;

		first_flag = true;
		pc_callback_flag = false;
		odom_callback_flag = false;
	}

}


void XmeansCluster::two_means_cluster(CloudIPtr pc_target_)
{
	size_t pc_target_size = pc_target_->points.size();
	
	std::random_device rnd;
		std::mt19937 mt(rnd());
		std::uniform_int_distribution<> rand2(0, 1);

	classified_pc_.pc_ptr_ = pc_target_;
	for(const auto& pt : pc_target_->points){
		classified_pc_.cls.push_back(rand2(mt));
	}

	Eigen::Vector4f center0_pre = Eigen::Vector4f::Zero();
	Eigen::Vector4f center1_pre = Eigen::Vector4f::Zero();
	Eigen::Vector4f center0, center1;
	float center0_displacement, center1_displacement;

	while(center0_displacement < EPS && center1_displacement < EPS){
		Eigen::Vector4f coordinate_sum0 = Eigen::Vector4f::Zero();
		Eigen::Vector4f coordinate_sum1 = Eigen::Vector4f::Zero();
		int cnt0 = 0, cnt1 = 1;
		for(size_t i = 0; i < pc_target_size; i++){
			if(classified_pc_.cls[i] == 0){
				coordinate_sum0[0] += classified_pc_.pc_ptr_->points[i].x;
				coordinate_sum0[1] += classified_pc_.pc_ptr_->points[i].y;
				coordinate_sum0[2] += classified_pc_.pc_ptr_->points[i].z;
				coordinate_sum0[3] += classified_pc_.pc_ptr_->points[i].intensity;
				cnt0++;
			}else{
				coordinate_sum1[0] += classified_pc_.pc_ptr_->points[i].x;
				coordinate_sum1[1] += classified_pc_.pc_ptr_->points[i].y;
				coordinate_sum1[2] += classified_pc_.pc_ptr_->points[i].z;
				coordinate_sum1[3] += classified_pc_.pc_ptr_->points[i].intensity;
				cnt1++;
			}
		}
		center0 = coordinate_sum0 / cnt0;
		center1 = coordinate_sum1 / cnt1;

		for(size_t i = 0; i < pc_target_size; i++){
			float dist0x = center0[0] - classified_pc_.pc_ptr_->points[i].x;
			float dist1x = center1[0] - classified_pc_.pc_ptr_->points[i].x;
			float dist0y = center0[1] - classified_pc_.pc_ptr_->points[i].y;
			float dist1y = center1[1] - classified_pc_.pc_ptr_->points[i].y;
			float dist0z = center0[2] - classified_pc_.pc_ptr_->points[i].z;
			float dist1z = center1[2] - classified_pc_.pc_ptr_->points[i].z;
			float dist0intensity = center0[3] - classified_pc_.pc_ptr_->points[i].intensity;
			float dist1intensity = center1[3] - classified_pc_.pc_ptr_->points[i].intensity;
			float dist0 = sqrt(dist0x*dist0x + dist0y*dist0y + dist0z*dist0z + dist0intensity*dist0intensity);
			float dist1 = sqrt(dist1x*dist1x + dist1y*dist1y + dist1z*dist1z + dist1intensity*dist1intensity);
			
			if(dist0 < dist1){
				classified_pc_.cls[i] = 0;
			}else{
				classified_pc_.cls[i] = 1;
			}
		}
		
		center0_displacement = (center0 - center0_pre).norm();
		center1_displacement = (center1 - center1_pre).norm();
	
		center0_pre = center0;
		center1_pre = center1;
	}
}



