
#include <ros/ros.h>
#include <cstdlib> 

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "road_recognizer/make_image.h"
using std::vector;


class makeimage{
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub_rmground;
		ros::Subscriber sub_grass;
		ros::Publisher pub_image;
		std::string pub_frameid;
		ros::Time pub_stamp;
		bool firstcallback_rmg = true;
		bool firstcallback_grs = true;
		MakeImage MI;

	public:
		makeimage();
		void CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackGrass(const sensor_msgs::PointCloud2ConstPtr& msg);
		void Publication(void);
		void process();
};

makeimage::makeimage()
{
	sub_rmground = nh.subscribe("/rm_ground", 1, &makeimage::CallbackRmGround, this);
	sub_grass = nh.subscribe("/grass", 1, &makeimage::CallbackGrass, this);
	pub_image = nh.advertise<sensor_msgs::Image>("/image", 1);
}
void makeimage::CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout<<"callbackrmgrourd"<<std::endl;
	firstcallback_rmg = false;
	pcl::fromROSMsg(*msg, *MI.rmground);

	MI.ExtractPCInRange(MI.rmground);

}
void makeimage::CallbackGrass(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout<<"callbacgrass"<<std::endl;
	firstcallback_grs = false;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	pcl::fromROSMsg(*msg, *MI.grass);

	MI.ExtractPCInRange(MI.grass);
	
	pub_frameid = msg->header.frame_id;
	pub_stamp = msg->header.stamp;

	MI.make_image();
	
	Publication();
}
void makeimage::Publication(void)
{
	MI.image_ros->header.frame_id = pub_frameid;
	MI.image_ros->header.stamp = pub_stamp;
	pub_image.publish(MI.image_ros);
}

void makeimage::process()
{
	ros::spin();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "make_image");
	// std::cout << "= make_image =" << std::endl;
	
	makeimage makeimage;
	makeimage.process();
	return 0;
}

