
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
		ros::NodeHandle private_nh;
		ros::Subscriber sub_rmground;
		ros::Subscriber sub_grass;
		ros::Publisher pub_image;
		ros::Publisher pub_edge;
		ros::Publisher pub_pcl;
		std::string pub_frameid;
		ros::Time pub_stamp;
		bool firstcallback_rmg = true;
		bool firstcallback_grs = true;
		double IMAGE_WIDTH;
		double IMAGE_HEIGHT;
		double IMAGE_RESOLUTION;
		bool HOUGHLINE_FLAG;
		int BEAM_ANGLE_NUM;
		double MAX_BEAM_RANGE;
		MakeImage MI;

	public:
		makeimage();
		void CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackGrass(const sensor_msgs::PointCloud2ConstPtr& msg);
		void Publication(void);
		void process();
};

makeimage::makeimage()
	: private_nh("~")
{
	
	private_nh.param("IMAGE_WIDTH", IMAGE_WIDTH, {40});
	private_nh.param("IMAGE_HEIGHT", IMAGE_HEIGHT, {40});
	private_nh.param("IMAGE_RESOLUTION", IMAGE_RESOLUTION, {0.1});
	private_nh.param("HOUGHLINE_FLAG", HOUGHLINE_FLAG, {true});
	private_nh.param("BEAM_ANGLE_NUM", BEAM_ANGLE_NUM, {360});
	private_nh.param("MAX_BEAM_RANGE", MAX_BEAM_RANGE, {1023});
	std::cout << "IMAGE_WIDTH 		: " << IMAGE_WIDTH << std::endl;
	std::cout << "IMAGE_HEIGHT 		: " << IMAGE_HEIGHT << std::endl;
	std::cout << "IMAGE_RESOLUTION 	: " << IMAGE_RESOLUTION << std::endl;
	std::cout << "HOUGHLINE_FLAG 	: " << HOUGHLINE_FLAG << std::endl;
	std::cout << "BEAM_ANGLE_NUM 	: " << BEAM_ANGLE_NUM << std::endl;
	std::cout << "MAX_BEAM_RANGE	: " << MAX_BEAM_RANGE << std::endl;
	
	MI.houghline_flag=HOUGHLINE_FLAG;
	MI.set_param(IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_RESOLUTION, BEAM_ANGLE_NUM, MAX_BEAM_RANGE);
	sub_rmground = nh.subscribe("/rm_ground", 1, &makeimage::CallbackRmGround, this);
	sub_grass = nh.subscribe("/grass", 1, &makeimage::CallbackGrass, this);
	pub_image = nh.advertise<sensor_msgs::Image>("/image/grass", 1);
	pub_edge = nh.advertise<sensor_msgs::Image>("/image/edge", 1);
	pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/grass_points", 1);
}
void makeimage::CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout<<"callbackrmgrourd"<<std::endl;
	firstcallback_rmg = false;
	pcl::fromROSMsg(*msg, *MI.rmground);

	MI.extract_pc_in_range(MI.rmground);

}
void makeimage::CallbackGrass(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout<<"callbacgrass"<<std::endl;
	firstcallback_grs = false;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	pcl::fromROSMsg(*msg, *MI.grass);

	MI.extract_pc_in_range(MI.grass);
	
	pub_frameid = msg->header.frame_id;
	pub_stamp = msg->header.stamp;

	MI.make_image();
	
	Publication();
}
void makeimage::Publication(void)
{
	MI.image_ros->header.frame_id = pub_frameid;
	MI.image_ros->header.frame_id = pub_frameid;
	MI.image_ros2->header.stamp = pub_stamp;
	MI.image_ros2->header.stamp = pub_stamp;
	MI.grass_pc2.header.frame_id = pub_frameid;
	MI.grass_pc2.header.stamp = pub_stamp;
	pub_image.publish(MI.image_ros);
	pub_edge.publish(MI.image_ros2);
	pub_pcl.publish(MI.grass_pc2);
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

