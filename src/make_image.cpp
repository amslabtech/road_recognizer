
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
using std::vector;


class MakeImage{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_rmground;
		ros::Subscriber sub_grass;
		/*publish*/
		ros::Publisher pub_image;
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZI>::Ptr rmground {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZI>::Ptr grass {new pcl::PointCloud<pcl::PointXYZI>};
		/*publish infomations*/
		std::string pub_frameid;
		ros::Time pub_stamp;
		/*const values*/
		const double w = 30.0;	//x[m]
		const double h = 30.0;	//y[m]
		const double resolution = 0.05;	//[m]
		const double resolution_inv = 1/resolution;	
		const int image_w = int(w*resolution_inv);
		const int image_h = int(h*resolution_inv);
		/* flags */
		bool firstcallback_rmg = true;
		bool firstcallback_grs = true;
		/*image*/
		sensor_msgs::ImagePtr image_ros;

	public:
		MakeImage();
		// void Initialization(void);
		void CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackGrass(const sensor_msgs::PointCloud2ConstPtr& msg);
		void ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc);
		void make_image(void);
		void AddPointData(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, cv::Mat& image,int& max);
		void AddPointData_obs(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, cv::Mat& image);
		// void AddPointData2(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, vector<vector<int>>& image,int& max);
		void normalize(cv::Mat& image,int max);
		void amp(cv::Mat& image,double gain);
		int MeterpointToPixel_x(double y);
		int MeterpointToPixel_y(double x);
		void HoughLineP(cv::Mat& image,cv::Mat& image_c);
		void Publication(void);
		void process();
};

MakeImage::MakeImage()
{
	sub_rmground = nh.subscribe("/rm_ground", 1, &MakeImage::CallbackRmGround, this);
	sub_grass = nh.subscribe("/grass", 1, &MakeImage::CallbackGrass, this);
	pub_image = nh.advertise<sensor_msgs::Image>("/image", 1);
}
void MakeImage::CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout<<"callbackrmground"<<std::endl;
	firstcallback_rmg = false;
	pcl::fromROSMsg(*msg, *rmground);

	ExtractPCInRange(rmground);

}
void MakeImage::CallbackGrass(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout<<"callbacgrass"<<std::endl;
	firstcallback_grs = false;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	pcl::fromROSMsg(*msg, *grass);

	ExtractPCInRange(grass);
	
	pub_frameid = msg->header.frame_id;
	pub_stamp = msg->header.stamp;

	make_image();
	
	Publication();
}
void MakeImage::ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc)
{
	// std::cout<<"extractpcinrange"<<std::endl;
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-w*0.5, w*0.5);
	pass.filter(*pc);
	pass.setInputCloud(pc);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-h*0.5, h*0.5);
	pass.filter(*pc);
}
void MakeImage::make_image(void)
{
	// std::cout<<"make_image"<<std::endl;
	cv::Mat image(cv::Size(image_w,image_h), CV_8UC1, cv::Scalar(0));
	cv::Mat image_c(cv::Size(image_w,image_h), CV_8UC3, cv::Scalar(0));
	vector<vector<int>> image_vec(image_h, vector<int>(image_w, 0));
	
	int max = 0;
	AddPointData(grass,image,max);

	////////////////////////////////////////////////////////////
	
	// IplConvKernel* karnel = cvCreateStructuringElementEx(7,7,4,4,2); //	size_x,y,offset_x,y,shape

	// cv::medianBlur(image, image, 7);
	// cv::erode(image, image, cv::Mat(), cv::Point(-1,-1), 1); 
	// AddPointData_obs(rmground,image);
	// cv::GaussianBlur(image, image, cv::Size(7,7), 10, 10);// src_img, out_img，karnel_size，標準偏差x, y
	// cv::erode(image, image, cv::Mat(), cv::Point(-1,-1), 1); 
	// cv::medianBlur(image, image, 7);
	// cv::erode(image, image, cv::Mat(), cv::Point(-1,-1), 1); 
	// cv::dilate(image, image, cv::Mat(), cv::Point(-1,-1), 1); 

	
	cv::Mat element = (cv::Mat_<uchar>(7,7)<< 0, 0, 1, 1, 1, 0, 0,
											  0, 1, 1, 1, 1, 1, 0,
											  1, 1, 1, 1, 1, 1, 1,
											  1, 1, 1, 1, 1, 1, 1,
											  1, 1, 1, 1, 1, 1, 1,
											  0, 1, 1, 1, 1, 1, 0,
											  0, 0, 1, 1, 1, 0, 0);

	cv::dilate(image, image, element, cv::Point(-1,-1), 2); 
	cv::erode(image, image, element, cv::Point(-1,-1), 4); 
	cv::dilate(image, image, element, cv::Point(-1,-1), 2); 
	// cv::GaussianBlur(image, image, cv::Size(7,7), 10, 10);
	cv::medianBlur(image, image, 7);







	// normalize(image,max);
	// cv::threshold(image, image, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
	/////////////////////////////////////////////////////////////
	// cv::Canny(image, image, 50, 200, 3); 
	cvtColor(image, image_c, CV_GRAY2RGB);

	HoughLineP(image, image_c);
	cv::circle(image_c, cv::Point(image_w*0.5,image_h*0.5), 3, cv::Scalar(100,255,0), -1, CV_AA);
	image_ros = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_c).toImageMsg();
}

void MakeImage::AddPointData(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, cv::Mat& image, int& max)
{
	// std::cout<<"AddPointData"<<std::endl;
	if(size_t lim = pc->points.size()){
		for(size_t i=0;i<lim;i++){
			int px_x = MeterpointToPixel_x(pc->points[i].y);
			int px_y = MeterpointToPixel_y(pc->points[i].x);
			double r = sqrt(pc->points[i].x*pc->points[i].x+pc->points[i].y*pc->points[i].y);
			int ind = px_y*image_w + px_x;
			// image.data[ind] += 1;
			// image.data[ind] += int(r*r*0.5+4);
			image.data[ind] = 255;
			if(image.data[ind]>max){
				max = image.data[ind];
				// if(max>200)std::cout<<"max:"<<max<<", x:"<<px_x<< ", y:"<<px_y <<std::endl;
			}
		}
	}
}
void MakeImage::AddPointData_obs(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, cv::Mat& image)
{
	// std::cout<<"AddPointData"<<std::endl;
	if(size_t lim = pc->points.size()){
		for(size_t i=0;i<lim;i++){
			int px_x = MeterpointToPixel_x(pc->points[i].y);
			int px_y = MeterpointToPixel_y(pc->points[i].x);
			int ind = px_y*image_w + px_x;
			image.data[ind] = 255;
		}
	}
}
// void MakeImage::AddPointData2(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, vector<vector<int>>& image_2d,int& max)
// {
// 	std::cout<<"AddPointData2"<<std::endl;
// 	if(size_t lim = pc->points.size()){
// 		for(size_t i=0;i<lim;i++){
// 			int px_x = MeterpointToPixel_x(pc->points[i].y);
// 			int px_y = MeterpointToPixel_y(pc->points[i].x);
// 			image_2d[px_y][px_x] += 1;
// 			std::cout << image_2d[px_y][px_x]<<std::endl;
// 		}
// 	}
// }
void MakeImage::normalize(cv::Mat& image,int max)
{
	// std::cout<<"normalize"<<std::endl;
	size_t lim = image_w*image_h;
	double normalize_n = 255/max;
	for(size_t i=0;i<lim;i++){
		image.data[i] *= normalize_n;
	}
}
void MakeImage::amp(cv::Mat& image,double gain)
{
	// std::cout<<"amp"<<std::endl;
	size_t lim = image_w*image_h;
	
	for(size_t i=0;i<lim;i++){
		if(image.data[i]*gain<256)image.data[i] *= gain;
		else image.data[i] = 255;
	}
}
void MakeImage::HoughLineP(cv::Mat& image,cv::Mat& image_c)
{
	// std::cout<<"HoughLineP"<<std::endl;
	cv::Canny(image, image, 50, 200, 3); 
	std::vector<cv::Vec4i> lines;
	// 入力画像，出力，距離分解能，角度分解能，閾値，線分の最小長さ，
	// 2点が同一線分上にあると見なす場合に許容される最大距離
	// cv::HoughLinesP(image, lines, 1, CV_PI/180*0.5, 80, 120, 15);
	cv::HoughLinesP(image, lines, 1, CV_PI/180*0.5, 70, 100, 30);
	std::vector<cv::Vec4i>::iterator it = lines.begin();
	std::cout << "number of lines: " << lines.size() << std::endl;
	for(; it!=lines.end(); ++it) {
		cv::Vec4i l = *it;
	    cv::line(image_c, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 2, CV_AA);
	}
}

int MakeImage::MeterpointToPixel_x(double y)
{
	int x_ = -y*resolution_inv + image_w*0.5 - 0.5;
	return x_;
}

int MakeImage::MeterpointToPixel_y(double x)
{
	int y_ = -x*resolution_inv + image_h*0.5 - 0.5;
	return y_;
}
void MakeImage::Publication(void)
{
	image_ros->header.frame_id = pub_frameid;
	image_ros->header.stamp = pub_stamp;
	pub_image.publish(image_ros);
}

void MakeImage::process()
{
	ros::spin();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "make_image");
	// std::cout << "= make_image =" << std::endl;
	
	MakeImage makeimage;
	makeimage.process();
	return 0;
}

