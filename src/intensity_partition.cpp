/* intensity_partition.cpp */

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <boost/thread.hpp>


typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

typedef pcl::PointXYZHSV PointIRT;
typedef pcl::PointCloud<PointIRT> CloudIRT;
typedef pcl::PointCloud<PointIRT>::Ptr CloudIRTPtr;


class IntensityPartition
{
	private:
		bool pc_callback_flag = false;
		bool first_flag = false;

		int n_grass = 0, n_asphalt = 0;

		const int GRASS = 1;
		const int ASPHALT = 2;
		const static int r_resolution = 50;
		const static int theta_resolution = 360;

		float Hz = 100.0;
		float range_max = 20.0;
		float dr = range_max / (float)r_resolution;
		float dtheta = 2*M_PI / (float)theta_resolution;
		float intensity_max = 0.0;
		float avr_grass = 0.0, avr_asphalt = 0.0;
		float sum_grass = 0.0, sum_asphalt = 0.0;

		float S_max[r_resolution];
		float OtsuThreshold[r_resolution];

		int PolarGrid_pt_count[r_resolution][theta_resolution];
		float PolarGrid_avrIntensity[r_resolution][theta_resolution];
		float PolarGrid_sumIntensity[r_resolution][theta_resolution];

		ros::Subscriber pc_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher grass_pc_publisher;

		sensor_msgs::PointCloud2 pub_pc;

		CloudIPtr input_pc_ {new CloudI};
		CloudIRTPtr polar_pc_ {new CloudIRT};

		struct WB{
			float within;
			float between;
		};
		struct GAA{
			float grass;
			float asphalt;
			float all;
		};

	public:
		IntensityPartition(void);

		void pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void execution(void);
		void initialize(void);
		void cartesian2PolarGrid(CloudIPtr);
		float calc_variance(std::vector<std::array<int, r_resolution> >, int, int, const int);
		void calc_OtsuBinary(void);
		CloudIPtr OtsuPC_generator(void);
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "intensity_partition");

	return 0;
}


IntensityPartition::IntensityPartition(void)
{
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	//nh.getParam("range_max", range_max);

	pc_subscriber = n.subscribe("/velodyne_clear", 10, &IntensityPartition::pc_callback, this);
    //odom_subscriber = n.subscribe("/estimated_pose/pose", 10, &IntensityPartition::odom_callback, this);

	grass_pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/grass_pc", 10);
}


void IntensityPartition::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *input_pc_);
	pcl::copyPointCloud(*input_pc_, *polar_pc_);
	pc_callback_flag = true;
}


void IntensityPartition::execution(void)
{	
	ros::Rate r(Hz);
	while(ros::ok()){
		if(pc_callback_flag){
			CloudIPtr Otsu_PC_ {new CloudI};
			cartesian2PolarGrid(input_pc_);
			calc_OtsuBinary();
			Otsu_PC_ = OtsuPC_generator();
			pcl::toROSMsg(*Otsu_PC_, pub_pc);
			grass_pc_publisher.publish(pub_pc);
		}
		r.sleep();
		ros::spinOnce();
	}
	
	//ros::spin();
}


void IntensityPartition::initialize(void)
{
	for(int i=0; i<r_resolution; i++){
		for(int j=0; j<theta_resolution; j++){
			PolarGrid_pt_count[i][j] = 0;
			PolarGrid_sumIntensity[i][j] = 0;
			PolarGrid_avrIntensity[i][j] = 0;
		}
		S_max[i] = 0.0;
		OtsuThreshold[i] = 0.0;
	}
}


void IntensityPartition::cartesian2PolarGrid(CloudIPtr cartesian_pc_)
{
	float r_tmp, theta_tmp;
	size_t i = 0;
	initialize();
	
	for(auto& pt : cartesian_pc_->points){
		r_tmp = sqrt(pt.x * pt.x + pt.y * pt.y);
		theta_tmp = atan2(pt.y,pt.x);
		if(theta_tmp < 0){
			theta_tmp += 2 * M_PI;
		}
		
		bool r_flag = false, theta_flag = false;
		for(int r_g=0; r_g<r_resolution; r_g++){
			for(int theta_g=0; theta_g<theta_resolution; theta_g++){
				if((r_g == (int)r_tmp / dr) && (theta_g == (int)theta_tmp / dtheta)){
					PolarGrid_pt_count[r_g][theta_g] += 1;
					PolarGrid_sumIntensity[r_g][theta_g] += pt.intensity;
					PolarGrid_avrIntensity[r_g][theta_g] = PolarGrid_sumIntensity[r_g][theta_g] / (float)PolarGrid_pt_count[r_g][theta_g];

					/* if need below */
					/* polar_pc_->points[i].x = pt.x; */
					/* polar_pc_->points[i].y = pt.y; */
					/* polar_pc_->points[i].z = pt.z; */
					polar_pc_->points[i].h = pt.intensity;
					polar_pc_->points[i].s = r_tmp;
					/* polar_pc_->points[i].v = theta_tmp; */
					
					r_flag = true;
					theta_flag = true;
				}
				if(r_flag) break;
			}
			if(theta_flag) break;
		}
		
		if(intensity_max < pt.intensity){
			intensity_max = pt.intensity;
		}
		i++;
	}
}


float IntensityPartition::calc_variance(std::vector<std::array<int, r_resolution> > Histogram, int Rg, int Threshold, int GA)
{
	float variance = 0, sum = 0, distpow_sum = 0, mu = 0;
	int cnt = 0;
	
	switch(GA){ 
		case 1: // grass
			for(int i=0; i<Threshold; i++){
				sum += (float)(Histogram.at(i)).at(Rg);
				cnt++;
			}
			sum_grass = sum;
			n_grass = cnt;
			mu = sum / cnt;
			avr_grass = mu;
			for(int i=0; i<Threshold; i++){
				float dist = (float)(Histogram.at(i)).at(Rg) - mu;
				distpow_sum += dist * dist;
			}
			variance = distpow_sum / cnt;
			break;

		case 2: //asphalt
			for(int i=Threshold; i<(int)intensity_max; i++){
				sum += (float)(Histogram.at(i)).at(Rg);
				cnt++;
			}
			sum_asphalt = sum;
			n_asphalt = cnt;
			mu = sum / cnt;
			avr_asphalt = mu;
			for(int i=Threshold; i<(int)intensity_max; i++){
				float dist = (float)(Histogram.at(i)).at(Rg) - mu;
				distpow_sum += dist * dist;
			}
			variance = distpow_sum / cnt;
			break;
	}

	return variance;
}


void IntensityPartition::calc_OtsuBinary(void)
{
	// initialize
	const int histogram_size = (int)intensity_max;
	std::array<int, r_resolution> r_res_array;
	std::vector<std::array<int, r_resolution> > histogram;
	for(int i=0; i<r_resolution; i++){
		r_res_array[i] = 0;
	}
	for(int j=0; j<histogram_size; j++){
			histogram.push_back(r_res_array);
	}

	// calc histogram
	for(int r_g=0; r_g<r_resolution; r_g++){
		for(int theta_g=0; theta_g<theta_resolution; theta_g++){
			int intensity_tmp = (int)PolarGrid_avrIntensity[r_g][theta_g];
			(histogram.at(intensity_tmp)).at(r_g) += 1;
		}
	}

	// calc separation
	struct GAA var[r_resolution][histogram_size-1];
	struct GAA num[r_resolution][histogram_size-1];
	struct GAA avr[r_resolution][histogram_size-1];
	for(int r_g=0; r_g<r_resolution; r_g++){
		for(int Ti=1; Ti<histogram_size; Ti++){
			var[r_g][Ti-1].grass = calc_variance(histogram, r_g, Ti, GRASS);
			var[r_g][Ti-1].asphalt = calc_variance(histogram, r_g, Ti, ASPHALT);
			num[r_g][Ti-1].grass = (float)n_grass;
			num[r_g][Ti-1].asphalt = (float)n_asphalt;
			num[r_g][Ti-1].all = (float)n_grass + (float)n_asphalt;
			avr[r_g][Ti-1].grass = avr_grass;
			avr[r_g][Ti-1].asphalt = avr_asphalt;
			avr[r_g][Ti-1].all = (sum_grass + sum_asphalt) / (float)num[r_g][Ti-1].all;
		}
	}
	struct WB varWB;
	for(int r_g=0; r_g<r_resolution; r_g++){
		for(int Ti=1; Ti<histogram_size; Ti++){
			float ng = (float)num[r_g][Ti-1].grass;
			float na = (float)num[r_g][Ti-1].asphalt;
			float nAll = (float)num[r_g][Ti-1].all;
			float avrAll = avr[r_g][Ti-1].all;
			varWB.within = (ng * var[r_g][Ti-1].grass + na * var[r_g][Ti-1].asphalt) / nAll;
			float distpow_mg = (avr[r_g][Ti-1].grass - avrAll) * (avr[r_g][Ti-1].grass - avrAll);
			float distpow_ma = (avr[r_g][Ti-1].asphalt - avrAll) * (avr[r_g][Ti-1].asphalt - avrAll);
			varWB.between = (ng * distpow_mg + na * distpow_ma) / nAll;
			float S_tmp = varWB.between / varWB.within;
			if(S_max[r_g] < S_tmp){
				S_max[r_g] = S_tmp;
				OtsuThreshold[r_g] = (float)Ti;
			}
		}
	}

	//r_res_array.clear();
	//histogram.clear();
}


CloudIPtr IntensityPartition::OtsuPC_generator(void)
{
	for(auto& pt : polar_pc_->points){
		for(int r_g=0; r_g<r_resolution; r_g++){
			if(((float)r_g <= pt.s && pt.s < (float)r_g+1.0) && OtsuThreshold[r_g] < pt.h){
				pt.h = -1.0;
			}
		}
	}

	pcl::PassThrough<PointIRT> pass;
	CloudIRTPtr filtered_pc_ {new CloudIRT};
	pass.setInputCloud(polar_pc_);
	pass.setFilterFieldName ("h");
	pass.setFilterLimits(0.0, intensity_max);
	pass.filter(*filtered_pc_);

	size_t otsu_size = filtered_pc_->points.size();

	CloudIPtr otsu_pc_ {new CloudI};
	otsu_pc_->points.resize(otsu_size);
	otsu_pc_->header = input_pc_->header;
	size_t i = 0;
	for(auto& pt : otsu_pc_->points){
		pt.x = filtered_pc_->points[i].x;
		pt.y = filtered_pc_->points[i].y;
		pt.z = filtered_pc_->points[i].z;
		pt.intensity = filtered_pc_->points[i].h;
		i++;
	}

	return otsu_pc_;
}


