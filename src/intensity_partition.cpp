/* intensity_partition.cpp */

#include <iostream>
#include <vector>
#include <array>

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


class IntensityPartition
{
	private:
		bool pc_callback_flag = false;
		bool first_flag = false;

		int n_grass = 0, n_asphalt = 0;

		const int GRASS = 1;
		const int ASPHALT = 2;
		const static int R_RESOLUTION = 50;
		const static int THETA_RESOLUTION = 360;

		float RANGE_MAX = 20.0;
		float dR = RANGE_MAX / (float)R_RESOLUTION;
		float dTheta = 2*M_PI / (float)THETA_RESOLUTION;
		float intensity_max = 0.0;
		float avr_grass = 0.0, avr_asphalt = 0.0;
		float sum_grass = 0.0, sum_asphalt = 0.0;

		float s_max[R_RESOLUTION];
		float otsu_threshold_tmp[R_RESOLUTION];

		int polar_grid_pt_cnt[R_RESOLUTION][THETA_RESOLUTION];
		float polar_grid_avr_intensity[R_RESOLUTION][THETA_RESOLUTION];
		float PolarGrid_sumIntensity[R_RESOLUTION][THETA_RESOLUTION];

		ros::Subscriber pc_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher grass_pc_publisher;

		sensor_msgs::PointCloud2 pub_pc;

		CloudIPtr input_pc_ {new CloudI};
		CloudIPtr polar_pc_ {new CloudI};

		std::vector<float> ptz_list;

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
		void cartesian_pt_2_polar_grid(CloudIPtr);
		float calc_variance(std::vector<std::array<int, R_RESOLUTION> >, int, int, const int);
		void calc_otsu_binary(void);
		CloudIPtr otsu_pc_generator(void);
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "intensity_partition");

	IntensityPartition intensity_partition;
	intensity_partition.execution();

	return 0;
}


IntensityPartition::IntensityPartition(void)
{
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	//nh.getParam("RANGE_MAX", RANGE_MAX);

	pc_subscriber = n.subscribe("/velodyne_clear", 10, &IntensityPartition::pc_callback, this);

	grass_pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/grass_pc", 10);
}


void IntensityPartition::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *input_pc_);
	polar_pc_ = input_pc_;

	CloudIPtr otsu_binary_pc_ {new CloudI};
	
	cartesian_pt_2_polar_grid(input_pc_);
	calc_otsu_binary();
	otsu_binary_pc_ = otsu_pc_generator();
	
	pcl::toROSMsg(*otsu_binary_pc_, pub_pc);
	grass_pc_publisher.publish(pub_pc);
	
	ptz_list.clear();
}


void IntensityPartition::execution(void)
{	
	ros::spin();
}


void IntensityPartition::initialize(void)
{
	for(int i=0; i<R_RESOLUTION; i++){
		for(int j=0; j<THETA_RESOLUTION; j++){
			polar_grid_pt_cnt[i][j] = 0;
			PolarGrid_sumIntensity[i][j] = 0;
			polar_grid_avr_intensity[i][j] = 0;
		}
		s_max[i] = 0.0;
		otsu_threshold_tmp[i] = 0.0;
	}
}


void IntensityPartition::cartesian_pt_2_polar_grid(CloudIPtr cartesian_pc_)
{
	float r_tmp, theta_tmp;
	size_t i = 0;
	initialize();
	
	for(auto& pt : cartesian_pc_->points){
		r_tmp = sqrt(pt.x * pt.x + pt.y * pt.y);
		theta_tmp = atan2(pt.y,pt.x);
		if(theta_tmp < 0){
			theta_tmp = 2 * M_PI + theta_tmp;
		}
		
		bool r_flag = false, theta_flag = false;
		for(int r_g=0; r_g<R_RESOLUTION; r_g++){
			for(int theta_g=0; theta_g<THETA_RESOLUTION; theta_g++){
				if((r_g == (int)(r_tmp / dR)) && (theta_g == (int)(theta_tmp / dTheta))){
					polar_grid_pt_cnt[r_g][theta_g] += 1;
					PolarGrid_sumIntensity[r_g][theta_g] += pt.intensity;
					polar_grid_avr_intensity[r_g][theta_g] = PolarGrid_sumIntensity[r_g][theta_g] / (float)polar_grid_pt_cnt[r_g][theta_g];
					
					polar_pc_->points[i].z = r_tmp;
					ptz_list.push_back(pt.z);

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


float IntensityPartition::calc_variance(std::vector<std::array<int, R_RESOLUTION> > histogram_list, int RG, int threshold_tmp, int grass_or_asphalt)
{
	float variance = 0, sum = 0, distpow_sum = 0, mu = 0;
	int cnt = 0;
	
	switch(grass_or_asphalt){ 
		case 1: // grass
			for(int i=0; i<threshold_tmp; i++){
				sum += (float)(histogram_list.at(i)).at(RG);
				cnt++;
			}
			sum_grass = sum;
			n_grass = cnt;
			mu = sum / cnt;
			avr_grass = mu;
			for(int i=0; i<threshold_tmp; i++){
				float dist = (float)(histogram_list.at(i)).at(RG) - mu;
				distpow_sum += dist * dist;
			}
			variance = distpow_sum / cnt;
			break;

		case 2: //asphalt
			for(int i=threshold_tmp; i<(int)intensity_max; i++){
				sum += (float)(histogram_list.at(i)).at(RG);
				cnt++;
			}
			sum_asphalt = sum;
			n_asphalt = cnt;
			mu = sum / cnt;
			avr_asphalt = mu;
			for(int i=threshold_tmp; i<(int)intensity_max; i++){
				float dist = (float)(histogram_list.at(i)).at(RG) - mu;
				distpow_sum += dist * dist;
			}
			variance = distpow_sum / cnt;
			break;
	}

	return variance;
}


void IntensityPartition::calc_otsu_binary(void)
{
	// initialize
	const int histogram_size = (int)intensity_max;
	std::array<int, R_RESOLUTION> r_res_array;
	std::vector<std::array<int, R_RESOLUTION> > histogram;
	for(int i=0; i<R_RESOLUTION; i++){
		r_res_array[i] = 0;
	}
	for(int j=0; j<histogram_size; j++){
		histogram.push_back(r_res_array);
	}

	// calc histogram
	for(int r_g=0; r_g<R_RESOLUTION; r_g++){
		for(int theta_g=0; theta_g<THETA_RESOLUTION; theta_g++){
			int intensity_tmp = (int)polar_grid_avr_intensity[r_g][theta_g];
			histogram[r_g][intensity_tmp] += 1;
		}
	}

	// calc separation
	struct GAA var[R_RESOLUTION][histogram_size -1];
	struct GAA num[R_RESOLUTION][histogram_size -1];
	struct GAA avr[R_RESOLUTION][histogram_size -1];
	for(int r_g=0; r_g<R_RESOLUTION; r_g++){
		for(int i_threshold=1; i_threshold<histogram_size; i_threshold++){
			var[r_g][i_threshold-1].grass = calc_variance(histogram, r_g, i_threshold, GRASS);
			var[r_g][i_threshold-1].asphalt = calc_variance(histogram, r_g, i_threshold, ASPHALT);
			num[r_g][i_threshold-1].grass = (float)n_grass;
			num[r_g][i_threshold-1].asphalt = (float)n_asphalt;
			num[r_g][i_threshold-1].all = (float)n_grass + (float)n_asphalt;
			avr[r_g][i_threshold-1].grass = avr_grass;
			avr[r_g][i_threshold-1].asphalt = avr_asphalt;
			avr[r_g][i_threshold-1].all = (sum_grass + sum_asphalt) / (float)num[r_g][i_threshold-1].all;
		}
	}
	struct WB var_wb;
	for(int r_g=0; r_g<R_RESOLUTION; r_g++){
		for(int i_threshold=1; i_threshold<histogram_size; i_threshold++){
			float ng = (float)num[r_g][i_threshold-1].grass;
			float na = (float)num[r_g][i_threshold-1].asphalt;
			float n_all = (float)num[r_g][i_threshold-1].all;
			float avr_all = avr[r_g][i_threshold-1].all;
			var_wb.within = (ng * var[r_g][i_threshold-1].grass + na * var[r_g][i_threshold-1].asphalt) / n_all;
			float distpow_mg = (avr[r_g][i_threshold-1].grass - avr_all) * (avr[r_g][i_threshold-1].grass - avr_all);
			float distpow_ma = (avr[r_g][i_threshold-1].asphalt - avr_all) * (avr[r_g][i_threshold-1].asphalt - avr_all);
			var_wb.between = (ng * distpow_mg + na * distpow_ma) / n_all;
			float s_tmp = var_wb.between / var_wb.within;
			if(s_max[r_g] < s_tmp){
				s_max[r_g] = s_tmp;
				otsu_threshold_tmp[r_g] = (float)i_threshold;
			}
		}
	}

	histogram.clear();
}


CloudIPtr IntensityPartition::otsu_pc_generator(void)
{
	size_t iz = 0;
	for(auto& pt : polar_pc_->points){
		for(int r_g=0; r_g<R_RESOLUTION; r_g++){
			if(((float)r_g <= pt.z && pt.z < (float)r_g+1.0) && otsu_threshold_tmp[r_g] > pt.intensity){
				pt.intensity = -1.0;
			}
		}
		pt.z = ptz_list[iz];
		iz++;
	}
	
	pcl::PassThrough<PointI> pass;
	CloudIPtr filtered_pc_ {new CloudI};
	pass.setInputCloud(polar_pc_);
	pass.setFilterFieldName ("intensity");
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
		pt.intensity = filtered_pc_->points[i].intensity;
		i++;
	}
	
	return otsu_pc_;
}


