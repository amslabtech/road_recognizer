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

#include "road_recognizer/OtsuBinary.h"
#include "road_recognizer/Intensity.h"
#include "road_recognizer/Analysis.h"
#include "road_recognizer/Distribution.h"

#include <visualization_msgs/Marker.h>


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
		
		/* ---------- Tuning Params ---------- */
		const static int RANGE_RESOLUTION = 20;
		const static int THETA_RESOLUTION = 360;
		float OTSU_BINARY_INTENSITY_STD_DEVIATION_THRESHOLD = 1.0;
		float OTSU_BINARY_SEPARATION_THRESHOLD = 0.2;
		float OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD = 58.0;
		float RANGE_MAX = 20.0;
		bool PC_PUBLISH_FLAG = true; // true : publish grass points
		//bool PC_PUBLISH_FLAG = false; // false : publish only otsu_binary_msg
		/* ----------------------------------- */

		float dR = RANGE_MAX / (float)RANGE_RESOLUTION;
		float dTheta = 2*M_PI / (float)THETA_RESOLUTION;
		float intensity_max_all;
		float intensity_max[RANGE_RESOLUTION];
		float intensity_min[RANGE_RESOLUTION];
		float avr_grass = 0.0, avr_asphalt = 0.0;

		float s_max[RANGE_RESOLUTION];
		float s_max_var_b[RANGE_RESOLUTION];
		float otsu_threshold_tmp[RANGE_RESOLUTION];

		int pub_count = 0;
		int polar_grid_pt_cnt[RANGE_RESOLUTION][THETA_RESOLUTION];
		// float range_grid_sum_intensity[RANGE_RESOLUTION];
		float polar_grid_avr_intensity[RANGE_RESOLUTION][THETA_RESOLUTION];
		float polar_grid_sum_intensity[RANGE_RESOLUTION][THETA_RESOLUTION];
		float time_sum_otsu[RANGE_RESOLUTION];	
		float time_sum_otsu_diffpow[RANGE_RESOLUTION];	
		float time_mu_otsu[RANGE_RESOLUTION];	
		float range_mu_otsu;	
		float otsu_range_std_deviation;	

		ros::NodeHandle nh;
		ros::NodeHandle local_nh;
		ros::Subscriber pc_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher grass_pc_publisher;
		ros::Publisher ground_pc_publisher;
		ros::Publisher otsu_binary_publisher;

		ros::Publisher marker_pub;

		sensor_msgs::PointCloud2 pub_pc;
		sensor_msgs::PointCloud2 pub_pc_tail;

		CloudIPtr input_pc_ {new CloudI};
		CloudIPtr polar_pc_ {new CloudI};

		std::vector<float> ptz_list;
		std::vector<float> ptz_list_tail;

		road_recognizer::OtsuBinary otsu_binary_msg; 

		struct WB{
			float within;
			float between;
		};
		struct GA{
			float grass;
			float asphalt;
		};


		visualization_msgs::Marker line_strip;


	public:
		IntensityPartition(void);

		void visualization(void);

		void pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void execution(void);
		void initialize(void);
		void cartesian_pt_2_polar_grid(CloudIPtr);
		//float calc_variance(std::vector<std::array<int, RANGE_RESOLUTION> >, int, int, int);
		float calc_variance(const std::vector<std::vector<int> >&, int, int, int);
		void calc_otsu_binary(void);
		void calc_diff_from_avr(void);
		void emergency_judge(void);
		
		CloudIPtr otsu_pc_generator(void);
		CloudIPtr otsu_pc_generator_tail(void);
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "intensity_partition");

	
	IntensityPartition intensity_partition;
	intensity_partition.execution();

	return 0;
}


IntensityPartition::IntensityPartition(void)
:local_nh("~")
{
	line_strip.id = 1;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.scale.x = 0.01;
	line_strip.color.r = 255.0;
	line_strip.color.g = 255.0;
	line_strip.color.b = 255.0;

	otsu_binary_msg.range_division_num = RANGE_RESOLUTION;
	otsu_binary_msg.theta_division_num = THETA_RESOLUTION;
	otsu_binary_msg.range_max = RANGE_MAX;
	otsu_binary_msg.otsubinary_separation_threshold = OTSU_BINARY_SEPARATION_THRESHOLD;
	otsu_binary_msg.otsubinary_sum_of_diff_from_avr_threshold = OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD;
	otsu_binary_msg.intensity.resize(RANGE_RESOLUTION);
	otsu_binary_msg.analysis.resize(RANGE_RESOLUTION);

	pc_subscriber = nh.subscribe("/velodyne_clear", 10, &IntensityPartition::pc_callback, this);

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	otsu_binary_publisher = local_nh.advertise<road_recognizer::OtsuBinary>("otsu_binary_info", 10);
	grass_pc_publisher = nh.advertise<sensor_msgs::PointCloud2>("/grass_pc", 10);
	ground_pc_publisher = nh.advertise<sensor_msgs::PointCloud2>("/ground_pc", 10);
}


void IntensityPartition::visualization(void)
{
	for(float r_g = 0; r_g < RANGE_MAX; r_g += dR){
		for(float theta_g = 0; theta_g < 2 * M_PI; theta_g += dTheta){
			geometry_msgs::Point p;
			p.x = r_g * cos(theta_g);
			p.y = r_g * sin(theta_g);
			p.z = -0.01;
			line_strip.points.push_back(p);
		}
	}
	marker_pub.publish(line_strip);
}


void IntensityPartition::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	//visualization();

	pcl::fromROSMsg(*msg, *input_pc_);
	polar_pc_ = input_pc_;

	CloudIPtr otsu_binary_pc_ {new CloudI};
	CloudIPtr otsu_binary_pc_tail_ {new CloudI};
	
	cartesian_pt_2_polar_grid(input_pc_);
	calc_otsu_binary();


	if(PC_PUBLISH_FLAG){
		otsu_binary_pc_ = otsu_pc_generator();
		otsu_binary_pc_tail_ = otsu_pc_generator_tail();
		pcl::toROSMsg(*otsu_binary_pc_, pub_pc);
		pcl::toROSMsg(*otsu_binary_pc_tail_, pub_pc_tail);
		grass_pc_publisher.publish(pub_pc);
		ground_pc_publisher.publish(pub_pc_tail);
		pub_count++;
		std::cout << "pub_count : " << pub_count << std::endl;
	}

	otsu_binary_publisher.publish(otsu_binary_msg);
	
	first_flag = true;

	ptz_list.clear();
	ptz_list_tail.clear();
}


void IntensityPartition::execution(void)
{	
	ros::spin();
}


void IntensityPartition::initialize(void)
{
	otsu_binary_msg.emergency = false;
	for(int r_g = 0; r_g <RANGE_RESOLUTION; r_g++){
		for(int theta_g = 0; theta_g <THETA_RESOLUTION; theta_g++){
			polar_grid_pt_cnt[r_g][theta_g] = 0;
			polar_grid_sum_intensity[r_g][theta_g] = 0.0;
			polar_grid_avr_intensity[r_g][theta_g] = -1.0;
		}
		s_max[r_g] = 0.0;
		s_max_var_b[r_g] = 0.0;
		otsu_threshold_tmp[r_g] = 0.0;
		//range_grid_sum_intensity[r_g] = 0.0;
		time_sum_otsu[r_g] = 0.0;
		time_sum_otsu_diffpow[r_g] = 0.0;
		intensity_max[r_g] = 0.0;
		intensity_min[r_g] = 999.9;
	}
	intensity_max_all = 0.0;
}


void IntensityPartition::cartesian_pt_2_polar_grid(CloudIPtr cartesian_pc_)
{
	float r_tmp, theta_tmp, z_tmp;
	size_t i = 0;
	initialize();
	
	for(auto& pt : cartesian_pc_->points){
		r_tmp = sqrt(pt.x * pt.x + pt.y * pt.y);
		theta_tmp = atan2(pt.y,pt.x);
		if(theta_tmp < 0){
			theta_tmp = 2 * M_PI + theta_tmp;
		}
		
		// create polar grid informations
		bool r_flag = false, theta_flag = false;
		for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
			for(int theta_g = 0; theta_g < THETA_RESOLUTION; theta_g++){
				if((r_g == (int)(r_tmp / dR)) && (theta_g == (int)(theta_tmp / dTheta))){
					polar_grid_pt_cnt[r_g][theta_g] += 1;
					polar_grid_sum_intensity[r_g][theta_g] += pt.intensity;
					polar_grid_avr_intensity[r_g][theta_g] = polar_grid_sum_intensity[r_g][theta_g] / (float)polar_grid_pt_cnt[r_g][theta_g]; 

					if(PC_PUBLISH_FLAG){
						z_tmp = polar_pc_->points[i].z;
						ptz_list.push_back(z_tmp);
						ptz_list_tail.push_back(z_tmp);
					}
					polar_pc_->points[i].z = r_tmp;

					r_flag = true;
					theta_flag = true;
				}
				
				if(intensity_min[r_g] > polar_grid_avr_intensity[r_g][theta_g] || !first_flag){
					intensity_min[r_g] = polar_grid_avr_intensity[r_g][theta_g];
				}
				if(intensity_max[r_g] < polar_grid_avr_intensity[r_g][theta_g] || !first_flag){
					intensity_max[r_g] = polar_grid_avr_intensity[r_g][theta_g];
				}

				if(theta_flag) break;
			}
			
		}
		
		if(intensity_max_all < pt.intensity){
			intensity_max_all = pt.intensity;
		}

		i++;
	}

	for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
		if(r_g == 0){
			otsu_binary_msg.intensity[r_g].min = 0.0;
			otsu_binary_msg.intensity[r_g].max = 0.0;
		}else{
			otsu_binary_msg.intensity[r_g].min = intensity_min[r_g];
			otsu_binary_msg.intensity[r_g].max = intensity_max[r_g];
		}
		/* for(int theta_g = 0; theta_g < THETA_RESOLUTION; theta_g++){ */
		/* 	range_grid_sum_intensity[r_g] += polar_grid_avr_intensity[r_g][theta_g]; */
		/* } */
	}
}


//float IntensityPartition::calc_variance(std::vector<std::array<int, RANGE_RESOLUTION> > histogram_list, int r_g, int threshold_tmp, int grass_or_asphalt)
float IntensityPartition::calc_variance(const std::vector<std::vector<int> >& histogram_list, int r_g, int threshold_tmp, int grass_or_asphalt)
{
	float variance = 0, sum = 0, multi_sum = 0, diffpow_sum = 0, mu = 0;
	int cnt = 0;

	switch(grass_or_asphalt){ 
		case 1: // grass
			for(int i = 0; i < threshold_tmp; i++){
				multi_sum += (float)histogram_list[i][r_g] * i;
				sum += (float)histogram_list[i][r_g];
				cnt++;
			}
			n_grass = cnt;
			mu = multi_sum / sum;
			avr_grass = mu;
			for(int i = 0; i < threshold_tmp; i++){
				float diff = (float)histogram_list[i][r_g] - mu;
				diffpow_sum += diff * diff;
			}
			variance = diffpow_sum / cnt;
			break;

		case 2: //asphalt
			for(int i = threshold_tmp; i < (int)intensity_max[r_g]; i++){
				multi_sum += (float)histogram_list[i][r_g] * i;
				sum += (float)histogram_list[i][r_g];
				cnt++;
			}
			n_asphalt = cnt;
			mu = multi_sum / sum;
			avr_asphalt = mu;
			for(int i = threshold_tmp; i < (int)intensity_max[r_g]; i++){
				float diff = (float)histogram_list[i][r_g] - mu;
				diffpow_sum += diff * diff;
			}
			variance = diffpow_sum / cnt;
			break;
	}
	
	return variance;
}


void IntensityPartition::calc_diff_from_avr(void)
{
	float range_sum_otsu = 0.0;
	for(int r_g = 1; r_g < RANGE_RESOLUTION; r_g++){
		range_sum_otsu += otsu_binary_msg.intensity[r_g].threshold;
	}
	float range_mu_otsu = range_sum_otsu / (RANGE_RESOLUTION - 1);

	for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
		float diff = otsu_binary_msg.intensity[r_g].threshold - range_mu_otsu;
		otsu_binary_msg.analysis[r_g].otsubinary_diff_from_thresholds_avr = sqrt(diff * diff);
	}
}


void IntensityPartition::emergency_judge(void)
{
	float sum_diff_from_avr = 0.0;
	for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
		sum_diff_from_avr += otsu_binary_msg.analysis[r_g].otsubinary_diff_from_thresholds_avr;
	}
	if(sum_diff_from_avr > OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD){
		otsu_binary_msg.emergency = true;
	}
}


void IntensityPartition::calc_otsu_binary(void)
{
	//const int histogram_size = (int)intensity_max_all;
	const int histogram_size = 256;
	float sum_all[RANGE_RESOLUTION], avr_all[RANGE_RESOLUTION], var_all[RANGE_RESOLUTION], diff_all[histogram_size], 
		  sum_tmp1[RANGE_RESOLUTION], sum_tmp2[RANGE_RESOLUTION], 
		  multi_sum_all[RANGE_RESOLUTION];
	/* std::array<int, RANGE_RESOLUTION> r_res_array; */
	std::vector<int> r_res_array;
	//std::vector<std::array<int, RANGE_RESOLUTION> > histogram;
	std::vector<std::vector<int> > histogram;

	// initialize
	for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
		r_res_array.push_back((int)0);
		multi_sum_all[r_g] = 0;
		sum_all[r_g] = 0;
		sum_tmp1[r_g] = 0;
		sum_tmp2[r_g] = 0;
	}
	for(int i = 0; i < histogram_size; i++){
		histogram.push_back(r_res_array);
	}
	
	// make histogram
	for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
		for(int theta_g = 0; theta_g < THETA_RESOLUTION; theta_g++){
			int intensity_tmp = (int)polar_grid_avr_intensity[r_g][theta_g];
			if(intensity_tmp > 0){
				histogram.at(intensity_tmp).at(r_g) += 1;
			}
		}
	}
	
	// for debag
	for(int r_g=0; r_g < RANGE_RESOLUTION; r_g++){
		otsu_binary_msg.analysis[r_g].distribution.resize(histogram_size);
		for(int idx_intensity = 0; idx_intensity < histogram_size; idx_intensity++){
			otsu_binary_msg.analysis[r_g].distribution[idx_intensity].intensity = (float)histogram[idx_intensity][r_g];
		}
	}

	
	// calc separation
	struct GA var[RANGE_RESOLUTION][histogram_size-1];
	struct GA num[RANGE_RESOLUTION][histogram_size-1];
	struct GA avr[RANGE_RESOLUTION][histogram_size-1];
	for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
		for(int i_threshold = 1; i_threshold < (int)intensity_max[r_g]; i_threshold++){
			var[r_g][i_threshold-1].grass = calc_variance(histogram, r_g, i_threshold, GRASS);
			var[r_g][i_threshold-1].asphalt = calc_variance(histogram, r_g, i_threshold, ASPHALT);
			num[r_g][i_threshold-1].grass = (float)n_grass;
			num[r_g][i_threshold-1].asphalt = (float)n_asphalt;
			avr[r_g][i_threshold-1].grass = avr_grass;
			avr[r_g][i_threshold-1].asphalt = avr_asphalt;
		}
	}
	
	// calc whole variance
	for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
		for(int idx_intensity = 0; idx_intensity < (int)intensity_max[r_g]; idx_intensity++){
			multi_sum_all[r_g] += histogram[idx_intensity][r_g] * idx_intensity;
			sum_all[r_g] += histogram[idx_intensity][r_g];
		}
		avr_all[r_g] = (float)multi_sum_all[r_g] / (float)sum_all[r_g];
		for(int i_threshold = 0; i_threshold < (int)intensity_max[r_g]; i_threshold++){
			diff_all[i_threshold] = (float)histogram[i_threshold][r_g] - avr_all[r_g];
			sum_tmp1[r_g] += diff_all[i_threshold] * diff_all[i_threshold];
		}
		var_all[r_g] = sum_tmp1[r_g] / (float)(sum_all[r_g]);
		/* std::cout << "var_all[" << r_g << "] : " << var_all[r_g] << std::endl; */
		otsu_binary_msg.analysis[r_g].intensity_std_deviation = sqrt(var_all[r_g]);
		/* std::cout << "intensity_std_deviation[" << r_g << "] : " << otsu_binary_msg.analysis[r_g].intensity_std_deviation << std::endl; */
	}

	
	struct WB var_wb;

	/* std::cout << "==========" << std::endl; */
	for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
		for(int i_threshold = 1; i_threshold < (int)intensity_max[r_g]; i_threshold++){
			float ng = (float)num[r_g][i_threshold-1].grass;
			float na = (float)num[r_g][i_threshold-1].asphalt;
			var_wb.within = (ng * var[r_g][i_threshold-1].grass + na * var[r_g][i_threshold-1].asphalt) / (ng + na);
			float diff_mg = avr[r_g][i_threshold-1].grass - avr_all[r_g];
			float diff_ma = avr[r_g][i_threshold-1].asphalt - avr_all[r_g];
			var_wb.between = (ng * diff_mg * diff_mg + na * diff_ma * diff_ma) / (ng + na);
			//var_wb.between = ng * na * (avr[r_g][i_threshold-1].grass - avr[r_g][i_threshold-1].asphalt) * (avr[r_g][i_threshold-1].grass - avr[r_g][i_threshold-1].asphalt) / (ng + na) * (ng + na);
			float s_tmp = var_wb.between / var_wb.within;
			
			if(s_max[r_g] < s_tmp){
				s_max[r_g] = s_tmp;
				s_max_var_b[r_g] = var[r_g][i_threshold-1].asphalt;
				otsu_threshold_tmp[r_g] = (float)i_threshold;
			}
		}
		
		if(r_g == 0){
			otsu_binary_msg.intensity[r_g].threshold = 0.0;
			otsu_binary_msg.analysis[r_g].separation = 0.0;
		}else{
			otsu_binary_msg.intensity[r_g].threshold = otsu_threshold_tmp[r_g];
			otsu_binary_msg.analysis[r_g].separation = s_max[r_g];
		}
		/* std::cout << "----------" << std::endl; */
		/* std::cout << "r = " << r_g << ", threshold = " << otsu_binary_msg.intensity[r_g].threshold << std::endl; */
		/* std::cout << "r = " << r_g << ", S = " << otsu_binary_msg.analysis[r_g].separation << std::endl; */
		//std::cout << "r = " << r_g << ", intensity_max = " << intensity_max[r_g] << std::endl;
		
		for(int i = 0; i < intensity_max[r_g]; i++){
			if(otsu_binary_msg.analysis[r_g].distribution[i].intensity > 0){
				//std::cout << "r = " << r_g << ", histogram[" << i << "] = " << otsu_binary_msg.analysis[r_g].distribution[i].intensity << std::endl;
			}
		}
		//std::cout << "----------" << std::endl;
	}

	//std::cout << "==========" << std::endl;


	// calc trust range
	

	// for debag

	// calc skewness
	for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
		for(int idx_intensity = 1; idx_intensity < (int)intensity_max[r_g]; idx_intensity++){
			float z_score = diff_all[idx_intensity-1] / var_all[r_g];
			sum_tmp2[r_g] += z_score * z_score * z_score;
		}
		/* std::cout << "sum_tmp2[" << r_g << "] : " << sum_tmp2[r_g] << std::endl; */
		/* std::cout << "n_all[" << r_g << "] : " << n_all[r_g] << std::endl; */
		/* if(r_g == 0){ */
		/* 	otsu_binary_msg.analysis[r_g].skewness = 0.0; */
		/* }else{ */
		otsu_binary_msg.analysis[r_g].skewness = sum_tmp2[r_g] / (float)sum_all[r_g];
		/* std::cout << "otsu_binary_msg.analysis[" << r_g << "].skewness : " << otsu_binary_msg.analysis[r_g].skewness << std::endl; */
		// }
	}

	// calc threshold histogram in range
	calc_diff_from_avr();

	
	// judge emergency
	emergency_judge();

	r_res_array.clear();
	histogram.clear();
}



CloudIPtr IntensityPartition::otsu_pc_generator(void)
{
	size_t iz = 0;
	CloudIPtr p_ {new CloudI};
	p_ = polar_pc_;
	for(auto& pt : p_->points){
		int RG = sqrt(pt.x * pt.x + pt.y * pt.y) / dR;
		int thetag = atan2(pt.y,pt.x);
		if(thetag < 0){
			thetag = 2 * M_PI + thetag;
		}
		int THETAG = thetag / dTheta;

		for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
			//for(int theta_g = 0; theta_g < THETA_RESOLUTION; theta_g++){
				//if(((float)r_g <= pt.z && pt.z < (float)r_g+dR) && ((float)theta_g <= thetag && thetag < (float)theta_g + dTheta)
				if(((float)r_g <= pt.z && pt.z < (float)r_g+dR)
					/* && ((otsu_threshold_tmp[r_g] - 1.0 > polar_grid_avr_intensity[RG][THETAG]) */
					&& ((otsu_threshold_tmp[r_g] > pt.intensity)
						//|| (otsu_binary_msg.analysis[r_g].intensity_std_deviation <  OTSU_BINARY_INTENSITY_STD_DEVIATION_THRESHOLD)
						|| (otsu_binary_msg.analysis[r_g].separation < OTSU_BINARY_SEPARATION_THRESHOLD)
						)){
					pt.intensity = -1.0;
				}
			//}
		}
		pt.z = ptz_list.at(iz);
		iz++;
	}
	
	if(otsu_binary_msg.emergency){
		//intensity_max_all = 0.0;
	}

	pcl::PassThrough<PointI> pass;
	CloudIPtr filtered_pc_ {new CloudI};
	pass.setInputCloud(polar_pc_);
	pass.setFilterFieldName ("intensity");
	pass.setFilterLimits(0.0, intensity_max_all);
	//pass.setFilterLimitsNegative (true);
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


CloudIPtr IntensityPartition::otsu_pc_generator_tail(void)
{
	size_t iz = 0;
	CloudIPtr p_ {new CloudI};
	p_ = polar_pc_;
	for(auto& pt : p_->points){
		int RG = sqrt(pt.x * pt.x + pt.y * pt.y) / dR;
		int thetag = atan2(pt.y,pt.x);
		if(thetag < 0){
			thetag = 2 * M_PI + thetag;
		}
		int THETAG = thetag / dTheta;

		for(int r_g = 0; r_g < RANGE_RESOLUTION; r_g++){
			//for(int theta_g = 0; theta_g < THETA_RESOLUTION; theta_g++){
				//if(((float)r_g <= pt.z && pt.z < (float)r_g+dR) && ((float)theta_g <= thetag && thetag < (float)theta_g + dTheta)
				if(((float)r_g <= pt.z && pt.z < (float)r_g+dR)
					/* && ((otsu_threshold_tmp[r_g] - 1.0 > polar_grid_avr_intensity[RG][THETAG]) */
					&& ((otsu_threshold_tmp[r_g] > pt.intensity)
						//|| (otsu_binary_msg.analysis[r_g].intensity_std_deviation <  OTSU_BINARY_INTENSITY_STD_DEVIATION_THRESHOLD)
						|| (otsu_binary_msg.analysis[r_g].separation < OTSU_BINARY_SEPARATION_THRESHOLD)
						)){
					pt.intensity = -1.0;
				}
			//}
		}
		pt.z = ptz_list.at(iz);
		/* if(pt.z > 0.5){ */
		/* 	pt.intensity = -1.0; */
		/* } */
		iz++;
	}
	
	if(otsu_binary_msg.emergency){
		//intensity_max_all = 0.0;
	}

	pcl::PassThrough<PointI> pass;
	CloudIPtr filtered_pc_ {new CloudI};
	pass.setInputCloud(polar_pc_);
	pass.setFilterFieldName ("intensity");
	pass.setFilterLimits(0.0, intensity_max_all);
	pass.setFilterLimitsNegative (true);
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




