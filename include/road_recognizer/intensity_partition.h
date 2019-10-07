#ifndef __INTENSITY_PARTITION_H
#define __INTENSITY_PARTITION_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "road_recognizer/OtsuBinary.h"
#include "road_recognizer/Intensity.h"
#include "road_recognizer/Analysis.h"
#include "road_recognizer/Distribution.h"


class IntensityPartition
{
public:
	typedef pcl::PointXYZI PointI;
	typedef pcl::PointCloud<PointI> CloudI;
	typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;
	typedef pcl::PointXYZINormal PointINormal;
	typedef pcl::PointCloud<PointINormal> CloudINormal;
	typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;

	IntensityPartition(int, int, float, float, float, float, float);

	CloudINormalPtr execution(CloudIPtr);
	void initialize(void);
	void cartesian_pt_2_polar_grid(CloudIPtr);
	float calc_variance(const std::vector<std::vector<int> >&, int, int, int);
	void calc_otsu_binary(void);
	void calc_diff_from_avr(void);
	void emergency_judge(void);
	void separated_histogram_peak_filter(float, float, int);
	CloudIPtr otsu_pc_generator(void);

private:
	const int GRASS = 1;
	const int ASPHALT = 2;
	int n_grass;//=0
	int	n_asphalt;// = 0;
	int RANGE_DIVISION_NUM_;//= 20
	int THETA_DIVISION_NUM_;//= 360;
	float PEAK_DIFF_THRESHOLD_;
	float OTSU_BINARY_SEPARATION_THRESHOLD_;// = 0.2;
	float OTSU_BINARY_DIFF_FROM_AVR_THRESHOLD_;// = 3.0;
	float OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD_;// = 58.0;
	float RANGE_MAX_;// = 20.0;
	float avr_grass = 0.0, avr_asphalt = 0.0;
	float dR;// = RANGE_MAX / (float)RANGE_DIVISION_NUM;
	float dTheta;// = 2*M_PI / (float)THETA_DIVISION_NUM;
	float intensity_max_all;
	float range_mu_otsu;	
	float otsu_range_std_deviation;	
	float max_std_deviation;
	std::vector<float> max_std_deviation_list;
	std::vector<float> polar_grid_pt_cnt_row;
	std::vector<float> polar_grid_avr_intensity_row;
	std::vector<float> polar_grid_sum_intensity_row;
	std::vector<CloudI> polar_grid_pc_row;
	std::vector<float> intensity_max;
	std::vector<float> intensity_min;
	std::vector<float> s_max;//[RANGE_DIVISION_NUM]
	std::vector<float> otsu_threshold_tmp;//[RANGE_DIVISION_NUM]
	std::vector<bool> peak_filter;
	std::vector<float> time_mu_otsu;//[RANGE_DIVISION_NUM];	
	std::vector<float> ptz_list;
	std::vector<std::vector<float> > polar_grid_pt_cnt;//[RANGE_DIVISION_NUM][THETA_DIVISION_NUM];
	std::vector<std::vector<float> > polar_grid_avr_intensity;//[RANGE_DIVISION_NUM][THETA_DIVISION_NUM];
	std::vector<std::vector<float> > polar_grid_sum_intensity;//[RANGE_DIVISION_NUM][THETA_DIVISION_NUM];
	std::vector<std::vector<float> > polar_grid_diff_sum_pow_intensity;//[RANGE_DIVISION_NUM][THETA_DIVISION_NUM];
	std::vector<std::vector<float> > polar_grid_std_deviation_intensity;//[RANGE_DIVISION_NUM][THETA_DIVISION_NUM];
	std::vector<std::vector<CloudI> > polar_grid_pc;//[RANGE_DIVISION_NUM][THETA_DIVISION_NUM];
	sensor_msgs::PointCloud2 pub_pc;
	CloudIPtr input_pc_ {new CloudI};
	CloudIPtr polar_pc_ {new CloudI};
	CloudIPtr otsu_binary_pc_ {new CloudI};
	struct GA{
		std::vector<std::vector<float> > grass;
		std::vector<std::vector<float> > asphalt;
	};
	struct WB{
		float within;
		float between;
	};
	road_recognizer::OtsuBinary otsu_binary_msg; 
};



#endif// __INTENSITY_PARTITION
