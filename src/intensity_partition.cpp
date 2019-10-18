
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "road_recognizer/OtsuBinary.h"
#include "road_recognizer/Intensity.h"
#include "road_recognizer/Analysis.h"
#include "road_recognizer/Distribution.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"
#include "Eigen/Eigenvalues"
#include "Eigen/Geometry"


class IntensityPartition
{
public:
	typedef pcl::PointXYZI PointI;
	typedef pcl::PointCloud<PointI> CloudI;
	typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;
	typedef pcl::PointXYZINormal PointINormal;
	typedef pcl::PointCloud<PointINormal> CloudINormal;
	typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;
	typedef pcl::PointXYZRGBA PointRGBA;
	typedef pcl::PointCloud<PointRGBA> CloudRGBA;
	typedef pcl::PointCloud<PointRGBA>::Ptr CloudRGBAPtr;

	IntensityPartition(void);

	void execution(void);
	void initialize(void);
	void cartesian_pt_2_polar_grid(CloudINormalPtr);
	void fresh_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
	void odom_callback(const nav_msgs::OdometryConstPtr&);
	float calc_variance(const std::vector<std::vector<int> >&, int, int, int);
	void calc_otsu_binary(void);
	void calc_diff_from_avr(void);
	void emergency_judge(void);
	CloudIPtr otsu_pc_generator(void);

private:
	const int GRASS = 1;
	const int ASPHALT = 2;
	int n_grass;//=0
	int	n_asphalt;// = 0;
	int RANGE_DIVISION_NUM_;//= 20
	int THETA_DIVISION_NUM_;//= 360;
	float VAR_BETWEEN_THRESHOLD_;
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
	std::vector<float> intensity_max;
	std::vector<float> intensity_min;
	std::vector<float> max_s_var_between;//[RANGE_DIVISION_NUM]
	std::vector<float> otsu_threshold_tmp;//[RANGE_DIVISION_NUM]
	std::vector<float> polar_grid_pt_cnt_row;
	std::vector<float> polar_grid_avr_intensity_row;
	std::vector<float> polar_grid_sum_intensity_row;
	std::vector<float> time_mu_otsu;//[RANGE_DIVISION_NUM];
	std::vector<float> ptz_list;
	std::vector<std::vector<float> > polar_grid_pt_cnt;//[RANGE_DIVISION_NUM][THETA_DIVISION_NUM];
	std::vector<std::vector<float> > polar_grid_avr_intensity;//[RANGE_DIVISION_NUM][THETA_DIVISION_NUM];
	std::vector<std::vector<float> > polar_grid_sum_intensity;//[RANGE_DIVISION_NUM][THETA_DIVISION_NUM];
	CloudINormalPtr input_pc_ {new CloudINormal};
	CloudINormalPtr polar_pc_ {new CloudINormal};
	CloudIPtr store_pc {new CloudI};
	struct GA{
		std::array<std::vector<float>, 256> grass;
		std::array<std::vector<float>, 256> asphalt;
	};
	struct WB{
		float within;
		float between;
	};
	road_recognizer::OtsuBinary otsu_binary_msg;

	float Hz = 5;
	ros::Subscriber sub_pc;
	ros::Subscriber sub_odom;
	ros::Publisher advertise_pc;
	ros::Publisher new_otsu_pub;
	ros::Publisher old_otsu_pub;
	ros::NodeHandle n;
	bool pc_callback_flag;
	bool tf_listen_flag;
	bool odom_callback_flag;
	bool first_flag;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	std::string target_frame = "/odom";

	
    Eigen::Affine3d affine_transform;
    Eigen::Vector3d last_add_position;
    Eigen::Vector3d current_position;
    double last_add_yaw;
    double current_yaw;

	std::vector<float> intensity_study;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_robosym");
	
	IntensityPartition intensity_partition;
	intensity_partition.execution();
	
	return 0;
}


IntensityPartition::IntensityPartition(void)
{
	RANGE_DIVISION_NUM_ = 20;
	THETA_DIVISION_NUM_ = 720;
	RANGE_MAX_ = 20;
	VAR_BETWEEN_THRESHOLD_ = 150;
	OTSU_BINARY_SEPARATION_THRESHOLD_ = 0;
	OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD_ = 0;
	OTSU_BINARY_DIFF_FROM_AVR_THRESHOLD_ = 0;

	otsu_binary_msg.range_division_num = RANGE_DIVISION_NUM_;
	otsu_binary_msg.theta_division_num = THETA_DIVISION_NUM_;
	otsu_binary_msg.range_max = RANGE_MAX_;
	otsu_binary_msg.otsubinary_separation_threshold = OTSU_BINARY_SEPARATION_THRESHOLD_;
	otsu_binary_msg.intensity.resize(RANGE_DIVISION_NUM_);
	otsu_binary_msg.analysis.resize(RANGE_DIVISION_NUM_);

	dR = RANGE_MAX_ / (float)RANGE_DIVISION_NUM_;
	dTheta = 2*M_PI / (float)THETA_DIVISION_NUM_;

	sub_pc = n.subscribe("/velodyne_clear", 10, &IntensityPartition::fresh_pc_callback, this);
	sub_odom = n.subscribe("/odom/complement", 10, &IntensityPartition::odom_callback, this);
	advertise_pc = n.advertise<sensor_msgs::PointCloud2>("/robosym_pc", 10);
}


void IntensityPartition::execution(void)
{
	pc_callback_flag = false;
	tf_listen_flag = false;
	first_flag = true;
	odom_callback_flag = false;
	store_pc->points.resize(0);
	int cnt = 0;

	ros::Rate r(Hz);
	while(ros::ok()){
		/* try{ */
       	/* 	listener.lookupTransform("/odom","/velodyne", ros::Time(0), transform); */
		/* 	tf_listen_flag = true; */
     	/* }    */
     	/* catch (tf::TransformException ex){ */
       	/* 	ROS_ERROR("%s",ex.what()); */
       	/* 	ros::Duration(1.0).sleep(); */
    	/* }  */
		/* if(tf_listen_flag && pc_callback_flag){ */
		
		if(odom_callback_flag && pc_callback_flag){
			std::cout << "start!" << std::endl;

			CloudIPtr otsu_binary_pc_ {new CloudI};

			polar_pc_ = input_pc_;

			cartesian_pt_2_polar_grid(input_pc_);
			calc_otsu_binary();
			otsu_binary_pc_ = otsu_pc_generator();
			otsu_binary_pc_->header = input_pc_->header;
			
			/* int grass_pt[20][720]; */
			/* int asphalt_pt[20][720]; */
			/* float score[20][720]; */
			/* for(int r_g = 0; r_g < 20; r_g++){ */
			/* 	for(int theta_g = 0; theta_g < 720; theta_g++){ */
			/* 		grass_pt[r_g][theta_g] = 0; */
			/* 		asphalt_pt[r_g][theta_g] = 0; */
			/* 		score[r_g][theta_g] = 0; */
			/* 	} */
			/* } */
			/*  */
			/* for(auto& pt : otsu_binary_pc_->points){ */
			/* 	float r_tmp = sqrt(pt.x * pt.x + pt.y * pt.y); */
			/* 	float theta_tmp = atan2(pt.y,pt.x); */
			/* 	if(theta_tmp < 0){ */
			/* 		theta_tmp = 2 * M_PI + theta_tmp; */
			/* 	} */
            /*  */
			/* 	bool check_flag = false; */
			/* 	for(int r_g = 0; r_g < 20; r_g++){ */
			/* 		for(int theta_g = 0; theta_g < 120; theta_g++){ */
			/* 			if((r_g == (int)(r_tmp / 1.0)) && (theta_g == (int)(theta_tmp / (2*M_PI/120.0) ))){ */
			/* 				if(pt.intensity == 0){ */
			/* 					asphalt_pt[r_g][theta_g] += 1; */
			/* 				}else{ */
			/* 					grass_pt[r_g][theta_g] += 1; */
			/* 				} */
			/* 				check_flag = true; */
			/* 			} */
			/* 			if(check_flag) break; */
			/* 		} */
			/* 		if(check_flag) break; */
			/* 	} */
			/* } */


			// sensor_msgs::PointCloud2 tmp_pc;
			// sensor_msgs::PointCloud2 tmp_pc2;
			// sensor_msgs::PointCloud2 transformed_pc;
			// sensor_msgs::PointCloud2 transformed_pc2;
			// CloudIPtr otsu_binary_transformed_pc {new CloudI};
			// CloudIPtr store_transformed_pc {new CloudI};
            //
			// pcl::toROSMsg(*otsu_binary_pc_, tmp_pc);
			// pcl_ros::transformPointCloud(target_frame, tmp_pc, transformed_pc, listener);
			// pcl::fromROSMsg(transformed_pc, *otsu_binary_transformed_pc);
            //
			// if(store_pc->points.size() > 0){
			// 	pcl::toROSMsg(*store_pc, tmp_pc2);
			// 	pcl_ros::transformPointCloud(target_frame, tmp_pc2, transformed_pc2, listener);
			// 	pcl::fromROSMsg(transformed_pc2, *store_transformed_pc);
			// 	store_pc = store_transformed_pc;
			// }
            //
			// size_t size = otsu_binary_transformed_pc->points.size();
			// *store_pc += *otsu_binary_transformed_pc;
			// if(cnt > 15){
			// 	store_pc->points.erase(store_pc->points.begin(), store_pc->points.begin() + size);
			// }



			// pcl::transformPointCloud(*otsu_binary_pc_, *otsu_binary_pc_, affine_transform);
			/* pcl::transformPointCloud(*store_pc, *store_pc, affine_transform); */
			/* if(((current_position - last_add_position).norm() > 0.1) || (fabs(current_yaw - last_add_yaw) > 0.1)){ */
			/* 	// store pointcloud only when robot moves */
			/* 	*store_pc += *otsu_binary_pc_; */
            /*  */
			/* 	size_t size = otsu_binary_pc_->points.size(); */
			/* 	if(cnt > 25){ */
			/* 		store_pc->points.erase(store_pc->points.begin(), store_pc->points.begin() + size); */
			/* 	} */
            /*  */
			/* 	last_add_position = current_position; */
			/* 	last_add_yaw = current_yaw; */
			/* } */
	


			CloudRGBAPtr rgba_pc {new CloudRGBA};
			rgba_pc->points.resize(0);
			// for(auto& pt : store_pc->points){
			for(auto& pt : otsu_binary_pc_->points){
				CloudRGBAPtr tmp_rgba_pt {new CloudRGBA};
				tmp_rgba_pt->points.resize(1);
				tmp_rgba_pt->points[0].x = pt.x;
				tmp_rgba_pt->points[0].y = pt.y;
				tmp_rgba_pt->points[0].z = pt.z;
				tmp_rgba_pt->points[0].a = pt.intensity;
				tmp_rgba_pt->points[0].r = 0;
				tmp_rgba_pt->points[0].g = 0;
				tmp_rgba_pt->points[0].b = 0;
				*rgba_pc += *tmp_rgba_pt;
			}

			/* for(int r_g = 0; r_g < 20; r_g++){ */
			/* 	for(int theta_g = 0; theta_g < 120; theta_g++){ */
			/* 		if(asphalt_pt[r_g][theta_g] + grass_pt[r_g][theta_g] > 0){ */
			/* 			score[r_g][theta_g] = (float)(grass_pt[r_g][theta_g]) / (float)(asphalt_pt[r_g][theta_g] + grass_pt[r_g][theta_g]); */
			/* 		} */
            /*  */
			/* 		for(auto& pt : rgba_pc->points){ */
			/* 			float r_tmp = sqrt(pt.x * pt.x + pt.y * pt.y); */
			/* 			float theta_tmp = atan2(pt.y,pt.x); */
			/* 			if(theta_tmp < 0){ */
			/* 				theta_tmp = 2 * M_PI + theta_tmp; */
			/* 			} */
            /*  */
			/* 			if((r_g == (int)(r_tmp / 1.0)) && (theta_g == (int)(theta_tmp / (2*M_PI/120.0) ))){ */
			/* 				if(score[r_g][theta_g] >= 0.5){ */
			/* 					pt.r = 0.0; */
			/* 					pt.g = 255.0; */
			/* 					pt.b = 0.0; */
			/* 					pt.a = 255.0; */
			/* 				}else{ */
			/* 					pt.r = 255.0; */
			/* 					pt.g = 255.0; */
			/* 					pt.b = 255.0; */
			/* 					pt.a = 150.0; */
			/* 				} */
			/* 			} */
			/* 		} */
			/* 	} */
			/* } */

			/* std::cout << "size : " << rgba_pc->points.size() << std::endl; */

			rgba_pc->header = input_pc_->header;
			rgba_pc->width = rgba_pc->points.size();
			sensor_msgs::PointCloud2 pub_pc;
			pcl::toROSMsg(*rgba_pc, pub_pc);
			advertise_pc.publish(pub_pc);

			ptz_list.clear();
			intensity_max.clear();
			intensity_min.clear();
			max_s_var_between.clear();
			otsu_threshold_tmp.clear();
			polar_grid_pt_cnt_row.clear();
			polar_grid_avr_intensity_row.clear();
			polar_grid_sum_intensity_row.clear();
			polar_grid_pt_cnt.clear();
			polar_grid_avr_intensity.clear();
			polar_grid_sum_intensity.clear();
			intensity_study.clear();
			tf_listen_flag = false;
			pc_callback_flag = false;
			odom_callback_flag = false;
			cnt++;
		}
		r.sleep();
		ros::spinOnce();
	}
}


void IntensityPartition::fresh_pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *input_pc_);
	pc_callback_flag = true;
}


void IntensityPartition::odom_callback(const nav_msgs::OdometryConstPtr& msg_odom)
{
    current_position << msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z;
    current_yaw = tf::getYaw(msg_odom->pose.pose.orientation);
    static Eigen::Vector3d last_position;
    static double last_yaw;

    if(!first_flag){
        double d_yaw = current_yaw - last_yaw;
        /* double d_yaw = -2.0*(current_yaw - last_yaw); */
        d_yaw = atan2(sin(d_yaw), cos(d_yaw));
        Eigen::Matrix3d r;
        r = Eigen::AngleAxisd(-d_yaw, Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d last_yaw_rotation;
        last_yaw_rotation = Eigen::AngleAxisd(-last_yaw, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d _current_position = last_yaw_rotation * current_position;
        Eigen::Vector3d _last_position = last_yaw_rotation * last_position;
        Eigen::Translation<double, 3> t(_last_position - _current_position);

        affine_transform = t * r;
		
		odom_callback_flag = true;
	}else{
        first_flag = false;
        last_position = current_position;
        last_add_position = current_position;
        last_yaw = current_yaw;
        last_add_yaw = current_yaw;
    }
    last_position = current_position;
    last_yaw = current_yaw;
}


void IntensityPartition::initialize(void)
{
	otsu_binary_msg.emergency = false;
	for(int theta_g = 0; theta_g < THETA_DIVISION_NUM_; theta_g++){
		polar_grid_pt_cnt_row.push_back(0.0);
		polar_grid_avr_intensity_row.push_back(-1.0);
		polar_grid_sum_intensity_row.push_back(0.0);
	}
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		intensity_max.push_back(0.0);
		intensity_min.push_back(999.9);
		otsu_threshold_tmp.push_back(0.0);
		max_s_var_between.push_back(0.0);
		polar_grid_pt_cnt.push_back(polar_grid_pt_cnt_row);
		polar_grid_avr_intensity.push_back(polar_grid_avr_intensity_row);
		polar_grid_sum_intensity.push_back(polar_grid_sum_intensity_row);
		otsu_binary_msg.intensity[r_g].threshold = 0.0;
		otsu_binary_msg.analysis[r_g].separation = -99.9;
		intensity_study.push_back(0.0);
	}
	intensity_max_all = 0.0;
}


void IntensityPartition::cartesian_pt_2_polar_grid(CloudINormalPtr cartesian_pc_)
{
	float r_tmp, theta_tmp, z_tmp;
	size_t i = 0;
	initialize();
	
	std::cout << "new" << std::endl;
	for(auto& pt : cartesian_pc_->points){
		r_tmp = sqrt(pt.x * pt.x + pt.y * pt.y);
		theta_tmp = atan2(pt.y,pt.x);
		if(theta_tmp < 0){
			theta_tmp = 2 * M_PI + theta_tmp;
		}
		
		// create polar grid informations
		bool get_pt_flag = false;
		for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
			for(int theta_g = 0; theta_g < THETA_DIVISION_NUM_; theta_g++){
				if((r_g == (int)(r_tmp / dR)) && (theta_g == (int)(theta_tmp / dTheta))){
					polar_grid_pt_cnt[r_g][theta_g] += 1;
					polar_grid_sum_intensity[r_g][theta_g] += pt.intensity;
					polar_grid_avr_intensity[r_g][theta_g] = polar_grid_sum_intensity[r_g][theta_g] / (float)polar_grid_pt_cnt[r_g][theta_g];
					z_tmp = polar_pc_->points[i].z;
					ptz_list.push_back(z_tmp);
					polar_pc_->points[i].z = r_tmp;

					get_pt_flag = true;
				}
				if(get_pt_flag) break;
			}
			if(get_pt_flag) break;

		}

		if(intensity_max_all < pt.intensity){
			intensity_max_all = pt.intensity;
		}

		i++;
	}

	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		for(int theta_g = 0; theta_g < THETA_DIVISION_NUM_; theta_g++){
			if(intensity_min[r_g] > polar_grid_avr_intensity[r_g][theta_g]){
				intensity_min[r_g] = polar_grid_avr_intensity[r_g][theta_g];
			}
			if(intensity_max[r_g] < polar_grid_avr_intensity[r_g][theta_g]){
				intensity_max[r_g] = polar_grid_avr_intensity[r_g][theta_g];
			}
			if(theta_g > 0 && theta_g < 10 && polar_grid_avr_intensity[r_g][theta_g] > 0){
				std::cout << "theta : " << theta_g << ", r : " << r_g << ", intensity : " << polar_grid_avr_intensity[r_g][theta_g] << std::endl;
			}
			if(theta_g > 520 && theta_g < 530 && polar_grid_avr_intensity[r_g][theta_g] > 0){
				std::cout << "theta : " << theta_g << ", r : " << r_g << ", intensity : " << polar_grid_avr_intensity[r_g][theta_g] << std::endl;
			}
		}
		if(r_g == 0){
			otsu_binary_msg.intensity[r_g].min = 0.0;
			otsu_binary_msg.intensity[r_g].max = 0.0;
		}else{
			otsu_binary_msg.intensity[r_g].min = intensity_min[r_g];
			otsu_binary_msg.intensity[r_g].max = intensity_max[r_g];
		}
	}
}


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
	for(int r_g = 1; r_g < RANGE_DIVISION_NUM_; r_g++){
		range_sum_otsu += otsu_binary_msg.intensity[r_g].threshold;
	}
	float range_mu_otsu = range_sum_otsu / (RANGE_DIVISION_NUM_);
	/* std::cout << "range_mu_otsu : " << range_mu_otsu <<std::endl; */
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		float diff = otsu_binary_msg.intensity[r_g].threshold - range_mu_otsu;
		otsu_binary_msg.analysis[r_g].otsubinary_diff_from_thresholds_avr = sqrt(diff * diff);
	}
}




void IntensityPartition::emergency_judge(void)
{
	float sum_diff_from_avr = 0.0;
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		sum_diff_from_avr += otsu_binary_msg.analysis[r_g].otsubinary_diff_from_thresholds_avr;
	}
	if(sum_diff_from_avr < OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD_){
		otsu_binary_msg.emergency = true;
	}
}



void IntensityPartition::calc_otsu_binary(void)
{
	const static int histogram_size = 256;
	std::array<float, histogram_size> sum_all;
	std::array<float, histogram_size> avr_all;
	std::array<float, histogram_size> diff_all;
	std::array<float, histogram_size> sum_tmp1;
	std::array<float, histogram_size> multi_sum_all;
	std::vector<int> r_res_array;
	std::vector<std::vector<int> > histogram;
	std::vector<float> var_num_avr_row;
	struct GA var;
	struct GA num;
	struct GA avr;

	// initialize
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		sum_all[r_g] = 0.0;
		avr_all[r_g] = 0.0;
		diff_all[r_g] = 0.0;
		sum_tmp1[r_g] = 0.0;
		multi_sum_all[r_g] = 0.0;
		r_res_array.push_back((int)0);
		var_num_avr_row.push_back(0.0);
	}

	for(int i = 0; i < histogram_size; i++){
		histogram.push_back(r_res_array);
		var.grass[i] = var_num_avr_row;
		num.grass[i] = var_num_avr_row;
		avr.grass[i] = var_num_avr_row;
		var.asphalt[i] = var_num_avr_row;
		num.asphalt[i] = var_num_avr_row;
		avr.asphalt[i] = var_num_avr_row;
	}

	// make histogram
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		for(int theta_g = 0; theta_g < THETA_DIVISION_NUM_; theta_g++){
			int intensity_tmp = (int)polar_grid_avr_intensity[r_g][theta_g];
			if(intensity_tmp >= 0){
				/* histogram.at(intensity_tmp).at(r_g) += 1; */
				histogram[intensity_tmp][r_g] += 1;
			}
		}
	}



	// calc separation
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		/* std::cout << "----r_g = " << r_g << "----" << std::endl; */
		for(int i_threshold = 1; i_threshold < (int)intensity_max[r_g]; i_threshold++){
			var.grass[i_threshold-1][r_g] = calc_variance(histogram, r_g, i_threshold, GRASS);
			var.asphalt[i_threshold-1][r_g] = calc_variance(histogram, r_g, i_threshold, ASPHALT);
			num.grass[i_threshold-1][r_g] = (float)n_grass;
			num.asphalt[i_threshold-1][r_g] = (float)n_asphalt;
			avr.grass[i_threshold-1][r_g] = avr_grass;
			avr.asphalt[i_threshold-1][r_g] = avr_asphalt;

			/* printf("%d", i_threshold); */
			/* int i = 0; */
			/* while(i < histogram[i_threshold][r_g]){ */
			/* 	printf("*"); */
			/* 	i++; */
			/* } */
			/* printf("\n"); */
		}
	}
		// calc whole variance
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		for(int idx_intensity = 0; idx_intensity < (int)intensity_max[r_g]; idx_intensity++){
			multi_sum_all[r_g] += histogram[idx_intensity][r_g] * idx_intensity;
			sum_all[r_g] += histogram[idx_intensity][r_g];
		}
		avr_all[r_g] = (float)multi_sum_all[r_g] / (float)sum_all[r_g];
		for(int i_threshold = 0; i_threshold < (int)intensity_max[r_g]; i_threshold++){
			diff_all[i_threshold] = (float)histogram[i_threshold][r_g] - avr_all[r_g];
			sum_tmp1[r_g] += diff_all[i_threshold] * diff_all[i_threshold];
		}
	}

	// calc variance between and within
	struct WB var_wb;
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		for(int i_threshold = 0; i_threshold < (int)intensity_max[r_g]; i_threshold++){
			float ng = (float)num.grass[i_threshold][r_g];
			float na = (float)num.asphalt[i_threshold][r_g];
			var_wb.within = (ng * var.grass[i_threshold][r_g] + na * var.asphalt[i_threshold][r_g]) / (ng + na);
			float diff_mg = avr.grass[i_threshold][r_g] - avr_all[r_g];
			float diff_ma = avr.asphalt[i_threshold][r_g] - avr_all[r_g];
			var_wb.between = (ng * diff_mg * diff_mg + na * diff_ma * diff_ma) / (ng + na);
			float s_tmp = var_wb.between / var_wb.within;

			if(otsu_binary_msg.analysis[r_g].separation < s_tmp){
				otsu_binary_msg.analysis[r_g].separation = s_tmp;
				otsu_binary_msg.intensity[r_g].threshold = (float)i_threshold;
				max_s_var_between[r_g] = var_wb.between;
			}
		}
		// otsu_binary_msg.analysis[r_g].separation = s_tmp;
		/* std::cout << "threshold[" << r_g << "] = " << otsu_binary_msg.intensity[r_g].threshold << std::endl; */
		/* std::cout << "separation[" << r_g << "] = " << otsu_binary_msg.analysis[r_g].separation << std::endl; */
		/* std::cout << "max_s_var_between[" << r_g << "] = " << max_s_var_between[r_g] << std::endl; */
	}

	// calc threshold histogram in range
	calc_diff_from_avr();

	// judge emergency
	emergency_judge();

	r_res_array.clear();
	var_num_avr_row.clear();
	histogram.clear();
}



pcl::PointCloud<pcl::PointXYZI>::Ptr IntensityPartition::otsu_pc_generator(void)
{
	size_t iz = 0;
	/* for(auto& pt : polar_pc_->points){ */
	/* 	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){ */
	/* 		if(((float)r_g <= pt.z && pt.z < (float)r_g+dR) */
	/* 			&& (otsu_threshold_tmp[r_g] - 1.0 > pt.intensity */
	/* 				//|| (otsu_binary_msg.analysis[r_g].otsubinary_diff_from_thresholds_avr > OTSU_BINARY_DIFF_FROM_AVR_THRESHOLD_) */
	/* 				|| (otsu_binary_msg.analysis[r_g].separation < OTSU_BINARY_SEPARATION_THRESHOLD_ && 1 < otsu_binary_msg.analysis[r_g].separation) */
	/* 				|| (pt.x == 0.0 && pt.y == 0.0) */
	/* 		   		) */
	/* 			){ */
	/* 			pt.intensity = -1.0; */
	/* 		} */
	/* 	} */
	/* 	pt.z = ptz_list.at(iz); */
	/* 	iz++; */
	/* } */


	for(auto& pt : polar_pc_->points){
		float r_tmp = sqrt(pt.x * pt.x + pt.y * pt.y);
		float theta_tmp = atan2(pt.y,pt.x);
		if(theta_tmp < 0){
			theta_tmp = 2 * M_PI + theta_tmp;
		}

		pt.z = ptz_list.at(iz);
		iz++;
		
		
		bool check_flag = false;
		for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
			for(int theta_g = 0; theta_g < THETA_DIVISION_NUM_; theta_g++){
				if((r_g == (int)(r_tmp / dR)) && (theta_g == (int)(theta_tmp / dTheta))){
					if(polar_grid_avr_intensity[r_g][theta_g] < otsu_binary_msg.intensity[r_g].threshold){
						/* pt.intensity = 0.0; */
						pt.intensity = -1.0;
					/* }else{ */
					/* 	pt.intensity = 1.0; */
					}
					if(max_s_var_between[r_g] < VAR_BETWEEN_THRESHOLD_){
						pt.intensity = -1.0;
					}
					check_flag = true;
				}
				if(check_flag) break;
			}
			if(check_flag) break;
		}
		
		/* if(pt.intensity / (0.08 * r_tmp + 16.9) < 1.0){ */
		/* 	#<{(| pt.intensity = 0.0; |)}># */
		/* 	pt.intensity = -1.0; */
		/* #<{(| }else{ |)}># */
		/* #<{(| 	pt.intensity = 1.0; |)}># */
		/* } */
		
	}

	if(otsu_binary_msg.emergency){
		//intensity_max_all = 0.0;
	}

	pcl::PassThrough<PointINormal> pass;
	CloudINormalPtr filtered_pc_ {new CloudINormal};
	pass.setInputCloud(polar_pc_);
	pass.setFilterFieldName ("intensity");
	/* pass.setFilterFieldName ("z"); */
	pass.setFilterLimits(0.0, intensity_max_all);
	/* pass.setFilterLimits(-999, 999); */
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
	/* return polar_pc_; */
}





