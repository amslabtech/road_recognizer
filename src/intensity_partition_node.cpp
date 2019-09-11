#include "road_recognizer/intensity_partition.h"

IntensityPartition::IntensityPartition(int range_division_num, int theta_division_num, float range_max
									, float otsu_binary_separation_threshold, float otsu_binary_diff_from_avr_threshold, float otsu_binary_sum_of_diff_from_avr_threshold)
{
	RANGE_DIVISION_NUM_ = range_division_num;
	THETA_DIVISION_NUM_ = theta_division_num;
	RANGE_MAX_ = range_max;
	OTSU_BINARY_SEPARATION_THRESHOLD_ = otsu_binary_separation_threshold;
	OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD_ = otsu_binary_sum_of_diff_from_avr_threshold;
	OTSU_BINARY_DIFF_FROM_AVR_THRESHOLD_ = otsu_binary_diff_from_avr_threshold;

	otsu_binary_msg.range_resolution = RANGE_DIVISION_NUM_;
	otsu_binary_msg.theta_resolution = THETA_DIVISION_NUM_;
	otsu_binary_msg.range_max = RANGE_MAX_;
	otsu_binary_msg.otsubinary_separation_threshold = OTSU_BINARY_SEPARATION_THRESHOLD_;
	otsu_binary_msg.otsubinary_sum_of_diff_from_avr_threshold = OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD_;
	otsu_binary_msg.intensity.resize(RANGE_DIVISION_NUM_);
	otsu_binary_msg.analysis.resize(RANGE_DIVISION_NUM_);

	dR = RANGE_MAX_ / (float)RANGE_DIVISION_NUM_;
	dTheta = 2*M_PI / (float)THETA_DIVISION_NUM_;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr IntensityPartition::execution(CloudIPtr ground_pc_)
{
	CloudIPtr otsu_binary_pc_ {new CloudI};
	CloudINormalPtr grass_pc_ {new CloudINormal};

	input_pc_ = ground_pc_;
	polar_pc_ = input_pc_;
	
	cartesian_pt_2_polar_grid(input_pc_);
	calc_otsu_binary();
	otsu_binary_pc_ = otsu_pc_generator();
	pcl::copyPointCloud(*otsu_binary_pc_, *grass_pc_);
	for(auto& pt : grass_pc_->points){
		pt.normal_x = 0.0;
		pt.normal_y = 0.0;
		pt.normal_z = 0.0;
	}
	
	ptz_list.clear();
	intensity_max.clear();
	intensity_min.clear();
	s_max.clear();
	otsu_threshold_tmp.clear();
	polar_grid_pt_cnt_row.clear();
	polar_grid_avr_intensity_row.clear();
	polar_grid_sum_intensity_row.clear();
	polar_grid_pt_cnt.clear();
	polar_grid_avr_intensity.clear();
	polar_grid_sum_intensity.clear();

	return grass_pc_;
}

void IntensityPartition::initialize(void)
{
	otsu_binary_msg.emergency = false;
	for(int i = 0; i < THETA_DIVISION_NUM_; i++){
		s_max.push_back(0.0);
		otsu_threshold_tmp.push_back(0.0);
		intensity_max.push_back(0.0);
		intensity_min.push_back(999.9);
		polar_grid_pt_cnt_row.push_back(0.0);
		polar_grid_avr_intensity_row.push_back(-1.0);
		polar_grid_sum_intensity_row.push_back(0.0);
	}
	for(int j = 0; j < RANGE_DIVISION_NUM_; j++){
		polar_grid_pt_cnt.push_back(polar_grid_pt_cnt_row);
		polar_grid_avr_intensity.push_back(polar_grid_avr_intensity_row);
		polar_grid_sum_intensity.push_back(polar_grid_sum_intensity_row);
	}
	intensity_max_all = 0.0;
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		for(int theta_g = 0; theta_g < THETA_DIVISION_NUM_; theta_g++){
		}
	}
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
		bool get_pt_flag = false, r_flag = false, theta_flag = false;
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
					r_flag = true;
					theta_flag = true;
				}
				
				if(intensity_min[r_g] > polar_grid_avr_intensity[r_g][theta_g] && get_pt_flag){
					intensity_min[r_g] = polar_grid_avr_intensity[r_g][theta_g];
				}
				if(intensity_max[r_g] < polar_grid_avr_intensity[r_g][theta_g] && get_pt_flag){
					intensity_max[r_g] = polar_grid_avr_intensity[r_g][theta_g];
				}

				if(theta_flag) break;
			}
			if(r_flag) break;
			
		}
		
		if(intensity_max_all < pt.intensity){
			intensity_max_all = pt.intensity;
		}

		i++;
	}

	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
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
	float range_mu_otsu = range_sum_otsu / (RANGE_DIVISION_NUM_ - 1);

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
	if(sum_diff_from_avr > OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD_){
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
		// var.grass[i].push_back(var_num_avr_row);
		// num.grass[i].push_back(var_num_avr_row);
		// avr.grass[i].push_back(var_num_avr_row);
		// var.asphalt[i].push_back(var_num_avr_row);
		// num.asphalt[i].push_back(var_num_avr_row);
		// avr.asphalt[i].push_back(var_num_avr_row);
		
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
			if(intensity_tmp > 0){
				histogram.at(intensity_tmp).at(r_g) += 1;
			}
		}
	}
	
	
	// calc separation
	for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
		for(int i_threshold = 1; i_threshold < (int)intensity_max[r_g]; i_threshold++){
			/* var.grass[r_g][i_threshold-1] = calc_variance(histogram, r_g, i_threshold, GRASS); */
			/* var.asphalt[r_g][i_threshold-1] = calc_variance(histogram, r_g, i_threshold, ASPHALT); */
			/* num.grass[r_g][i_threshold-1] = (float)n_grass; */
			/* num.asphalt[r_g][i_threshold-1] = (float)n_asphalt; */
			/* avr.grass[r_g][i_threshold-1] = avr_grass; */
			/* avr.asphalt[r_g][i_threshold-1] = avr_asphalt; */
			
			var.grass[i_threshold-1][r_g] = calc_variance(histogram, r_g, i_threshold, GRASS);
			var.asphalt[i_threshold-1][r_g] = calc_variance(histogram, r_g, i_threshold, ASPHALT);
			num.grass[i_threshold-1][r_g] = (float)n_grass;
			num.asphalt[i_threshold-1][r_g] = (float)n_asphalt;
			avr.grass[i_threshold-1][r_g] = avr_grass;
			avr.asphalt[i_threshold-1][r_g] = avr_asphalt;

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
		for(int i_threshold = 1; i_threshold < (int)intensity_max[r_g]; i_threshold++){
			float ng = (float)num.grass[r_g][i_threshold-1];
			float na = (float)num.asphalt[r_g][i_threshold-1];
			var_wb.within = (ng * var.grass[r_g][i_threshold-1] + na * var.asphalt[r_g][i_threshold-1]) / (ng + na);
			float diff_mg = avr.grass[r_g][i_threshold-1] - avr_all[r_g];
			float diff_ma = avr.asphalt[r_g][i_threshold-1] - avr_all[r_g];
			var_wb.between = (ng * diff_mg * diff_mg + na * diff_ma * diff_ma) / (ng + na);
			float s_tmp = var_wb.between / var_wb.within;
			
			if(s_max[r_g] < s_tmp){
				s_max[r_g] = s_tmp;
				otsu_threshold_tmp[r_g] = (float)i_threshold;
			}
		}
		// search for max separation
		if(r_g == 0){
			otsu_binary_msg.intensity[r_g].threshold = 0.0;
			otsu_binary_msg.analysis[r_g].separation = 0.0;
		}else{
			otsu_binary_msg.intensity[r_g].threshold = otsu_threshold_tmp[r_g];
			otsu_binary_msg.analysis[r_g].separation = s_max[r_g];
		}
				
	}
	
	// calc threshold histogram in range
	calc_diff_from_avr();

	// judge emergency
	emergency_judge();
	
	//sum_all.clear();
	//avr_all.clear();
	//diff_all.clear();
	//sum_tmp1.clear();
	//multi_sum_all.clear();
	r_res_array.clear();
	var_num_avr_row.clear();
	//var.grass.clear();
	//num.grass.clear();
	//avr.grass.clear();
	//var.asphalt.clear();
	//num.asphalt.clear();
	//avr.asphalt.clear();
	histogram.clear();
}



//CloudIPtr IntensityPartition::otsu_pc_generator(void)
pcl::PointCloud<pcl::PointXYZI>::Ptr IntensityPartition::otsu_pc_generator(void)
{
	size_t iz = 0;
	for(auto& pt : polar_pc_->points){
		/*
		int RG = sqrt(pt.x * pt.x + pt.y * pt.y) / dR;
		int thetag = atan2(pt.y,pt.x);
		if(thetag < 0){
			thetag = 2 * M_PI + thetag;
		}
		int THETAG = thetag / dTheta;
		*/

		for(int r_g = 0; r_g < RANGE_DIVISION_NUM_; r_g++){
			//for(int theta_g = 0; theta_g < THETA_DIVISION_NUM; theta_g++){
				//if(((float)r_g <= pt.z && pt.z < (float)r_g+dR) && ((float)theta_g <= thetag && thetag < (float)theta_g + dTheta)
				if(((float)r_g <= pt.z && pt.z < (float)r_g+dR)
					/* && ((otsu_threshold_tmp[r_g] - 1.0 > polar_grid_avr_intensity[RG][THETAG]) */
					&& ((otsu_threshold_tmp[r_g] > pt.intensity)
						//|| (otsu_binary_msg.analysis[r_g].otsubinary_diff_from_thresholds_avr > OTSU_BINARY_DIFF_FROM_AVR_THRESHOLD_)
						|| (otsu_binary_msg.analysis[r_g].separation < OTSU_BINARY_SEPARATION_THRESHOLD_)
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
	pass.setFilterLimits(4.9, intensity_max_all);
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





