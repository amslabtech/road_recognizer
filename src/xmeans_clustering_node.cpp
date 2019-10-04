#include "road_recognizer/xmeans_clustering.h"


XmeansClustering::XmeansClustering(bool clustering_method, int n, int width_division_num, int height_division_num, double max_width, double max_height, double eps)
{
	kmeans = false;
	kmeans_pp = true;
	N_ = n;
	CLUSTERING_METHOD_ = clustering_method;
	WIDTH_DIVISION_NUM_ = width_division_num;
	HEIGHT_DIVISION_NUM_ = height_division_num;
	MAX_WIDTH_ = max_width;
	MAX_HEIGHT_ = max_height;
	dX = max_width / (float)width_division_num;
	dY = max_height / (float)height_division_num;
	EPS_ = eps;
}


pcl::PointCloud<pcl::PointXYZINormal>::Ptr XmeansClustering::execution(CloudIPtr imput_pc)
{
	std::cout << "start" << std::endl;

	CloudINormalPtr road_side_pc {new CloudINormal};
	
	initialization();
	grid_partition(imput_pc);
	xmeans_clustering();
	road_side_pc = points_extraction();
	road_side_pc->header = imput_pc->header;
	
	cells.affiliation.clear();
	cells.intensity_average.clear();
	cells.intensity_std_deviation.clear();
	cells.point_cloud.clear();
	point_counter.clear();
	intensity_sum.clear();
	sum_diff_pow.clear();

	std::cout << "return pc" << std::endl;
	return road_side_pc;
}


void XmeansClustering::initialization(void)
{
	std::cout << "initialization" << std::endl;

	std::vector<int> int_vector;
	std::vector<float> float_vector;
	std::vector<CloudIPtr> pcl_vector;
	CloudIPtr pc_tmp {new CloudI};
	pc_tmp->points.resize(0);

	for(int j = 0; j < HEIGHT_DIVISION_NUM_; j++){
		int_vector.push_back(0);
		float_vector.push_back(0.0);
		pcl_vector.push_back(pc_tmp);
	}
	
	for(int i = 0; i < WIDTH_DIVISION_NUM_; i++){
		cells.affiliation.push_back(int_vector); // .affiliation[i][j] = 0 <=> There is no point cloud in the grid
		cells.intensity_average.push_back(float_vector);
		cells.intensity_std_deviation.push_back(float_vector);
		cells.point_cloud.push_back(pcl_vector);
		point_counter.push_back(int_vector);
		intensity_sum.push_back(float_vector);
		sum_diff_pow.push_back(float_vector);
	}
}


void XmeansClustering::grid_partition(CloudIPtr not_partitioned_pc)
{
	std::cout << "grid_partition" << std::endl;

	for(int i = 0; i < WIDTH_DIVISION_NUM_; i++){
		for(int j = 0; j < HEIGHT_DIVISION_NUM_; j++){
			for(auto& pt : not_partitioned_pc->points){
				if((i*dX - (MAX_WIDTH_/2) <= pt.x && pt.x < (i+1)*dX - (MAX_WIDTH_/2))
				&& (j*dY - (MAX_HEIGHT_/2) <= pt.y && pt.y < (j+1)*dY - (MAX_HEIGHT_/2))){
					CloudIPtr tmp_pt {new CloudI};
					tmp_pt->points.resize(1);
					tmp_pt->points[0].x = pt.x;
					tmp_pt->points[0].y = pt.y;
					tmp_pt->points[0].z = pt.z;
					tmp_pt->points[0].intensity = pt.intensity;
					point_counter[i][j] += 1;
					intensity_sum[i][j] += pt.intensity;
					cells.intensity_average[i][j] = intensity_sum[i][j] / (float)point_counter[i][j];
					*cells.point_cloud[i][j] += *tmp_pt;
					/* std::cout << "cells.point_cloud[" << i << "][" << j << "]->points.size() = " << cells.point_cloud[i][j]->points.size() << std::endl; */
				}
			}
			cells.affiliation[i][j] = 0;
		}
	}

	for(int i = 0; i < WIDTH_DIVISION_NUM_; i++){
		for(int j = 0; j < HEIGHT_DIVISION_NUM_; j++){
			if(point_counter[i][j] > 0){
				for(int s = 0; s < point_counter[i][j]; s++){
					sum_diff_pow[i][j] += my_pow(cells.point_cloud[i][j]->points[s].intensity - cells.intensity_average[i][j]);
				}
				cells.intensity_std_deviation[i][j] = sqrt(sum_diff_pow[i][j] / point_counter[i][j]);
			}
		}
	}
}


pcl::PointCloud<pcl::PointXYZI>::Ptr XmeansClustering::partitional_optimization(CloudIPtr i_j_std_class_ex, int block) // i_j_std_class_ex's class is partitioned virtually
{
	std::cout << "partitional_optimization" << std::endl;

	float all_center_move = 0.0;
	Eigen::Vector3f eigen_zero = Eigen::Vector3f::Zero();
	std::vector<int> class_counter;
	std::vector<Eigen::Vector3f> centers;
	std::vector<Eigen::Vector3f> pre_centers;
	std::vector<Eigen::Vector3f> coordinate_sums;
	
	// chose center
	for(int k = 0; k < 2; k++){
		coordinate_sums.push_back(eigen_zero);
		class_counter.push_back(0);
		if(CLUSTERING_METHOD_ == kmeans){
			Eigen::Vector3f eigen_random = Eigen::Vector3f::Random();
			pre_centers.push_back(eigen_random);
		}else if(CLUSTERING_METHOD_ == kmeans_pp){
			while(1){
				float rnd_idx = randomization(i_j_std_class_ex->points.size());
				if(i_j_std_class_ex->points[rnd_idx].intensity == block + k){
					Eigen::Vector3f initial_center;
					initial_center << i_j_std_class_ex->points[rnd_idx].x, i_j_std_class_ex->points[rnd_idx].y, i_j_std_class_ex->points[rnd_idx].z;
					pre_centers.push_back(initial_center);
					break;
				}
			}
		}
	}

	while(all_center_move > EPS_){
		// Calculate each of class's center, and make each of class's position(by point cloud)
		all_center_move = 0;
		for(int k = block; k < 2 + block; k++){
			coordinate_sums[k] = eigen_zero;
			class_counter[k] = 0;
			for(auto& position : i_j_std_class_ex->points){
				if(position.intensity == block){
					Eigen::Vector3f eigen_tmp_point;
					eigen_tmp_point << position.x, position.y, position.z;
					coordinate_sums[k] += eigen_tmp_point;
					class_counter[k] += 1;
				}
			}
			centers[k] = coordinate_sums[k] / class_counter[k];
			Eigen::Vector3f eigen_center_tmp = centers[k] - pre_centers[k];
			all_center_move += eigen_center_tmp.norm();
			pre_centers[k] = centers[k];
		}

		// Reregister each of cells.affiliation[i][j]
		for(auto& position : i_j_std_class_ex->points){
			int count = 0;
			float min_range = WIDTH_DIVISION_NUM_ * HEIGHT_DIVISION_NUM_;
			for(int k = block; k < 2 + block; k++){
				Eigen::Vector3f eigen_tmp_point;
				eigen_tmp_point << position.x, position.y, position.z;
				Eigen::Vector3f coordinate_distance_from_center = eigen_tmp_point - centers[k];
				if(min_range > coordinate_distance_from_center.norm()){
					min_range = coordinate_distance_from_center.norm();
					position.intensity = (float)k;
				}
			}
			count++;
		}
	}

	return i_j_std_class_ex;
}


void XmeansClustering::xmeans_clustering(void)
{
	std::cout << "xmeans_clustering" << std::endl;

	int block = 0;
	int upper_block_num = 1;
	std::vector<bool> bic_flags;
	std::vector<bool> block_manager;
	std::vector<float> bic_list;
	CloudIPtr i_j_std_class_solo {new CloudI};
	CloudIPtr i_j_std_class {new CloudI};

	while(1){
		i_j_std_class->points.clear();
		i_j_std_class_solo->points.clear();
		for(int i = 0; i < WIDTH_DIVISION_NUM_; i++){
			for(int j = 0; j < HEIGHT_DIVISION_NUM_; j++){
				if(cells.affiliation[i][j] == block){
					CloudIPtr tmp_pos {new CloudI};
					tmp_pos->points.resize(1);
					tmp_pos->points[0].x = (float)i;
					tmp_pos->points[0].y = (float)j;
					tmp_pos->points[0].z = cells.intensity_std_deviation[i][j];
					tmp_pos->points[0].intensity = block;
					*i_j_std_class += *tmp_pos;
					*i_j_std_class_solo += *tmp_pos;
				}
			}
		}
		
		float bic = bic_calculation(false, i_j_std_class_solo);
		CloudIPtr tmp_pos {new CloudI};
		tmp_pos = virtual_class_partition(i_j_std_class_solo, block);
		i_j_std_class = partitional_optimization(tmp_pos, block);
		float bic_dash = bic_calculation(true, i_j_std_class);

		std::cout << "bic = " << bic << std::endl;
		std::cout << "bic_dash = " << bic_dash << std::endl;

		if(bic > bic_dash){
			for(auto& position : i_j_std_class->points){
				int ix = (int)position.x;
				int jy = (int)position.y;
				int class_intensity = (int)position.intensity;
				cells.affiliation[ix][jy] = class_intensity;
			}
			upper_block_num += 2;
			bic_list.push_back(bic_dash);
			bic_flags.push_back(false);
			block_manager.push_back(false);
		}else{
			bic_flags.push_back(true);
			block_manager.push_back(true);
		}

		if(block > 0){
			block_manager[block-1] = true;
		}

		int true_counter = 0;
		if(block == upper_block_num - 1){
			for(int b = 0; b < upper_block_num; b++){
				if(bic_flags[b] == true && block_manager[b] == true){
					true_counter++;
				}
			}
		}

		if(upper_block_num == true_counter){
			std::cout << "upper_block_num : " << upper_block_num << std::endl;
			std::cout << "block : " << block << std::endl;
			break;
		}

		block_manager[block] = true;

		block++;
		std::cout << "block num : " << block << std::endl;
	}
}


pcl::PointCloud<pcl::PointXYZINormal>::Ptr XmeansClustering::points_extraction(void)
{
	std::cout << "points_extraction" << std::endl;

	CloudIPtr xmeans_pc {new CloudI};
	CloudINormalPtr xmeans_normal_pc {new CloudINormal};
	
	xmeans_pc->points.resize(0);
	
	int observation_x = (int)(0.5 * WIDTH_DIVISION_NUM_);
	int observation_y = (int)(0.5 * HEIGHT_DIVISION_NUM_);
	int observed_position_class = cells.affiliation[observation_x][observation_y];
	
	for(int i = 0; i < WIDTH_DIVISION_NUM_; i++){
		for(int j = 0; j < HEIGHT_DIVISION_NUM_; j++){
			/* std::cout << "cells.affiliation[" << i << "][" << j << "] = " << cells.affiliation[i][j] << std::endl; */
			if(cells.affiliation[i][j] != observed_position_class && cells.point_cloud[i][j]->points.size() > 0){
				*xmeans_pc += *cells.point_cloud[i][j];
			}
		}
	}

	pcl::copyPointCloud(*xmeans_pc, *xmeans_normal_pc);
	for(auto& pt : xmeans_normal_pc->points){
		pt.normal_x = 0.0;
		pt.normal_y = 0.0;
		pt.normal_z = 0.0;
		pt.curvature = 0.0;
	}

	return xmeans_normal_pc;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr XmeansClustering::virtual_class_partition(CloudIPtr solo_class, int block)
{
	/* std::cout << "virtual_class_partition" << std::endl; */

	CloudIPtr virtual_parrtition_class {new CloudI};
	virtual_parrtition_class = solo_class;
	for(auto& position : virtual_parrtition_class->points){
		position.intensity = block + randomization(1);
	}
	return virtual_parrtition_class;
}


float XmeansClustering::density_function(CloudIPtr i_j_std_class_ex, Eigen::Vector3f pos_data, Eigen::Vector3f mu)
{
	/* std::cout << "density_function" << std::endl; */

	Eigen::Matrix3f cov_matrix = covariance_matrix(i_j_std_class_ex);
	float cov_matrix_determinant = cov_matrix.determinant();
	/* std::cout << "cov_matrix = " << std::endl; */
	/* std::cout << cov_matrix << std::endl; */
	Eigen::Matrix3f cov_matrix_inverse = cov_matrix.inverse();
	Eigen::MatrixXf cov_matrix_inverse_x(3,3);
	cov_matrix_inverse_x << cov_matrix_inverse(0,0), cov_matrix_inverse(0,1), cov_matrix_inverse(0,2),
							cov_matrix_inverse(1,0), cov_matrix_inverse(1,1), cov_matrix_inverse(1,2),
							cov_matrix_inverse(2,0), cov_matrix_inverse(2,1), cov_matrix_inverse(2,2);
	Eigen::MatrixXf diff_from_mu(3,1);
	Eigen::MatrixXf diff_from_mu_t(3,1);
	diff_from_mu << pos_data[0] - mu[0], pos_data[1] - mu[1], pos_data[2] - mu[2];
	diff_from_mu_t = diff_from_mu;
	diff_from_mu_t.transposeInPlace();
	Eigen::MatrixXf eigen_dot_value(1,1);
	eigen_dot_value << diff_from_mu_t * cov_matrix_inverse_x * diff_from_mu;
	float dot_value = eigen_dot_value(0,0);
	float distribution_density = pow(2*M_PI, -1.5) * pow(cov_matrix_determinant, -0.5) * exp(-0.5 * dot_value);
	/* std::cout << "distribution_density = " << distribution_density << std::endl; */

	return distribution_density;
}


Eigen::Matrix3f XmeansClustering::covariance_matrix(CloudIPtr ci_class)
{
	int ci_size = ci_class->points.size();
	Eigen::Vector3f sum = Eigen::Vector3f::Zero();
	for(auto& position : ci_class->points){
		sum[0] += position.x;
		sum[1] += position.y;
		sum[2] += position.z;
	}
	Eigen::Vector3f mu = sum / ci_size;
	
	float sigma_xx = 0.0;
	float sigma_yy = 0.0;
	float sigma_zz = 0.0;
	float sigma_xy = 0.0;
	float sigma_xz = 0.0;
	float sigma_yz = 0.0;
	for(auto& position : ci_class->points){
		sigma_xx += my_pow(position.x - mu[0]);
		sigma_yy += my_pow(position.y - mu[1]);
		sigma_zz += my_pow(position.z - mu[2]);
		sigma_xy += (position.x - mu[0]) * (position.y - mu[1]);
		sigma_xz += (position.x - mu[0]) * (position.z - mu[2]);
		sigma_yz += (position.y - mu[1]) * (position.z - mu[2]);
	}
	sigma_xx /= ci_size;
	sigma_yy /= ci_size;
	sigma_zz /= ci_size;
	sigma_xy /= ci_size;
	sigma_xz /= ci_size;
	sigma_yz /= ci_size;

	Eigen::Matrix3f cov_matrix;
	cov_matrix << sigma_xx, sigma_xy, sigma_xz
				, sigma_xy, sigma_yy, sigma_yz
				, sigma_xz, sigma_yz, sigma_zz;

	return cov_matrix;
}


float XmeansClustering::bic_calculation(bool dash, CloudIPtr i_j_std_class_ex)
{
	/* std::cout << "bic_calculation" << std::endl; */
	int ci_num;
	float bic;
	float alpha;
	float beta;
	std::vector<Eigen::Vector3f> mu_list;
	std::vector<Eigen::Vector3f> sum_list;
	std::vector<CloudIPtr> ci_class_list;
	std::vector<Eigen::Matrix3f> cov_list;
	std::vector<float> likelihood_list;

	if(!dash){
		ci_num = 1;
	}else{
		ci_num = 2;
	}

	for(int ci = 0; ci < ci_num; ci++){
		Eigen::Vector3f mu;
		Eigen::Vector3f sum = Eigen::Vector3f::Zero();
		Eigen::Matrix3f cov;

		CloudIPtr ci_class {new CloudI};
		ci_class->points.resize(0);
		for(auto& position : i_j_std_class_ex->points){
			if((int)position.intensity == ci){
				CloudIPtr tmp_pt {new CloudI};
				tmp_pt->points.resize(1);
				tmp_pt->points[0].x = position.x;
				tmp_pt->points[0].y = position.y;
				tmp_pt->points[0].z = position.z;
				tmp_pt->points[0].intensity = position.intensity;
				*ci_class += *tmp_pt;
			}
		}
		ci_class_list.push_back(ci_class);

		for(auto& position : ci_class->points){
			sum[0] += position.x;
			sum[1] += position.y;
			sum[2] += position.z;
		}
		sum_list.push_back(sum);
		
		mu = sum / (float)i_j_std_class_ex->points.size();
		mu_list.push_back(mu);

		cov = covariance_matrix(ci_class);
		cov_list.push_back(cov);
	}

	if(!dash){
		float likelihood = 1.0;
		for(auto& position : i_j_std_class_ex->points){
			Eigen::Vector3f pos_data;
			pos_data << position.x, position.y, position.z;
			likelihood *= density_function(ci_class_list[0], pos_data, mu_list[0]);
			/* std::cout << "solo likelihood = " << likelihood << std::endl; */
		}
		bic = -2 * log(likelihood) + 6 * log(i_j_std_class_ex->points.size());
	}else{
		float mu_diff_norm = (mu_list[0] - mu_list[1]).norm();
		float mu_diff_norm_pow = my_pow(mu_diff_norm);
		beta = sqrt(mu_diff_norm_pow / (cov_list[0].determinant() + cov_list[1].determinant()) );
		std::cout << "beta = " << beta << std::endl;
		float lower_probability = std_normal_distribution_integral(beta);
		alpha = 0.5 / lower_probability;
		std::cout << "alpha = " << alpha << std::endl;

		for(int ci = 0; ci < 2; ci++){
			float likelihood = 1.0;
			CloudIPtr tmp_pos {new CloudI};
			tmp_pos = ci_class_list[ci];
			for(auto& position : tmp_pos->points){
				Eigen::Vector3f pos_data;
				pos_data << position.x, position.y, position.z;
				likelihood *= density_function(ci_class_list[ci], pos_data, mu_list[0]);
			}
			/* std::cout << "likelihood[" << ci << "] = " << likelihood << std::endl; */
			likelihood_list.push_back(likelihood);
		}

		bic = -2 * (i_j_std_class_ex->points.size() * log(alpha) + log(likelihood_list[0]) +  log(likelihood_list[1])) + 12.0 * log(ci_class_list[0]->points.size());
	}

	return bic;
}


int XmeansClustering::randomization(int num)
{
	/* std::cout << "randomization" << std::endl; */

	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_int_distribution<> rand_kpp(0, num);

	return rand_kpp(mt);
}


float XmeansClustering::std_normal_distribution_integral(float beta)
{
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_real_distribution<> rand(-10.0, 10.0);
	float answer = 0.0;
	float min = 99.9;
	int n_counter = 0;

	for(int n = 0; n < N_; n++){
		float result = rand(mt);
		float result_pow = my_pow(result);
		if(result < beta){
			answer += exp(-0.5 * result_pow) / sqrt(2 * M_PI);
			n_counter++;
		}
		
		if(min > result){
			min = result;
		}
	}
	
	answer *= (beta - min) / n_counter;

	return answer;
}


float XmeansClustering::my_pow(float arg)
{
	/* std::cout << "my_pow" << std::endl; */

	return arg * arg;
}
