#include "road_recognizer/xmeans_clustering.h"


XmeansClustering::XmeansClustering(bool clustering_method, int width_division_num, int height_division_num, double max_width, double max_height, double eps)
{
	kmeans = false;
	kmeans_pp = true;
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
	initialization();
	grid_partition(imput_pc);
	xmeans_clustering(2);

	return xmeans_pc;
}


void XmeansClustering::initialization(void)
{
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
				}
			}
			cells.affiliation[i][j] = randomization(true, 2); // k-means initialize
		}
	}


	i_j_std_class->points.resize(0);
	for(int i = 0; i < WIDTH_DIVISION_NUM_; i++){
		for(int j = 0; j < HEIGHT_DIVISION_NUM_; j++){
			if(point_counter[i][j] > 0){
				for(int s = 0; s < point_counter[i][j]; s++){
					sum_diff_pow[i][j] += my_pow(cells.point_cloud[i][j]->points[s].intensity - cells.intensity_average[i][j]);
				}
				cells.intensity_std_deviation[i][j] = sqrt(sum_diff_pow[i][j] / point_counter[i][j]);
				CloudIPtr tmp_pt {new CloudI};
				tmp_pt->points.resize(1);
				tmp_pt->points[0].x = (float)i;
				tmp_pt->points[0].y = (float)j;
				tmp_pt->points[0].z = cells.intensity_std_deviation[i][j];
				tmp_pt->points[0].intensity = cells.affiliation[i][j];
				*i_j_std_class += *tmp_pt;
				tmp_pt->points[0].intensity = 1.0;
				*i_j_std_solo_class += *tmp_pt;
			}
		}
	}
}


int XmeansClustering::randomization(bool method, int num)
{
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_int_distribution<> rand_k(1, num);
	std::uniform_int_distribution<> rand_kpp(0, num);

	if(method){
		return rand_k(mt);
	}else{
		return	rand_kpp(mt);
	}
}


void XmeansClustering::partitional_optimization(int k, int ci, CloudIPtr i_j_std_class_ex)
{
	float all_center_move = 0.0;
	Eigen::Vector3f eigen_zero = Eigen::Vector3f::Zero();
	for(int k_ = 0; k_ < k; k_++){
		coordinate_sums.push_back(eigen_zero);
		class_counter.push_back(0);
		if(CLUSTERING_METHOD_ == kmeans){
			Eigen::Vector3f eigen_random = Eigen::Vector3f::Random();
			pre_centers.push_back(eigen_random);
		}else if(CLUSTERING_METHOD_ == kmeans_pp){
			while(1){
				float rnd_idx = randomization(kmeans_pp, WIDTH_DIVISION_NUM_ * HEIGHT_DIVISION_NUM_ - 1);
				if(i_j_std_class_ex->points[rnd_idx].intensity == ci){
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
		for(int k_ = ci - 1; k_ < k + ci - 1; k_++){
			coordinate_sums[k_] = eigen_zero;
			class_counter[k_] = 0;
			for(auto& position : i_j_std_class_ex->points){
				if(position.intensity == ci){
					Eigen::Vector3f eigen_tmp_point;
					eigen_tmp_point << position.x, position.y, position.z;
					coordinate_sums[k_] += eigen_tmp_point;
					class_counter[k_] += 1;
				}
			}
			centers[k_] = coordinate_sums[k_] / class_counter[k_];
			Eigen::Vector3f eigen_center_tmp = centers[k_] - pre_centers[k_];
			all_center_move += eigen_center_tmp.norm();
			pre_centers[k_] = centers[k_];
		}

		// Reregister each of cells.affiliation[i][j]
		for(auto& position : i_j_std_class_ex->points){
			int count = 0;
			float min_range = WIDTH_DIVISION_NUM_ * HEIGHT_DIVISION_NUM_;
			for(int k_ = ci - 1; k_ < k + ci - 1; k_++){
				Eigen::Vector3f eigen_tmp_point;
				eigen_tmp_point << position.x, position.y, position.z;
				Eigen::Vector3f coordinate_distance_from_center = eigen_tmp_point - centers[k_];
				if(min_range > coordinate_distance_from_center.norm()){
					min_range = coordinate_distance_from_center.norm();
					position.intensity = (float)(k_ + 1);
				}
			}
			count++;
		}
	}
}


void XmeansClustering::xmeans_clustering(int k)
{
	float bic;
	float bic_dash;
	int ci = 1;
	bic = bic_calculation(i_j_std_solo_class);
	i_j_std_class_list.push_back(partitional_optimization(k, ci, i_j_std_class)); // class 1, 2

}


float XmeansClustering::density_function(CloudIPtr i_j_std_class_ex, Eigen::Vector3f pos_data, Eigen::Vector3f mu)
{
	Matrix3f cov_matrix = covariance_matrix(i_j_std_class_ex);
	Eigen::MatrixXf diff_from_mu(1,3);
	diff_from_mu << pos_data[0] - mu[0], pos_data[1] - mu[1], pos_data[2] - mu[2];
	float distribution_density = pow(2*M_PI, -1.5) * pow(cov_matrix.determinant(), -0.5) * exp(-0.5 * diff_from_mu.transposeInPlace() * cov_matrix.inverse() * diff_from_mu);

	return distribution_density;
}


float XmeansClustering::covariance_matrix(CloudIPtr i_j_std_class_ex)
{
	pcl::PCA<PointIVoxel> pca;
	pca.setInputCloud(i_j_std_class_ex);
	Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
	
	return eigen_vectors;
}


float XmeansClustering::bic_calculation(CloudIPtr i_j_std_class_ex)
{
	Eigen::Vector3f mu;
	Eigen::Vector3f sum;
	sum << 0.0, 0.0, 0.0;
	float L = 1.0;
	for(auto& position : i_j_std_class_ex->points){
		sum[0] += position.x;
		sum[1] += position.y;
		sum[2] += position.z;
	}
	mu = sum / (float)i_j_std_class_ex->points.size();
	
	for(auto& position : i_j_std_class_ex->points){
		Eigen::Vector3f pos_data;
		pos_data << position.x, position.y, position.z;
		L *= density_function(i_j_std_class_ex, pos_data, mu)
	}
}


float XmeansClustering::my_pow(float arg)
{
	return arg * arg;
}
