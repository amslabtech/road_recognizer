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
			if(CLUSTERING_METHOD_ == kmeans){
				cells.affiliation[i][j] = randomization(kmeans, 2); // k-means initialize
			}else if(CLUSTERING_METHOD_ == kmeans_pp){
				cells.affiliation[i][j] = 1; // k-means++ initialize
			}
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


int XmeansClustering::randomization(bool method, int num)
{
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_int_distribution<> rand_k(1, num);
	std::uniform_int_distribution<> rand_kpp(0, num);

	if(method == kmeans){
		return rand_k(mt);
	}else if(method == kmeans_pp){
		return	rand_kpp(mt);
	}
}


void XmeansClustering::partitional_optimization(int k, int ci)
{
	float all_center_move = 0.0;
	Eigen::Vector3f eigen_zero = Eigen::Vector3f::Zero();
	std::vector<int> class_counter;
	std::vector<Eigen::Vector3f> centers;
	std::vector<Eigen::Vector3f> pre_centers;
	std::vector<Eigen::Vector3f> coordinate_sums;
	for(int k_ = 0; k_ < k; k++){
		coordinate_sums.push_back(eigen_zero);
		class_counter.push_back(0);
		if(CLUSTERING_METHOD_ == kmeans){
			Eigen::Vector3f eigen_random = Eigen::Vector3f::Random();
			pre_centers.push_back(eigen_random);
		}else if(CLUSTERING_METHOD_ == kmeans_pp){
			bool get_initial_centroid = false;
			while(!get_initial_centroid){
				float rnd_i = randomization(kmeans_pp, WIDTH_DIVISION_NUM_);
				float rnd_j = randomization(kmeans_pp, HEIGHT_DIVISION_NUM_);
				if(point_counter[rnd_i][rnd_j] > 0){
					get_initial_centroid = true;
				}
				Eigen::Vector3f initial_center;
				initial_center << (float)rnd_i, (float)rnd_j, cells.intensity_std_deviation[rnd_i][rnd_j];
				pre_centers.push_back(initial_center);
			}
		}
		CloudPtr initial_pc {new Cloud};
		initial_pc->points.resize(0);
		grid_points.push_back(initial_pc);
	}

	while(all_center_move > EPS_){
		// Calculate each of class's center, and make each of class's positin(by point cloud)
		for(int k_ = 0; k_ < k; k++){
			coordinate_sums[k_] = eigen_zero;
			class_counter[k_] = 0;
			grid_points[k_]->points.clear();
			for(int i = 0; i < WIDTH_DIVISION_NUM_; i++){
				for(int j = 0; j < HEIGHT_DIVISION_NUM_; j++){
					CloudPtr tmp_grid_point {new Cloud};
					tmp_grid_point->points.resize(1);
					tmp_grid_point->points[0].x = (float)i;
					tmp_grid_point->points[0].y = (float)j;
					tmp_grid_point->points[0].z = cells.intensity_std_deviation[i][j];
					Eigen::Vector3f eigen_tmp_point;
					eigen_tmp_point << (float)i, (float)j, cells.intensity_std_deviation[i][j];
					if(k_+1 == cells.affiliation[i][j]){
						*grid_points[k_] += *tmp_grid_point;
						coordinate_sums[k_] += eigen_tmp_point;
						class_counter[k_] += 1;
					}
				}
			}
			centers[k_] = coordinate_sums[k_] / class_counter[k_];
			Eigen::Vector3f eigen_center_tmp = centers[k_] - pre_centers[k_];
			all_center_move += eigen_center_tmp.norm();
			pre_centers[k_] = centers[k_];
		}

		// Reregister each of cells.affiliation[i][j]
		for(int i = 0; i < WIDTH_DIVISION_NUM_; i++){
			for(int j = 0; j < HEIGHT_DIVISION_NUM_; j++){
				float min_range = (float)(WIDTH_DIVISION_NUM_ * HEIGHT_DIVISION_NUM_);
				for(int k_ = 0; k_ < k; k_++){
					if(point_counter[i][j] > 0){
						Eigen::Vector3f eigen_tmp_point;
						eigen_tmp_point << (float)i, (float)j, cells.intensity_std_deviation[i][j];
						Eigen::Vector3f coordinate_distance_from_center = eigen_tmp_point - centers[k_];
						if(min_range > coordinate_distance_from_center.norm()){
							min_range = coordinate_distance_from_center.norm();
							cells.affiliation[i][j] = k_+1;
						}
					}
				}
			}
		}
	}
}


void XmeansClustering::xmeans_clustering(int k)
{
	float bic;
	float bic_dash;
	int ci = 0;
	while(1){
		partitional_optimization(k, ci);
	}
}


float XmeansClustering::density_function(int c)
{
	float distribution_density = 0;

	return distribution_density;
}

float XmeansClustering::my_pow(float arg){
	return arg * arg;
}
