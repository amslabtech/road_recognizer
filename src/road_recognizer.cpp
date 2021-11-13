#include "road_recognizer/road_recognizer.h"

RoadRecognizer::RoadRecognizer(void)
:local_nh("~"), road_stored_cloud_sub(nh, "cloud/road/stored", 10), obstacles_cloud_sub(nh, "/velodyne_obstacles", 10), sync(sync_subs(10), road_stored_cloud_sub, obstacles_cloud_sub)
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("LEAF_SIZE", LEAF_SIZE, {0.1});
    local_nh.param("OUTLIER_REMOVAL_K", OUTLIER_REMOVAL_K, {50});
    local_nh.param("OUTLIER_REMOVAL_THRESHOLD", OUTLIER_REMOVAL_THRESHOLD, {1.0});
    local_nh.param("MAX_RANDOM_SAMPLE_SIZE", MAX_RANDOM_SAMPLE_SIZE, {5000});
    local_nh.param("RANDOM_SAMPLE_RATIO", RANDOM_SAMPLE_RATIO, {0.25});
    local_nh.param("ENABLE_VISUALIZATION", ENABLE_VISUALIZATION, {false});
    local_nh.param("BEAM_ANGLE_NUM", BEAM_ANGLE_NUM, {720});
    local_nh.param("MAX_BEAM_RANGE", MAX_BEAM_RANGE, {20});
    local_nh.param("MIN_BEAM_RANGE", MIN_BEAM_RANGE, {1.0});
    local_nh.param("RANSAC_DISTANCE_THRESHOLD", RANSAC_DISTANCE_THRESHOLD, {0.20});
    local_nh.param("RANSAC_MIN_LINE_LENGTH_THRESHOLD", RANSAC_MIN_LINE_LENGTH_THRESHOLD, {3.0});
    local_nh.param("RANSAC_MIN_LINE_DENSITY_THRESHOLD", RANSAC_MIN_LINE_DENSITY_THRESHOLD, {1.0});
    local_nh.param("EUCLIDEAN_CLUSTERING_TOLERANCE", EUCLIDEAN_CLUSTERING_TOLERANCE, {1.0});
    local_nh.param("MAX_ROAD_EDGE_DIRECTION_DIFFERENCE", MAX_ROAD_EDGE_DIRECTION_DIFFERENCE, {0.1});
    local_nh.param("MIN_ROAD_WIDTH", MIN_ROAD_WIDTH, {1.0});
    local_nh.param("BEAM_MEDIAN_N", BEAM_MEDIAN_N, {9});
    local_nh.param("LINE_NORMAL_MEAN_INNER_PRODUCT_THRESHOLD", LINE_NORMAL_MEAN_INNER_PRODUCT_THRESHOLD, {0.5});
    local_nh.param("RADIUS_FOR_2D_NORMAL", RADIUS_FOR_2D_NORMAL, {1.0});

    downsampled_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/downsampled", 1);
    filtered_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/filtered", 1);
    beam_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/beam_model", 1);
    linear_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/linear", 1);
    beam_array_pub = nh.advertise<std_msgs::Float64MultiArray>("beam_array", 1);
    line_markers_pub = local_nh.advertise<visualization_msgs::MarkerArray>("road/edge_lines", 1);
    road_pub = nh.advertise<amsl_navigation_msgs::RoadArray>("road", 1);

    sync.registerCallback(boost::bind(&RoadRecognizer::callback, this, _1, _2));

    filtered_cloud = CloudXYZINPtr(new CloudXYZIN);

    if(ENABLE_VISUALIZATION){
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer);
        viewer->setBackgroundColor(0, 0, 0);
    }

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "LEAF_SIZE: " << LEAF_SIZE << std::endl;
    std::cout << "OUTLIER_REMOVAL_K: " << OUTLIER_REMOVAL_K << std::endl;
    std::cout << "OUTLIER_REMOVAL_THRESHOLD: " << OUTLIER_REMOVAL_THRESHOLD << std::endl;
    std::cout << "MAX_RANDOM_SAMPLE_SIZE: " << MAX_RANDOM_SAMPLE_SIZE << std::endl;
    std::cout << "RANDOM_SAMPLE_RATIO: " << RANDOM_SAMPLE_RATIO << std::endl;
    std::cout << "ENABLE_VISUALIZATION: " << ENABLE_VISUALIZATION << std::endl;
    std::cout << "BEAM_ANGLE_NUM: " << BEAM_ANGLE_NUM << std::endl;
    std::cout << "MAX_BEAM_RANGE: " << MAX_BEAM_RANGE << std::endl;
    std::cout << "MIN_BEAM_RANGE: " << MIN_BEAM_RANGE << std::endl;
    std::cout << "RANSAC_DISTANCE_THRESHOLD: " << RANSAC_DISTANCE_THRESHOLD << std::endl;
    std::cout << "RANSAC_MIN_LINE_LENGTH_THRESHOLD: " << RANSAC_MIN_LINE_LENGTH_THRESHOLD << std::endl;
    std::cout << "RANSAC_MIN_LINE_DENSITY_THRESHOLD: " << RANSAC_MIN_LINE_DENSITY_THRESHOLD << std::endl;
    std::cout << "EUCLIDEAN_CLUSTERING_TOLERANCE: " << EUCLIDEAN_CLUSTERING_TOLERANCE << std::endl;
    std::cout << "MAX_ROAD_EDGE_DIRECTION_DIFFERENCE: " << MAX_ROAD_EDGE_DIRECTION_DIFFERENCE << std::endl;
    std::cout << "MIN_ROAD_WIDTH: " << MIN_ROAD_WIDTH << std::endl;
    std::cout << "BEAM_MEDIAN_N: " << BEAM_MEDIAN_N << std::endl;
    std::cout << "LINE_NORMAL_MEAN_INNER_PRODUCT_THRESHOLD: " << LINE_NORMAL_MEAN_INNER_PRODUCT_THRESHOLD << std::endl;
    std::cout << "RADIUS_FOR_2D_NORMAL: " << RADIUS_FOR_2D_NORMAL << std::endl;
}

void RoadRecognizer::callback(const sensor_msgs::PointCloud2ConstPtr& msg_road_stored_cloud, const sensor_msgs::PointCloud2ConstPtr& msg_obstacles_cloud)
{
    std::cout << "=== road recognizer ===" << std::endl;
    double start = ros::Time::now().toSec();

    CloudXYZINPtr road_cloud(new CloudXYZIN);
    pcl::fromROSMsg(*msg_road_stored_cloud, *road_cloud);

    CloudXYZIPtr obstacles_cloud_without_normal(new CloudXYZI);
    CloudXYZINPtr obstacles_cloud(new CloudXYZIN);
    // pcl::fromROSMsg(*msg_obstacles_cloud, *obstacles_cloud);
    pcl::fromROSMsg(*msg_obstacles_cloud, *obstacles_cloud_without_normal);
    pcl::copyPointCloud(*obstacles_cloud_without_normal, *obstacles_cloud);

    *road_cloud += *obstacles_cloud;

    int cloud_size = road_cloud->points.size();
    std::cout << "road cloud size: " <<  cloud_size << std::endl;

    std::cout << "before cloud size: " << road_cloud->points.size() << std::endl;
    // random sampling
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    int random_sample_size = std::min(cloud_size * RANDOM_SAMPLE_RATIO, (double)MAX_RANDOM_SAMPLE_SIZE);

    CloudXYZINPtr downsampled_cloud(new CloudXYZIN);
    downsampled_cloud->points.resize(random_sample_size);
    for(int i=0;i<random_sample_size;i++){
        int index =  cloud_size * dist(mt);
        downsampled_cloud->points[i] = road_cloud->points[index];
    }
    downsampled_cloud->header = road_cloud->header;
    std::cout << "after voxel grid cloud size: " << downsampled_cloud->points.size() << std::endl;

    /*
    std::cout << "before cloud size: " << downsampled_cloud->points.size() << std::endl;
    pcl::StatisticalOutlierRemoval<PointXYZIN> sor;
    sor.setInputCloud(downsampled_cloud);
    sor.setMeanK(OUTLIER_REMOVAL_K);
    sor.setStddevMulThresh(OUTLIER_REMOVAL_THRESHOLD);
    sor.filter(*filtered_cloud);
    std::cout << "after statistical outlier removal cloud size: " << filtered_cloud->points.size() << std::endl;
    */

    std::cout << "--- beam model ---" << std::endl;
    CloudXYZPtr beam_cloud(new CloudXYZ);
    beam_cloud->header = road_cloud->header;
    //get_beam_cloud(filtered_cloud, beam_cloud);
    get_beam_cloud(downsampled_cloud, beam_cloud);
    std::cout << "beam cloud size: " << beam_cloud->points.size() << std::endl;

    extract_lines(beam_cloud);

    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(*downsampled_cloud, cloud1);
    downsampled_pub.publish(cloud1);

    // sensor_msgs::PointCloud2 cloud2;
    // pcl::toROSMsg(*filtered_cloud, cloud2);
    // filtered_pub.publish(cloud2);

    if(ENABLE_VISUALIZATION){
        visualize_cloud();
    }

    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void RoadRecognizer::visualize_cloud(void)
{
    viewer->removeAllPointClouds();

    pcl::PointCloud<pcl::PointXYZ>::Ptr view_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*filtered_cloud, *view_cloud);
    viewer->addPointCloud(view_cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cloud");

    pcl::PointCloud<pcl::PointNormal>::Ptr view_normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*filtered_cloud, *view_normal);
    viewer->addPointCloudNormals<pcl::PointNormal>(view_normal, 1, 1, "normal");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "normal");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normal");

    viewer->spinOnce();
}

void RoadRecognizer::extract_lines(const CloudXYZPtr input_cloud)
{
    std::cout << "extract lines" << std::endl;
    CloudXYZPtr cloud(new CloudXYZ);
    CloudXYZINPtr cloud_n(new CloudXYZIN);
    std::vector<CloudXYZPtr> linear_clouds;
    pcl::copyPointCloud(*input_cloud, *cloud);
    std::cout << "cloud: " << cloud->points.size() << std::endl;

    estimate_normal_2d(cloud, cloud_n);
    cloud_n->header = input_cloud->header;
    std::cout << "cloud_n: " << cloud_n->points.size() << std::endl;

    sensor_msgs::PointCloud2 cloud3;
    pcl::toROSMsg(*cloud_n, cloud3);
    beam_cloud_pub.publish(cloud3);
    get_linear_clouds(cloud_n, linear_clouds);

    publish_linear_clouds(linear_clouds);

    std::vector<LineInformation> line_list;
    get_line_information_from_linear_clouds(linear_clouds, line_list);

    std::vector<pcl::PointIndices> cluster_indices;
    get_clustered_lines(line_list, cluster_indices);

    make_and_publish_line_marker(cluster_indices, line_list, pcl_conversions::fromPCL(input_cloud->header));

    amsl_navigation_msgs::RoadArray road_array;
    road_array.header = pcl_conversions::fromPCL(input_cloud->header);
    std::vector<int> added_indices;
    int line_num = cluster_indices.size();
    for(int i=0;i<line_num;i++){
        if(std::find(added_indices.begin(), added_indices.end(), i) == added_indices.end()){
            int index = cluster_indices[i].indices[0];
            LineInformation line = line_list[index];
            double min_distance = 1e3;
            int min_index = i;
            for(int j=i+1;j<line_num;j++){
                if(std::find(added_indices.begin(), added_indices.end(), j) == added_indices.end()){
                    int index2 = cluster_indices[j].indices[0];
                    LineInformation line2 = line_list[index2];
                    if(fabs(std::get<2>(line) - std::get<2>(line2)) < MAX_ROAD_EDGE_DIRECTION_DIFFERENCE){
                        double distance = fabs(std::get<3>(line) - std::get<3>(line2));
                        if(distance > MIN_ROAD_WIDTH){
                            //std::cout << "line " << i << " and " << j << " can be pair" << std::endl;
                            //std::cout << "distance is " << distance << "[m]" << std::endl;;
                            if(distance < min_distance){
                                min_distance = distance;
                                min_index = j;
                            }
                        }
                    }
                }
            }
            amsl_navigation_msgs::Road road;
            road.direction = std::get<2>(line);
            road.distance_to_right = std::get<3>(line);
            road.length = std::get<4>(line);
            added_indices.push_back(i);
            if(min_index != i){
                road.width = min_distance;
                added_indices.push_back(min_index);
                if(road.distance_to_right < 0){
                    double right = road.distance_to_right + road.width;
                    road.distance_to_right = right;
                }
            }
            road_array.roads.push_back(road);
        }
    }
    road_pub.publish(road_array);
}

template<typename PointT>
double RoadRecognizer::get_distance(const PointT& p0, const PointT& p1)
{
    Eigen::Vector3d v0(p0.x, p0.y, p0.z);
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    return (v0 - v1).norm();
}

double RoadRecognizer::get_length_from_linear_cloud(const CloudXYZPtr& cloud)
{
    std::vector<double> distance_list(cloud->points.size());
    Eigen::Vector2d begin_vector(cloud->points[0].x, cloud->points[0].y);
    Eigen::Vector2d end_vector(cloud->points.back().x, cloud->points.back().y);
    Eigen::Vector2d direction_vector = end_vector - begin_vector;
    double line_direction = atan2(direction_vector(1), direction_vector(0));
    double max_d_p = 0;
    double max_d_n = 0;
    for(const auto& pt : cloud->points){
        Eigen::Vector2d point(pt.x, pt.y);
        Eigen::Vector2d relative_point = point - begin_vector;
        double angle = line_direction - atan2(relative_point(1), relative_point(0));
        double d = relative_point.norm() * cos(angle);
        if(fabs(angle) > M_PI){
            d = -d;
        }
        if(max_d_p <= d){
            max_d_p = d;
        }
        if(max_d_n >= d){
            max_d_n = d;
        }
    }
    return fabs(max_d_p) + fabs(max_d_n);
}

double RoadRecognizer::get_length_from_linear_cloud(const CloudXYZPtr& cloud, Eigen::Vector2d& p0, Eigen::Vector2d& p1)
{
    std::vector<double> distance_list(cloud->points.size());
    Eigen::Vector2d begin_vector(cloud->points[0].x, cloud->points[0].y);
    Eigen::Vector2d end_vector(cloud->points.back().x, cloud->points.back().y);
    Eigen::Vector2d direction_vector = end_vector - begin_vector;
    double line_direction = atan2(direction_vector(1), direction_vector(0));
    double max_d_p = 0;
    double max_d_n = 0;
    for(const auto& pt : cloud->points){
        Eigen::Vector2d point(pt.x, pt.y);
        Eigen::Vector2d relative_point = point - begin_vector;
        double angle = line_direction - atan2(relative_point(1), relative_point(0));
        double d = relative_point.norm() * cos(angle);
        if(fabs(angle) > M_PI){
            d = -d;
        }
        if(max_d_p <= d){
            max_d_p = d;
            p0 << pt.x, pt.y;
        }
        if(max_d_n >= d){
            max_d_n = d;
            p1 << pt.x, pt.y;
        }
    }
    return fabs(max_d_p) + fabs(max_d_n);
}

void RoadRecognizer::publish_linear_clouds(const std::vector<CloudXYZPtr>& linear_clouds)
{
    int linear_cloud_size = linear_clouds.size();
    std::cout << "linear cloud num: " << linear_cloud_size << std::endl;
    // coloring cloud for visualization
    std::cout << "coloring cloud for visualization" << std::endl;
    if(linear_cloud_size > 0){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i=0;i<linear_cloud_size;i++){
            pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
            pcl::copyPointCloud(*linear_clouds[i], *hsv_cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(auto& pt : hsv_cloud->points){
                pt.h = 255.0 * i / (double)linear_cloud_size;
                pt.s = 1.0;
                pt.v = 1.0;
                pcl::PointXYZRGB rgb_pt;
                pcl::PointXYZHSVtoXYZRGB(pt, rgb_pt);
                rgb_cloud->points.push_back(rgb_pt);
            }
            *colored_cloud += *rgb_cloud;
        }
        colored_cloud->header = linear_clouds[0]->header;
        sensor_msgs::PointCloud2 _cloud;
        pcl::toROSMsg(*colored_cloud, _cloud);
        linear_cloud_pub.publish(_cloud);
    }
}

void RoadRecognizer::get_linear_clouds(const CloudXYZINPtr input_cloud, std::vector<CloudXYZPtr>& linear_clouds)
{
    while(input_cloud->points.size() > 0){
        //std::cout << "ransac" << std::endl;
        std::cout << "remaining cloud size: " << input_cloud->points.size() << std::endl;
        pcl::SampleConsensusModelLine<PointXYZIN>::Ptr model_l(new pcl::SampleConsensusModelLine<PointXYZIN>(input_cloud));
        pcl::RandomSampleConsensus<PointXYZIN> ransac(model_l);
        bool computed = false;
        if(input_cloud->points.size() > 1){
            ransac.setDistanceThreshold(RANSAC_DISTANCE_THRESHOLD);
            computed = ransac.computeModel();
        }
        if(computed){
            std::vector<int> inliers;
            ransac.getInliers(inliers);
            CloudXYZPtr linear_cloud(new CloudXYZ);
            linear_cloud->header = input_cloud->header;
            pcl::copyPointCloud(*input_cloud, inliers, *linear_cloud);
            int linear_cloud_size = linear_cloud->size();
            std::cout << "linear cloud size: " << linear_cloud->size() << std::endl;
            if(linear_cloud_size > 1){
                Eigen::VectorXf coeff;
                ransac.getModelCoefficients(coeff);
                Eigen::Vector2f line_vector = coeff.segment(3, 2);
                // std::cout << coeff.transpose() << std::endl;
                // std::cout << line_vector.transpose() << std::endl;
                double cost = 0;
                for(int i=0;i<linear_cloud_size;i++){
                    Eigen::Vector2f normal(input_cloud->points[inliers[i]].normal_x, input_cloud->points[inliers[i]].normal_y);
                    // std::cout << i << ": " << normal.transpose() << std::endl;
                    cost += fabs(normal.dot(line_vector));
                }
                cost /= (double)linear_cloud_size;
                std::cout << "cost: " << cost << std::endl;
                if(cost < LINE_NORMAL_MEAN_INNER_PRODUCT_THRESHOLD){
                    double line_length = get_length_from_linear_cloud(linear_cloud);
                    std::cout << "line length: " << line_length << "[m]" << std::endl;
                    if(line_length > RANSAC_MIN_LINE_LENGTH_THRESHOLD){
                        if(linear_cloud->size() / line_length > RANSAC_MIN_LINE_DENSITY_THRESHOLD){
                            // new linear cloud
                            //std::cout << "new linear cloud" << std::endl;
                            linear_clouds.push_back(linear_cloud);
                            //std::cout << "linear cloud num: " << linear_clouds.size() << std::endl;
                        }else{
                            std::cout << "line length is NOT denser than threshold!" << std::endl;
                        }
                    }else{
                        std::cout << "line length is NOT longer than threshold!" << std::endl;
                    }
                }else{
                    std::cout << "difference between line and cloud normals is NOT smaller than threshold!" << std::endl;
                }
            }else{
                std::cout << "cloud size is NOT biggner than threshold!" << std::endl;
                break;
            }
            // remove inliers from cloud
            //std::cout << "remove inliers from cloud" << std::endl;
            int size = input_cloud->points.size();
            std::vector<int> outliers;
            outliers.reserve(size);
            for(int i=0;i<size;i++){
                if(std::find(inliers.begin(), inliers.end(), i) == inliers.end()){
                    outliers.push_back(i);
                }
            }
            pcl::copyPointCloud(*input_cloud, outliers, *input_cloud);
        }else{
            // no line was detected
            //std::cout << "no line was detected" << std::endl;
            break;
        }
    }
}

void RoadRecognizer::make_and_publish_line_marker(const std::vector<pcl::PointIndices>& cluster_indices, const std::vector<LineInformation>& line_list, const std_msgs::Header& header)
{
    visualization_msgs::MarkerArray line_markers;
    int cluster_num = cluster_indices.size();
    static int last_cluster_num = 0;
    int i = 0;
    for(;i<cluster_num;i++){
        auto line_data = line_list[cluster_indices[i].indices[0]];
        visualization_msgs::Marker line_marker;
        line_marker.header = header;
        line_marker.ns = "road_recognizer";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.scale.x = 0.05;
        line_marker.color.g = 1.0;
        line_marker.color.a = 1.0;
        line_marker.lifetime = ros::Duration(0);
        geometry_msgs::Point p;
        p.x = std::get<0>(line_data)(0);
        p.y = std::get<0>(line_data)(1);
        line_marker.points.push_back(p);
        p.x = std::get<1>(line_data)(0);
        p.y = std::get<1>(line_data)(1);
        line_marker.points.push_back(p);
        line_markers.markers.push_back(line_marker);
    }
    for(;i<last_cluster_num;i++){
        visualization_msgs::Marker line_marker;
        line_marker.header = header;
        line_marker.ns = "road_recognizer";
        line_marker.id = i;
        line_marker.action = visualization_msgs::Marker::DELETE;
        line_markers.markers.push_back(line_marker);
    }
    last_cluster_num = cluster_num;
    line_markers_pub.publish(line_markers);
}

void RoadRecognizer::get_clustered_lines(const std::vector<LineInformation>& line_list, std::vector<pcl::PointIndices>& cluster_indices)
{
    CloudXYZPtr line_points(new CloudXYZ);
    for(const auto& line : line_list){
        Eigen::Vector2d vec = std::get<6>(line);
        PointXYZ pt(vec(0), vec(1), 0.0);
        line_points->points.push_back(pt);
    }
    line_points->height = 1;
    line_points->width = line_points->points.size();
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
    // std::cout << "before morishita" << std::endl;
    std::cout << "line_points size: " << line_points->points.size() << std::endl;
    if(line_points->points.size() != 0){
        tree->setInputCloud(line_points);
        // std::cout << "after morishita" << std::endl;
        pcl::EuclideanClusterExtraction<PointXYZ> ec;
        ec.setClusterTolerance(EUCLIDEAN_CLUSTERING_TOLERANCE);
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(line_points->points.size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(line_points);
        ec.extract(cluster_indices);
        int count = 0;
        for(const auto& indices : cluster_indices){
            std::cout << "cluster" << count << std::endl;
            for(const auto& i : indices.indices){
                std::cout << line_points->points[i] << std::endl;
            }
            count++;
        }
    }
}

void RoadRecognizer::get_line_information_from_linear_clouds(const std::vector<CloudXYZPtr>& linear_clouds, std::vector<LineInformation>& line_list)
{
    std::cout << "lines" << std::endl;
    for(const auto& linear_cloud : linear_clouds){
        Eigen::Vector2d p0;
        Eigen::Vector2d p1;
        double length = get_length_from_linear_cloud(linear_cloud, p0, p1);
        Eigen::Vector2d direction_vector = p1 - p0;
        if(fabs(direction_vector(0)) > 1e-6){
            // ax + by + c = 0
            double a = direction_vector(1) / direction_vector(0);
            double b = -1;
            double c = p0(1) - a * p0(0);
            double distance_from_origin = fabs(c) / sqrt(a * a + b * b);
            double direction = atan(a);
            constexpr double PI_2 = M_PI * 0.5;
            if(direction > PI_2){
                direction -= M_PI;
            }else if(direction < -PI_2){
                direction += M_PI;
            }
            double perpendicular_angle = (c >= 0 ? PI_2 : -PI_2) + direction;
            Eigen::Vector2d perpendicular_intersection_point(distance_from_origin * cos(perpendicular_angle), distance_from_origin * sin(perpendicular_angle));
            if(c >= 0){
                // if the line is placed on the left side of the robot
                distance_from_origin = -distance_from_origin;
            }
            auto line = std::make_tuple(p0, p1, direction, distance_from_origin, length, perpendicular_angle, perpendicular_intersection_point, linear_cloud->points.size());
            line_list.push_back(line);
            std::cout << std::get<2>(line) << "[rad], " << std::get<3>(line) << "[m], " << std::get<4>(line) << "[m], " << std::get<5>(line) << "[rad], " << std::get<7>(line) << std::endl;
            std::cout << "point: " << std::get<6>(line).transpose() << std::endl;
        }
    }
}

void RoadRecognizer::get_beam_cloud(const CloudXYZINPtr& input_cloud, CloudXYZPtr& beam_cloud)
{
    const double ANGLE_INCREMENT = 2.0 * M_PI / (double)BEAM_ANGLE_NUM;
    std::vector<double> beam_list(BEAM_ANGLE_NUM, MAX_BEAM_RANGE);
    for(const auto& pt : input_cloud->points){
        double distance = sqrt(pt.x * pt.x + pt.y * pt.y);
        if(distance < MIN_BEAM_RANGE){
            continue;
        }
        double angle = atan2(pt.y, pt.x);
        int index = (angle + M_PI) / ANGLE_INCREMENT;
        if(0 <= index && index < BEAM_ANGLE_NUM){
            if(beam_list[index] > distance){
                beam_list[index] = distance;
            }
        }
    }
    // publish beam list
    std_msgs::Float64MultiArray beam_array;
    for(const auto& distance : beam_list){
        beam_array.data.push_back(distance);
    }
    beam_array_pub.publish(beam_array);

    std::vector<double> filtered_beam;
    //apply_median_filter(beam_list, filtered_beam);
    apply_mean_filter(beam_list, filtered_beam);

    for(int i=0;i<BEAM_ANGLE_NUM;i++){
        if(filtered_beam[i] < MAX_BEAM_RANGE){
            double angle = i * ANGLE_INCREMENT - M_PI;
            PointXYZ pt;
            pt.x = filtered_beam[i] * cos(angle);
            pt.y = filtered_beam[i] * sin(angle);
            beam_cloud->points.push_back(pt);
            //std::cout << i << ": " << median_beam[i] << "[m]" << std::endl;
        }
    }
    beam_cloud->height = 1;
    beam_cloud->width = beam_cloud->points.size();
}

void RoadRecognizer::apply_median_filter(const std::vector<double>& beam, std::vector<double>& filtered_beam)
{
    static const int N_2 = BEAM_MEDIAN_N * 0.5;
    std::vector<double> median_beam(BEAM_ANGLE_NUM, MAX_BEAM_RANGE);
    for(int i=0;i<BEAM_ANGLE_NUM;i++){
        std::vector<double> beams;
        double median = 0;
        int count = 0;
        if(beam[i] >= MAX_BEAM_RANGE){
            median_beam[i] = beam[i];
            continue;
        }
        for(int j=i-N_2;j<=i+N_2;j++){
            double range = beam[(j + BEAM_ANGLE_NUM) % BEAM_ANGLE_NUM];
            if(range < MAX_BEAM_RANGE){
                beams.push_back(range);
                count++;
            }
        }
        if(count > 0){
            std::sort(beams.begin(), beams.end());
            // if(count > 1){
            //     if(count % 2 == 0){
            //         count--;
            //     }
            //     median = beams[count / 2 + 1];
            // }else{
            //     median = beams[0];
            // }
            //std::cout << "count: " << count << ", " << median << "[m]" << std::endl;
            if(count > 2){
                median = beams[count / 2 + 1];
            }else{
                median = MAX_BEAM_RANGE;
            }
        }else{
            median = MAX_BEAM_RANGE;
        }
        median_beam[i] = median;
        //std::cout << i << ": " << median_beam[i] << "[m]" << std::endl;
        //std::cout << beam[i] << ", " << median_beam[i] << std::endl;
    }
    filtered_beam = median_beam;
}

void RoadRecognizer::apply_mean_filter(const std::vector<double>& beam, std::vector<double>& filtered_beam)
{
    static const int N_2 = BEAM_MEDIAN_N * 0.5;
    std::vector<double> mean_beam(BEAM_ANGLE_NUM, MAX_BEAM_RANGE);
    for(int i=0;i<BEAM_ANGLE_NUM;i++){
        std::vector<double> beams;
        double mean = 0;
        int count = 0;
        if(beam[i] >= MAX_BEAM_RANGE){
            mean_beam[i] = beam[i];
            continue;
        }
        for(int j=i-N_2;j<=i+N_2;j++){
            double range = beam[(j + BEAM_ANGLE_NUM) % BEAM_ANGLE_NUM];
            if(range < MAX_BEAM_RANGE){
                beams.push_back(range);
                count++;
            }
        }
        if(count > 0){
            std::sort(beams.begin(), beams.end());
            if(count > 2){
                for(int j=0;j<count;j++){
                    mean += beams[j];
                }
                mean /= (double)count;
            }else{
                mean = MAX_BEAM_RANGE;
            }
        }else{
            mean = MAX_BEAM_RANGE;
        }
        mean_beam[i] = mean;
        //std::cout << i << ": " << mean_beam[i] << "[m]" << std::endl;
        //std::cout << beam[i] << ", " << mean_beam[i] << std::endl;
    }
    filtered_beam = mean_beam;
}

void RoadRecognizer::estimate_normal_2d(const CloudXYZPtr& input_cloud, CloudXYZINPtr& output_cloud)
{
    pcl::KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud);

    int input_size = input_cloud->points.size();
    std::vector<int> computed_indices;
    computed_indices.reserve(input_size);
    for(int i=0;i<input_size;i++){
        computed_indices.push_back(i);
        std::vector<int> indices;
        std::vector<float> distances;
        kdtree.radiusSearch(input_cloud->points[i], RADIUS_FOR_2D_NORMAL, indices, distances);
        // add itself
        indices.push_back(i);
        int num = indices.size();
        if(num > 2){
            double sum_x = 0;
            double sum_y = 0;
            for(int j=0;j<num;j++){
                sum_x += input_cloud->points[indices[j]].x;
                sum_y += input_cloud->points[indices[j]].y;
                computed_indices.push_back(indices[j]);
            }
            double ave_x = sum_x / (double)num;
            double ave_y = sum_y / (double)num;
            double sum_sigma_x = 0;
            double sum_sigma_y = 0;
            double sum_sigma_xy = 0;
            for(int j=0;j<num;j++){
                sum_sigma_x += (input_cloud->points[indices[j]].x - ave_x) * (input_cloud->points[indices[j]].x - ave_x);
                sum_sigma_y += (input_cloud->points[indices[j]].y - ave_y) * (input_cloud->points[indices[j]].y - ave_y);
                sum_sigma_xy = (input_cloud->points[indices[j]].x - ave_x) * (input_cloud->points[indices[j]].y - ave_y);
            }
            double sigma_x = sum_sigma_x / (double)num;
            double sigma_y = sum_sigma_y / (double)num;
            double sigma_xy = sum_sigma_xy / (double)num;
            Eigen::Matrix2d mat;
            mat << sigma_x, sigma_xy,
                   sigma_xy, sigma_y;
            Eigen::EigenSolver<Eigen::Matrix2d> eigensolver(mat);
            Eigen::Vector2d e_values = eigensolver.eigenvalues().real();
            Eigen::Matrix2d e_vectors = eigensolver.eigenvectors().real();
            Eigen::Vector2d second = e_vectors.col((e_values(0) > e_values(1)) ? 1 : 0);
            PointXYZIN pt;
            pt.x = input_cloud->points[i].x;
            pt.y = input_cloud->points[i].y;
            pt.z = input_cloud->points[i].z;
            pt.normal_x = second(0);
            pt.normal_y = second(1);
            pt.normal_z = 0;
            output_cloud->points.push_back(pt);
        }
    }
}

bool RoadRecognizer::check_line_with_normal(const CloudXYZINPtr& cloud, std::vector<int>& indices)
{
    return true;
}

void RoadRecognizer::process(void)
{
    ros::spin();
}
