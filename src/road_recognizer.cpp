#include "road_recognizer/road_recognizer.h"

RoadRecognizer::RoadRecognizer(void)
:local_nh("~")
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
    local_nh.param("RANSAC_DISTANCE_THRESHOLD", RANSAC_DISTANCE_THRESHOLD, {0.20});
    local_nh.param("RANSAC_MIN_LINE_LENGTH_THRESHOLD", RANSAC_MIN_LINE_LENGTH_THRESHOLD, {3.0});
    local_nh.param("RANSAC_MIN_LINE_DENSITY_THRESHOLD", RANSAC_MIN_LINE_DENSITY_THRESHOLD, {1.0});

    downsampled_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/downsampled", 1);
    filtered_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/filtered", 1);
    beam_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/beam_model", 1);
    linear_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/linear", 1);
    beam_array_pub = nh.advertise<std_msgs::Float64MultiArray>("beam_array", 1);
    line_markers_pub = local_nh.advertise<visualization_msgs::MarkerArray>("road/edge_lines", 1);

    road_stored_cloud_sub = nh.subscribe("cloud/road/stored", 1, &RoadRecognizer::road_cloud_callback, this);

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
    std::cout << "RANSAC_DISTANCE_THRESHOLD: " << RANSAC_DISTANCE_THRESHOLD << std::endl;
    std::cout << "RANSAC_MIN_LINE_LENGTH_THRESHOLD: " << RANSAC_MIN_LINE_LENGTH_THRESHOLD << std::endl;
    std::cout << "RANSAC_MIN_LINE_DENSITY_THRESHOLD: " << RANSAC_MIN_LINE_DENSITY_THRESHOLD << std::endl;
}

void RoadRecognizer::road_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "=== road recognizer ===" << std::endl;
    double start = ros::Time::now().toSec();
    CloudXYZINPtr road_cloud(new CloudXYZIN);
    pcl::fromROSMsg(*msg, *road_cloud);
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

    std::cout << "before cloud size: " << downsampled_cloud->points.size() << std::endl;
    pcl::StatisticalOutlierRemoval<PointXYZIN> sor;
    sor.setInputCloud(downsampled_cloud);
    sor.setMeanK(OUTLIER_REMOVAL_K);
    sor.setStddevMulThresh(OUTLIER_REMOVAL_THRESHOLD);
    sor.filter(*filtered_cloud);
    std::cout << "after statistical outlier removal cloud size: " << filtered_cloud->points.size() << std::endl;

    std::cout << "--- beam model ---" << std::endl;
    const double ANGLE_INCREMENT = 2.0 * M_PI / (double)BEAM_ANGLE_NUM;
    std::vector<double> beam_list(BEAM_ANGLE_NUM, MAX_BEAM_RANGE);
    for(const auto& pt : filtered_cloud->points){
        double distance = sqrt(pt.x * pt.x + pt.y * pt.y);
        double angle = atan2(pt.y, pt.x);
        int index = (angle + M_PI) / ANGLE_INCREMENT;
        if(0 <= index && index < BEAM_ANGLE_NUM){
            if(beam_list[index] > distance){
                beam_list[index] = distance;
            }
        }
    }
    CloudXYZPtr beam_cloud(new CloudXYZ);
    beam_cloud->header = road_cloud->header;
    for(int i=0;i<BEAM_ANGLE_NUM;i++){
        if(beam_list[i] < MAX_BEAM_RANGE){
            double angle = i * ANGLE_INCREMENT - M_PI;
            PointXYZ pt;
            pt.x = beam_list[i] * cos(angle);
            pt.y = beam_list[i] * sin(angle);
            beam_cloud->points.push_back(pt);
        }
    }
    beam_cloud->height = 1;
    beam_cloud->width = beam_cloud->points.size();
    std::cout << "beam cloud size: " << beam_cloud->points.size() << std::endl;

    extract_lines(beam_cloud);

    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(*downsampled_cloud, cloud1);
    downsampled_pub.publish(cloud1);

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(*filtered_cloud, cloud2);
    filtered_pub.publish(cloud2);

    sensor_msgs::PointCloud2 cloud3;
    pcl::toROSMsg(*beam_cloud, cloud3);
    beam_cloud_pub.publish(cloud3);

    std_msgs::Float64MultiArray beam_array;
    for(const auto& distance : beam_list){
        beam_array.data.push_back(distance);
    }
    beam_array_pub.publish(beam_array);

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
    std::vector<CloudXYZPtr> linear_clouds;
    pcl::copyPointCloud(*input_cloud, *cloud);
    while(cloud->points.size() > 0){
        std::cout << "ransac" << std::endl;
        std::cout << "remaining cloud size: " << cloud->points.size() << std::endl;
        pcl::SampleConsensusModelLine<PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<PointXYZ>(cloud));
        pcl::RandomSampleConsensus<PointXYZ> ransac(model_l);
        ransac.setDistanceThreshold(RANSAC_DISTANCE_THRESHOLD);
        bool computed = ransac.computeModel();
        if(computed){
            std::vector<int> inliers;
            ransac.getInliers(inliers);
            CloudXYZPtr linear_cloud(new CloudXYZ);
            linear_cloud->header = input_cloud->header;
            pcl::copyPointCloud(*cloud, inliers, *linear_cloud);
            std::cout << "linear cloud size: " << linear_cloud->size() << std::endl;
            if(linear_cloud->size() > 1){
                double line_length = get_distance(linear_cloud->points[0], linear_cloud->points.back());
                std::cout << "line length: " << line_length << "[m]" << std::endl;
                if(line_length > RANSAC_MIN_LINE_LENGTH_THRESHOLD){
                    if(linear_cloud->size() / line_length > RANSAC_MIN_LINE_DENSITY_THRESHOLD){
                        // new linear cloud
                        std::cout << "new linear cloud" << std::endl;
                        linear_clouds.push_back(linear_cloud);
                        std::cout << "linear cloud num: " << linear_clouds.size() << std::endl;
                    }else{
                        std::cout << "line length is NOT denser than threshold!" << std::endl;
                    }
                }else{
                    std::cout << "line length is NOT longer than threshold!" << std::endl;
                }
            }else{
                std::cout << "cloud size is NOT longer than threshold!" << std::endl;
            }
            // remove inliers from cloud
            std::cout << "remove inliers from cloud" << std::endl;
            int size = cloud->points.size();
            std::vector<int> outliers;
            outliers.reserve(size);
            for(int i=0;i<size;i++){
                if(std::find(inliers.begin(), inliers.end(), i) == inliers.end()){
                    outliers.push_back(i);
                }
            }
            pcl::copyPointCloud(*cloud, outliers, *cloud);
        }else{
            // no line was detected
            std::cout << "no line was detected" << std::endl;
            break;
        }
    }

    publish_linear_clouds(linear_clouds);

    std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d, double, double, double, Eigen::Vector2d, int> > line_list;
    std::cout << "lines" << std::endl;
    for(const auto& linear_cloud : linear_clouds){
        Eigen::Vector2d p0(linear_cloud->points[0].x, linear_cloud->points[0].y);
        Eigen::Vector2d p1(linear_cloud->points.back().x, linear_cloud->points.back().y);
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
            double length = direction_vector.norm();
            auto line = std::make_tuple(p0, p1, direction, distance_from_origin, length, perpendicular_intersection_point, linear_cloud->points.size());
            line_list.push_back(line);
            std::cout << std::get<2>(line) << "[rad], " << std::get<3>(line) << "[m], " << std::get<4>(line) << "[m], " << perpendicular_angle << "[rad], " << std::get<6>(line) << std::endl;
            std::cout << "point: " << std::get<5>(line).transpose() << std::endl;
        }
    }
    // clustering lines
    CloudXYZPtr line_points(new CloudXYZ);
    for(const auto& line : line_list){
        Eigen::Vector2d vec = std::get<5>(line);
        PointXYZ pt(vec(0), vec(1), 0.0);
        line_points->points.push_back(pt);
    }
    line_points->height = 1;
    line_points->width = line_points->points.size();
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
    tree->setInputCloud(line_points);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance(1.0);
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
    // make marker
    visualization_msgs::MarkerArray line_markers;
    int cluster_num = cluster_indices.size();
    static int last_cluster_num = 0;
    int i = 0;
    for(;i<cluster_num;i++){
        auto line_data = line_list[cluster_indices[i].indices[0]];
        visualization_msgs::Marker line_marker;
        line_marker.header = pcl_conversions::fromPCL(input_cloud->header);
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
        line_marker.header = pcl_conversions::fromPCL(input_cloud->header);
        line_marker.ns = "road_recognizer";
        line_marker.id = i;
        line_marker.action = visualization_msgs::Marker::DELETE;
        line_markers.markers.push_back(line_marker);
    }
    last_cluster_num = cluster_num;
    line_markers_pub.publish(line_markers);
}

template<typename PointT>
double RoadRecognizer::get_distance(const PointT& p0, const PointT& p1)
{
    Eigen::Vector3d v0(p0.x, p0.y, p0.z);
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    return (v0 - v1).norm();
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

void RoadRecognizer::process(void)
{
    ros::spin();
}
