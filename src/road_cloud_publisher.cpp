#include "road_recognizer/road_cloud_publisher.h"

RoadCloudPublisher::RoadCloudPublisher(void)
:local_nh("~")
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("NORMAL_ESTIMATION_RADIUS", NORMAL_ESTIMATION_RADIUS, {0.03});
    local_nh.param("LEAF_SIZE", LEAF_SIZE, {0.1});
    local_nh.param("OUTLIER_REMOVAL_K", OUTLIER_REMOVAL_K, {50});
    local_nh.param("OUTLIER_REMOVAL_THRESHOLD", OUTLIER_REMOVAL_THRESHOLD, {1.0});
    local_nh.param("CURVATURE_THRESHOLD", CURVATURE_THRESHOLD, {0.1});
    local_nh.param("INTENSITY_UPPER_THRESHOLD", INTENSITY_UPPER_THRESHOLD, {15});
    local_nh.param("INTENSITY_LOWER_THRESHOLD", INTENSITY_LOWER_THRESHOLD, {1});
    local_nh.param("HEIGHT_THRESHOLD", HEIGHT_THRESHOLD, {0});
    local_nh.param("MAX_RANDOM_SAMPLE_SIZE", MAX_RANDOM_SAMPLE_SIZE, {5000});
    local_nh.param("RANDOM_SAMPLE_RATIO", RANDOM_SAMPLE_RATIO, {0.25});
    local_nh.param("IS_OTSU", IS_OTSU , {true});
    local_nh.param("RANGE_MAX", RANGE_MAX, {20.0});
    local_nh.param("RANGE_DIVISION_NUM", RANGE_DIVISION_NUM, {20});
    local_nh.param("THETA_DIVISION_NUM", THETA_DIVISION_NUM, {360});
    local_nh.param("OTSU_BINARY_SEPARATION_THRESHOLD", OTSU_BINARY_SEPARATION_THRESHOLD, {0.8});
    local_nh.param("OTSU_BINARY_DIFF_FROM_AVR_THRESHOLD", OTSU_BINARY_DIFF_FROM_AVR_THRESHOLD, {3.0});
    local_nh.param("OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD", OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD, {999.9});
    local_nh.param("VAR_BETWEEN_THRESHOLD", VAR_BETWEEN_THRESHOLD, {100.0});
    local_nh.param("CHEAT_INTENSITY_WIDTH", CHEAT_INTENSITY_WIDTH, {5.0});
    local_nh.param("IGNORE_INTENSITY_DEFAULT", IGNORE_INTENSITY_DEFAULT, {false});
    local_nh.param("USE_NORMAL_Z_AS_CURVATURE", USE_NORMAL_Z_AS_CURVATURE, {false});

    curvature_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/curvature", 1);
    intensity_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/intensity", 1);
    downsampled_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/downsampled", 1);
    road_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud/road", 1);
    obstacles_sub = nh.subscribe("/velodyne_obstacles", 1, &RoadCloudPublisher::obstacles_callback, this);
    ground_sub = nh.subscribe("/velodyne_clear", 1, &RoadCloudPublisher::ground_callback, this);
    ignore_intensity_sub = nh.subscribe("/task/ignore_intensity", 1, &RoadCloudPublisher::ignore_intensity_callback, this);

    obstacles_cloud = CloudXYZINPtr(new CloudXYZIN);
    ground_cloud = CloudXYZINPtr(new CloudXYZIN);
    curvature_cloud = CloudXYZINPtr(new CloudXYZIN);
    intensity_cloud = CloudXYZINPtr(new CloudXYZIN);
    road_cloud = CloudXYZINPtr(new CloudXYZIN);

    obstacles_cloud_updated = false;
    ground_cloud_updated = false;
	ignore_intensity_flag = IGNORE_INTENSITY_DEFAULT;

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "NORMAL_ESTIMATION_RADIUS: " << NORMAL_ESTIMATION_RADIUS << std::endl;;
    std::cout << "LEAF_SIZE: " << LEAF_SIZE << std::endl;
    std::cout << "OUTLIER_REMOVAL_K: " << OUTLIER_REMOVAL_K << std::endl;
    std::cout << "OUTLIER_REMOVAL_THRESHOLD: " << OUTLIER_REMOVAL_THRESHOLD << std::endl;
    std::cout << "CURVATURE_THRESHOLD: " << CURVATURE_THRESHOLD << std::endl;
    std::cout << "INTENSITY_UPPER_THRESHOLD: " << INTENSITY_UPPER_THRESHOLD << std::endl;
    std::cout << "INTENSITY_LOWER_THRESHOLD: " << INTENSITY_LOWER_THRESHOLD << std::endl;
    std::cout << "HEIGHT_THRESHOLD: " << HEIGHT_THRESHOLD << std::endl;
    std::cout << "MAX_RANDOM_SAMPLE_SIZE: " << MAX_RANDOM_SAMPLE_SIZE << std::endl;
    std::cout << "RANDOM_SAMPLE_RATIO: " << RANDOM_SAMPLE_RATIO << std::endl;
    std::cout << "IS_OTSU: " << IS_OTSU << std::endl;
    std::cout << "IGNORE_INTENSITY_DEFAULT: " << IGNORE_INTENSITY_DEFAULT<< std::endl;
    std::cout << "USE_NORMAL_Z_AS_CURVATURE: " << USE_NORMAL_Z_AS_CURVATURE << std::endl;
}

void RoadCloudPublisher::obstacles_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *obstacles_cloud);
    obstacles_cloud_updated = true;
}

void RoadCloudPublisher::ground_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *ground_cloud);
    road_cloud->header = ground_cloud->header;
    ground_cloud_updated = true;
}

void RoadCloudPublisher::ignore_intensity_callback(const std_msgs::BoolConstPtr& msg)
{
	ignore_intensity_flag = msg->data;
}

void RoadCloudPublisher::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        if(obstacles_cloud_updated && ground_cloud_updated){
            double start = ros::Time::now().toSec();
            std::cout << "=== road cloud publisher ===" << std::endl;

            std::cout << "--- downsampling ---" << std::endl;
            downsample();

            // std::cout << "--- normal estimation ---" << std::endl;
            // estimate_normal();

            std::cout << "--- passthrough filter ---" << std::endl;
            filter_curvature();
            std::cout << "curvature cloud size: " << curvature_cloud->points.size() << std::endl;
			*road_cloud = *curvature_cloud;
            if(!ignore_intensity_flag){
                filter_intensity();
                std::cout << "intensity cloud size: " << intensity_cloud->points.size() << std::endl;
                *road_cloud += *intensity_cloud;
            }
            // *road_cloud = *intensity_cloud;

            filter_height();
            std::cout << "after passthrough filter cloud size: " << road_cloud->points.size() << std::endl;

            publish_clouds();

            obstacles_cloud_updated = false;
            ground_cloud_updated = false;

            std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RoadCloudPublisher::publish_clouds(void)
{
    std::cout << "publish downsampled ground cloud" << std::endl;
    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(*ground_cloud, cloud1);
    downsampled_cloud_pub.publish(cloud1);
    ground_cloud->points.clear();

    std::cout << "publish curvature cloud" << std::endl;
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(*curvature_cloud, cloud2);
    curvature_cloud_pub.publish(cloud2);
    curvature_cloud->points.clear();

    std::cout << "publish intensity cloud" << std::endl;
    sensor_msgs::PointCloud2 cloud3;
    pcl::toROSMsg(*intensity_cloud, cloud3);
    intensity_cloud_pub.publish(cloud3);
    intensity_cloud->points.clear();

    std::cout << "publish road cloud" << std::endl;
    sensor_msgs::PointCloud2 cloud4;
    pcl::toROSMsg(*road_cloud, cloud4);
    road_cloud_pub.publish(cloud4);
    road_cloud->points.clear();
}

void RoadCloudPublisher::downsample(void)
{
    std::cout << "before cloud size: " << ground_cloud->points.size() << std::endl;
    // random sampling
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    int cloud_size = ground_cloud->points.size();
    int random_sample_size = std::min(cloud_size * RANDOM_SAMPLE_RATIO, (double)MAX_RANDOM_SAMPLE_SIZE);

    CloudXYZINPtr downsampled_cloud(new CloudXYZIN);
    downsampled_cloud->points.resize(random_sample_size);
    downsampled_cloud->header = ground_cloud->header;
    for(int i=0;i<random_sample_size;i++){
        int index =  cloud_size * dist(mt);
        downsampled_cloud->points[i] = ground_cloud->points[index];
    }
    *ground_cloud = *downsampled_cloud;
    std::cout << "after random sampling size: " << ground_cloud->points.size() << std::endl;
}

void RoadCloudPublisher::estimate_normal(void)
{
    CloudXYZPtr xyz_cloud(new CloudXYZ);
    pcl::copyPointCloud(*ground_cloud, *xyz_cloud);
    pcl::NormalEstimationOMP<PointXYZ, PointN> ne;
    ne.setInputCloud(xyz_cloud);
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());
    //pcl::KdTreeFLANN<PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<PointXYZ>());
    ne.setSearchMethod(tree);

    CloudNPtr cloud_normals(new CloudN);
    ne.setRadiusSearch(NORMAL_ESTIMATION_RADIUS);

    ne.compute(*cloud_normals);
    std::cout << "after normal estimation road_cloud size: " << cloud_normals->points.size() << std::endl;

    road_cloud->header = ground_cloud->header;
    int size = cloud_normals->points.size();
    road_cloud->points.resize(size);

    #pragma omp parallel for
    for(int i=0;i<size;i++){
        road_cloud->points[i].x = ground_cloud->points[i].x;
        road_cloud->points[i].y = ground_cloud->points[i].y;
        road_cloud->points[i].z = ground_cloud->points[i].z;
        road_cloud->points[i].intensity = ground_cloud->points[i].intensity;
        road_cloud->points[i].normal_x = cloud_normals->points[i].normal_x;
        road_cloud->points[i].normal_y = cloud_normals->points[i].normal_y;
        road_cloud->points[i].normal_z = cloud_normals->points[i].normal_z;
        road_cloud->points[i].curvature = cloud_normals->points[i].curvature;
    }
}

void RoadCloudPublisher::filter_curvature(void)
{
    pcl::PassThrough<PointXYZIN> pass;
    pass.setInputCloud(ground_cloud);
    if(!USE_NORMAL_Z_AS_CURVATURE){
        pass.setFilterFieldName("curvature");
        pass.setFilterLimits(0, CURVATURE_THRESHOLD);
        pass.setFilterLimitsNegative(true);
    }else{
        pass.setFilterFieldName("normal_z");
        pass.setFilterLimits(-CURVATURE_THRESHOLD, CURVATURE_THRESHOLD);
    }
    curvature_cloud->header = ground_cloud->header;
    curvature_cloud->points.clear();
    pass.filter(*curvature_cloud);
}

void RoadCloudPublisher::filter_intensity(void)
{
    if(IS_OTSU){
        IntensityPartition intensity_partition(RANGE_DIVISION_NUM, THETA_DIVISION_NUM, RANGE_MAX, VAR_BETWEEN_THRESHOLD, OTSU_BINARY_SEPARATION_THRESHOLD, OTSU_BINARY_DIFF_FROM_AVR_THRESHOLD, OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD, INTENSITY_LOWER_THRESHOLD, CHEAT_INTENSITY_WIDTH);
        intensity_cloud = intensity_partition.execution(ground_cloud);
        intensity_cloud->header = ground_cloud->header;
    }else{
        pcl::PassThrough<PointXYZIN> intensity_pass;
        intensity_pass.setInputCloud(ground_cloud);
        intensity_pass.setFilterFieldName("intensity");
        intensity_pass.setFilterLimits(INTENSITY_LOWER_THRESHOLD, INTENSITY_UPPER_THRESHOLD);
        intensity_cloud->header = ground_cloud->header;
        intensity_pass.filter(*intensity_cloud);
        intensity_cloud->width = intensity_cloud->points.size();
    }
}

void RoadCloudPublisher::filter_height(void)
{
    pcl::PassThrough<PointXYZIN> height_pass;
    height_pass.setInputCloud(road_cloud);
    height_pass.setFilterFieldName("z");
    height_pass.setFilterLimits(HEIGHT_THRESHOLD, 100);
    height_pass.setFilterLimitsNegative(true);
    road_cloud->header = ground_cloud->header;
    height_pass.filter(*road_cloud);
}
