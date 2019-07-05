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

    downsampled_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/downsampled", 1);
    filtered_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/filtered", 1);
    beam_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/beam_model", 1);

    road_stored_cloud_sub = nh.subscribe("cloud/road/stored", 1, &RoadRecognizer::road_cloud_callback, this);

    filtered_cloud = CloudXYZINPtr(new CloudXYZIN);

    if(ENABLE_VISUALIZATION){
        viewer.setBackgroundColor(0, 0, 0);
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
    /*
    pcl::VoxelGrid<PointXYZIN> vg;
    vg.setInputCloud(road_cloud);
    vg.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    vg.filter(*downsampled_cloud);
    */
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
        //int index = (angle / M_PI + 1) * BEAM_ANGLE_NUM * 0.5;
        int index = (angle + M_PI) / ANGLE_INCREMENT;
        if(0 <= index && index < BEAM_ANGLE_NUM){
            if(beam_list[index] > distance){
                beam_list[index] = distance;
            }
        }
    }
    CloudXYZPtr beam_cloud(new CloudXYZ);
    beam_cloud->points.resize(BEAM_ANGLE_NUM);
    beam_cloud->header = road_cloud->header;
    for(int i=0;i<BEAM_ANGLE_NUM;i++){
        double angle = (i / (BEAM_ANGLE_NUM * 0.5) - 1) * M_PI;
        beam_cloud->points[i].x = beam_list[i] * cos(angle);
        beam_cloud->points[i].y = beam_list[i] * sin(angle);
    }
    std::cout << "beam cloud size: " << beam_cloud->points.size() << std::endl;

    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(*downsampled_cloud, cloud1);
    downsampled_pub.publish(cloud1);

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(*filtered_cloud, cloud2);
    filtered_pub.publish(cloud2);

    sensor_msgs::PointCloud2 cloud3;
    pcl::toROSMsg(*beam_cloud, cloud3);
    beam_cloud_pub.publish(cloud3);

    if(ENABLE_VISUALIZATION){
        visualize_cloud();
    }

    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void RoadRecognizer::visualize_cloud(void)
{
    viewer.removeAllPointClouds();

    pcl::PointCloud<pcl::PointXYZ>::Ptr view_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*filtered_cloud, *view_cloud);
    viewer.addPointCloud(view_cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cloud");

    pcl::PointCloud<pcl::PointNormal>::Ptr view_normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*filtered_cloud, *view_normal);
    viewer.addPointCloudNormals<pcl::PointNormal>(view_normal, 1, 1, "normal");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "normal");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normal");

    viewer.spinOnce();
}

void RoadRecognizer::process(void)
{
    ros::spin();
}
