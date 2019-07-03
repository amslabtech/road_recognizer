#include "road_recognizer/road_recognizer.h"

RoadRecognizer::RoadRecognizer(void)
:local_nh("~")
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("NORMAL_ESTIMATION_RADIUS", NORMAL_ESTIMATION_RADIUS, {0.03});
    local_nh.param("LEAF_SIZE", LEAF_SIZE, {0.1});
    local_nh.param("OUTLIER_REMOVAL_K", OUTLIER_REMOVAL_K, {50});
    local_nh.param("OUTLIER_REMOVAL_THRESHOLD", OUTLIER_REMOVAL_THRESHOLD, {1.0});

    curvature_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/curvature", 1);
    downsampled_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/downsampled", 1);
    obstacles_sub = nh.subscribe("/velodyne_obstacles", 1, &RoadRecognizer::obstacles_callback, this);
    ground_sub = nh.subscribe("/velodyne_clear", 1, &RoadRecognizer::ground_callback, this);

    obstacles_cloud = CloudXYZIPtr(new CloudXYZI);
    ground_cloud = CloudXYZIPtr(new CloudXYZI);
    obstacles_cloud_updated = false;
    ground_cloud_updated = false;

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "NORMAL_ESTIMATION_RADIUS: " << NORMAL_ESTIMATION_RADIUS << std::endl;;
    std::cout << "LEAF_SIZE: " << LEAF_SIZE << std::endl;
    std::cout << "OUTLIER_REMOVAL_K: " << OUTLIER_REMOVAL_K << std::endl;
    std::cout << "OUTLIER_REMOVAL_THRESHOLD: " << OUTLIER_REMOVAL_THRESHOLD << std::endl;
}

void RoadRecognizer::obstacles_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *obstacles_cloud);
    obstacles_cloud_updated = true;
}

void RoadRecognizer::ground_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *ground_cloud);
    ground_cloud_updated = true;
}

void RoadRecognizer::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        if(obstacles_cloud_updated && ground_cloud_updated){
            double start = ros::Time::now().toSec();
            std::cout << "=== road recognizer ===" << std::endl;

            std::cout << "--- downsampling ---" << std::endl;
            std::cout << "before cloud size: " << ground_cloud->points.size() << std::endl;
            pcl::VoxelGrid<PointXYZI> vg;
            vg.setInputCloud(ground_cloud);
            vg.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
            vg.filter(*ground_cloud);
            std::cout << "after voxel grid cloud size: " << ground_cloud->points.size() << std::endl;
            pcl::StatisticalOutlierRemoval<PointXYZI> sor;
            sor.setInputCloud(ground_cloud);
            sor.setMeanK(OUTLIER_REMOVAL_K);
            sor.setStddevMulThresh(OUTLIER_REMOVAL_THRESHOLD);
            sor.filter(*ground_cloud);
            std::cout << "after statistical outlier removal cloud size: " << ground_cloud->points.size() << std::endl;

            std::cout << "--- normal estimation ---" << std::endl;
            pcl::NormalEstimationOMP<PointXYZI, PointXYZIN> ne;
            //pcl::NormalEstimation<PointXYZI, PointXYZIN> ne;
            ne.setInputCloud(ground_cloud);
            pcl::search::KdTree<PointXYZI>::Ptr tree(new pcl::search::KdTree<PointXYZI>());
            ne.setSearchMethod(tree);

            CloudXYZINPtr cloud_normals(new CloudXYZIN);
            ne.setRadiusSearch(NORMAL_ESTIMATION_RADIUS);

            ne.compute(*cloud_normals);
            std::cout << "after normal estimation cloud size: " << cloud_normals->points.size() << std::endl;

            /*
            for(const auto& pt : cloud_normals->points){
                std::cout << "(x,y,z): " << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
                std::cout << "(nx,ny,nz): " << pt.normal_x << ", " << pt.normal_y << ", " << pt.normal_z << std::endl;
                std::cout << "(curv): " << pt.curvature << std::endl;
            }
            */

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*ground_cloud, cloud);
            downsampled_cloud_pub.publish(cloud);
            sensor_msgs::PointCloud2 cloud2;
            pcl::toROSMsg(*cloud_normals, cloud2);
            curvature_cloud_pub.publish(cloud2);

            obstacles_cloud_updated = false;
            ground_cloud_updated = false;

            std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
