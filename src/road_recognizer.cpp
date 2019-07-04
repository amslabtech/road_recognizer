#include "road_recognizer/road_recognizer.h"

RoadRecognizer::RoadRecognizer(void)
:local_nh("~")
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("NORMAL_ESTIMATION_RADIUS", NORMAL_ESTIMATION_RADIUS, {0.03});
    local_nh.param("LEAF_SIZE", LEAF_SIZE, {0.1});
    local_nh.param("OUTLIER_REMOVAL_K", OUTLIER_REMOVAL_K, {50});
    local_nh.param("OUTLIER_REMOVAL_THRESHOLD", OUTLIER_REMOVAL_THRESHOLD, {1.0});
    local_nh.param("CURVATURE_THRESHOLD", CURVATURE_THRESHOLD, {0.1});

    curvature_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/curvature", 1);
    downsampled_cloud_pub = local_nh.advertise<sensor_msgs::PointCloud2>("cloud/downsampled", 1);
    obstacles_sub = nh.subscribe("/velodyne_obstacles", 1, &RoadRecognizer::obstacles_callback, this);
    ground_sub = nh.subscribe("/velodyne_clear", 1, &RoadRecognizer::ground_callback, this);

    obstacles_cloud = CloudXYZIPtr(new CloudXYZI);
    ground_cloud = CloudXYZIPtr(new CloudXYZI);
    cloud_normals = CloudXYZINPtr(new CloudXYZIN);
    obstacles_cloud_updated = false;
    ground_cloud_updated = false;

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "NORMAL_ESTIMATION_RADIUS: " << NORMAL_ESTIMATION_RADIUS << std::endl;;
    std::cout << "LEAF_SIZE: " << LEAF_SIZE << std::endl;
    std::cout << "OUTLIER_REMOVAL_K: " << OUTLIER_REMOVAL_K << std::endl;
    std::cout << "OUTLIER_REMOVAL_THRESHOLD: " << OUTLIER_REMOVAL_THRESHOLD << std::endl;
    std::cout << "CURVATURE_THRESHOLD: " << CURVATURE_THRESHOLD << std::endl;
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
            std::cout << "after normal estimation cloud size: " << cloud_normals->points.size() << std::endl;

            CloudXYZINPtr cloud(new CloudXYZIN);
            cloud->header = ground_cloud->header;
            int size = cloud_normals->points.size();
            cloud->points.resize(size);

            #pragma omp parallel for
            for(int i=0;i<size;i++){
                cloud->points[i].x = ground_cloud->points[i].x;
                cloud->points[i].y = ground_cloud->points[i].y;
                cloud->points[i].z = ground_cloud->points[i].z;
                cloud->points[i].intensity = ground_cloud->points[i].intensity;
                cloud->points[i].normal_x = cloud_normals->points[i].normal_x;
                cloud->points[i].normal_y = cloud_normals->points[i].normal_y;
                cloud->points[i].normal_z = cloud_normals->points[i].normal_z;
                cloud->points[i].curvature = cloud_normals->points[i].curvature;
            }

            std::cout << "--- passthrough filter ---" << std::endl;
            pcl::PassThrough<PointXYZIN> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("curvature");
            pass.setFilterLimits(0, CURVATURE_THRESHOLD);
            pass.setFilterLimitsNegative(true);
            pass.filter(*cloud);
            std::cout << "after passthrough filter cloud size: " << cloud->points.size() << std::endl;

            sensor_msgs::PointCloud2 cloud1;
            pcl::toROSMsg(*ground_cloud, cloud1);
            downsampled_cloud_pub.publish(cloud1);
            sensor_msgs::PointCloud2 cloud2;
            pcl::toROSMsg(*cloud, cloud2);
            curvature_cloud_pub.publish(cloud2);

            obstacles_cloud_updated = false;
            ground_cloud_updated = false;

            std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
