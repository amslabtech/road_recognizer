#include "road_recognizer/velodyne_differentiator.h"

VelodyneDifferentiator::VelodyneDifferentiator(void)
:local_nh("~")
{
    local_nh.param("LAYER_NUM", LAYER_NUM, {32});
    local_nh.param("DISTANCE_THRESHOLD", DISTANCE_THRESHOLD, {0.3});
    local_nh.param("SECOND_DIFFERENTIAL_THRESHOLD", SECOND_DIFFERENTIAL_THRESHOLD, {0.01});

    derivative_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_differentiator/first", 1);
    second_derivative_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_differentiator/second", 1);
    edge_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_differentiator/edge", 1);
    velodyne_sub = nh.subscribe("/velodyne_points", 1, &VelodyneDifferentiator::velodyne_callback, this);

    std::cout << "=== velodyne differentiator ===" << std::endl;
    std::cout << "LAYER_NUM: " << LAYER_NUM << std::endl;
    std::cout << "DISTANCE_THRESHOLD: " << DISTANCE_THRESHOLD << std::endl;
    std::cout << "SECOND_DIFFERENTIAL_THRESHOLD: " << SECOND_DIFFERENTIAL_THRESHOLD << std::endl;
}

void VelodyneDifferentiator::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "callback" << std::endl;
    double start_time = ros::Time::now().toSec();
    pcl::PointCloud<pcl::PointXYZI>::Ptr vp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *vp);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr vp_n(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::copyPointCloud(*vp, *vp_n);
    size_t size = vp->points.size();

    #pragma omp parallel for
    for(size_t i=0;i<size;i++){
        // size_t ring_index = get_ring_index_from_firing_order(i % LAYER_NUM);

        Eigen::Vector3d point(vp->points[i].x, vp->points[i].y, vp->points[i].z);
        if(point.norm() < 0.1){
            continue;
        }

        if((int)i + LAYER_NUM >= (int)size - 1){
            continue;
        }
        size_t next_index = i + LAYER_NUM;
        Eigen::Vector3d next_point(vp->points[next_index].x, vp->points[next_index].y, vp->points[next_index].z);
        if(next_point.norm() < 0.1){
            continue;
        }

        if((int)i - LAYER_NUM < 0){
            continue;
        }
        size_t prev_index = i - LAYER_NUM;
        Eigen::Vector3d prev_point(vp->points[prev_index].x, vp->points[prev_index].y, vp->points[prev_index].z);
        if(prev_point.norm() < 0.1){
            continue;
        }

        double distance_surface = (next_point.segment(0, 2) - prev_point.segment(0, 2)).norm();
        if(fabs(distance_surface) > DISTANCE_THRESHOLD){
            continue;
        }
        double derivative = (next_point(2) - prev_point(2));// / distance_surface;

        vp_n->points[i].curvature = fabs(derivative);
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr vp_n_2(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::copyPointCloud(*vp_n, *vp_n_2);
    #pragma omp parallel for
    for(size_t i=0;i<size;i++){
        Eigen::Vector3d point(vp->points[i].x, vp->points[i].y, vp->points[i].z);
        if(point.norm() < 0.1){
            continue;
        }

        if((int)i + LAYER_NUM >= (int)size - 1){
            continue;
        }
        size_t next_index = i + LAYER_NUM;
        Eigen::Vector3d next_point(vp->points[next_index].x, vp->points[next_index].y, vp->points[next_index].z);
        if(next_point.norm() < 0.1){
            continue;
        }

        if((int)i - LAYER_NUM < 0){
            continue;
        }
        size_t prev_index = (i - LAYER_NUM + size) % size;
        Eigen::Vector3d prev_point(vp->points[prev_index].x, vp->points[prev_index].y, vp->points[prev_index].z);
        if(prev_point.norm() < 0.1){
            continue;
        }

        double distance_surface = (next_point.segment(0, 2) - point.segment(0, 2)).norm();
        if(fabs(distance_surface) > DISTANCE_THRESHOLD){
            continue;
        }
        double derivative = (vp_n->points[next_index].curvature - vp_n->points[prev_index].curvature);// / distance_surface;

        vp_n_2->points[i].curvature = fabs(derivative);
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr vp_edge(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PassThrough<pcl::PointXYZINormal> pass;
    pass.setInputCloud(vp_n_2);
    pass.setFilterFieldName("curvature");
    pass.setFilterLimits(0.0, SECOND_DIFFERENTIAL_THRESHOLD);
    pass.setFilterLimitsNegative(true);
    pass.filter(*vp_edge);
    std::cout << vp_edge->points.size() << " points is extracted" << std::endl;

    sensor_msgs::PointCloud2 f_d_cloud;
    pcl::toROSMsg(*vp_n, f_d_cloud);
    derivative_cloud_pub.publish(f_d_cloud);
    sensor_msgs::PointCloud2 s_d_cloud;
    pcl::toROSMsg(*vp_n_2, s_d_cloud);
    second_derivative_cloud_pub.publish(s_d_cloud);
    sensor_msgs::PointCloud2 edge_cloud;
    pcl::toROSMsg(*vp_edge, edge_cloud);
    edge_cloud_pub.publish(edge_cloud);
    std::cout << "time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
}

size_t VelodyneDifferentiator::get_ring_index_from_firing_order(size_t order)
{
    if(order % 2){
        return order / 2 + LAYER_NUM / 2;
    }else{
        return order / 2;
    }
}

void VelodyneDifferentiator::process(void)
{
    ros::spin();
}
