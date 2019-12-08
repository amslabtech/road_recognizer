#include "road_recognizer/velodyne_differentiator.h"

GridState::GridState(void)
:min_z(1e6), max_z(0), avg_z(0), hit_count(0)
{
}

void GridState::set_height(double height)
{
    min_z = std::min(height, min_z);
    max_z = std::max(height, max_z);
    avg_z = (hit_count * avg_z + height) / (hit_count + 1);
    hit_count++;
}

VelodyneDifferentiator::VelodyneDifferentiator(void)
:local_nh("~")
{
    local_nh.param("LAYER_NUM", LAYER_NUM, {32});
    local_nh.param("DISTANCE_THRESHOLD", DISTANCE_THRESHOLD, {0.3});
    local_nh.param("SECOND_DIFFERENTIAL_THRESHOLD", SECOND_DIFFERENTIAL_THRESHOLD, {0.01});
    local_nh.param("VELODYNE_HEIGHT", VELODYNE_HEIGHT, {1.2});

    derivative_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_differentiator/first", 1);
    second_derivative_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_differentiator/second", 1);
    edge_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_differentiator/edge", 1);
    velodyne_sub = nh.subscribe("/velodyne_points", 1, &VelodyneDifferentiator::velodyne_callback, this);
    odom_sub = nh.subscribe("/odom", 1, &VelodyneDifferentiator::odom_callback, this);

    std::cout << "=== velodyne differentiator ===" << std::endl;
    std::cout << "LAYER_NUM: " << LAYER_NUM << std::endl;
    std::cout << "DISTANCE_THRESHOLD: " << DISTANCE_THRESHOLD << std::endl;
    std::cout << "SECOND_DIFFERENTIAL_THRESHOLD: " << SECOND_DIFFERENTIAL_THRESHOLD << std::endl;
    std::cout << "VELODYNE_HEIGHT: " << VELODYNE_HEIGHT << std::endl;
}

void VelodyneDifferentiator::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    std::cout << "odom callback" << std::endl;
    static Eigen::Vector2d last_pose = Eigen::Vector2d::Zero();
    static double last_yaw = 0;

    Eigen::Vector2d pose(msg->pose.pose.position.x, msg->pose.pose.position.y);
    double yaw = tf::getYaw(msg->pose.pose.orientation);

    if(!first_odom_flag){
        Eigen::Vector2d diff_pose = pose - last_pose;
        std::cout << diff_pose << std::endl;
        double diff_yaw = yaw - last_yaw + 2 * M_PI;
        diff_yaw = atan2(sin(diff_yaw), cos(diff_yaw));
        Eigen::Rotation2D<double> rotation(-last_yaw);
        diff_pose = rotation * diff_pose;
        dx = diff_pose(0);
        dy = diff_pose(1);
        dyaw = diff_yaw;
        std::cout << "dx, dy, dyaw: " << dx << ", " << dy << ", " << dyaw << std::endl;
    }else{
        first_odom_flag = false;
    }

    last_yaw = yaw;
    last_pose = pose;
}

void VelodyneDifferentiator::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "velodyne callback" << std::endl;
    if(first_odom_flag){
        std::cout << "odom has not been subscribed" << std::endl;
        return;
    }
    double start_time = ros::Time::now().toSec();
    pcl::PointCloud<pcl::PointXYZI>::Ptr vp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *vp);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr vp_n(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::copyPointCloud(*vp, *vp_n);
    size_t size = vp->points.size();

    const int WIDTH = 200;
    const double RESOLUTION = 0.10;
    std::vector<std::vector<GridState> > grid_states;
    grid_states.resize(WIDTH);
    for(auto& grids : grid_states){
        grids.resize(WIDTH);
    }
    static cv::Mat road_image(cv::Size(WIDTH, WIDTH), CV_64FC1, cv::Scalar(0));

    #pragma omp parallel for
    for(size_t i=0;i<size;i++){
        int ring_index = get_ring_index_from_firing_order(i % LAYER_NUM);

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

        if(ring_index < LAYER_NUM / 2){
            int u = vp->points[i].x / RESOLUTION + 0.5 * WIDTH;
            int v = vp->points[i].y / RESOLUTION + 0.5 * WIDTH;
            if(0<=u && u<road_image.rows && 0<=v && v<road_image.cols){
                grid_states[u][v].set_height(vp->points[i].z + VELODYNE_HEIGHT);
            }
        }
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

    cv::Point2d center_of_image(road_image.cols/2, road_image.rows/2);
    cv::Mat affine_mat = cv::getRotationMatrix2D(center_of_image, -dyaw / M_PI * 180.0, 1);
    affine_mat.at<double>(0, 2) += -dy / RESOLUTION;// y
    affine_mat.at<double>(1, 2) += -dx / RESOLUTION;// x
    std::cout << affine_mat << std::endl;
    cv::warpAffine(road_image, road_image, affine_mat, road_image.size(), CV_INTER_LINEAR);

    apply_grid_states_to_image(grid_states, road_image);

    cv::namedWindow("road_image", cv::WINDOW_NORMAL);
    cv::imshow("road_image", road_image);
    cv::Mat filtered_image;
    cv::GaussianBlur(road_image, filtered_image, cv::Size(1, 1), 0);
    cv::namedWindow("filtered_image", cv::WINDOW_NORMAL);
    cv::imshow("filtered_image", filtered_image);
    cv::Mat laplacian_image;
    cv::Laplacian(filtered_image, laplacian_image, CV_64FC1, 3);
    cv::namedWindow("laplacian_image", cv::WINDOW_NORMAL);
    cv::imshow("laplacian_image", laplacian_image);
    cv::waitKey(1);

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

void VelodyneDifferentiator::apply_grid_states_to_image(const std::vector<std::vector<GridState> >& grid_states, cv::Mat& image)
{
    int width = image.size().width;
    size_t size = image.size().width * image.size().height;
    for(size_t i=0;i<size;i++){
        int u = i / width;
        int v = i % width;
        if(grid_states[u][v].hit_count != 0){
            if(image.at<double>(u, v) != 0.0){
                image.at<double>(u, v) = (grid_states[u][v].hit_count * grid_states[u][v].avg_z + image.at<double>(u, v)) / (grid_states[u][v].hit_count + 1);
                // std::cout << i << ": " << image.at<double>(u, v) << std::endl;
            }else{
                image.at<double>(u, v) = grid_states[u][v].avg_z;
            }
        }
    }
}

void VelodyneDifferentiator::process(void)
{
    ros::spin();
}
