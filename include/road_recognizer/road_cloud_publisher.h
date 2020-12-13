#ifndef __ROAD_CLOUD_PUBLISHER
#define __ROAD_CLOUD_PUBLISHER

#include <random>

#include <ros/ros.h>

// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "road_recognizer/intensity_partition.h"
#include "road_recognizer/xmeans_clustering.h"

#include <std_msgs/Bool.h>
// OMP
#include <omp.h>


class RoadCloudPublisher
{
public:
    typedef pcl::PointXYZINormal PointXYZIN;
    typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
    typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;
    typedef pcl::PointXYZ PointXYZ;
    typedef pcl::PointCloud<PointXYZ> CloudXYZ;
    typedef pcl::PointCloud<PointXYZ>::Ptr CloudXYZPtr;
    typedef pcl::PointNormal PointN;
    typedef pcl::PointCloud<PointN> CloudN;
    typedef pcl::PointCloud<PointN>::Ptr CloudNPtr;

    RoadCloudPublisher(void);

    void process(void);
    void obstacles_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void ground_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void ignore_intensity_callback(const std_msgs::BoolConstPtr&);
    void publish_clouds(void);
    void downsample(void);
    void estimate_normal(void);
    void filter_curvature(void);
    void filter_intensity(void);
    void filter_height(void);

private:
    double HZ;
    double NORMAL_ESTIMATION_RADIUS;
    double LEAF_SIZE;
    int OUTLIER_REMOVAL_K;
    double OUTLIER_REMOVAL_THRESHOLD;
    double CURVATURE_THRESHOLD;
    double INTENSITY_UPPER_THRESHOLD;
    double INTENSITY_LOWER_THRESHOLD;
    double HEIGHT_THRESHOLD;
    int MAX_RANDOM_SAMPLE_SIZE;
    double RANDOM_SAMPLE_RATIO;
	bool IS_OTSU;
    bool IGNORE_INTENSITY_DEFAULT;
    bool USE_NORMAL_Z_AS_CURVATURE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher curvature_cloud_pub;
    ros::Publisher downsampled_cloud_pub;
    ros::Publisher intensity_cloud_pub;
    ros::Publisher road_cloud_pub;
    ros::Subscriber obstacles_sub;
    ros::Subscriber ground_sub;
    ros::Subscriber ignore_intensity_sub;

	//IntensityPartition intensity_partition;

    CloudXYZINPtr obstacles_cloud;
    CloudXYZINPtr ground_cloud;
    CloudXYZINPtr curvature_cloud;
    CloudXYZINPtr intensity_cloud;
    CloudXYZINPtr road_cloud;
    bool obstacles_cloud_updated;
    bool ground_cloud_updated;
	bool ignore_intensity_flag;


	int RANGE_DIVISION_NUM;//= 20
	int THETA_DIVISION_NUM;//= 360;
	float OTSU_BINARY_SEPARATION_THRESHOLD;// = 0.2;
	float OTSU_BINARY_DIFF_FROM_AVR_THRESHOLD;// = 3.0;
	float OTSU_BINARY_SUM_OF_DIFF_FROM_AVR_THRESHOLD;// = 58.0;
	float RANGE_MAX;// = 20.0;
	float VAR_BETWEEN_THRESHOLD;
	float CHEAT_INTENSITY_WIDTH;
};

#endif// __ROAD_CLOUD_PUBLISHER
