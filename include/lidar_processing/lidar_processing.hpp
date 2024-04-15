#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>
#include <thread>

/**
* @brief 3D lidar using pointcloud2 data type
*/
#include <sensor_msgs/PointCloud2.h>

//#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/**
* @brief Filter that available on PCL
*/
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

///pcl convertions
#include "pcl_conversions/pcl_conversions.h"


class LidarProcessing 
{
    public:
    LidarProcessing();

    private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher voxelgrid_filtered_pub_;
    ros::Publisher passthrough_filtered_pub_;
    ros::Publisher ransac_filtered_pub_;
    void pointcloudCallbackHandler(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

};
