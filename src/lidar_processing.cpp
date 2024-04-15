#include<lidar_processing/lidar_processing.hpp>

LidarProcessing::LidarProcessing()
{
    /**
    * @brief Initialize Subscriber
    */
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/carla/ego_vehicle/vlp16_1",
        10,
        boost::bind(&LidarProcessing::pointcloudCallbackHandler, this, ::_1));
        ros::Rate loop_rate(100);    
    
    /**
    * @brief Initialize Publisher
    */
    voxelgrid_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/voxelgrid_filtered", 10);
    passthrough_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/passthrough_filtered", 10);
    ransac_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ransac_filtered", 10);
    
    // smart pointers
    //scaned_cloud_ptr_ = boost::make_shared<PointCloudT>();

};


void LidarProcessing::pointcloudCallbackHandler(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // setClear();
    
    // if (cloud_msg->data.empty()){
    //     return;
    // }
    /**
     * @brief Inisialisasi kontainer, penting untuk memproses cloud yang berupa pointer dimana data yang masuk akan lansung hilang 
     */
    // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);
    /**
     * @brief Cara penulisan lain : sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2);
     */
    
    /**
     * @brief langkah untuk konversi data pointer dari cloud ke kontainer untuk di proses
     */
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    
    /**
     * @brief filter voxelgrid. titik cloud yang berdekatan dengan radius n akan dikelompokkan menjadi titik 
              cloud yang lebih besar.
              ini adalah proses Downsampling 
              leaf size 0.1 = 0.1m
     */
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (*cloudFilteredPtr);

    /**
     * @brief Passthrough filter
     */
    //kontainer
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr (xyz_cloud);

    // convert pcl::PointCloud2 ke pcl::PointCloud<pcl::PointXYZ>
    pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

    //kontainer
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);
    
    /**
     * @brief Passthrough filter flag
     */

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (xyzCloudPtr);
    pass.setFilterFieldName ("z");
    //pass.setFilterLimits (-0.1, 0.1); // -> untuk x, y
    pass.setFilterLimits (-1.2, 0.0); // -> untuk z
    //pass.setFilterLimits (-2.8, 0.9);
    //pass.setFilterLimitsNegative (true);
    //pass.setNegative (true);
    pass.filter (*xyzCloudPtrFiltered);

    /**
     * @brief note : vlp16_1 publisher lidar memiliki  rgb field, tapi hanya digunakan di rviz
                    kita tidak dapat mengolah data ini,
                    warning msg : Failed to find match for field 'rgb' atau 'rgba' dll
                    read : https://answers.ros.org/question/373789/colour-coding-in-rviz/
     */

    //ransac filter

    //kontainer
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_ransac_filtered_outliner = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (xyz_cloud_ransac_filtered_outliner);

    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_ransac_filtered_inliner = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (xyz_cloud_ransac_filtered_inliner);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.07);
    //additional
    seg.setMaxIterations (100);
    
    seg.setInputCloud (xyzCloudPtrFiltered);
    seg.segment (*inliers, *coefficients);
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    //extract.setInputCloud (xyzCloudPtrFiltered);
    extract.setInputCloud (xyzCloudPtrFiltered);
    extract.setIndices (inliers);
    extract.setNegative (true);
    //true : outliner == planar
    //false : outliner == object
    extract.filter (*xyzCloudPtrRansacFiltered);

    // pcl::PointCloud<pcl::PointXYZ> ego_cloud;
    // pcl::fromROSMsg<pcl::PointXYZ>(*cloud_msg, ego_cloud);

    //for(auto each : *xyzCloudPtrFiltered)    
};

