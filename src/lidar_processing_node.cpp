#include<lidar_processing/lidar_processing.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_mapping");
  LidarProcessing node;
  ros::spin();
  return 0;
}
