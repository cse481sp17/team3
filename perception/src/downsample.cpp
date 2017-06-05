#include "perception/downsample.h"
// Add this to your #includes
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"


// Add these typedefs after your #includes
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

perception::Downsampler::Downsampler(const ros::Publisher& pub) : pub_(pub) {}

void perception::Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cropped_cloud);
  ROS_INFO("Got point cloud with %ld points", cropped_cloud->size());

  PointCloudC::Ptr downsampled_cloud(new PointCloudC());

  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(cropped_cloud);
  double voxel_size;
  ros::param::param("voxel_size", voxel_size, 0.01);
  vox.setLeafSize(voxel_size, voxel_size, voxel_size);
  vox.filter(*downsampled_cloud);
  ROS_INFO("Downsampled to %ld points", downsampled_cloud->size());
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*downsampled_cloud, msg_out);
  pub_.publish(msg_out);
  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*cropped_cloud, min_pcl, max_pcl);
  ROS_INFO("min: %f, max: %f", min_pcl.x, max_pcl.x);
}
