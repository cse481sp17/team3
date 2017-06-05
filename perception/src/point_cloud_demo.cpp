#include "ros/ros.h"
#include "perception/crop.h"
#include "perception/downsample.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/segmentation.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;

  ROS_INFO("Starting point cloud demo");

  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);

  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("bounding_boxes", 1, true);

  ros::Publisher crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);

  // TODO(emersonn): THIS WAS ORIGINALLY CLOUD_IN BEFOREHAND!!!!!
  ros::Subscriber sub = nh.subscribe("seg_pipe_go", 1, &perception::Cropper::Callback, &cropper);

  // perception::Segmenter segmenter(table_pub);
  perception::Segmenter segmenter2(table_pub, marker_pub);

  // ros::Subscriber sub =
  //    nh.subscribe("cloud_in", 1, &perception::Segmenter::Callback, &segmenter);
  ros::Subscriber sub2 =
      nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter2);

  ros::spin();
  return 0;
  // ros::init(argc, argv, "point_cloud_demo");
  // ros::NodeHandle nh;
  // ros::Publisher crop_pub =
  //   nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  // ros::Publisher downsampler_pub =
  //   nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
  // perception::Cropper cropper(crop_pub);
  // ros::Subscriber sub =
  //     nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  // perception::Downsampler downsampler(downsampler_pub);
  // ros::Subscriber sub2 =
  //     nh.subscribe("cloud_in", 1, &perception::Downsampler::Callback, &downsampler);
  // ros::spin();
  // return 0;
}
