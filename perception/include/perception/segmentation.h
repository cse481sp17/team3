#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/MarkerArray.h"
#include "pcl/ModelCoefficients.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
// Finds the largest horizontal surface in the given point cloud.
// This is useful for adding a collision object to MoveIt.
//
// Args:
//  cloud: The point cloud to extract a surface from.
//  indices: The indices of points in the point cloud that correspond to the
//    surface. Empty if no surface was found.
void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff);

// Computes the axis-aligned bounding box of a point cloud.
//
// Args:
//  cloud: The point cloud
//  pose: The output pose. Because this is axis-aligned, the orientation is just
//    the identity. The position refers to the center of the box.
//  dimensions: The output dimensions, in meters.
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions);
class Segmenter {
 public:
  Segmenter(const ros::Publisher& surface_points_pub);
  Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);
  void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices);
  void FindAllTables(PointCloudC::Ptr cloud);
  visualization_msgs::MarkerArray SegmentSurfaceObjectsOnTable(
    PointCloudC::Ptr cloud,
    double ground_height,
    pcl::PointIndices::Ptr table_inliers,
    int table,
    pcl::ModelCoefficients::Ptr coeff
  );

 private:
  ros::Publisher surface_points_pub_;
  ros::Publisher marker_pub_;
  ros::NodeHandle nh_;
  ros::Publisher input_pub_;
  ros::Publisher collision_pub_;
  ros::Publisher items_pub_;
};
}  // namespace perception
