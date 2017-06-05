#include "perception/segmentation.h"

#include "pcl/common/angles.h"
#include "pcl/common/common.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "visualization_msgs/Marker.h"
#include "pcl/PointIndices.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/filters/crop_box.h"
#include "perception/box_fitter.h"

#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/CollisionObject.h"

#include "visualization_msgs/MarkerArray.h"

#include "acciobot_main/CollisionList.h"
#include "acciobot_main/BoxCollision.h"
#include "acciobot_main/PerceivedItems.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  // pcl::ModelCoefficients coeff;

  seg.segment(indices_internal, *coeff);

  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  // Build custom indices that ignores points above the plane.
  for (size_t i = 0; i < cloud->size(); ++i) {
    const PointC& pt = cloud->points[i];
    float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
                coeff->values[2] * pt.z + coeff->values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(i);
    }
  }

  // Comment this out
  //*indices = indices_internal;
  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }

}

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
  pose->orientation.w = 1;
  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
  pose->position.x = (max_pcl.x + min_pcl.x) / 2;
  pose->position.y = (max_pcl.y + min_pcl.y) / 2;
  pose->position.z = (max_pcl.z + min_pcl.z) / 2;
  // ROS_INFO("%f, %f, %f", pose->position.x, pose->position.y, pose->position.z);
  dimensions->x = (max_pcl.x - min_pcl.x);
  dimensions->y = (max_pcl.y - min_pcl.y);
  dimensions->z = (max_pcl.z - min_pcl.z);
}


Segmenter::Segmenter(const ros::Publisher& surface_points_pub)
    : surface_points_pub_(surface_points_pub) {}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub)
    : nh_(), surface_points_pub_(surface_points_pub), marker_pub_(marker_pub) {
      input_pub_ =
          nh_.advertise<sensor_msgs::PointCloud2>("input_cloud", 1, true);

      // TODO: PARAMETERS?
      collision_pub_ =
          nh_.advertise<acciobot_main::CollisionList>("accio_collisions", 1, true);

      items_pub_ =
          nh_.advertise<acciobot_main::PerceivedItems>("accio_items", 1, true);
    }

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  ROS_ERROR("We are running the segmentation pipeline everyone!");
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  input_pub_.publish(msg);

  /*
  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  SegmentSurface(cloud, table_inliers);
  PointCloudC::RPtr subset_cloud(new PointCloudC());
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.filter(*subset_cloud);
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*subset_cloud, msg_out);
  // Reused later for above surface segmentation
  surface_points_pub_.publish(msg_out);

  // Segmentation of objects
  std::vector<pcl::PointIndices> object_indices;
  SegmentSurfaceObjects(cloud, table_inliers, &object_indices);

  pcl::ExtractIndices<PointC> extract_objects;
  PointCloudC::Ptr annoy_object_cloud(new PointCloudC());

  extract_objects.setInputCloud(cloud);
  extract_objects.setIndices(table_inliers);

  // We are reusing the extract object created earlier in the callback.
  extract_objects.setNegative(true);
  extract_objects.filter(*annoy_object_cloud);
  // pcl::toROSMsg(*annoy_object_cloud, msg_out);
  // surface_points_pub_.publish(msg_out);

  // for (size_t i = 1; i < 2; ++i) {
  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    // TODO: fill in object_cloud using indices
    ROS_INFO("trying to draw %ld ", indices->indices.size());
    // for (size_t j = 0; j < indices->indices.size(); j++) {
    //     ROS_INFO("%ld", indices->indices[j]);
    // }
    pcl::ExtractIndices<PointC> extract_seg;

    extract_seg.setInputCloud(cloud);
    extract_seg.setIndices(indices);

    // extract_seg.setNegative(true);
    extract_seg.filter(*object_cloud);
    // pcl::toROSMsg(*object_cloud, msg_out);
    // surface_points_pub_.publish(msg_out);
    //

    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                              &object_marker.scale);
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    marker_pub_.publish(object_marker);
  }
  //
  */

  // TODO(emersonn): TABLE MARKER FOR SINGLE TABLE!!!!!
  // visualization_msgs::Marker table_marker;
  // table_marker.ns = "table";
  // table_marker.header.frame_id = "base_link";
  // table_marker.type = visualization_msgs::Marker::CUBE;
  // GetAxisAlignedBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
  // table_marker.color.r = 1;
  // table_marker.color.a = 0.8;
  // if( marker_pub_ ) {
  //   marker_pub_.publish(table_marker);
  // }
  //
  // acciobot_main::CollisionList new_collisions;
  //
  // acciobot_main::BoxCollision one_box;
  // one_box.pose = table_marker.pose;
  // one_box.scale = table_marker.scale;
  //
  // std::vector<acciobot_main::BoxCollision> collision_list;
  // collision_list.push_back(one_box);
  //
  // new_collisions.collisions = collision_list;
  //
  // // TODO: FIGURE THIS OUT?
  // collision_pub_.publish(new_collisions);

  // moveit::planning_interface::PlanningSceneInterface planning_scene("base_link");
  // planning_scene.clear();
  //
  // // planning_scene.addBox('bottom_shelf', 0.5, 1, 0.9, 0.9, 0, 0.30)
  // std::vector<std::string> object_ids;
  // object_ids.push_back("shelf");
  //
  // planning_scene.removeCollisionObjects(object_ids);
  // // planning_scene.addBox('shelf');
  // std::vector<moveit_msgs::CollisionObject> objects;
  // moveit_msgs::CollisionObject new_object();
  FindAllTables(cloud);
}

void Segmenter::FindAllTables(
  PointCloudC::Ptr cloud
) {
  // Heights of every shelf in meters from the ground
  double heights[4] = {0.04, 0.4, 0.750, 1.11};

  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, 0.5); // 0.3
  ros::param::param("crop_min_y", min_y, -1.0);
  ros::param::param("crop_min_z", min_z, 0.5);
  ros::param::param("crop_max_x", max_x, 1.1); // 0.9
  ros::param::param("crop_max_y", max_y, 1.0);
  ros::param::param("crop_max_z", max_z, 1.5);
  // ROS_INFO("%f %f %f %f %f %f", min_x, min_y, min_z, max_x, max_y, max_z);

  acciobot_main::CollisionList new_collisions;
  std::vector<acciobot_main::BoxCollision> collision_list;

  acciobot_main::PerceivedItems new_items;
  std::vector<visualization_msgs::MarkerArray> table_arrays;

  for (int i = 0; i < 4; i++) {
    double ground_height = heights[i];

    Eigen::Vector4f min_pt(min_x, min_y, ground_height - 0.05, 1);
    Eigen::Vector4f max_pt(max_x, max_y, ground_height + 0.05, 1);

    pcl::CropBox<PointC> crop;

    PointCloudC::Ptr cropped_cloud(new PointCloudC());
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cropped_cloud);

    if (cropped_cloud->size() < 100) {
      // ROS_INFO("%d %ld\n", i, cropped_cloud->size());
      visualization_msgs::MarkerArray new_array;
      table_arrays.push_back(new_array);
      continue;
    }

    // if (i == 3) {
    //   sensor_msgs::PointCloud2 msg_out;
    //   pcl::toROSMsg(*cropped_cloud, msg_out);
    //   surface_points_pub_.publish(msg_out);
    // }

    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    SegmentSurface(cropped_cloud, table_inliers, coefficients);

    PointCloudC::Ptr subset_cloud(new PointCloudC());
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(cropped_cloud);
    extract.setIndices(table_inliers);
    extract.filter(*subset_cloud);

    if (i == 2) {
      sensor_msgs::PointCloud2 msg_out;
      pcl::toROSMsg(*subset_cloud, msg_out);
      surface_points_pub_.publish(msg_out);
    }

    visualization_msgs::Marker table_marker;
    table_marker.ns = "table";
    table_marker.id = i;
    table_marker.header.frame_id = "base_link";
    table_marker.type = visualization_msgs::Marker::CUBE;
    GetAxisAlignedBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
    table_marker.color.r = 1;
    table_marker.color.a = 0.8;
    if (marker_pub_) {
      marker_pub_.publish(table_marker);
    }

    acciobot_main::BoxCollision one_box;
    one_box.pose = table_marker.pose;
    one_box.scale = table_marker.scale;

    collision_list.push_back(one_box);

    // OBJECT SEGMENTATION
    pcl::PointIndices::Ptr fake_table_inliers(new pcl::PointIndices());
    visualization_msgs::MarkerArray new_array = SegmentSurfaceObjectsOnTable(cloud, ground_height, fake_table_inliers, i, coefficients);
    table_arrays.push_back(new_array);
  }

  new_items.tables = table_arrays;
  items_pub_.publish(new_items);

  // TODO: FIGURE THIS OUT?
  new_collisions.collisions = collision_list;
  collision_pub_.publish(new_collisions);
}

visualization_msgs::MarkerArray Segmenter::SegmentSurfaceObjectsOnTable(
  PointCloudC::Ptr cloud,
  double ground_height,
  pcl::PointIndices::Ptr table_inliers,
  int table,
  pcl::ModelCoefficients::Ptr coeff
) {
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, 0.3);
  ros::param::param("crop_min_y", min_y, -1.0);
  ros::param::param("crop_min_z", min_z, 0.5);
  ros::param::param("crop_max_x", max_x, 0.9);
  ros::param::param("crop_max_y", max_y, 1.0);
  ros::param::param("crop_max_z", max_z, 1.5);

  // TODO(emersonn): Crop the cloud for each table appropriately, dfiferent marker color for different tables,t hen maek a message
  Eigen::Vector4f min_obj_pt(min_x, min_y, ground_height + 0.030, 1);
  Eigen::Vector4f max_obj_pt(max_x, max_y, ground_height + 0.300, 1);

  pcl::CropBox<PointC> crop_for_objects;

  PointCloudC::Ptr object_cropped_cloud(new PointCloudC());
  crop_for_objects.setInputCloud(cloud);
  crop_for_objects.setMin(min_obj_pt);
  crop_for_objects.setMax(max_obj_pt);
  crop_for_objects.filter(*object_cropped_cloud);

  if (table == 3) {
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*object_cropped_cloud, msg_out);
    surface_points_pub_.publish(msg_out);
  }

  std::vector<pcl::PointIndices> object_indices;

  // TODO: TESTING 1, 2,3, FOR THE CROPED CLOUDS, THINGS MIGHT BE NOT IN THAT CROPPED CLOUD RESPECTIVE TSUFF
  // MIGHT AS EWLL USE MODEL COEFFICIENTS, OR CROPPING ABOVE THE TABLE INSTEAD TO IGNORE THE TABLE AND JUST USE AN
  // EMPTY TABLE_INLIERS
  // table_inliers = new pcl::PointIndices();

  // Use model coefficients to find the points above and below plane
  SegmentSurfaceObjects(object_cropped_cloud, table_inliers, &object_indices);
  ROS_ERROR("We found: %ld objects on table %d\n", object_indices.size(), table);

  visualization_msgs::MarkerArray found_items;
  std::vector<visualization_msgs::Marker> items_list;

  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];

    PointCloudC::Ptr object_cloud(new PointCloudC());
    pcl::ExtractIndices<PointC> extract_seg;

    extract_seg.setInputCloud(object_cropped_cloud);
    extract_seg.setIndices(indices);
    extract_seg.filter(*object_cloud);

    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i + 100 * table;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;

    //GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
    //                          &object_marker.scale);
    //
    PointCloudC::Ptr output_cloud(new PointCloudC());
    shape_msgs::SolidPrimitive shape;
    geometry_msgs::Pose pose;
    perception::FitBox(*object_cloud, coeff, *output_cloud, shape, pose);

    object_marker.scale.x = shape.dimensions[shape.BOX_X];
    object_marker.scale.y = shape.dimensions[shape.BOX_Y];
    object_marker.scale.z = shape.dimensions[shape.BOX_Z];
    object_marker.pose = pose;
    //

    ROS_ERROR("%f", object_marker.scale.z);
    // TODO(emersonn): HARD CODED LOL
    if (object_marker.scale.z >= 0.27) {
      // continue;
    }

    items_list.push_back(object_marker);

    object_marker.color.g = 1;
    object_marker.color.a = 0.3;

    marker_pub_.publish(object_marker);
  }

  found_items.markers = items_list;
  return found_items;
}

void Segmenter::SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices) {
  pcl::ExtractIndices<PointC> extract;
  pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
  extract.setInputCloud(cloud);
  extract.setIndices(surface_indices);
  extract.setNegative(true);
  extract.filter(above_surface_indices->indices);

  // ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 100000);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setIndices(above_surface_indices);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*object_indices);

  // Find the size of the smallest and the largest object,
  // where size = number of points in the cluster
  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < object_indices->size(); ++i) {
    size_t cluster_size = object_indices->at(i).indices.size();

    // ROS_INFO("WTF %ld", cluster_size);

    if (cluster_size < min_size) {
      min_size = cluster_size;
    }

    if (cluster_size > max_size) {
      max_size = cluster_size;
    }
  }

  // ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
  //         object_indices->size(), min_size, max_size);

}


}  // namespace perception
