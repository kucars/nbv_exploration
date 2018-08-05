#ifndef NBV_EXPLORATION_VIEW_SELECTER_BASE_H
#define NBV_EXPLORATION_VIEW_SELECTER_BASE_H

#include <iostream>
#include <ros/ros.h>

#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/mapping_module.h"


class ViewSelecterBase
{
public:
  int   info_iteration_;
  float info_distance_total_;
  float info_entropy_total_;

  std::vector<float> info_utilities_;
  float info_selected_utility_;
  float info_selected_utility_density_;
  float info_selected_utility_distance_;
  float info_selected_utility_entropy_;
  float info_selected_utility_prediction_;
  int   info_selected_occupied_voxels_;

  float temp_utility_density_;
  float temp_utility_distance_;
  float temp_utility_entropy_;
  float temp_utility_prediction_;
  int   temp_occupied_voxels_;

  std::vector<Eigen::Matrix3d> camera_rotation_mtx_; // Cameras rotation mtx
  std::vector<Eigen::Vector3d> camera_translation_;//Cameras translation
  int camera_count_;
  double fov_horizontal_;
  double fov_vertical_;
  double range_max_;
  double range_min_;

  ViewSelecterBase();

  void evaluate();
  virtual std::string getMethodName();
  geometry_msgs::Pose  getTargetPose();
  void   getCameraRotationMtxs();
  void setCameraSettings(double fov_h, double fov_v, double r_max, double r_min);
  void setViewGenerator(ViewGeneratorBase* v);
  void setMappingModule(MappingModule* m);
  virtual void update();

protected:
  ViewGeneratorBase* view_gen_;
  MappingModule* mapping_module_;
  
  PointCloudXYZ::Ptr cloud_occupied_ptr_;
  octomap::OcTree* tree_;

  geometry_msgs::Pose current_pose_;
  geometry_msgs::Pose selected_pose_;

//  int camera_count_;
//  std::vector<Eigen::Matrix3d> camera_rotation_mtx_; // Camera rotation mtx
  int vehicle_type_;
  
//  double fov_horizontal_;
//  double fov_vertical_;
//  double range_max_;
//  double range_min_;
  double tree_resolution_;

  bool is_debug_;
  bool must_see_occupied_;
  bool is_ignoring_clamping_entropies_;
  
  std::vector<Eigen::Vector3d> rays_far_plane_;
  std::vector<octomap::point3d> rays_far_plane_at_pose_;
  
  visualization_msgs::Marker ray_msg;
  visualization_msgs::Marker trajectory_msg;

  // Topic handlers
  ros::Publisher marker_pub;
  ros::Publisher pose_pub;
  ros::Publisher trajectory_pub;

protected:
  bool isEntropyLow();
  bool isNodeInBounds(octomap::OcTreeKey &key);
  bool isNodeFree(octomap::OcTreeNode node);
  bool isNodeOccupied(octomap::OcTreeNode node);
  bool isNodeUnknown(octomap::OcTreeNode node);
  bool isPointInBounds(octomap::point3d &p);

//  void   getCameraRotationMtxs();
  double getNodeOccupancy(octomap::OcTreeNode* node);
  double getNodeEntropy(octomap::OcTreeNode* node);
  int    getPointCountAtOcTreeKey(octomap::OcTreeKey key);

  double computeRelativeRays();
  void   computeRaysAtPose(geometry_msgs::Pose p);


  double calculateIG(geometry_msgs::Pose p);
  double calculateDistance(geometry_msgs::Pose p);
  double calculateAngularDistance(geometry_msgs::Pose p);
  virtual double calculateUtility(geometry_msgs::Pose p);
  

  void addToRayMarkers(octomap::point3d origin, octomap::point3d endpoint);
  void clearRayMarkers();
  void publishRayMarkers();
  void publishPose(geometry_msgs::Pose p);
  void publishTrajectory();
};

#endif
