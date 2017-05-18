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

typedef geometry_msgs::Pose Pose;

struct OctomapKeyCompare {
  bool operator() (const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const
  {
    size_t h1 = size_t(lhs.k[0]) + 1447*size_t(lhs.k[1]) + 345637*size_t(lhs.k[2]);
    size_t h2 = size_t(rhs.k[0]) + 1447*size_t(rhs.k[1]) + 345637*size_t(rhs.k[2]);
    return h1< h2;
  }
};

class ViewSelecterBase
{
public:
  int   info_iteration_;
  float info_distance_total_;
  float info_entropy_total_;
  std::vector<float> info_utilities_;
  float info_utility_max_;
  float info_utility_med_;

  ViewSelecterBase();

  void evaluate();
  virtual std::string getMethodName();
  Pose  getTargetPose();
  void setCameraSettings(double fov_h, double fov_v, double r_max, double r_min);
  void setViewGenerator(ViewGeneratorBase* v);
  virtual void update();

protected:
  ViewGeneratorBase* view_gen_;
  
  PointCloudXYZ::Ptr cloud_occupied_ptr_;
  octomap::OcTree* tree_;
  std::map<octomap::OcTreeKey, int, OctomapKeyCompare> pointCountInKey;
  
  Pose current_pose_;
  Pose selected_pose_;
  Eigen::Matrix3d rotation_mtx_;
  
  double fov_horizontal_;
  double fov_vertical_;
  double range_max_;
  double range_min_;
  double tree_resolution_;

  bool is_debug_;
  bool must_see_occupied_;
  bool is_ignoring_clamping_entropies_;
  
  std::vector<Eigen::Vector3d> rays_near_plane_;
  std::vector<Eigen::Vector3d> rays_far_plane_;
  
  visualization_msgs::Marker line_list;

  // Topic handlers
  ros::Publisher marker_pub;
  ros::Publisher pose_pub;

protected:
  bool isEntropyLow();
  bool isNodeInBounds(octomap::OcTreeKey &key);
  bool isNodeFree(octomap::OcTreeNode node);
  bool isNodeOccupied(octomap::OcTreeNode node);
  bool isNodeUnknown(octomap::OcTreeNode node);
  bool isPointInBounds(octomap::point3d &p);

  double getNodeOccupancy(octomap::OcTreeNode* node);
  double getNodeEntropy(octomap::OcTreeNode* node);
  int    getPointCountAtOcTreeKey(octomap::OcTreeKey key);

  double computeRelativeRays();
  void computeOrientationMatrix(Pose p);
  octomap::point3d transformToGlobalRay(Eigen::Vector3d ray_dir);


  double calculateIG(Pose p);
  double calculateDistance(Pose p);
  double calculateAngularDistance(Pose p);
  virtual double calculateUtility(Pose p);
  

  void addToRayMarkers(octomap::point3d origin, octomap::point3d endpoint);
  void clearRayMarkers();
  void publishRayMarkers();
  void publishPose(Pose p);
};

#endif
