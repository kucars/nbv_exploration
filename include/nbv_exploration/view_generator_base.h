#ifndef NBV_EXPLORATION_VIEW_GENERATOR_BASE_H
#define NBV_EXPLORATION_VIEW_GENERATOR_BASE_H

#include <iostream>

#include <geometry_msgs/Pose.h>
#include <fcl/shape/geometric_shapes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/OcTree.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "nbv_exploration/common.h"
#include "nbv_exploration/nbv_history.h"

typedef geometry_msgs::Pose Pose;


class ViewGeneratorBase
{
protected:
  double res_x_, res_y_, res_z_, res_yaw_;
  
  double collision_radius_;
  double nav_bounds_x_max_, nav_bounds_y_max_, nav_bounds_z_max_;
  double nav_bounds_x_min_, nav_bounds_y_min_, nav_bounds_z_min_;
  bool is_debug_;
  std::vector<fcl::CollisionObject*> collision_boxes_;
  NBVHistory* nbv_history_;
  
public:
  // ==========
  // == Variables
  // ==========
  double obj_bounds_x_max_, obj_bounds_y_max_, obj_bounds_z_max_; //Made public for viewselector. May benefit from getter
  double obj_bounds_x_min_, obj_bounds_y_min_, obj_bounds_z_min_;
  
  PointCloudXYZ::Ptr cloud_occupied_ptr_;
  octomap::OcTree* tree_;
  octomap::OcTree* tree_prediction_;
  Pose current_pose_;
  std::vector<Pose> generated_poses;

  // Visualizer
  int vis_marker_array_prev_size_;
  int vis_sphere_counter_;
  ros::Publisher pub_view_marker_array_;
  ros::Publisher pub_collision_marker_;

  // ==========
  // == Methods
  // ==========
  ViewGeneratorBase();

  virtual void generateViews()
  {
    std::cout << "[WARNING] Call to ViewGeneratorBase::generateViews(). Impliment function in derived class. No poses generated." << std::endl;
  }

  virtual bool isCollidingWithOctree(Pose p);
  virtual bool isInsideBounds(Pose p);
  virtual bool isValidViewpoint(Pose p);

  void setCollisionRadius(double r);
  void setCloud(PointCloudXYZ::Ptr in_occ_cloud);
  void setCurrentPose(Pose p);
  void setDebug(bool b);
  void setHistory(NBVHistory* h);
  void setNavigationBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
  void setMap(octomap::OcTree* oct);
  void setMapPrediction(octomap::OcTree* oct);
  void setObjectBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
  
  virtual void updateCollisionBoxesFromOctomap();

  virtual void visualize(std::vector<Pose> valid_poses, std::vector<Pose> invalid_poses);
  visualization_msgs::Marker visualizeDeleteArrowMarker(int id);
  visualization_msgs::Marker visualizeCreateArrowMarker(int id, Pose pose, bool valid, double max_z = 0, double min_z = 0);
  void visualizeDrawSphere(Pose p, double r);
};

#endif
