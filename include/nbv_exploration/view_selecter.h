#ifndef NBV_EXPLORATION_VIEW_SELECTER_H
#define NBV_EXPLORATION_VIEW_SELECTER_H

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

#include "nbv_exploration/view_generator.h"

typedef geometry_msgs::Pose Pose;

class ViewSelecterBase
{
protected:
  ViewGeneratorBase* view_gen_;
  
  PointCloudXYZ::Ptr cloud_occupied_ptr_;
  octomap::OcTree* tree_;
  
  Pose current_pose_;
  Pose selected_pose_;
  Eigen::Matrix3d rotation_mtx_;
  
  double fov_horizontal_;
  double fov_vertical_;
  double range_max_;
  double range_min_;
  double tree_resolution_;
  double last_max_utility_;
  
  bool is_debug_;
  
  std::vector<Eigen::Vector3d> rays_near_plane_;
  std::vector<Eigen::Vector3d> rays_far_plane_;
  
  visualization_msgs::Marker line_list;

  // Topic handlers
  ros::Publisher marker_pub;
  ros::Publisher pose_pub;
  ros::Publisher ig_pub;
  
  
public:
  ViewSelecterBase();
  ~ViewSelecterBase(){}
  
  void setViewGenerator(ViewGeneratorBase* v)
  {
    view_gen_ = v;
  }
  
  void update()
  {
    cloud_occupied_ptr_ = view_gen_->cloud_occupied_ptr_;
    tree_               = view_gen_->tree_;
    current_pose_       = view_gen_->current_pose_;
    
    tree_resolution_ = tree_->getResolution();
    
    octomap::point3d min (view_gen_->obj_bounds_x_min_,
			  view_gen_->obj_bounds_y_min_,
			  view_gen_->obj_bounds_z_min_);
    octomap::point3d max (view_gen_->obj_bounds_x_max_,
			  view_gen_->obj_bounds_y_max_,
			  view_gen_->obj_bounds_z_max_);
			  
    std::cout << "Bounds min: " << view_gen_->obj_bounds_x_min_ << ", " << view_gen_->obj_bounds_y_min_ << ", " << view_gen_->obj_bounds_z_min_ << "\n";
    std::cout << "Bounds max: " << view_gen_->obj_bounds_x_max_ << ", " << view_gen_->obj_bounds_y_max_ << ", " << view_gen_->obj_bounds_z_max_ << "\n";
		    
    tree_->setBBXMin( min );	       
    tree_->setBBXMax( max );
    
    computeRelativeRays();
  }
  

  Pose getTargetPose()
  {
    return selected_pose_;
  }

  void setCameraSettings(double fov_h, double fov_v, double r_max, double r_min)
  {
    fov_horizontal_ = fov_h;
    fov_vertical_ = fov_v;
    range_max_ = r_max;
    range_min_ = r_min;
  }

  void setDebug(bool b)
  {
    is_debug_ = b;
  }

  void evaluate();
  bool isEntropyLow();
  
  double getNodeOccupancy(octomap::OcTreeNode* node);
  double getNodeEntropy(octomap::OcTreeNode* node);
  
  double computeRelativeRays();
  void computeOrientationMatrix(Pose p);
  octomap::point3d transformToGlobalRay(Eigen::Vector3d ray_dir);
  bool isNodeInBounds(octomap::OcTreeKey &key);
  bool isPointInBounds(octomap::point3d &p);
  
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
