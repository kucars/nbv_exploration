#ifndef NBV_EXPLORATION_VIEW_SELECTER_H
#define NBV_EXPLORATION_VIEW_SELECTER_H

#include <iostream>

#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>

#include <nbv_exploration/view_generator.h>

//typedef pcl::PointXYZRGBA PointT;
typedef geometry_msgs::Pose Pose;

class ViewSelecterBase
{
protected:
  ViewGeneratorBase* view_gen_;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_occupied_ptr_;
  octomap::OcTree* tree_;
  
  Pose current_pose_;
  Pose selected_pose_;
  Eigen::Matrix3d rotation_mtx_;
  
  double fov_horizontal_;
  double fov_vertical_;
  double range_max_;
  double range_min_;
  double tree_resolution_;
  
  std::vector<Eigen::Vector3d> ray_directions_;
  
  visualization_msgs::Marker line_list;
  
  
public:
  ViewSelecterBase();
  ~ViewSelecterBase(){}
  
  void setViewGenerator(ViewGeneratorBase* v)
  {
    view_gen_ = v;
    
    fov_horizontal_ = 60;
    fov_vertical_  = 45;
    range_max_ = 6.0;
    range_min_ = 0.05;
  }
  
  void update()
  {
    cloud_occupied_ptr_ = view_gen_->cloud_occupied_ptr_;
    tree_               = view_gen_->tree_;
    current_pose_       = view_gen_->current_pose_;
    
    tree_resolution_ = tree_->getResolution();
    
    computeRelativeRays();
  }
  
  
  
  Pose getTargetPose()
  {
    return selected_pose_;
  }

  void evaluate();
  
  double getNodeOccupancy(octomap::OcTreeNode* node);
  double getNodeEntropy(octomap::OcTreeNode* node);
  
  double computeRelativeRays();
  void computeOrientationMatrix(Pose p);
  octomap::point3d getGlobalRayDirection(Eigen::Vector3d ray_dir);
  
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
