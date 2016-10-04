#ifndef NBV_EXPLORATION_VIEW_GENERATOR_H
#define NBV_EXPLORATION_VIEW_GENERATOR_H

#include <iostream>

#include <geometry_msgs/Pose.h>

#include "fcl/shape/geometric_shapes.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <octomap/OcTree.h>

#include <tf_conversions/tf_eigen.h>

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
  
public:
  // == Variables
  double obj_bounds_x_max_, obj_bounds_y_max_, obj_bounds_z_max_; //Made public for viewselector. May benefit from getter
  double obj_bounds_x_min_, obj_bounds_y_min_, obj_bounds_z_min_;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_occupied_ptr_;
  octomap::OcTree* tree_;
  Pose current_pose_;
  std::vector<Pose> generated_poses;

  // == Constructor and Destructor
  ViewGeneratorBase();
  ~ViewGeneratorBase(){}

  
  // == Setters
  void setCollisionRadius(double r){collision_radius_ = r;}
  
  void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_occ_cloud){cloud_occupied_ptr_ = in_occ_cloud;}
  
  void setCurrentPose(Pose p){current_pose_ = p;}
  
  void setDebug(bool b){is_debug_ = b;}
  
  void setNavigationBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
  {
    nav_bounds_x_min_ = x_min;
    nav_bounds_x_max_ = x_max;
    nav_bounds_y_min_ = y_min;
    nav_bounds_y_max_ = y_max;
    nav_bounds_z_min_ = z_min;
    nav_bounds_z_max_ = z_max;
  }
  
  void setMap(octomap::OcTree* oct){
    tree_ = oct;
    updateCollisionBoxesFromOctomap();
  }
  
  void setObjectBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
  {
    obj_bounds_x_min_ = x_min;
    obj_bounds_x_max_ = x_max;
    obj_bounds_y_min_ = y_min;
    obj_bounds_y_max_ = y_max;
    obj_bounds_z_min_ = z_min;
    obj_bounds_z_max_ = z_max;
  }
  
  // Not used by all derived classes
  void setResolution(double x, double y, double z, double yaw)
  {
    res_x_   = x;
    res_y_   = y;
    res_z_   = z;
    res_yaw_ = yaw;
  }
  
  
  // == Methods
  virtual void generateViews()
  {
    std::cout << "[WARNING] Call to ViewGeneratorBase::generateViews(). Impliment function in derived class. No poses generated." << std::endl;
  }
  
  virtual void visualize(std::vector<Pose> valid_poses, std::vector<Pose> invalid_poses);
  virtual void updateCollisionBoxesFromOctomap();
  virtual bool isCollidingWithOctree(Pose p);
  virtual bool isInsideBounds(Pose p);
  virtual bool isValidViewpoint(Pose p);
};


// Nearest neighbor
class ViewGeneratorNN : public ViewGeneratorBase
{
public:
  ViewGeneratorNN(){}
  ~ViewGeneratorNN(){}

  void generateViews();
  
};

// Frontier
class ViewGeneratorFrontier : public ViewGeneratorBase
{
public:
  ViewGeneratorFrontier(){}
  ~ViewGeneratorFrontier(){}

  void generateViews();
};


#endif
