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
  double res_x_, res_y_, res_z_;
  double res_yaw_ = M_PI_4;
  
  double bounds_x_max_, bounds_y_max_, bounds_z_max_;
  double bounds_x_min_, bounds_y_min_, bounds_z_min_;
  double collision_radius_;
  bool is_debug_;
  std::vector<fcl::CollisionObject*> collision_boxes_;
  
public:
  // == Variables
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_occupied_ptr_;
  octomap::OcTree* tree_;
  Pose current_pose_;
  std::vector<Pose> generated_poses;

  // == Constructor and Destructor
  ViewGeneratorBase();
  ~ViewGeneratorBase(){}

  
  // == Setters
  void setBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
  {
    bounds_x_min_ = x_min;
    bounds_x_max_ = x_max;
    bounds_y_min_ = y_min;
    bounds_y_max_ = y_max;
    bounds_z_min_ = z_min;
    bounds_z_max_ = z_max;
  }
  
  void setCurrentPose(Pose p){current_pose_ = p;}
  
  void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_occ_cloud){cloud_occupied_ptr_ = in_occ_cloud;}
  
  void setCollisionRadius(double r){collision_radius_ = r;}
  
  void setDebug(bool b){is_debug_ = b;}
  
  void setMap(octomap::OcTree* oct){
    tree_ = oct;
    updateCollisionBoxesFromOctomap();
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
