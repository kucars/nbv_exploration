#ifndef NBV_EXPLORATION_VIEW_SELECTER_H
#define NBV_EXPLORATION_VIEW_SELECTER_H

#include <iostream>

#include <geometry_msgs/Pose.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nbv_exploration/view_generator.h>

//typedef pcl::PointXYZRGBA PointT;
typedef geometry_msgs::Pose Pose;


class ViewSelecterBase
{
protected:
  ViewGeneratorBase* view_gen_;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_occupied_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_free_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined_ptr_;
  
  Pose current_pose_;
  Pose selected_pose_;
  
public:
  ViewSelecterBase(){}
  ~ViewSelecterBase(){}
  
  
  Pose getTargetPose()
  {
    return selected_pose_;
  }
  
  void setViewGenerator(ViewGeneratorBase* v)
  {
    view_gen_           = v;
    cloud_occupied_ptr_ = v->cloud_occupied_ptr_;
    cloud_free_ptr_     = v->cloud_free_ptr_;
    cloud_combined_ptr_ = v->cloud_combined_ptr_;
    current_pose_       = v->current_pose_;
  }

  void evaluate();
  double calculateIG(Pose p);
  double calculateDistance(Pose p);
  double calculateAngularDistance(Pose p);
  virtual double calculateUtility(Pose p);
};

#endif
