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
  ViewGeneratorBase* _viewGen;
  Pose _currentPose;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloudPtr;
  
public:
  ViewSelecterBase(){}
  ~ViewSelecterBase(){}
  
  Pose selected_pose;
  
  
  Pose getTargetPose()
  {
    return selected_pose;
  }
  void setViewGenerator(ViewGeneratorBase* v)
  {
    _viewGen   = v;
    _cloudPtr  = v->cloudPtr;
    _currentPose = v->currentPose;
  }

  void evaluate();
  double calculateIG(Pose p);
  double calculateDistance(Pose p);
  double calculateAngularDistance(Pose p);
  
  virtual double calculateUtility(Pose p);
};

#endif
