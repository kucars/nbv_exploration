#ifndef NBV_EXPLORATION_VIEW_SELECTER_IG_EXP_DISTANCE_H
#define NBV_EXPLORATION_VIEW_SELECTER_IG_EXP_DISTANCE_H

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

#include "nbv_exploration/view_selecter_base.h"
#include "nbv_exploration/common.h"


class ViewSelecterIgExpDistance: public ViewSelecterBase
{
public:
  ViewSelecterIgExpDistance();

protected:
  double w_dist_; //weight of distance in exp

  double calculateUtility(geometry_msgs::Pose p);
  std::string getMethodName();
};

#endif
