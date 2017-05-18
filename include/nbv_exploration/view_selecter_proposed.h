#ifndef NBV_EXPLORATION_VIEW_SELECTER_PROPOSED_H
#define NBV_EXPLORATION_VIEW_SELECTER_PROPOSED_H

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


class ViewSelecterProposed: public ViewSelecterBase
{
public:
  ViewSelecterProposed();

protected:
  octomap::OcTree* tree_predicted_;
  double calculateUtility(Pose p);
  std::string getMethodName();
  void update();

  double weight_density_;
  double weight_entropy_;
  double weight_prediction_;
  double max_density_;

};

#endif
