#ifndef NBV_EXPLORATION_VIEW_SELECTER_IG_H
#define NBV_EXPLORATION_VIEW_SELECTER_IG_H

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


class ViewSelecterIg: public ViewSelecterBase
{
public:
  ViewSelecterIg();

protected:
  double calculateUtility(geometry_msgs::Pose p);
  std::string getMethodName();
};

#endif
