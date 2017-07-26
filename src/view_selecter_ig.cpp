#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf_conversions/tf_eigen.h>
//#include <culling/occlusion_culling.h>

#include "nbv_exploration/view_selecter_ig.h"

ViewSelecterIg::ViewSelecterIg():
  ViewSelecterBase() //Call base class constructor
{
}

double ViewSelecterIg::calculateUtility(geometry_msgs::Pose p)
{
  double IG = calculateIG(p);
  return IG;
}

std::string ViewSelecterIg::getMethodName()
{
  return "Classic IG";
}
