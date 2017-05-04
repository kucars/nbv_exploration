#include <iostream>
#include <ros/ros.h>

#include "nbv_exploration/view_selecter_ig_exp_distance.h"

ViewSelecterIgExpDistance::ViewSelecterIgExpDistance():
  ViewSelecterBase() //Call base class constructor
{

  ros::param::param<double>("~view_selecter_weight_distance", w_dist_, 1.0);
}

double ViewSelecterIgExpDistance::calculateUtility(Pose p)
{
  double IG = calculateIG(p);
  double dist = calculateDistance(p);
  return IG*exp(-dist*w_dist_);
}

std::string ViewSelecterIgExpDistance::getMethodName()
{
  return "IG_exp_" + std::to_string(w_dist_) +"_dist";
}
