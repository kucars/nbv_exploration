#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf_conversions/tf_eigen.h>
//#include <culling/occlusion_culling.h>

#include "nbv_exploration/view_selecter_symmetry_prediction.h"

ViewSelecterSymmetryPrediction::ViewSelecterSymmetryPrediction():
  ViewSelecterBase() //Call base class constructor
{
}

double ViewSelecterSymmetryPrediction::calculateUtility(Pose p)
{
  double IG = calculateIG(p);
  return IG;
}

std::string ViewSelecterSymmetryPrediction::getMethodName()
{
  return "Symmetry Prediction";
}
