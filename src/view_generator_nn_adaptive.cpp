#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include "nbv_exploration/view_generator_nn_adaptive.h"
#include "nbv_exploration/common.h"

ViewGeneratorNNAdaptive::ViewGeneratorNNAdaptive():
  ViewGeneratorNN(), //Call base class constructor
  scale_factor_(1)
{
  ros::param::param<int>("~view_generator_nn_adaptive_local_minima_iterations", minima_iterations_, 3);
  //ros::param::param<double>("~view_generator_nn_adaptive_local_minima_threshold", minima_threshold_, 0.01);
  ros::param::param<double>("~view_generator_nn_adaptive_utility_threshold", minima_threshold_, 0.01);
}

bool ViewGeneratorNNAdaptive::isStuckInLocalMinima()
{
  if (minima_iterations_ > nbv_history_->iteration)
    return false;

  // See if the last few moves are repeating
  if (nbv_history_->isRepeatingMotions(4))
    return true;

  // Find max entropy change in the past few iterations
  float utility = nbv_history_->getMaxUtility(minima_iterations_);
  if (utility < minima_threshold_)
    return true;

  /*
  double max_change = nbv_history_->getMaxEntropyDiffPerVoxel(minima_iterations_);
  if (max_change < minima_threshold_)
    return true;
  */

  return false;
}

void ViewGeneratorNNAdaptive::generateViews()
{
  if (ViewGeneratorNNAdaptive::isStuckInLocalMinima())
  {
    scale_factor_*= 1.5;
    std::cout << "[ViewGeneratorNNAdaptive]: " << cc.yellow << "Local minima detected. Increasing scale factor to " << scale_factor_ << "\n" << cc.reset;

    if (scale_factor_ >= 7.5)
    {
      std::cout << "[ViewGeneratorNNAdaptive]: " << cc.red << "Warning: Scale factor very large: " << scale_factor_ << "\n" << cc.reset;
      scale_factor_ = 7.5;
    }
  }
  else
  {
    scale_factor_ = 1;
  }

  // Scale up sampling resolution
  double backup_res_x_ = res_x_;
  double backup_res_y_ = res_y_;
  double backup_res_z_ = res_z_;

  res_x_ *= scale_factor_;
  res_y_ *= scale_factor_;
  res_z_ *= scale_factor_;

  // Call base function
  if (scale_factor_ > 1)
    // Do not generate any viewpoints in current location to escape local minima
    ViewGeneratorNN::generateViews(false);
  else
    ViewGeneratorNN::generateViews(true);

  // Return scale to normal
  res_x_ = backup_res_x_;
  res_y_ = backup_res_y_;
  res_z_ = backup_res_z_;
}

std::string ViewGeneratorNNAdaptive::getMethodName()
{
  return "NN Adaptive";
}
