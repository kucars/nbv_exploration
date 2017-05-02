#include <ros/ros.h>

#include "nbv_exploration/termination_check_base.h"

TerminationCheckBase::TerminationCheckBase():
  current_iteration_(0)
{
  ros::param::param("~termination_max_iterations", max_iterations_, 300);
}

void TerminationCheckBase::update()
{
  current_iteration_++;
  ROS_INFO("termination_check: Iteration %d", current_iteration_);
}

bool TerminationCheckBase::isTerminated()
{
  if (current_iteration_ > max_iterations_)
    return true;

  return false;

  /*
  if (viewSel->isEntropyLow())
    state = NBVState::TERMINATION_MET;
  */
}
