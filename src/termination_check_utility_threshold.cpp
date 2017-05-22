#include "nbv_exploration/termination_check_utility_threshold.h"

TerminationCheckUtilityThreshold::TerminationCheckUtilityThreshold()
{
  ros::param::param<double>("~termination_utility_min_threshold", min_utility_threshold_, 0.01);
  ros::param::param<int>("~termination_window_size", window_size_, 1);
}

bool TerminationCheckUtilityThreshold::isTerminated()
{
  // If we haven't gone through enough iterations, continue
  if (window_size_ > nbv_history_->iteration)
    return false;

  // Find max entropy change in the past few iterations
  float utility = nbv_history_->getMaxUtility(window_size_);
  if (utility < min_utility_threshold_)
    return true;

  // See if the last few moves are repeating
  if (nbv_history_->isRepeatingMotions(repeat_window_size_))
    return true;

  return false;
}

void TerminationCheckUtilityThreshold::update()
{
}
