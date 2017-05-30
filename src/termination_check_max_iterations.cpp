#include "nbv_exploration/termination_check_max_iterations.h"

TerminationCheckMaxIterations::TerminationCheckMaxIterations()
{
  ros::param::param("~termination_iterations_max", max_iterations_, 300);
}

bool TerminationCheckMaxIterations::isTerminated()
{
  if (nbv_history_->iteration >= max_iterations_)
    return true;

  // See if the last few moves are repeating
  if (nbv_history_->isRepeatingMotions(repeat_window_size_))
    return true;

  return false;
}
