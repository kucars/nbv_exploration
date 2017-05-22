#include "nbv_exploration/termination_check_max_iterations.h"

TerminationCheckMaxIterations::TerminationCheckMaxIterations():
  current_iteration_(0)
{
  ros::param::param("~termination_iterations_max", max_iterations_, 300);
}

void TerminationCheckMaxIterations::update()
{
  current_iteration_++;
  std::cout << "[TerminationCheckMaxIterations]: " << cc.green << "Iteration " << current_iteration_ << "\n" << cc.reset;
}

bool TerminationCheckMaxIterations::isTerminated()
{
  if (current_iteration_ > max_iterations_)
    return true;

  // See if the last few moves are repeating
  if (nbv_history_->isRepeatingMotions(repeat_window_size_))
    return true;

  return false;
}
