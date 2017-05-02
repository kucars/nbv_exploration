#include "nbv_exploration/termination_check_entropy_stagnation.h"

TerminationCheckEntropyStagnation::TerminationCheckEntropyStagnation()
{
  ros::param::param<double>("~termination_entropy_min_change_threshold", min_entropy_threshold_, 0.01);
  ros::param::param<int>("~termination_entropy_window_size", window_size_, 1);
}

bool TerminationCheckEntropyStagnation::isTerminated()
{
  // If we haven't gone through enough iterations, continue
  if (entropy_change_history_.size() < window_size_)
    return false;

  // Find max entropy change in the past few iterations
  float max_change = 0;
  int end = entropy_change_history_.size() - 1;

  for (int i=0; i<window_size_; i++)
  {
    float change = entropy_change_history_[end-i];
    if (change > max_change)
      max_change = change;
  }


  if (max_change < min_entropy_threshold_)
    return true;

  return false;
}

void TerminationCheckEntropyStagnation::update()
{
  // Get last entropy value and append it
  float entropy_current = view_selecter_->info_entropy_total_;
  entropy_history_.push_back(entropy_current);

  int end = entropy_history_.size()-1;
  if (end == 0)
    entropy_change_history_.push_back(1);
  else
  {
    // Compute difference in entropy as a percentage difference
    double entropy_prev = entropy_history_[end-1];
    double change = (entropy_prev-entropy_current)/((entropy_prev+entropy_current)/2.0);
    change = fabs(change); // Make it positive
    entropy_change_history_.push_back( change );
  }

  std::cout << "[TerminationCheckEntropyStagnation]: " << cc.green << "Entropy: " << entropy_history_.back() << "\tEntropy change: " << entropy_change_history_.back()*100.0 << "%\n" << cc.reset;
}
