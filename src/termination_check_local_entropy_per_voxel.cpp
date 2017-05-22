#include "nbv_exploration/termination_check_local_entropy_per_voxel.h"

TerminationCheckLocalEntropyPerVoxel::TerminationCheckLocalEntropyPerVoxel()
{
  ros::param::param<double>("~termination_entropy_local_min_change_threshold", min_entropy_threshold_, 0.01);
  ros::param::param<int>("~termination_window_size", window_size_, 1);
}

bool TerminationCheckLocalEntropyPerVoxel::isTerminated()
{
  // If we haven't gone through enough iterations, continue
  if (window_size_ > nbv_history_->iteration)
    return false;

  // Find max entropy change in the past few iterations
  float max_change = nbv_history_->getMaxEntropyDiffPerVoxel(window_size_);

  if (fabs(max_change) < min_entropy_threshold_)
    return true;

  return false;
}

void TerminationCheckLocalEntropyPerVoxel::update()
{
  /*
  std::cout << "[TerminationCheckLocalEntropyPerVoxel]: " << cc.green
            << "Entropy: " << nbv_history_->total_entropy.back()
            << "\tEntropy change: " << nbv_history_->avg_entropy_diff_per_voxel.back() << "\n" << cc.reset;
  */
}
