#include "nbv_exploration/termination_check_local_entropy_per_voxel.h"

TerminationCheckLocalEntropyPerVoxel::TerminationCheckLocalEntropyPerVoxel():
  is_error_initializing_(false)
{
  ros::param::param<double>("~termination_entropy_local_min_change_threshold", min_entropy_threshold_, 0.01);
  ros::param::param<int>("~termination_entropy_window_size", window_size_, 1);

  double fov_vert, fov_hor, range_min, range_max, octree_res;
  ros::param::param<double>("~depth_range_max", range_max, -1);
  ros::param::param<double>("~depth_range_min", range_min, -1);
  ros::param::param<double>("~fov_vertical", fov_vert, -1);
  ros::param::param<double>("~fov_horizontal", fov_hor, -1);
  ros::param::param<double>("~mapping_octree_resolution", octree_res, -1);


  if (range_min < 0 || range_max <= 0 || fov_vert <= 0 || fov_hor <= 0 )
  {
    std::cout << "[TerminationCheckLocalEntropyPerVoxel]: " << cc.red << "ERROR: Camera settings no initialized. Exiting.\n" << cc.reset;
    is_error_initializing_ = true;
    return;
  }

  if (octree_res <= 0)
  {
    std::cout << "[TerminationCheckLocalEntropyPerVoxel]: " << cc.red << "ERROR: Mapping voxel resolution not set. Exiting.\n" << cc.reset;
    is_error_initializing_ = true;
    return;
  }

  // Compute volume of FOV
  fov_vert *= M_PI/180.0;//Convert to radians
  fov_hor *= M_PI/180.0;//Convert to radians

  double angle_square = tan(fov_vert/2)*tan(fov_hor/2); // Denotes the area the square face of a pyramid whose height is 1
  double A1 = range_max*range_max*angle_square; //area of far face of frustum
  double A2 = range_min*range_min*angle_square; //area of near face of frustum
  double height = range_max - range_min;

  double volume_frustum = height*(A1 + A2 + sqrt(A1*A2))/3;

  // Compute volume of voxel
  double volume_voxel = octree_res*octree_res*octree_res;

  // Compute number of voxels (approximately)
  num_voxels_ = volume_frustum/volume_voxel;
}

bool TerminationCheckLocalEntropyPerVoxel::isTerminated()
{
  if (is_error_initializing_)
    true;

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

void TerminationCheckLocalEntropyPerVoxel::update()
{
  // Get last entropy value and append it
  float entropy_current = view_selecter_->info_entropy_total_;
  entropy_history_.push_back(entropy_current);

  int end = entropy_history_.size()-1;
  if (end == 0)
    entropy_change_history_.push_back(1);
  else
  {
    // Compute difference in entropy
    double entropy_prev = entropy_history_[end-1];
    double change = entropy_prev-entropy_current;

    // Get average change per voxel
    change = fabs(change/num_voxels_); // Make it positive
    entropy_change_history_.push_back( change );
  }

  std::cout << "[TerminationCheckLocalEntropyPerVoxel]: " << cc.green << "Entropy: " << entropy_history_.back() << "\tEntropy change: " << entropy_change_history_.back() << "\n" << cc.reset;
}
