#include "nbv_exploration/nbv_history.h"

NBVHistory::NBVHistory():
  iteration(0)
{
  // Load parameters to compute average entropy change per voxel

  double fov_vert, fov_hor, range_min, range_max, octree_res;
  ros::param::param<double>("~depth_range_max", range_max, -1);
  ros::param::param<double>("~depth_range_min", range_min, -1);
  ros::param::param<double>("~fov_vertical", fov_vert, -1);
  ros::param::param<double>("~fov_horizontal", fov_hor, -1);
  ros::param::param<double>("~mapping_octree_resolution", octree_res, -1);

  if (range_min < 0 || range_max <= 0 || fov_vert <= 0 || fov_hor <= 0 )
  {
    std::cout << "[TerminationCheckLocalEntropyPerVoxel]: " << cc.red << "ERROR: Camera settings no initialized. Exiting.\n" << cc.reset;
    return;
  }

  if (octree_res <= 0)
  {
    std::cout << "[TerminationCheckLocalEntropyPerVoxel]: " << cc.red << "ERROR: Mapping voxel resolution not set. Exiting.\n" << cc.reset;
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

  // Compute number of voxels in viewpoint (approximately)
  num_voxels_in_view_ = volume_frustum/volume_voxel;
}

void NBVHistory::computeEntropyDiff()
{
  if (iteration==0)
  {
    entropy_diff.push_back(0);
    avg_entropy_diff_per_voxel.push_back(0);
  }
  else
  {
    double diff = total_entropy[iteration] - total_entropy[iteration-1];
    entropy_diff.push_back(diff);
    avg_entropy_diff_per_voxel.push_back( fabs(diff/num_voxels_in_view_) );
  }
}

double NBVHistory::getMaxEntropyDiffPerVoxel (int N_iterations)
{
  if (N_iterations > iteration)
    return std::numeric_limits<double>::quiet_NaN();


  // Get max entropy difference per voxel in the past "N_iterations"
  double max_change = -std::numeric_limits<float>::infinity();
  for (int i=0; i<N_iterations; i++)
  {
    float change = avg_entropy_diff_per_voxel[iteration-i];
    if (change > max_change)
      max_change = change;
  }

  return max_change;
}

double NBVHistory::getMaxUtility (int N_iterations)
{
  if (N_iterations > iteration)
    return std::numeric_limits<double>::quiet_NaN();


  // Get max entropy difference per voxel in the past "N_iterations"
  double max_utility = -std::numeric_limits<float>::infinity();
  for (int i=0; i<N_iterations; i++)
  {
    float utility = selected_utility[iteration-i];
    if (utility > max_utility)
      max_utility = utility;
  }

  return max_utility;
}

bool NBVHistory::isRepeatingMotions(int window_size)
{
  // Checks if the last 4 moves are repeated, indicating a local minima
  if (window_size >= iteration)
    return false;

  for (int i=1; i<=window_size-2; i++)
  {
    if (selected_utility[iteration-i] != selected_utility[iteration-i-2])
      return false;
  }

  return true;
}

void NBVHistory::update()
{
  computeEntropyDiff();
  iteration++;
  //std::cout << "[NBVHistory]: " << cc.yellow << "update() not defined. Used derived class with proper implimentation.\n" << cc.reset;
}
