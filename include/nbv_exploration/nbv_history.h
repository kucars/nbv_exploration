#ifndef NBV_EXPLORATION_HISTORY_H
#define NBV_EXPLORATION_HISTORY_H

#include <ros/ros.h>
#include "nbv_exploration/common.h"

class NBVHistory
{
public:
  NBVHistory();
  double getMaxEntropyDiffPerVoxel (int N_iterations);
  double getMaxUtility (int N_iterations);
  bool isRepeatingMotions(int window_size);
  void update();

  int iteration;
  std::vector<geometry_msgs::Pose> selected_poses;
  std::vector<float> selected_utility;
  std::vector<float> selected_utility_entropy;
  std::vector<float> selected_utility_density;
  std::vector<float> selected_utility_prediction;
  std::vector<float> selected_utility_occupied_voxels;

  std::vector<float> total_entropy;
  std::vector<float> entropy_diff;
  std::vector<float> avg_entropy_diff_per_voxel;
  std::vector<float> avg_point_density;
  std::vector<float> voxels_observed;

  std::vector<float> time_per_iteration;



protected:
  double num_voxels_in_view_;

  void computeEntropyDiff();
};

#endif // end include
