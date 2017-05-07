#ifndef NBV_EXPLORATION_HISTORY_H
#define NBV_EXPLORATION_HISTORY_H

#include <ros/ros.h>
#include "nbv_exploration/common.h"

class NBVHistory
{
public:
  NBVHistory();
  double getMaxEntropyDiffPerVoxel (int N_iterations);
  void update();

  int iteration;
  std::vector<geometry_msgs::Pose> selected_poses;
  std::vector<float> selected_utility;
  std::vector<float> total_entropy;
  std::vector<float> entropy_diff;
  std::vector<float> avg_entropy_diff_per_voxel;

protected:
  double num_voxels_;

  void computeEntropyDiff();
};

#endif // end include
