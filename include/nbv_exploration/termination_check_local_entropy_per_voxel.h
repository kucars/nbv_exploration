#ifndef TERMINATION_CHECK_LOCAL_ENTROPY_PER_VOXEL_H
#define TERMINATION_CHECK_LOCAL_ENTROPY_PER_VOXEL_H

#include "nbv_exploration/termination_check_base.h"

class TerminationCheckLocalEntropyPerVoxel: public TerminationCheckBase
{
public:
  TerminationCheckLocalEntropyPerVoxel();
  void update();
  bool isTerminated();

protected:
  std::vector<double> entropy_history_;
  std::vector<double> entropy_change_history_;
  double min_entropy_threshold_;
  int window_size_;
  double num_voxels_;

  bool is_error_initializing_;
};

#endif // TERMINATIONCHECKBASE_H
