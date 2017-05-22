#ifndef TERMINATION_CHECK_UTILITY_THRESHOLD_H
#define TERMINATION_CHECK_UTILITY_THRESHOLD_H

#include "nbv_exploration/termination_check_base.h"

class TerminationCheckUtilityThreshold: public TerminationCheckBase
{
public:
  TerminationCheckUtilityThreshold();
  void update();
  bool isTerminated();

protected:
  double min_utility_threshold_;
  int window_size_;
};

#endif
