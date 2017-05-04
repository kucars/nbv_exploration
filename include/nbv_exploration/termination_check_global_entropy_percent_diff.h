#ifndef TERMINATION_CHECK_GLOBAL_ENTROPY_PERCENTAGE_DIFFERENCE_H
#define TERMINATION_CHECK_GLOBAL_ENTROPY_PERCENTAGE_DIFFERENCE_H

#include "nbv_exploration/termination_check_base.h"

class TerminationCheckGlobalEntropyPercentageDifference: public TerminationCheckBase
{
public:
  TerminationCheckGlobalEntropyPercentageDifference();
  void update();
  bool isTerminated();

protected:
  std::vector<double> entropy_history_;
  std::vector<double> entropy_change_history_;
  double min_entropy_threshold_;
  int window_size_;
};

#endif // TERMINATIONCHECKBASE_H
