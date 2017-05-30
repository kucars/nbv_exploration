#ifndef TERMINATION_CHECK_MAX_ITERATIONS_H
#define TERMINATION_CHECK_MAX_ITERATIONS_H

#include "nbv_exploration/termination_check_base.h"

class TerminationCheckMaxIterations: public TerminationCheckBase
{
public:
  TerminationCheckMaxIterations();
  bool isTerminated();

protected:
  int max_iterations_;
};

#endif // TERMINATIONCHECKBASE_H
