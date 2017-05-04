#ifndef TERMINATION_CHECK_BASE_H
#define TERMINATION_CHECK_BASE_H

#include <ros/ros.h>
#include "nbv_exploration/common.h"
#include "nbv_exploration/nbv_history.h"
#include "nbv_exploration/view_selecter_base.h"

class TerminationCheckBase
{
public:
  TerminationCheckBase();
  virtual bool isTerminated();
  void setHistory(NBVHistory* h);
  virtual void update();

protected:
  NBVHistory* nbv_history_;
};

#endif // TERMINATIONCHECKBASE_H
