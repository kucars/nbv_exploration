#ifndef TERMINATION_CHECK_BASE_H
#define TERMINATION_CHECK_BASE_H

#include <ros/ros.h>
#include "nbv_exploration/common.h"
#include "nbv_exploration/view_selecter_base.h"

class TerminationCheckBase
{
public:
  TerminationCheckBase();
  virtual bool isTerminated();
  void setViewSelecter(ViewSelecterBase* v);
  virtual void update();

protected:
  ViewSelecterBase* view_selecter_;
};

#endif // TERMINATIONCHECKBASE_H
