#include "nbv_exploration/termination_check_base.h"

TerminationCheckBase::TerminationCheckBase()
{
}

bool TerminationCheckBase::isTerminated()
{
  std::cout << "[TerminationCheckBase]: " << cc.yellow << "update() not defined. Used derived class with proper implimentation.\n" << cc.reset;
  return false;
}

void TerminationCheckBase::setViewSelecter(ViewSelecterBase* v)
{
  view_selecter_ = v;
}

void TerminationCheckBase::update()
{
  std::cout << "[TerminationCheckBase]: " << cc.yellow << "update() not defined. Used derived class with proper implimentation.\n" << cc.reset;
}
