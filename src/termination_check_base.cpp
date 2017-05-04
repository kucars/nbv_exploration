#include "nbv_exploration/termination_check_base.h"

TerminationCheckBase::TerminationCheckBase()
{
}

bool TerminationCheckBase::isTerminated()
{
  std::cout << "[TerminationCheckBase]: " << cc.yellow << "update() not defined. Used derived class with proper implimentation.\n" << cc.reset;
  return false;
}

void TerminationCheckBase::setHistory(NBVHistory* h)
{
  nbv_history_ = h;
}

void TerminationCheckBase::update()
{
  std::cout << "[TerminationCheckBase]: " << cc.yellow << "update() not defined. Used derived class with proper implimentation.\n" << cc.reset;
}
