#ifndef TERMINATIONCHECKBASE_H
#define TERMINATIONCHECKBASE_H

class TerminationCheckBase
{
public:
  int max_iterations_;
  int current_iteration_;

  TerminationCheckBase();
  void update();
  bool isTerminated();
};

#endif // TERMINATIONCHECKBASE_H
