#ifndef NBV_EXPLORATION_VIEW_GENERATOR_NN_H
#define NBV_EXPLORATION_VIEW_GENERATOR_NN_H

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/common.h"


// Nearest neighbor
class ViewGeneratorNN : public ViewGeneratorBase
{
public:
  ViewGeneratorNN();

  void generateViews();
  void generateViews(bool generate_at_current_location);
  std::string getMethodName();
};

#endif
