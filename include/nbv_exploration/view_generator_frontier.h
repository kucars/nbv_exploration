#ifndef NBV_EXPLORATION_VIEW_GENERATOR_FRONTIER_H
#define NBV_EXPLORATION_VIEW_GENERATOR_FRONTIER_H

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/common.h"


// Frontier
class ViewGeneratorFrontier : public ViewGeneratorBase
{
public:
  ViewGeneratorFrontier();

  void generateViews();
};

#endif
