#ifndef NBV_EXPLORATION_VIEW_GENERATOR_ADAPTIVE_NN_H
#define NBV_EXPLORATION_VIEW_GENERATOR_ADAPTIVE_NN_H

#include "nbv_exploration/view_generator_nn.h"
#include "nbv_exploration/common.h"


// Nearest neighbor
class ViewGeneratorAdaptiveNN : public ViewGeneratorNN
{
public:
  ViewGeneratorAdaptiveNN();

  void generateViews();

protected:
  bool isStuckInLocalMinima();

  double scale_factor_;
  double minima_iterations_;
  double minima_threshold_;

};

#endif
