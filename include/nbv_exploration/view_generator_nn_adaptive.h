#ifndef NBV_EXPLORATION_VIEW_GENERATOR_NN_ADAPTIVE_H
#define NBV_EXPLORATION_VIEW_GENERATOR_NN_ADAPTIVE_H

#include "nbv_exploration/view_generator_nn.h"
#include "nbv_exploration/common.h"


// Nearest neighbor
class ViewGeneratorNNAdaptive : public ViewGeneratorNN
{
public:
  ViewGeneratorNNAdaptive();
  void generateViews();
  std::string getMethodName();

private:
  int minima_iterations_;
  double minima_threshold_;
  bool isStuckInLocalMinima();

protected:
  double scale_factor_;
};

#endif
