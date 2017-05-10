#ifndef NBV_EXPLORATION_VIEW_GENERATOR_NN_FRONTIER_H
#define NBV_EXPLORATION_VIEW_GENERATOR_NN_FRONTIER_H

#include "nbv_exploration/view_generator_frontier.h"
#include "nbv_exploration/view_generator_nn_adaptive.h"
#include "nbv_exploration/common.h"


class ViewGeneratorNNFrontier : public ViewGeneratorNNAdaptive
{
public:
  ViewGeneratorNNFrontier();
  void generateViews();
  std::string getMethodName();

private:
  int frontier_minima_iterations_;
  double frontier_minima_threshold_;
  bool isStuckInLocalMinima();

protected:
  ViewGeneratorFrontier generator_frontier_;

  void setCollisionRadius(double r);
  void setCloud(PointCloudXYZ::Ptr in_occ_cloud);
  void setCurrentPose(Pose p);
  void setDebug(bool b);
  void setHistory(NBVHistory* h);
  void setNavigationBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
  void setMap(octomap::OcTree* oct);
  void setMapPrediction(octomap::OcTree* oct);
  void setObjectBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
};
#endif
