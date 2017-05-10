#include <iostream>
#include <ros/ros.h>

#include "nbv_exploration/view_generator_nn_frontier.h"
#include "nbv_exploration/common.h"

ViewGeneratorNNFrontier::ViewGeneratorNNFrontier():
  ViewGeneratorNNAdaptive() //Call base class constructor
{
  ros::param::param<int>("~view_generator_frontier_local_minima_iterations", frontier_minima_iterations_, 3);
  ros::param::param<double>("~view_generator_frontier_local_minima_threshold", frontier_minima_threshold_, 0.01);
}

bool ViewGeneratorNNFrontier::isStuckInLocalMinima()
{
  if (frontier_minima_iterations_ > nbv_history_->iteration)
    return false;

  double max_change = nbv_history_->getMaxEntropyDiffPerVoxel(frontier_minima_iterations_);

  if (max_change < frontier_minima_threshold_)
    return true;

  return false;
}

void ViewGeneratorNNFrontier::generateViews()
{
  if (ViewGeneratorNNFrontier::isStuckInLocalMinima())
  {
    std::cout << "[ViewGeneratorNNFrontier]: " << cc.yellow << "Local minima detected. Using frontiers\n" << cc.reset;
    generator_frontier_.generateViews();
    generated_poses = generator_frontier_.generated_poses;
  }
  else
  {
    ViewGeneratorNNAdaptive::generateViews();
  }
}

std::string ViewGeneratorNNFrontier::getMethodName()
{
  return "Frontier after Adaptive NN";
}

void ViewGeneratorNNFrontier::setCollisionRadius(double r)
{
  ViewGeneratorNNAdaptive::setCollisionRadius(r);
  generator_frontier_.setCollisionRadius(r);
}

void ViewGeneratorNNFrontier::setCloud(PointCloudXYZ::Ptr in_occ_cloud)
{
  ViewGeneratorNNAdaptive::setCloud(in_occ_cloud);
  generator_frontier_.setCloud(in_occ_cloud);
}

void ViewGeneratorNNFrontier::setCurrentPose(Pose p)
{
  ViewGeneratorNNAdaptive::setCurrentPose(p);
  generator_frontier_.setCurrentPose(p);
}

void ViewGeneratorNNFrontier::setDebug(bool b)
{
  ViewGeneratorNNAdaptive::setDebug(b);
  generator_frontier_.setDebug(b);
}

void ViewGeneratorNNFrontier::setHistory(NBVHistory* h)
{
  ViewGeneratorNNAdaptive::setHistory(h);
  generator_frontier_.setHistory(h);
}

void ViewGeneratorNNFrontier::setNavigationBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  ViewGeneratorNNAdaptive::setNavigationBounds(x_min, x_max, y_min, y_max, z_min, z_max);
  generator_frontier_.setNavigationBounds(x_min, x_max, y_min, y_max, z_min, z_max);
}

void ViewGeneratorNNFrontier::setMap(octomap::OcTree* oct){
  ViewGeneratorNNAdaptive::setMap(oct);
  generator_frontier_.setMap(oct);
}

void ViewGeneratorNNFrontier::setMapPrediction(octomap::OcTree* oct)
{
  ViewGeneratorNNAdaptive::setMapPrediction(oct);
  generator_frontier_.setMapPrediction(oct);
}

void ViewGeneratorNNFrontier::setObjectBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  ViewGeneratorNNAdaptive::setObjectBounds(x_min, x_max, y_min, y_max, z_min, z_max);
  generator_frontier_.setObjectBounds(x_min, x_max, y_min, y_max, z_min, z_max);
}
