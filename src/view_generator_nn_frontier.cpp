#include <iostream>
#include <ros/ros.h>

#include "nbv_exploration/view_generator_nn_frontier.h"
#include "nbv_exploration/common.h"

ViewGeneratorNNFrontier::ViewGeneratorNNFrontier():
  ViewGeneratorBase() //Call base class constructor
{
  ros::param::param<double>("~view_generator_nn_adaptive_local_minima_iterations", minima_iterations_, 3);
  ros::param::param<double>("~view_generator_nn_adaptive_local_minima_threshold", minima_threshold_, 0.01);
}

bool ViewGeneratorNNFrontier::isStuckInLocalMinima()
{
  if (minima_iterations_ > nbv_history_->iteration)
    return false;

  double max_change = nbv_history_->getMaxEntropyDiffPerVoxel(minima_iterations_);

  if (max_change < minima_threshold_)
    return true;

  return false;
}

void ViewGeneratorNNFrontier::generateViews()
{
  if (isStuckInLocalMinima())
  {
    std::cout << "[ViewGeneratorNNAdaptive]: " << cc.yellow << "Local minima detected. Using frontiers\n" << cc.reset;
    generator_frontier_.generateViews();
    generated_poses = generator_frontier_.generated_poses;
  }
  else
  {
    generator_nn_.generateViews();
    generated_poses = generator_nn_.generated_poses;
  }
}

void ViewGeneratorNNFrontier::setCollisionRadius(double r)
{
  ViewGeneratorBase::setCollisionRadius(r);
  generator_frontier_.setCollisionRadius(r);
  generator_nn_.setCollisionRadius(r);
}

void ViewGeneratorNNFrontier::setCloud(PointCloudXYZ::Ptr in_occ_cloud)
{
  ViewGeneratorBase::setCloud(in_occ_cloud);
  generator_frontier_.setCloud(in_occ_cloud);
  generator_nn_.setCloud(in_occ_cloud);
}

void ViewGeneratorNNFrontier::setCurrentPose(Pose p)
{
  ViewGeneratorBase::setCurrentPose(p);
  generator_frontier_.setCurrentPose(p);
  generator_nn_.setCurrentPose(p);
}

void ViewGeneratorNNFrontier::setDebug(bool b)
{
  ViewGeneratorBase::setDebug(b);
  generator_frontier_.setDebug(b);
  generator_nn_.setDebug(b);
}

void ViewGeneratorNNFrontier::setHistory(NBVHistory* h)
{
  ViewGeneratorBase::setHistory(h);
  generator_frontier_.setHistory(h);
  generator_nn_.setHistory(h);
}

void ViewGeneratorNNFrontier::setNavigationBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  ViewGeneratorBase::setNavigationBounds(x_min, x_max, y_min, y_max, z_min, z_max);
  generator_frontier_.setNavigationBounds(x_min, x_max, y_min, y_max, z_min, z_max);
  generator_nn_.setNavigationBounds(x_min, x_max, y_min, y_max, z_min, z_max);
}

void ViewGeneratorNNFrontier::setMap(octomap::OcTree* oct){
  ViewGeneratorBase::setMap(oct);
  generator_frontier_.setMap(oct);
  generator_nn_.setMap(oct);
}

void ViewGeneratorNNFrontier::setMapPrediction(octomap::OcTree* oct)
{
  ViewGeneratorBase::setMapPrediction(oct);
  generator_frontier_.setMapPrediction(oct);
  generator_nn_.setMapPrediction(oct);
}

void ViewGeneratorNNFrontier::setObjectBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  ViewGeneratorBase::setObjectBounds(x_min, x_max, y_min, y_max, z_min, z_max);
  generator_frontier_.setObjectBounds(x_min, x_max, y_min, y_max, z_min, z_max);
  generator_nn_.setObjectBounds(x_min, x_max, y_min, y_max, z_min, z_max);
}
