#ifndef NBV_EXPLORATION_VIEW_GENERATOR_FRONTIER_H
#define NBV_EXPLORATION_VIEW_GENERATOR_FRONTIER_H

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/common.h"


struct KeyIndex{
  octomap::OcTreeKey key;
  int index;
};

class ViewGeneratorFrontier : public ViewGeneratorBase
{
public:
  ViewGeneratorFrontier();
  void generateViews();
  std::string getMethodName();

protected:
  double density_threshold_;
  int minimum_frontier_size_;
  int nearest_frontiers_count_; // number of frontiers to extract when finding the nearest frontiers
  double cylinder_radius_; //radius of sampling cylinder
  double cylinder_height_;

  ros::Publisher pub_vis_frontier_points_;

  std::vector<std::vector<octomap::OcTreeKey> > findFrontierAdjacencies(std::vector<octomap::OcTreeKey>& cells);
  std::vector<octomap::OcTreeKey> findFrontierCells();
  std::vector<std::vector<octomap::OcTreeKey> > findFrontiers();
  std::vector<octomap::OcTreeKey> findLowDensityCells();


  bool isNear(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  bool isNodeFree(octomap::OcTreeNode node);
  bool isNodeOccupied(octomap::OcTreeNode node);
  bool isNodeUnknown(octomap::OcTreeNode node);
};

#endif
