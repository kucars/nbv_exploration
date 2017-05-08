#ifndef NBV_EXPLORATION_VIEW_GENERATOR_FRONTIER_H
#define NBV_EXPLORATION_VIEW_GENERATOR_FRONTIER_H

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/common.h"


// Frontier
struct Frontier{
  std::vector<octomap::OcTreeKey> nodes;
  std::vector<PointXYZ> centroids;
};

struct KeyIndex{
  octomap::OcTreeKey key;
  int index;
};

class ViewGeneratorFrontier : public ViewGeneratorBase
{
public:
  ViewGeneratorFrontier();
  void generateViews();

protected:
  std::vector<std::vector<octomap::OcTreeKey> > findFrontiers();
  std::vector<octomap::OcTreeKey> findFrontierCells();

  bool isNear(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  bool isNodeFree(octomap::OcTreeNode node);
  bool isNodeOccupied(octomap::OcTreeNode node);
  bool isNodeUnknown(octomap::OcTreeNode node);
};

#endif
