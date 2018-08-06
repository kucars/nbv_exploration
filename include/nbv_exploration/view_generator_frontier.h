#ifndef NBV_EXPLORATION_VIEW_GENERATOR_FRONTIER_H
#define NBV_EXPLORATION_VIEW_GENERATOR_FRONTIER_H

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/PoseArray.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>

#include "sspp/sensors.h"
#include "culling/occlusion_culling.h"

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/view_selecter_base.h"
#include "nbv_exploration/common.h"

#include <eigen3/Eigen/Dense>

#define SQRT2 0.70711
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
//  int nearest_frontiers_count_; // number of frontiers to extract when finding the nearest frontiers
  double cylinder_radius_; //radius of sampling cylinder
  double cylinder_height_;

  std::vector<double> cameraPitch_;
  std::vector<double> cameraHorizontalFoV_;
  std::vector<double> cameraVerticalFoV_;
  double maxDist_ ;
  std::vector<std::vector<Eigen::Vector3d> > camBoundNormals_;
  ViewSelecterBase* viewBase;
  ros::NodeHandle ros_node;

  ros::Publisher pub_vis_frontier_points_;
  ros::Publisher pub_vis_points_;
  ros::Publisher pub_vis_centroid_points_;
  ros::Publisher pub_marker_normals_;
  ros::Publisher pub_marker_planes_;
//  ros::Publisher pub_marker_lines_;

  std::vector<std::vector<octomap::OcTreeKey> > findFrontierAdjacencies(std::vector<octomap::OcTreeKey>& cells);
  std::vector<octomap::OcTreeKey> findFrontierCells();
  std::vector<std::vector<octomap::OcTreeKey> > findFrontiers();
  void findFrontiersPCL(std::vector<pcl::PointCloud<pcl::PointXYZ> >& clusters_frontiers_vec,  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& clusters_frontiers_ptr_vec,  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  std::vector<octomap::OcTreeKey> findLowDensityCells();
  void visualizeNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, geometry_msgs::PoseArray& normal_poses, bool pcl_visualize);
  void setCameraParams(std::vector<double> cameraPitch, std::vector<double> cameraHorizontalFoV, std::vector<double> cameraVerticalFoV, double maxDist);
  bool pointInFOV(Eigen::Vector4d state, pcl::PointXYZ pt);


  bool isNear(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  bool isNodeFree(octomap::OcTreeNode node);
  bool isNodeOccupied(octomap::OcTreeNode node);
  bool isNodeUnknown(octomap::OcTreeNode node);
};

#endif
