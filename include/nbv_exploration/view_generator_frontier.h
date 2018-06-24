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

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/common.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Segment_3.h>
#include <CGAL/AABB_triangle_primitive.h>


struct KeyIndex{
  octomap::OcTreeKey key;
  int index;
};

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Exact_predicates_exact_constructions_kernel exactKernel;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Plane_3 Plane3;
typedef K::Line_3 Line1;
typedef K::Point_3 Point;
typedef K::Segment_3 Segment;
typedef K::Direction_3 Direction;
typedef K::Triangle_3 CGALTriangle;
typedef std::list<CGALTriangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree1;
typedef CGAL::Cartesian_converter<K,exactKernel > SimpleToExactConverter;

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
  ros::Publisher pub_vis_centroid_points_;
  ros::Publisher pub_marker_normals_;

  std::vector<std::vector<octomap::OcTreeKey> > findFrontierAdjacencies(std::vector<octomap::OcTreeKey>& cells);
  std::vector<octomap::OcTreeKey> findFrontierCells();
  std::vector<std::vector<octomap::OcTreeKey> > findFrontiers();
  void findFrontiersPCL(std::vector<pcl::PointCloud<pcl::PointXYZ> >& clusters_frontiers_vec,  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& clusters_frontiers_ptr_vec,  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  std::vector<octomap::OcTreeKey> findLowDensityCells();
  void visualizeNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, geometry_msgs::PoseArray& normal_poses, bool pcl_visualize);
  void findClusterBB(pcl::PointCloud<pcl::PointXYZ> clusterPoints, geometry_msgs::Vector3& gridSize, geometry_msgs::Pose& gridStart);


  bool isNear(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  bool isNodeFree(octomap::OcTreeNode node);
  bool isNodeOccupied(octomap::OcTreeNode node);
  bool isNodeUnknown(octomap::OcTreeNode node);
};

#endif
