#include <iostream>
#include <queue>          // std::queue
#include <stdlib.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "nbv_exploration/view_generator_frontier.h"
#include "nbv_exploration/common.h"
#include <sensor_msgs/PointCloud2.h>

ViewGeneratorFrontier::ViewGeneratorFrontier():
  ViewGeneratorBase() //Call base class constructor
{

  ros::param::param<int>("~view_generator_frontier_minimum_size", minimum_frontier_size_, 10);
  ros::param::param<int>("~view_generator_frontier_nearest_count", nearest_frontiers_count_, 10);
  ros::param::param<double>("~view_generator_frontier_cylinder_radius", cylinder_radius_, 3.0);
  ros::param::param<double>("~view_generator_frontier_cylinder_height", cylinder_height_, 1.0);
  ros::param::param<double>("~view_generator_frontier_density_threshold", density_threshold_, 10);

  ros::NodeHandle ros_node;
  pub_vis_frontier_points_ = ros_node.advertise<sensor_msgs::PointCloud2>("nbv_exploration/generation/frontier_points", 10);
}

std::vector<std::vector<octomap::OcTreeKey> >
ViewGeneratorFrontier::findFrontierAdjacencies(std::vector<octomap::OcTreeKey>& cells)
{
  std::vector<std::vector<octomap::OcTreeKey> > list;
  int cell_count = cells.size();

  std::vector<bool> was_cell_checked;
  was_cell_checked.assign(cell_count, false); //fill the vector with "false"

  for (int i_cell=0; i_cell<cell_count && ros::ok(); i_cell++)
  {
    // Cell was already checked, move along
    if (was_cell_checked[i_cell])
      continue;

    // key_list contains the final linked keys
    // key_queue is pushed and popped recursively until we find all linked cells
    std::vector<octomap::OcTreeKey> key_list;
    std::queue<KeyIndex> key_queue;

    KeyIndex ki;
    ki.key = cells[i_cell];
    ki.index = i_cell;

    key_queue.push(ki);

    // @todo: requires HEAVY optimization
    // Looks around the current cell for linked cells
    while (key_queue.size() > 0)
    {
      KeyIndex ki_dequeue = key_queue.front();
      key_queue.pop();

      if (was_cell_checked[ki_dequeue.index])
        continue;
      was_cell_checked[ki_dequeue.index] = true;
      key_list.push_back(ki_dequeue.key);

      // Check if any remaining cells are neighbors
      for (int i_check=0; i_check<cell_count; i_check++)
      {
        if (was_cell_checked[i_check])
          continue;

        if ( isNear(cells[ki_dequeue.index], cells[i_check]) )
        {
          KeyIndex ki_new;
          ki_new.key = cells[i_check];
          ki_new.index = i_check;

          key_queue.push(ki_new);
        }
      } //end checking for neighbors
    } //end queue

    // Not enough frontier cells, do not count this as a frontier
    if (key_list.size() < minimum_frontier_size_)
      continue;

    list.push_back(key_list);
  } //end for

  return list;
}

std::vector<octomap::OcTreeKey> ViewGeneratorFrontier::findFrontierCells()
{
  int treeDepth = 16;
  std::vector<octomap::OcTreeKey> frontier_keys;


  for (octomap::OcTree::iterator it = tree_->begin(treeDepth), end = tree_->end(); it != end; ++it)
  {
    if (!isNodeUnknown(*it))
      continue;

    // check if cell is frontier (has 1 free and 1 occupied near it)
    bool found_free = false;
    bool found_occ = false;

    octomap::OcTreeKey key;
    octomap::OcTreeKey nKey = it.getKey();

    for (int k=-1; k <= 1; ++k)
    {
      key[2] = nKey[2] + k;

      for (int j=-1; j <= 1; ++j)
      {
        key[1] = nKey[1] + j;

        for (int i=-1; i <= 1; ++i)
        {
          key[0] = nKey[0] + i;

          if (key == nKey)
            continue;

          octomap::OcTreeNode* node = tree_->search(key);

          if (!node)
            continue;
          else if ( isNodeFree(*node) )
            found_free = true;
          else if ( isNodeOccupied(*node) )
            found_occ = true;
        } //end i: key[0]
      } //end j: key[1]
    } //end k: key[2]
    // end proximity check

    if (found_free && found_occ)
      frontier_keys.push_back(nKey);
  } //end for

  return frontier_keys;
}



std::vector<std::vector<octomap::OcTreeKey> > ViewGeneratorFrontier::findFrontiers()
{
  /*
  std::vector<octomap::OcTreeKey> frontier_cells = findFrontierCells();
  std::vector<std::vector<octomap::OcTreeKey> > frontier_list;
  frontier_list = findFrontierAdjacencies(frontier_cells);
  */

  std::vector<octomap::OcTreeKey> low_density_cells = findLowDensityCells();
  std::vector<std::vector<octomap::OcTreeKey> > density_list;
  density_list  = findFrontierAdjacencies(low_density_cells);

  //printf("[ViewGeneratorFrontier] Frontiers -- Entropy: %lu\tDensity: %lu\n", frontier_list.size(), density_list.size() );


  std::vector<std::vector<octomap::OcTreeKey> > final_list;
  //final_list.insert(final_list.end(), frontier_list.begin(), frontier_list.end());
  final_list.insert(final_list.end(), density_list.begin(), density_list.begin()+1);

  return final_list;
}

std::vector<octomap::OcTreeKey> ViewGeneratorFrontier::findLowDensityCells()
{
  int treeDepth = 16;
  std::vector<octomap::OcTreeKey> density_keys;

  for (octomap::OcTree::iterator it = tree_->begin(treeDepth), end = tree_->end(); it != end; ++it)
  {
    octomap::OcTreeKey key = it.getKey();
    // Ignore high density cells and invalid keys

    int density = mapping_module_->getDensityAtOcTreeKey(key);
    if (density < 0 || density >= density_threshold_)
      continue;

    //Store low density keys
    density_keys.push_back( key );
  } //end for

  return density_keys;
}


void ViewGeneratorFrontier::generateViews()
{
  generated_poses.clear();
  std::vector<geometry_msgs::Pose> rejected_poses;

  mapping_module_->updateVoxelDensities();

  // ==========
  // Find frontiers
  // ==========
  timer.start("[ViewGeneratorFrontier]findFrontiers");
  std::vector<std::vector<octomap::OcTreeKey> > frontiers = findFrontiers();
  if (frontiers.size() == 0)
    return;

  timer.stop("[ViewGeneratorFrontier]findFrontiers");

  // ===========
  // Get centroids of frontiers
  // ===========
  timer.start("[ViewGeneratorFrontier]getCentroids");
  PointCloudXYZ::Ptr centroid_cloud (new PointCloudXYZ);
  PointCloudXYZ::Ptr visualization_cloud (new PointCloudXYZ);

  for (int i_f=0; i_f<frontiers.size(); i_f++)
  {
    std::vector<octomap::OcTreeKey> f = frontiers[i_f];
    int c_count = f.size();

    PointXYZ centroid;
    centroid.x=0; centroid.y=0; centroid.z=0;

    for (int i_c=0; i_c<c_count; i_c++)
    {
      octomap::point3d pt = tree_->keyToCoord(f[i_c]);
      centroid.x += pt.x()/c_count;
      centroid.y += pt.y()/c_count;
      centroid.z += pt.z()/c_count;

      visualization_cloud->points.push_back( PointXYZ(pt.x(), pt.y(), pt.z()) );
    }

    centroid_cloud->points.push_back(centroid);
  }
  timer.stop("[ViewGeneratorFrontier]getCentroids");

  // Visualize
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*visualization_cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  pub_vis_frontier_points_.publish(cloud_msg);

  // ============
  // Get nearest frontiers
  // ============
  timer.start("[ViewGeneratorFrontier]getNearestFrontier");
  pcl::KdTreeFLANN<PointXYZ> kdtree;
  kdtree.setInputCloud (centroid_cloud);
  std::vector<int> pointIdxNKNSearch(nearest_frontiers_count_);
  std::vector<float> pointNKNSquaredDistance(nearest_frontiers_count_);

  PointXYZ current_pt (
        current_pose_.position.x,
        current_pose_.position.y,
        current_pose_.position.z
        );

  if ( kdtree.nearestKSearch (current_pt, nearest_frontiers_count_, pointIdxNKNSearch, pointNKNSquaredDistance) == 0 )
    return;

  for (int i=0; i<pointIdxNKNSearch.size(); i++)
  {
    int idx = pointIdxNKNSearch[i];
    PointXYZ centroid = centroid_cloud->points[idx];

    // Generate poses around the centroid
    for (int z_inc=-1; z_inc<=1; z_inc+=1)
    {
      for (double theta=0; theta<=2*M_PI; theta+=M_PI_2)
      {
        Pose pose;
        pose.position.x = centroid.x + cylinder_radius_*cos(theta);
        pose.position.y = centroid.y + cylinder_radius_*sin(theta);
        pose.position.z = centroid.z + z_inc*cylinder_height_;
        pose.orientation = pose_conversion::getQuaternionFromYaw(M_PI+theta); // Point to center of cylinder

        timer.start("[ViewGeneratorFrontier]validity");
        if ( isValidViewpoint(pose) )
          generated_poses.push_back(pose);
        else
          rejected_poses.push_back(pose);
        timer.stop("[ViewGeneratorFrontier]validity");
      }
    }
  }
  timer.stop("[ViewGeneratorFrontier]getNearestFrontier");

  // Visualize
  std::cout << "[ViewGeneratorNN] Generated " << generated_poses.size() << " poses (" << rejected_poses.size() << " rejected)" << std::endl;
  ViewGeneratorBase::visualize(generated_poses, rejected_poses);
}

std::string ViewGeneratorFrontier::getMethodName()
{
  return "Frontier";
}

bool ViewGeneratorFrontier::isNear(octomap::OcTreeKey k1, octomap::OcTreeKey k2)
{
  if (abs(k1[0]-k2[0]) <=2 &&
      abs(k1[1]-k2[1]) <=2 &&
      abs(k1[2]-k2[2]) <=2)
    return true;

  return false;
}

bool ViewGeneratorFrontier::isNodeFree(octomap::OcTreeNode node)
{
  //if (node.getOccupancy() <= 1-tree_->getOccupancyThres())
  if (node.getOccupancy() <= 0.4)
    return true;

  return false;
}

bool ViewGeneratorFrontier::isNodeOccupied(octomap::OcTreeNode node)
{
  //if (node.getOccupancy() >= tree_->getOccupancyThres())
  if (node.getOccupancy() >= 0.6)
    return true;

  return false;
}

bool ViewGeneratorFrontier::isNodeUnknown(octomap::OcTreeNode node)
{
  return !isNodeFree(node) && !isNodeOccupied(node);
}
