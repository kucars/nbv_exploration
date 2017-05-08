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

ViewGeneratorFrontier::ViewGeneratorFrontier():
  ViewGeneratorBase() //Call base class constructor
{

}

std::vector<std::vector<octomap::OcTreeKey> > ViewGeneratorFrontier::findFrontiers()
{
  std::vector<std::vector<octomap::OcTreeKey> > frontier_list;

  std::vector<octomap::OcTreeKey> frontier_cells = findFrontierCells();
  int cell_count = frontier_cells.size();

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
    ki.key = frontier_cells[i_cell];
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

        if ( isNear(frontier_cells[ki_dequeue.index], frontier_cells[i_check]) )
        {
          KeyIndex ki_new;
          ki_new.key = frontier_cells[i_check];
          ki_new.index = i_check;

          key_queue.push(ki_new);
        }
      } //end checking for neighbors
    } //end queue

    frontier_list.push_back(key_list);
  } //end for

  return frontier_list;
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

  std::cout << "[ViewpointGeneratorFrontier] " << cc.red << "Found " << frontier_keys.size() << " frontier cells\n" << cc.reset;

  return frontier_keys;
}

void ViewGeneratorFrontier::generateViews()
{
  generated_poses.clear();

  // ==========
  // Estimate points normals (parallelized)
  // ==========
  PointCloudN::Ptr cloud_normals (new PointCloudN);
  pcl::search::KdTree<PointXYZ>::Ptr kdtree_est (new pcl::search::KdTree<PointXYZ> ());
  pcl::NormalEstimationOMP<PointXYZ, PointN> norm_est;

  norm_est.setSearchMethod (kdtree_est);
  norm_est.setInputCloud (cloud_occupied_ptr_);
  norm_est.setKSearch (50);
  norm_est.compute (*cloud_normals);

  // ==========
  // Set up nearest neighbor search
  // ==========
  pcl::KdTreeFLANN<PointN> kdtree;
  kdtree.setInputCloud (cloud_normals);
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);




  // ==========
  // Find frontiers
  // ==========
  std::vector<std::vector<octomap::OcTreeKey> > frontiers = findFrontiers();

  // ===========
  // Get centroids of frontiers
  // ===========
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

      /*
      Pose pose;
      pose.position.x = pt.x();
      pose.position.y = pt.y();
      pose.position.z = pt.z();
      generated_poses.push_back(pose);
      */

    }

    // Get node normal
    /*
    std::vector< octomap::point3d > normals;
    octomap::point3d centroid_octo(
          centroid.x,
          centroid.y,
          centroid.z);

    tree_->getNormals(centroid_octo, normals);

    printf("Centroid: [%3.2lf, %3.2lf, %3.2lf], Count: %d, Normals: %lu\n", centroid.x, centroid.y, centroid.z, c_count, normals.size());

    Eigen::Vector3d v (0,0,0);

    for(unsigned i = 0; i < normals.size(); ++i)
    {
      v[0] += normals[i].x();
      v[1] += normals[i].y();
      v[2] += normals[i].z();

      std::cout << "\t" << normals[i].x() << "; " << normals[i].y() << "; " << normals[i].z() << std::endl;
    }

    v.normalize();
    std::cout << "\tFinal: " << v[0] << "; " << v[1] << "; " << v[2] << std::endl;
    */

    // ==============
    // Save pose for visualization
    // =============
    Pose pose;
    pose.position.x = centroid.x;
    pose.position.y = centroid.y;
    pose.position.z = centroid.z;

    /*
    if (normals.size() > 0)
      pose.orientation = pose_conversion::getQuaternionFromDirectionVector(v);
    */

    // ============
    // Get node normal
    // ============

    PointN searchPoint;
    searchPoint.x = centroid.x;
    searchPoint.y = centroid.y;
    searchPoint.z = centroid.z;

    pointIdxNKNSearch.clear();
    pointNKNSquaredDistance.clear();

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      // Find closest point in original cloud
      int idx = pointIdxNKNSearch[0];
      PointN pt_nn = cloud_normals->points[idx];

      Eigen::Vector3d v(
          pt_nn.normal[0],
          pt_nn.normal[1],
          pt_nn.normal[2]
          );

      std::cout << "\tFinal: " << v[0] << "; " << v[1] << "; " << v[2] << std::endl;
      pose.orientation = pose_conversion::getQuaternionFromDirectionVector(v);
    }


    generated_poses.push_back(pose);

  }

  // Visualize
  std::vector<geometry_msgs::Pose> invalid_poses;
  ViewGeneratorBase::visualize(generated_poses, invalid_poses);
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
