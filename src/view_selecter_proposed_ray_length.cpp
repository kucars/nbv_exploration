#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf_conversions/tf_eigen.h>

#include "nbv_exploration/view_selecter_proposed_ray_length.h"

ViewSelecterProposedRayLength::ViewSelecterProposedRayLength():
  ViewSelecterBase() //Call base class constructor
{
}

double ViewSelecterProposedRayLength::calculateUtility(geometry_msgs::Pose p)
{
  // Modified version of ViewSelecterBase::calculateIG()
  // Computes length of rays in original and predicted and uses those as weights for entropies
  //
  // Source: Borrowed partially from
  // https://github.com/uzh-rpg/rpg_ig_active_reconstruction/blob/master/ig_active_reconstruction_octomap/src/code_base/octomap_basic_ray_ig_calculator.inl
  double t_start, t_end;
  t_start = ros::Time::now().toSec();

  octomap::point3d origin (p.position.x, p.position.y, p.position.z);

  int nodes_traversed = 0;
  int nodes_free = 0;
  int nodes_occ = 0;
  int nodes_unknown = 0;
  int nodes_unobserved = 0;

  std::set<octomap::OcTreeKey, OctomapKeyCompare> nodes; //all nodes in a set are UNIQUE

  clearRayMarkers();
  double ig_total = 0;

  for (int i=0; i<rays_far_plane_at_pose_.size(); i++)
  {
    double ig_ray = 0;

    octomap::point3d endpoint, endpoint_predicted;
    double ray_len, ray_len_predicted;

    // Get length of beam to the far plane of sensor
    double range = rays_far_plane_at_pose_[i].norm();

    // Get the direction of the ray
    octomap::point3d dir = rays_far_plane_at_pose_[i].normalize();

    // Cast through unknown cells as well as free cells
    bool found_endpoint = tree_->castRay(origin, dir, endpoint, true, range);
    if (!found_endpoint)
    {
      endpoint = origin + dir * range;
      ray_len = range;
    }
    else
    {
      ray_len = (origin-endpoint).norm();
    }

    // Cast ray through predicted map
    found_endpoint = tree_predicted_->castRay(origin, dir, endpoint_predicted, true, range);
    if (!found_endpoint)
      ray_len_predicted = range;
    else
      ray_len_predicted = (origin-endpoint_predicted).norm();

    if (ray_len_predicted > ray_len)
      ray_len_predicted = ray_len;

    octomap::point3d start_pt, end_pt;
    bool entered_valid_range = false;
    bool exited_valid_range = false;

    /* Check ray
     *
     * We first draw a ray from the UAV to the endpoint
     *
     * For generality, we assume the UAV is outside the object bounds. We only consider nodes
     * 	that are inside the object bounds, and past the near-plane of the camera.
     *
     * The ray continues until the end point.
     * If the ray exits the bounds, we stop adding nodes to IG and discard the endpoint.
     */

    octomap::KeyRay ray;
    tree_->computeRayKeys( origin, endpoint, ray );

    for( octomap::KeyRay::iterator it = ray.begin() ; it!=ray.end(); ++it )
    {
      octomap::point3d p = tree_->keyToCoord(*it);

      if (!entered_valid_range)
      {
        // Find the first valid point
        if ( /*(origin-p).norm() >= min_dist &&*/ isPointInBounds(p) )
        {
          entered_valid_range = true;
          start_pt = p;
          end_pt = p;
        }

        else
          continue;
      }

      else
      {
        if (isPointInBounds(p))
          end_pt = p;

        else
        {
          exited_valid_range = true;
          break; //We've exited the valid range
        }
      }

      // Add point to IG
      octomap::OcTreeNode* node = tree_->search(*it);
      ig_ray += getNodeEntropy(node);

      //nodes.insert(*it);
      nodes_traversed++;

      double prob = getNodeOccupancy(node);
      if (prob > 0.8)
        nodes_occ++;
      else if (prob < 0.2)
        nodes_free++;
      else
        nodes_unknown++;

      if (prob == 0.5)
        nodes_unobserved++;
    }

    // Evaluate endpoint
    if (!exited_valid_range)
    {
      if ( isPointInBounds(endpoint) )
      {
        end_pt = endpoint;

        octomap::OcTreeKey end_key;
        if( tree_->coordToKeyChecked(endpoint, end_key) )
        {
          octomap::OcTreeNode* node = tree_->search(end_key);

          nodes.insert(end_key);
          nodes_traversed++;
        }

        // Weight the final value with predicted lengths
        ig_total += (2 - ray_len_predicted/ray_len)*ig_ray;
      }
    }

    /*
     * Project the discretized start and end point to the ray we started with
     * This just cleans up the lines for visualization
     */
    start_pt = origin + dir * (dir.dot(start_pt-origin)/dir.dot(dir));
    end_pt = origin + dir * (dir.dot(end_pt-origin)/dir.dot(dir));

    addToRayMarkers(start_pt, end_pt);
  }

  //if(is_debug_)
  //{
    publishRayMarkers();
    publishPose(p);
  //}

  int nodes_processed = nodes_traversed;

  // Views that do not see a single occupied cell are discarded
  // This is to encourage the vehivle to look at the object at all times
  if (must_see_occupied_ && nodes_occ == 0)
    ig_total = -1;

  t_end = ros::Time::now().toSec();
  if(is_debug_)
  {
    std::cout << "\nIG: " << ig_total << "\tAverage IG: " << ig_total/nodes_processed <<"\n";
    std::cout << "Unobserved: " << nodes_unobserved << "\tUnknown: " << nodes_unknown << "\tOcc: " << nodes_occ << "\tFree: " << nodes_free << "\n";
    std::cout << "Time: " << t_end-t_start << " sec\tNodes: " << nodes_processed << "/" << nodes_traversed<< " (" << 1000*(t_end-t_start)/nodes_processed << " ms/node)\n";
    std::cout << "\tAverage nodes per ray: " << nodes_traversed/rays_far_plane_at_pose_.size() << "\n";
  }

  return ig_total;
}

std::string ViewSelecterProposedRayLength::getMethodName()
{
  return "Proposed (Ray Length)";
}

void ViewSelecterProposedRayLength::update()
{
  ViewSelecterBase::update(); //Call base class update
  tree_predicted_ = view_gen_->tree_prediction_;
}
