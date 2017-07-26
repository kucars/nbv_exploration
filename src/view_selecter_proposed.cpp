#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf_conversions/tf_eigen.h>
//#include <culling/occlusion_culling.h>

#include "nbv_exploration/view_selecter_proposed.h"

ViewSelecterProposed::ViewSelecterProposed():
  ViewSelecterBase() //Call base class constructor
{
  ros::param::param("~view_selecter_proposed_weight_density", weight_density_, 0.5);
  ros::param::param("~view_selecter_proposed_weight_distance", weight_distance_, 0.5);
  ros::param::param("~view_selecter_proposed_weight_entropy", weight_entropy_, 0.5);
  ros::param::param("~view_selecter_proposed_weight_prediction", weight_prediction_, 0.5);
  ros::param::param("~view_selecter_proposed_use_node_count", use_node_count, true);

  //@todo: chech all weights are positive

  // Normalize weights
  double mag = weight_density_ + weight_prediction_;
  weight_density_    /= mag;
  weight_prediction_ /= mag;

  // ========
  // Compute max point density
  // ========
  /*
   * Max density based on filter size and occupancy size
   * Since the filter is much smaller than the octree resolution, and we know
   *  each filtered voxel will have at most one point, we can determine
   *  the max number of points in one unit area or unit volume of occupancy voxel
   *
   * The absolute max number of points per voxel is (octomap_vox_size/filter_vox_size)^3,
   *  where the entire volume of the voxel is filled with voxels
   *
   * The maximum practical number is 3*(octomap_vox_size/filter_vox_size)^2,
   *  where we can see at most 3 faces of the voxel at once
   */

  double octomap_vox_size, filter_vox_size;
  ros::param::param("~mapping_octree_resolution", octomap_vox_size, 0.2);
  ros::param::param("~mapping_voxel_grid_res_rgbd", filter_vox_size, 0.1);

  max_density_ = 3*std::pow( (octomap_vox_size/filter_vox_size), 2);
}

double ViewSelecterProposed::calculateUtility(geometry_msgs::Pose p)
{
  // Modified version of ViewSelecterBase::calculateIG()
  // Computes classical entropy as well as entropy in predicted voxel grid
  //
  // Source: Borrowed partially from
  // https://github.com/uzh-rpg/rpg_ig_active_reconstruction/blob/master/ig_active_reconstruction_octomap/src/code_base/octomap_basic_ray_ig_calculator.inl

  octomap::point3d origin (p.position.x, p.position.y, p.position.z);

  int num_nodes_traversed = 0;
  int num_nodes_free = 0;
  int num_nodes_occ = 0;
  int num_nodes_unknown = 0;
  int num_nodes_predicted = 0;

  int num_of_points = 0;

  std::set<octomap::OcTreeKey, OctomapKeyCompare> key_list;
  std::set<octomap::OcTreeKey, OctomapKeyCompare> key_predicted_list;



  clearRayMarkers();
  double ig_total = 0;

  for (int i=0; i<rays_far_plane_at_pose_.size(); i++)
  {
    octomap::point3d endpoint, endpoint_predicted;
    double ray_length, ray_predicted_length;

    // Get length of beam to the far plane of sensor
    double range = rays_far_plane_at_pose_[i].norm();

    // Get the direction of the ray
    octomap::point3d dir = rays_far_plane_at_pose_[i].normalize();

    // ========
    // Get endpoint of ray when cast through main octomap
    //  using castRay() is expensive when done repeatedly
    // ========
    bool found_endpoint = tree_->castRay(origin, dir, endpoint, true, range);
    if (found_endpoint)
    {
      ray_length = (origin-endpoint).norm();
    }
    else
    {
      ray_length = range;
    }

    // ========
    // Get endpoint of ray when cast through predicted octomap
    //  using castRay() is expensive when done repeatedly
    // ========
    if(weight_prediction_ > 0)
    {
      found_endpoint = tree_predicted_->castRay(origin, dir, endpoint_predicted, true, range);
      if (found_endpoint)
      {
        ray_predicted_length = (origin-endpoint_predicted).norm();

        // Check if predicted ray is not occluded
        if (ray_length >= ray_predicted_length)
        {
          octomap::OcTreeKey end_key;
          if( tree_->coordToKeyChecked(endpoint_predicted, end_key) )
          {
            // Predicted voxel is not occluded, count it
            insertKeyIfUnique(key_predicted_list, end_key);
          }
        }
      }
    }

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
    octomap::point3d start_pt, end_pt;
    bool entered_valid_range = false;
    bool exited_valid_range = false;

    octomap::KeyRay ray;
    tree_->computeRayKeys( origin, endpoint, ray );

    // ======
    // Iterate through each node in ray
    // ======
    for( octomap::KeyRay::iterator it = ray.begin() ; it!=ray.end(); ++it )
    {
      octomap::point3d p = tree_->keyToCoord(*it);

      // ======
      // Compute ray start and end point for visualization
      // ======
      if (!entered_valid_range)
      {
        // Find the first valid point
        if ( isPointInBounds(p) )
        {
          entered_valid_range = true;
          start_pt = p;
        }

        else
        {
          continue;
        }
      }

      else
      {
        if (isPointInBounds(p))
        {
          end_pt = p;
        }

        else
        {
          exited_valid_range = true;
          break; //We've exited the valid range
        }
      }

      insertKeyIfUnique(key_list, *it);
    }// end ray iterator


    // ======
    // Evaluate endpoint
    // ======
    if (found_endpoint && !exited_valid_range)
    {
      octomap::OcTreeKey end_key;
      if( tree_->coordToKeyChecked(endpoint, end_key) )
      {
        insertKeyIfUnique(key_list, end_key);
      }
    }

    /*
     * Project the discretized start and end point to the ray we started with
     * This just cleans up the lines for visualization
     */
    start_pt = origin + dir * (dir.dot(start_pt-origin)/dir.dot(dir));
    end_pt = origin + dir * (dir.dot(end_pt-origin)/dir.dot(dir));

    addToRayMarkers(start_pt, end_pt);
  }//end ray casting

  //========
  // Process all unique nodes
  //========
  // Predicted octomap
  num_nodes_predicted = key_predicted_list.size();

  // Normal octomap
  std::set<octomap::OcTreeKey>::iterator it;
  for (it = key_list.begin(); it != key_list.end(); ++it)
  {
    octomap::OcTreeKey key = *it;
    octomap::OcTreeNode* node = tree_->search(key);


    num_nodes_traversed++;
    if (!node)
      num_nodes_unknown++;
    else if (isNodeOccupied(*node))
    {
      num_nodes_occ++;

      // Get number of points to compute density
      num_of_points += getPointCountAtOcTreeKey(key);
    }
    else if (isNodeFree(*node))
      num_nodes_free++;
    else
      num_nodes_unknown++;

    // Entropy
    ig_total += getNodeEntropy(node);
  }

  //=======
  // Compute distance
  //=======
  double distance = calculateDistance(p);

  //=======
  // Compute density
  //=======
  double density;

  if (num_nodes_occ == 0)
    density = 0;
  else
    density = double(num_of_points)/num_nodes_occ;

  //=======
  // Compute utility
  //=======
  // Views that do not see a single occupied cell are discarded
  // This is to encourage the vehivle to look at the object at all times
  if (must_see_occupied_ && num_nodes_occ == 0)
  {
    ig_total = 0;
  }

  double normalized_density, normalized_entropy, normalized_prediction;

  // Density
  normalized_density = density / max_density_;

  // Entropy
  int nodes_for_entropy = num_nodes_unknown+num_nodes_occ+num_nodes_free;
  if (nodes_for_entropy > 0)
  {
    double max_entropy = nodes_for_entropy*(-log(0.5));
    normalized_entropy = ig_total/max_entropy;
  }
  else
  {
    normalized_entropy = 0;
  }

  // Predictied
  if (num_nodes_predicted+num_nodes_occ > 0)
  {
    normalized_prediction = double(num_nodes_predicted)/(num_nodes_predicted+num_nodes_occ);
  }
  else
  {
    normalized_prediction = 0;
  }

  // Node count


  // Multiply each component by its weight
  double weighted_density, weighted_entropy, weighted_prediction, weighted_distance, weighted_node_count;

  if (normalized_density > 0)
    weighted_density = weight_density_*(1-normalized_density);
  else
    // No points seen
    weighted_density = 0;

  weighted_entropy = weight_entropy_*normalized_entropy;
  weighted_prediction = weight_prediction_*normalized_prediction;
  weighted_distance = exp(-weight_distance_*distance);

  if (use_node_count)
    weighted_node_count = log10(num_nodes_occ);
  else
    // No points seen
    weighted_node_count = 1;

  // Value views that see more voxels more, as they're less likely to skim the edge of the surface
  double utility;
  if (num_nodes_occ == 0)
    utility = 0;
  else
    //utility = log10(num_nodes_occ) * weighted_distance * (weighted_density + weighted_entropy + weighted_prediction);
    utility = weighted_node_count * weighted_distance * (1+weighted_entropy) * (weighted_density + weighted_prediction);

  if(is_debug_)
  {
    printf("\nDensity: %f\tMax: %f\n", density, max_density_);
    printf("Normalized -- IG: %f, Density: %f, Predicted: %f\n", normalized_entropy, normalized_density, normalized_prediction);
    printf("Utility ----- IG: %f, Density: %f, Predicted: %f, Total: %f\n", weighted_entropy, weighted_density, weighted_prediction, utility);
    printf("Predicted: %d\tUnknown: %d\tOccupied: %d\tFree: %d\n", num_nodes_predicted, num_nodes_unknown, num_nodes_occ, num_nodes_free);

    //std::cout << "\tAverage nodes per ray: " << num_nodes_traversed/rays_far_plane_at_pose_.size() << "\n";
  }

  temp_utility_density_    = weighted_density;
  temp_utility_distance_   = weighted_distance;
  temp_utility_entropy_    = weighted_entropy;
  temp_utility_prediction_ = weighted_prediction;
  temp_occupied_voxels_    = num_nodes_occ;

  // ======
  // Visualize
  // ======
  publishRayMarkers();
  publishPose(p);

  return utility;
}

void ViewSelecterProposed::insertKeyIfUnique(std::set<octomap::OcTreeKey, OctomapKeyCompare>& list, octomap::OcTreeKey key)
{
  //TODO: This function takes most of the time in the evaluator
  if (list.count(key) == 0)
  {
    // Key not found, insert
    list.insert(key);
  }
}

std::string ViewSelecterProposed::getMethodName()
{
  std::ostringstream name_stream;
  name_stream.precision(2); //Set number of decimal spaces to display for floats

  name_stream << "(1+" << weight_entropy_ << "E)(" << weight_density_ << "D+"<<weight_prediction_ << "P)";

  if (weight_distance_ > 0)
    name_stream << "exp(-" << weight_distance_ << "d)";

  if (use_node_count)
    name_stream << "log10(N)";

  return name_stream.str();
  //return "Proposed Method";
}

void ViewSelecterProposed::update()
{
  ViewSelecterBase::update(); //Call base class update
  tree_predicted_ = view_gen_->tree_prediction_;
}
