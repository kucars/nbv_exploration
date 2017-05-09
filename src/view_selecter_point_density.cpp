#include <iostream>
#include <ros/ros.h>

#include "nbv_exploration/view_selecter_point_density.h"

ViewSelecterPointDensity::ViewSelecterPointDensity():
  ViewSelecterBase() //Call base class constructor
{
}

double ViewSelecterPointDensity::calculateUtility(Pose p)
{
  int num_of_voxels = 0;
  int num_of_points = 0;

  std::vector <octomap::OcTreeKey> checked_keys;
  clearRayMarkers();

  for (int i=0; i<rays_far_plane_.size(); i++)
  {
    octomap::point3d origin (p.position.x, p.position.y, p.position.z);
    octomap::point3d dir = transformToGlobalRay(rays_far_plane_[i]).normalize();
    octomap::point3d endpoint;

    // Get length of beam to the far plane of sensor
    double range = rays_far_plane_[i].norm();

    // Cast through unknown cells as well as free cells
    bool found_endpoint = tree_->castRay(origin, dir, endpoint, true, range);
    if (!found_endpoint)
      continue;

    addToRayMarkers(origin, endpoint);

    // Check if endpoint exists (ie. occupied)
    octomap::OcTreeKey end_key;
    if( !tree_->coordToKeyChecked(endpoint, end_key) )
      continue;

    // Check if endpoint has not been considered before
    bool unique=true;
    for (int c=0; c<checked_keys.size(); c++)
    {
      if (checked_keys[c] == end_key)
      {
        unique = false;
        break;
      }
    }

    if (!unique)
      continue;

    // Get number of points inside endpoint
    checked_keys.push_back(end_key);
    num_of_points += getPointCountInVoxel(end_key);
    num_of_voxels++;
  }

  publishRayMarkers();
  publishPose(p);

  if (num_of_voxels == 0)
    return -1;

  double density = double(num_of_points)/num_of_voxels;

  //printf("Density: %lf, Voxels: %d, Points: %d\n", density, num_of_voxels, num_of_points);
  //return 1.0/density;

  return calculateIG(p)/density;
}


int ViewSelecterPointDensity::getPointCountInVoxel(octomap::OcTreeKey key)
{
  octomap::point3d coord = tree_->keyToCoord(key);
  double res = tree_->getResolution()/2;

  double x_min = coord.x() - res;
  double x_max = coord.x() + res;
  double y_min = coord.y() - res;
  double y_max = coord.y() + res;
  double z_min = coord.z() - res;
  double z_max = coord.z() + res;

  int count = 0;

  for (int i=0; i<cloud_occupied_ptr_->points.size(); i++)
  {
    PointXYZ p = cloud_occupied_ptr_->points[i];

    if (p.x >= x_min && p.x <= x_max &&
        p.y >= y_min && p.y <= y_max &&
        p.z >= z_min && p.z <= z_max)
      count++;
  }

  return count;
}


std::string ViewSelecterPointDensity::getMethodName()
{
  return "Point Density";
}
