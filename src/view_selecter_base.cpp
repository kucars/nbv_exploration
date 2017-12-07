#include <iostream>
#include <cmath>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
//#include <culling/occlusion_culling.h>

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/view_selecter_base.h"
#include "nbv_exploration/common.h"


ViewSelecterBase::ViewSelecterBase():
  info_iteration_(0),
  info_distance_total_(0),
  info_selected_utility_(-std::numeric_limits<float>::infinity()), //-inf
  info_selected_utility_density_(std::numeric_limits<double>::quiet_NaN()),
  temp_utility_distance_(std::numeric_limits<double>::quiet_NaN()),
  info_selected_utility_entropy_(std::numeric_limits<double>::quiet_NaN()),
  info_selected_utility_prediction_(std::numeric_limits<double>::quiet_NaN()),
  info_selected_occupied_voxels_(0)
{
  ros::NodeHandle n;
  marker_pub     = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pose_pub       = n.advertise<geometry_msgs::PoseStamped>("visualization_marker_pose", 10);
  trajectory_pub = n.advertise<visualization_msgs::Marker>("nbv_exploration/trajectory", 10);

  // >>>>>>>>>>>>>>>>>
  // Read parameters
  // >>>>>>>>>>>>>>>>>
  ros::param::param("~debug_view_selecter", is_debug_, true);
  ros::param::param("~view_selecter_must_see_occupied", must_see_occupied_, true);
  ros::param::param("~view_selecter_ignore_entropies_at_clamping_points", is_ignoring_clamping_entropies_, true);



  double fov_h, fov_v, r_max, r_min;
  ros::param::param("~fov_horizontal", fov_h, 60.0);
  ros::param::param("~fov_vertical", fov_v, 45.0);
  ros::param::param("~depth_range_max", r_max, 5.0);
  ros::param::param("~depth_range_min", r_min, 0.05);
  ros::param::param("~camera_count", camera_count_, 1);

  setCameraSettings(fov_h, fov_v, r_max, r_min);


  ros::param::param("~vehicle_type", vehicle_type_, 0);

  // == Get camera orientation(s) ==
  getCameraRotationMtxs();
}

void ViewSelecterBase::addToRayMarkers(octomap::point3d origin, octomap::point3d endpoint)
{
  geometry_msgs::Point p;

  // Start
  p.x = origin.x();
  p.y = origin.y();
  p.z = origin.z();
  ray_msg.points.push_back(p);

  // End
  p.x = endpoint.x();
  p.y = endpoint.y();
  p.z = endpoint.z();
  ray_msg.points.push_back(p);
}

double ViewSelecterBase::calculateIG(geometry_msgs::Pose p)
{
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
    octomap::point3d endpoint;

    // Get length of beam to the far plane of sensor
    double range = rays_far_plane_at_pose_[i].norm();

    // Get the direction of the ray
    octomap::point3d dir = rays_far_plane_at_pose_[i].normalize();




    // Cast through unknown cells as well as free cells
    bool found_endpoint = tree_->castRay(origin, dir, endpoint, true, range);
    if (!found_endpoint)
    {
      endpoint = origin + dir * range;
    }


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
     *
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
          nodes.insert(end_key);
          nodes_traversed++;
        }

        ig_total += ig_ray;
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

  /*
  int nodes_processed = nodes.size();
  for (std::set<octomap::OcTreeKey>::iterator it=nodes.begin(); it!=nodes.end(); ++it)
  {
    octomap::OcTreeNode* node = tree_->search(*it);
    ig_total += getNodeEntropy(node);

    double prob = getNodeOccupancy(node);
    if (prob > 0.5)
      nodes_occ++;
    else if (prob < 0.5)
      nodes_free++;
    else
      nodes_unknown++;
  }
  */
  //ig_total /= nodes_processed;

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

double ViewSelecterBase::calculateDistance(geometry_msgs::Pose p)
{
  return sqrt(
        (p.position.x-current_pose_.position.x)*(p.position.x-current_pose_.position.x) +
        (p.position.y-current_pose_.position.y)*(p.position.y-current_pose_.position.y) +
        (p.position.z-current_pose_.position.z)*(p.position.z-current_pose_.position.z)
        );
}

double ViewSelecterBase::calculateAngularDistance(geometry_msgs::Pose p)
{
  double yaw1 = pose_conversion::getYawFromQuaternion(current_pose_.orientation);
  double yaw2 = pose_conversion::getYawFromQuaternion(p.orientation);

  // Set difference from -pi to pi
  double yaw_diff = fmod(yaw1 - yaw2, 2*M_PI);
  if (yaw_diff > M_PI)
  {
      yaw_diff = yaw_diff - 2*M_PI;
  }
  else if (yaw_diff < -M_PI)
  {
      yaw_diff = yaw_diff + 2*M_PI;
  }

  return fabs(yaw_diff);
}

double ViewSelecterBase::calculateUtility(geometry_msgs::Pose p)
{
  std::cout << "[ViewSelecterBase]: " << cc.yellow << "Warning: calculateUtility() not implimented, defaulting to classical IG calculation\n" << cc.reset;
  double IG = calculateIG(p);
  //double effort = calculateDistance(p) + calculateAngularDistance(p)/M_PI;

  return IG;// /effort;
}

void ViewSelecterBase::clearRayMarkers()
{
  ray_msg.id = 0;
  ray_msg.type = visualization_msgs::Marker::LINE_LIST;
  ray_msg.scale.x = 0.05;

  // Blue lines
  ray_msg.color.r = 1.0;
  ray_msg.color.g = 0;
  ray_msg.color.b = 1.0;
  ray_msg.color.a = 0.1;

  ray_msg.points.clear();

}

double ViewSelecterBase::computeRelativeRays()
{
  rays_far_plane_.clear();

  double deg2rad = M_PI/180;
  double min_x = -range_max_ * tan(fov_horizontal_/2 * deg2rad);
  double min_y = -range_max_ * tan(fov_vertical_/2 * deg2rad);
  double max_x =  range_max_ * tan(fov_horizontal_/2 * deg2rad);
  double max_y =  range_max_ * tan(fov_vertical_/2 * deg2rad);

  double x_step = tree_resolution_;
  double y_step = tree_resolution_;

  for( double x = min_x; x<=max_x; x+=x_step )
  {
    for( double y = min_y; y<=max_y; y+=y_step )
    {
      Eigen::Vector3d p_far(range_max_, x, y);
      rays_far_plane_.push_back(p_far);
    }
  }
}

void ViewSelecterBase::computeRaysAtPose(geometry_msgs::Pose p)
{
  rays_far_plane_at_pose_.clear();

  Eigen::Matrix3d r_pose, rotation_mtx_;
  r_pose = pose_conversion::getRotationMatrix( p );

  // For each camera, compute the rays
  for (int c=0; c<camera_count_; c++)
  {
    rotation_mtx_ = r_pose * camera_rotation_mtx_[c];

    for (int r=0; r<rays_far_plane_.size(); r++)
    {
      // Rotate ray to its final position
      Eigen::Vector3d temp = rotation_mtx_*rays_far_plane_[r];

      // Create an octomap point to later cast a ray
      octomap::point3d p (temp[0], temp[1], temp[2]);
      rays_far_plane_at_pose_.push_back(p);
    }
  }

}

void ViewSelecterBase::evaluate()
{
  update();

  timer.start("[ViewSelecterBase]evaluate");
  // Reset all variables of interest
  info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
  info_utilities_.clear();
  selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();


  for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
  {
    geometry_msgs::Pose p = view_gen_->generated_poses[i];
    computeRaysAtPose(p);

    double utility = calculateUtility(p);

    // Ignore invalid utility values (may arise if we rejected pose based on IG requirements)
    if (utility>=0)
      info_utilities_.push_back(utility);

    if (utility > info_selected_utility_)
    {
      info_selected_utility_            = utility;
      info_selected_utility_density_    = temp_utility_density_;
      info_selected_utility_distance_   = temp_utility_distance_;
      info_selected_utility_entropy_    = temp_utility_entropy_;
      info_selected_utility_prediction_ = temp_utility_prediction_;
      info_selected_occupied_voxels_    = temp_occupied_voxels_;

      selected_pose_ = p;
    }

    if (is_debug_)
    {
      std::cout << "Utility of pose[" << i << "]: " << utility << "\n";

      //std::cout << "[ViewSelecterBase::evaluate] Looking at pose[" << i << "]:\nx = " << p.position.x << "\ty = "  << p.position.y << "\tz = "  << p.position.z << "\n";
      std::cout << "Press ENTER to continue\n";
      std::cin.get();
    }
  } //end for


  // No valid poses found, end
  if(std::isnan(selected_pose_.position.x) )
  {
    return;
  }

  // Increase total distance travelled
  info_distance_total_ += calculateDistance(selected_pose_);
  publishTrajectory();

  timer.stop("[ViewSelecterBase]evaluate");
}

void ViewSelecterBase::getCameraRotationMtxs()
{
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;

  camera_rotation_mtx_.clear();

  for (int c=0; c<camera_count_; c++)
  {
    std::string camera_frame;
    std::string base_frame;

    if (vehicle_type_ == 0)
    {
      base_frame = "/floating_sensor/base_link";

      if (c==0)
        camera_frame = "/floating_sensor/camera_frame";
      else
      {
        char sb[ 100 ];
        sprintf( sb, "/floating_sensor/camera%d_frame", c+1 );
        camera_frame = sb;
      }
    }


    else if (vehicle_type_ == 1)
    {
      base_frame = "/iris/xtion_sensor/ground_truth/iris/xtion_sensor/ground_truth/odometry_sensor_link";

      if (c==0)
        camera_frame = "/iris/xtion_sensor/camera_depth_optical_frame";

      else
      {
        char sb[ 100 ];
        sprintf( sb, "/iris/xtion_sensor%d/camera_depth_optical_frame", c+1 );
        camera_frame = sb;
      }
    }

    // Keep trying until it succeeds
    // TF failures happen in the few moments after the system starts for the first time
    while(true)
    {
      try
      {
        tf_listener.lookupTransform(base_frame, camera_frame, ros::Time(0), transform);
        break; // Success, break out of the loop
      }
      catch (tf2::LookupException ex){
        ROS_INFO_THROTTLE(1,"[ViewSelecterBase] Waiting for Transformation %s",ex.what());
        ros::Duration(0.1).sleep();
      }
      catch (tf2::ExtrapolationException ex){
        ROS_INFO_THROTTLE(1,"[ViewSelecterBase] Waiting for Transformation %s",ex.what());
        ros::Duration(0.1).sleep();
      }
      catch (tf2::ConnectivityException ex){
        ROS_INFO_THROTTLE(1,"[ViewSelecterBase] Waiting for Transformation %s",ex.what());
        ros::Duration(0.1).sleep();
      }
    }
    camera_rotation_mtx_.push_back( pose_conversion::getRotationMatrix(transform) );
  }
}

std::string ViewSelecterBase::getMethodName()
{
  return "Base";
}

double ViewSelecterBase::getNodeOccupancy(octomap::OcTreeNode* node)
{
  double p;

  if( node==NULL )
  {
    p = 0.5;
  }
  else
  {
    p = node->getOccupancy();
  }
  return p;
}

double ViewSelecterBase::getNodeEntropy(octomap::OcTreeNode* node)
{
  double p = getNodeOccupancy(node);

  if (is_ignoring_clamping_entropies_ && (p <= tree_->getClampingThresMin() || p >= tree_->getClampingThresMax()) )
    return 0;

  return - p*log(p) - (1-p)*log(1-p);
}

geometry_msgs::Pose ViewSelecterBase::getTargetPose()
{
  return selected_pose_;
}

bool ViewSelecterBase::isNodeInBounds(octomap::OcTreeKey &key)
{
  octomap::point3d p = tree_->keyToCoord(key);
  return isPointInBounds(p);
}

bool ViewSelecterBase::isNodeFree(octomap::OcTreeNode node)
{
  if (node.getOccupancy() <= 1-tree_->getOccupancyThres())
    return true;

  return false;
}

bool ViewSelecterBase::isNodeOccupied(octomap::OcTreeNode node)
{
  if (node.getOccupancy() >= tree_->getOccupancyThres())
    return true;

  return false;
}

bool ViewSelecterBase::isNodeUnknown(octomap::OcTreeNode node)
{
  return !isNodeFree(node) && !isNodeOccupied(node);
}

bool ViewSelecterBase::isPointInBounds(octomap::point3d &p)
{
  bool result = (p.x() >= view_gen_->obj_bounds_x_min_ && p.x() <= view_gen_->obj_bounds_x_max_ &&
      p.y() >= view_gen_->obj_bounds_y_min_ && p.y() <= view_gen_->obj_bounds_y_max_ &&
      p.z() >= view_gen_->obj_bounds_z_min_ && p.z() <= view_gen_->obj_bounds_z_max_ );

  return result;
}

void ViewSelecterBase::publishRayMarkers()
{
  ray_msg.header.frame_id = "world";
  ray_msg.header.stamp = ros::Time::now();

  marker_pub.publish(ray_msg);
}

void ViewSelecterBase::publishPose(geometry_msgs::Pose p)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();

  msg.pose = p;

  pose_pub.publish(msg);
}

void ViewSelecterBase::publishTrajectory()
{
  // Add point to trajectory
  geometry_msgs::Point p = selected_pose_.position;

  trajectory_msg.points.push_back(p);
  trajectory_msg.id = 0;
  trajectory_msg.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_msg.scale.x = 0.05;

  // black lines
  trajectory_msg.color.r = 0.0;
  trajectory_msg.color.g = 0;
  trajectory_msg.color.b = 0.0;
  trajectory_msg.color.a = 0.7;

  // Publish
  trajectory_msg.header.frame_id = "world";
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_pub.publish(trajectory_msg);
}

void ViewSelecterBase::setCameraSettings(double fov_h, double fov_v, double r_max, double r_min)
{
  fov_horizontal_ = fov_h;
  fov_vertical_ = fov_v;
  range_max_ = r_max;
  range_min_ = r_min;
}

void ViewSelecterBase::setViewGenerator(ViewGeneratorBase* v)
{
  view_gen_ = v;
}

void ViewSelecterBase::setMappingModule(MappingModule* m)
{
  mapping_module_ = m;
}

void ViewSelecterBase::update()
{
  timer.start("[ViewSelecterBase]update");
  info_iteration_++;

  cloud_occupied_ptr_ = view_gen_->cloud_occupied_ptr_;
  tree_               = view_gen_->tree_;
  current_pose_       = view_gen_->current_pose_;

  // Update evaluation bounds
  tree_resolution_ = tree_->getResolution();

  octomap::point3d min (
      view_gen_->obj_bounds_x_min_,
      view_gen_->obj_bounds_y_min_,
      view_gen_->obj_bounds_z_min_);
  octomap::point3d max (
      view_gen_->obj_bounds_x_max_,
      view_gen_->obj_bounds_y_max_,
      view_gen_->obj_bounds_z_max_);

  tree_->setBBXMin( min );
  tree_->setBBXMax( max );

  computeRelativeRays();

  if (is_debug_)
  {
    std::cout << "[ViewSelecterBase]: Bounds min: " << view_gen_->obj_bounds_x_min_ << ", " << view_gen_->obj_bounds_y_min_ << ", " << view_gen_->obj_bounds_z_min_ << "\n";
    std::cout << "[ViewSelecterBase]: Bounds max: " << view_gen_->obj_bounds_x_max_ << ", " << view_gen_->obj_bounds_y_max_ << ", " << view_gen_->obj_bounds_z_max_ << "\n";
  }

  //=======
  // Find total entropy remaining in system
  // * This is the slowest part of the update, optimize this for additional performance
  //=======
  info_entropy_total_=0;

  std::vector<double> total_entropy_threaded;
  #ifdef _OPENMP
    #pragma omp parallel
    #pragma omp critical
    {
      if (omp_get_thread_num() == 0){
        total_entropy_threaded.resize(omp_get_num_threads());
      }

    } // end critical
  #else
    total_entropy_threaded.resize(1);
  #endif

  int num_threads = total_entropy_threaded.size();
  double res = tree_->getResolution();

  // OpenMP loops require integer counters, so we determine the number of iterations here
  int max_iteration = (view_gen_->obj_bounds_x_max_ - view_gen_->obj_bounds_x_min_)/res;

  #ifdef _OPENMP
  omp_set_num_threads(num_threads);
  #pragma omp parallel for schedule(guided)
  #endif
  for (int i=0; i < max_iteration; i++)
  {
    // Compute the corresponding "x" coordinate
    double x = view_gen_->obj_bounds_x_min_ + res*i;

    unsigned threadIdx = 0;
    #ifdef _OPENMP
    threadIdx = omp_get_thread_num();
    #endif

    for (double y=view_gen_->obj_bounds_y_min_; y < view_gen_->obj_bounds_y_max_; y+=res)
    {
      for (double z=view_gen_->obj_bounds_z_min_; z < view_gen_->obj_bounds_z_max_; z+=res)
      {
        octomap::OcTreeKey key = tree_->coordToKey(x,y,z);
        octomap::OcTreeNode* node = tree_->search(key);
        total_entropy_threaded[threadIdx] += getNodeEntropy(node);
      }
    }
  }

  // Merge threaded values
  for (int i=0; i<num_threads; i++)
  {
    info_entropy_total_ += total_entropy_threaded[i];
  }

  if (is_debug_)
  {
    std::cout << "[ViewSelecterBase]: Total entropy remaining: " << info_entropy_total_ << "\n";
  }

  timer.stop("[ViewSelecterBase]update");
}

int ViewSelecterBase::getPointCountAtOcTreeKey(octomap::OcTreeKey key)
{
  return mapping_module_->getDensityAtOcTreeKey(key);
}
