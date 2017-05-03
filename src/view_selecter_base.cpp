#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>

#include <tf_conversions/tf_eigen.h>
//#include <culling/occlusion_culling.h>

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/view_selecter_base.h"
#include "nbv_exploration/common.h"
#include "nbv_exploration/IterationInfo.h"


ViewSelecterBase::ViewSelecterBase():
  info_iteration_(0),
  info_distance_total_(0),
  info_utility_max_(-std::numeric_limits<float>::infinity()) //-inf
{
	ros::NodeHandle n;
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pose_pub   = n.advertise<geometry_msgs::PoseStamped>("visualization_marker_pose", 10);
  ig_pub     = n.advertise<nbv_exploration::IterationInfo>("nbv_exploration/iteration_info", 10);

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

  setCameraSettings(fov_h, fov_v, r_max, r_min);
}

void ViewSelecterBase::addToRayMarkers(octomap::point3d origin, octomap::point3d endpoint)
{
	geometry_msgs::Point p;
	
	// Start
	p.x = origin.x();
	p.y = origin.y();
	p.z = origin.z();
	line_list.points.push_back(p);

	// End
	p.x = endpoint.x();
	p.y = endpoint.y();
	p.z = endpoint.z();
	line_list.points.push_back(p);
}

double ViewSelecterBase::calculateIG(Pose p)
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

  for (int i=0; i<rays_far_plane_.size(); i++)
  {
    double ig_ray = 0;

    octomap::point3d dir = transformToGlobalRay(rays_far_plane_[i]).normalize();
    octomap::point3d endpoint;

    // Get length of beam to the far plane of sensor
    double range = rays_far_plane_[i].norm();
    double min_dist = rays_near_plane_[i].norm();

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
          octomap::OcTreeNode* node = tree_->search(end_key);

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
    std::cout << "\tAverage nodes per ray: " << nodes_traversed/rays_far_plane_.size() << "\n";
  }
  return ig_total;
}

double ViewSelecterBase::calculateDistance(Pose p)
{
  return sqrt(
        (p.position.x-current_pose_.position.x)*(p.position.x-current_pose_.position.x) +
        (p.position.y-current_pose_.position.y)*(p.position.y-current_pose_.position.y) +
        (p.position.z-current_pose_.position.z)*(p.position.z-current_pose_.position.z)
        );
}

double ViewSelecterBase::calculateAngularDistance(Pose p)
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

double ViewSelecterBase::calculateUtility(Pose p)
{
  std::cout << "[ViewSelecterBase]: " << cc.yellow << "Warning: calculateUtility() not implimented, defaulting to classical IG calculation\n" << cc.reset;
  double IG = calculateIG(p);
  //double effort = calculateDistance(p) + calculateAngularDistance(p)/M_PI;

  return IG;// /effort;
}

void ViewSelecterBase::clearRayMarkers()
{
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.05;
	
	// Blue lines
	line_list.color.r = 1.0;
	line_list.color.g = 0;
	line_list.color.b = 1.0;
  line_list.color.a = 0.1;
	
	line_list.points.clear();

}

double ViewSelecterBase::computeRelativeRays()
{
  rays_near_plane_.clear();
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
      Eigen::Vector3d p_far(y, x, range_max_);
      rays_far_plane_.push_back(p_far);

      Eigen::Vector3d p_near(y, x, range_min_);
      rays_near_plane_.push_back(p_near);
    }
  }
}

void ViewSelecterBase::computeOrientationMatrix(Pose p)
{
  Eigen::Quaterniond q(p.orientation.x,
                       p.orientation.y,
                       p.orientation.z,
                       p.orientation.w);

  /* Additional rotation to correct alignment of rays */
  Eigen::Matrix3d r;
  r << 0, 0, 1,
       0, 1, 0,
       1, 0, 0;

  rotation_mtx_ = r*q.toRotationMatrix();
}

void ViewSelecterBase::evaluate()
{
  update();

  // Reset all variables of interest
  info_utility_max_ = - std::numeric_limits<float>::infinity(); //-inf
  info_utility_med_ = - std::numeric_limits<float>::infinity(); //-inf
  info_utilities_.clear();


  for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
  {
    Pose p = view_gen_->generated_poses[i];
    computeOrientationMatrix(p);

    double utility = calculateUtility(p);

    // Ignore invalid utility values (may arise if we rejected pose based on IG requirements)
    if (utility>=0)
      info_utilities_.push_back(utility);

    if (utility > info_utility_max_)
    {
      info_utility_max_ = utility;
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

  // Compute median value
  std::vector<float> sorted_utilities(info_utilities_);
  std::sort(sorted_utilities.begin(), sorted_utilities.end());
  info_utility_med_ = sorted_utilities[sorted_utilities.size()/2];

  // Increase total distance travelled
  info_distance_total_ += calculateDistance(selected_pose_);

  // Publish information about our viewpoint selection scheme
  nbv_exploration::IterationInfo iteration_msg;
  iteration_msg.iteration = info_iteration_;
  iteration_msg.total_distance = info_distance_total_;
  iteration_msg.total_entropy = info_entropy_total_;
  iteration_msg.method = getMethodName();
  iteration_msg.utilities = info_utilities_;
  iteration_msg.max_utility = info_utility_max_;
  iteration_msg.med_utility = info_utility_med_;
  ig_pub.publish(iteration_msg);
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

Pose ViewSelecterBase::getTargetPose()
{
  return selected_pose_;
}

bool ViewSelecterBase::isNodeInBounds(octomap::OcTreeKey &key)
{
  octomap::point3d p = tree_->keyToCoord(key);
  return isPointInBounds(p);
}

bool ViewSelecterBase::isPointInBounds(octomap::point3d &p)
{
  if (p.x() >= view_gen_->obj_bounds_x_min_ && p.x() <= view_gen_->obj_bounds_x_max_ &&
      p.y() >= view_gen_->obj_bounds_y_min_ && p.y() <= view_gen_->obj_bounds_y_max_ &&
      p.z() >= view_gen_->obj_bounds_z_min_ && p.z() <= view_gen_->obj_bounds_z_max_ )
  {
    return true;
  }

  return false;
}

void ViewSelecterBase::publishRayMarkers()
{
  line_list.header.frame_id = "world";
  line_list.header.stamp = ros::Time::now();

  marker_pub.publish(line_list);
}

void ViewSelecterBase::publishPose(Pose p)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();

  msg.pose = p;

  pose_pub.publish(msg);
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

octomap::point3d ViewSelecterBase::transformToGlobalRay(Eigen::Vector3d ray_dir)
{
	Eigen::Vector3d temp = rotation_mtx_*ray_dir;
	octomap::point3d p (temp[0], temp[1], temp[2]);
	
	return p;
}

void ViewSelecterBase::update()
{
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

  // Find total entropy remaining in system
  info_entropy_total_=0;

  double res = tree_->getResolution();
  for (double x=view_gen_->obj_bounds_x_min_; x < view_gen_->obj_bounds_x_max_; x+=res)
  {
    for (double y=view_gen_->obj_bounds_y_min_; y < view_gen_->obj_bounds_y_max_; y+=res)
    {
      for (double z=view_gen_->obj_bounds_z_min_; z < view_gen_->obj_bounds_z_max_; z+=res)
      {
        octomap::OcTreeKey key = tree_->coordToKey(x,y,z);
        octomap::OcTreeNode* node = tree_->search(key);
        info_entropy_total_ += getNodeEntropy(node);
      }
    }
  }

  if (is_debug_)
  {
    std::cout << "[ViewSelecterBase]: Total entropy remaining: " << info_entropy_total_ << "\n";
  }
}
