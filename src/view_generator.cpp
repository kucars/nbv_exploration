#include <iostream>
#include <ros/ros.h>

#include <Eigen/Geometry>

#include <nbv_exploration/view_generator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/ccd/motion.h"
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"

#include "nbv_exploration/view_generator.h"
#include "nbv_exploration/common.h"

// ==============
// Base
// ==============
ros::Publisher marker_array_pub;
ros::Publisher visPub;
int sphereCounter = 0;

int marker_array_prev_size = 0;

ViewGeneratorBase::ViewGeneratorBase()
{
	ros::NodeHandle n;
  marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("generated_pose_marker_array", 10);
  visPub = n.advertise<visualization_msgs::Marker>("collision_marker", 10);

  // Read parameters
  double res_x, res_y, res_z, res_yaw;
  ros::param::param("~uav_position_resolution_x", res_x, 1.0);
  ros::param::param("~uav_position_resolution_y", res_y, 1.0);
  ros::param::param("~uav_position_resolution_z", res_z, 1.0);
  ros::param::param("~uav_position_resolution_yaw", res_yaw, M_PI_4);

  double obj_x_min, obj_x_max, obj_y_min, obj_y_max, obj_z_min, obj_z_max;
  ros::param::param("~object_bounds_x_min", obj_x_min,-1.0);
  ros::param::param("~object_bounds_x_max", obj_x_max, 1.0);
  ros::param::param("~object_bounds_y_min", obj_y_min,-1.0);
  ros::param::param("~object_bounds_y_max", obj_y_max, 1.0);
  ros::param::param("~object_bounds_z_min", obj_z_min, 0.0);
  ros::param::param("~object_bounds_z_max", obj_z_max, 1.0);

  double nav_x_min, nav_x_max, nav_y_min, nav_y_max, nav_z_min, nav_z_max;
  ros::param::param("~nav_bounds_x_min", nav_x_min,-5.0);
  ros::param::param("~nav_bounds_x_max", nav_x_max, 5.0);
  ros::param::param("~nav_bounds_y_min", nav_y_min,-5.0);
  ros::param::param("~nav_bounds_y_max", nav_y_max, 5.0);
  ros::param::param("~nav_bounds_z_min", nav_z_min, 1.0);
  ros::param::param("~nav_bounds_z_max", nav_z_max, 5.0);

  double collision_radius;
  ros::param::param("~uav_collision_radius", collision_radius, 1.0);

  setResolution(res_x, res_y, res_z, res_yaw);
  setObjectBounds(obj_x_min, obj_x_max, obj_y_min, obj_y_max, obj_z_min, obj_z_max);
  setNavigationBounds(nav_x_min, nav_x_max, nav_y_min, nav_y_max, nav_z_min, nav_z_max);
  setCollisionRadius(collision_radius);
}

void drawSphere(Pose p, double r)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.id = sphereCounter;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = p;
  marker.scale.x = r;
  marker.scale.y = r;
  marker.scale.z = r;
  marker.color.a = 0.3;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  //marker.lifetime = ros::Duration(0.3);
  visPub.publish( marker );
  
  sphereCounter++;
}

void ViewGeneratorBase::updateCollisionBoxesFromOctomap()
{
  // convert the octomap::octree to fcl::octree fcl_octree object
  fcl::OcTree* tree2 = new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(tree_));
  std::vector<boost::array<fcl::FCL_REAL, 6> > boxes = tree2->toBoxes();
  
  collision_boxes_.clear();
  for(std::size_t i = 0; i < boxes.size(); ++i)
  {
    fcl::FCL_REAL x = boxes[i][0];
    fcl::FCL_REAL y = boxes[i][1];
    fcl::FCL_REAL z = boxes[i][2];
    fcl::FCL_REAL size = boxes[i][3];
    fcl::FCL_REAL cost = boxes[i][4];
    fcl::FCL_REAL threshold = boxes[i][5];
    fcl::Box* box = new fcl::Box(size, size, size);
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    fcl::CollisionObject* obj = new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), fcl::Transform3f(fcl::Vec3f(x, y, z)));
    collision_boxes_.push_back(obj);
  }
}

bool ViewGeneratorBase::isInsideBounds(Pose p)
{
  if (p.position.x < nav_bounds_x_min_ || p.position.x > nav_bounds_x_max_ ||
      p.position.y < nav_bounds_y_min_ || p.position.y > nav_bounds_y_max_ ||
      p.position.z < nav_bounds_z_min_ || p.position.z > nav_bounds_z_max_)
  {
    return false;
  }
  
  return true;
}

bool ViewGeneratorBase::isCollidingWithOctree(Pose p)
{
  /* Collision detection based on octomap
   * 
   * Source: https://github.com/kuri-kustar/laser_collision_detection/blob/master/src/laser_obstacle_detect.cpp#L135-L228
   */
  
  // Create UAV collision object
  boost::shared_ptr<fcl::Sphere> Shpere0(new fcl::Sphere(collision_radius_));
  
  fcl::Transform3f tf0;
  tf0.setIdentity();
  tf0.setTranslation(fcl::Vec3f(p.position.x, p.position.y, p.position.z));

  fcl::CollisionObject co0(Shpere0, tf0);
  
  // Visualize
  drawSphere(p, collision_radius_);
  
  bool collisionDetected = false;
  
  double min_dist = 1/.0;
  
  for(size_t i = 0; i < collision_boxes_.size(); ++i)
  {
    fcl::CollisionObject* box =  collision_boxes_[i];
    static const int num_max_contacts = std::numeric_limits<int>::max();
    static const bool enable_contact = true ;

    fcl::CollisionResult result;
    fcl::CollisionRequest request(num_max_contacts, enable_contact);
    fcl::collide(&co0, box, request, result);

    if ( result.isCollision() )
    {
      collisionDetected = true;
      break;
    }
  }
  
  return collisionDetected;
}

bool ViewGeneratorBase::isValidViewpoint(Pose p)
{
  if (!isInsideBounds(p) )
  {
    return false;
  }
    
  if (isCollidingWithOctree(p) )
  {
    return false;
  }
    
  return true;
}

visualization_msgs::Marker deleteArrowMarker(int id)
{
  visualization_msgs::Marker pose_marker;
  
  pose_marker.header.frame_id = "world";
  pose_marker.header.stamp = ros::Time::now();
  pose_marker.id = id;
  
  pose_marker.action = visualization_msgs::Marker::DELETE;
  return pose_marker;
}

visualization_msgs::Marker createArrowMarker(int id, Pose pose, bool valid, double max_z = 0, double min_z = 0)
{
  visualization_msgs::Marker pose_marker;
  
  pose_marker.header.frame_id = "world";
  pose_marker.header.stamp = ros::Time::now();
  pose_marker.id = id;
  pose_marker.type = visualization_msgs::Marker::ARROW;
  pose_marker.pose = pose;

  // Style
  pose_marker.scale.x = 0.5;
  pose_marker.scale.y = 0.1;
  pose_marker.scale.z = 0.1;
  
  if( valid )
  {
    double curr_z = pose.position.z;
    double color = 1.0;
    if (max_z - min_z > 0)
    {
      color = (curr_z - min_z)/(max_z - min_z); // Normalize between 0.5 and 1
      color = 0.3*(curr_z - min_z)/(max_z - min_z) + 0.7; //Normalize between 0.7 and 1
    }
    
    pose_marker.color.r = 0;
    pose_marker.color.g = 0.5;
    pose_marker.color.b = color;
    pose_marker.color.a = 1.0;
  }
  else
  {
    pose_marker.color.r = 1.0;
    pose_marker.color.g = 0;
    pose_marker.color.b = 0;
    pose_marker.color.a = 1.0;
  }
  
  return pose_marker;
}

void ViewGeneratorBase::visualize(std::vector<Pose> valid_poses, std::vector<Pose> invalid_poses)
{
  visualization_msgs::MarkerArray pose_array;
  
  double max_z = -1/.0;
  double min_z =  1/.0;
  for (int i=0; i<generated_poses.size(); i++)
  {
    if (valid_poses[i].position.z > max_z)
      max_z = valid_poses[i].position.z;
    else if (valid_poses[i].position.z < min_z)
      min_z = valid_poses[i].position.z;
  }

  // Valid markers
  for (int i=0; i<valid_poses.size(); i++)
  {
    visualization_msgs::Marker pose_marker = createArrowMarker(i, valid_poses[i], true, max_z, min_z);
    pose_array.markers.push_back(pose_marker);
  }
  
  // Invalid markers
  int offset = valid_poses.size();
  for (int i=0; i<invalid_poses.size(); i++)
  {
    visualization_msgs::Marker pose_marker = createArrowMarker(i+offset, invalid_poses[i], false);
    pose_array.markers.push_back(pose_marker);
  }
  
  // Delete old markers
  for (int i=valid_poses.size() + invalid_poses.size(); i<marker_array_prev_size; i++)
  {
    visualization_msgs::Marker pose_marker = deleteArrowMarker(i);    
    pose_array.markers.push_back(pose_marker);
  }
  
  marker_array_prev_size = valid_poses.size() + invalid_poses.size();
  marker_array_pub.publish(pose_array);
  
  std::cout << "Press ENTER to continue\n";
  std::cin.get();
}


// ==============
// Nearest Neighbor
// ==============
void ViewGeneratorNN::generateViews()
{
  std::vector<Pose> initial_poses;
  generated_poses.clear();
  
  double currX = current_pose_.position.x;
  double currY = current_pose_.position.y;
  double currZ = current_pose_.position.z;
  double currYaw = pose_conversion::getYawFromQuaternion(current_pose_.orientation);
  
  //@TODO: Must check if viewpoints area reachable
  
  if (cloud_occupied_ptr_->points.size() < 0)
  {
    std::cout << "[ViewpointGen::NN] No points in map. Rotating" << std::endl;
    
    Pose p;
    p.position.x = std::numeric_limits<double>::quiet_NaN(); //Set to NaN
    p.position.y = std::numeric_limits<double>::quiet_NaN();;
    p.position.z = std::numeric_limits<double>::quiet_NaN();;
    p.orientation = pose_conversion::getQuaternionFromYaw(res_yaw_); //Rotate 22.5 deg
    
    generated_poses.push_back(p);
  }
  else
  {
    std::cout << "[ViewpointGen::NN] Generating 4-D state lattice" << std::endl;
    
    for (int i_x=-1; i_x<=1; i_x++)
    {
      for (int i_y=-1; i_y<=1; i_y++)
      {
        for (int i_z=-1; i_z<=1; i_z++)
        {
          for (int i_yaw=-1; i_yaw<=1; i_yaw++)
          {
            /*
            if (i_x==0 && i_y==0 && i_z==0 && i_yaw==0)
              continue;
            */
            
            Pose p;
            p.position.x = currX + res_x_*i_x*cos(currYaw) + res_y_*i_y*sin(currYaw);
            p.position.y = currY - res_x_*i_x*sin(currYaw) + res_y_*i_y*cos(currYaw);
            p.position.z = currZ + res_z_*i_z;
            
            p.orientation = pose_conversion::getQuaternionFromYaw(currYaw + res_yaw_*i_yaw);
            initial_poses.push_back(p);
          }
        }
      }
    }
    
    std::vector<Pose> rejected_poses;
    for (int i=0; i<initial_poses.size(); i++)
    {
      if ( isValidViewpoint(initial_poses[i]) )
      {
        generated_poses.push_back(initial_poses[i]);
      }
      else
      {
        rejected_poses.push_back(initial_poses[i]);
      }
    }
    
    if (is_debug_)
    {
      std::cout << "[ViewpointGen::NN] Generated " << generated_poses.size() << " poses (" << rejected_poses.size() << " rejected)" << std::endl;
      visualize(generated_poses, rejected_poses);
    }
  }
}

// ==============
// Frontier class
// ==============
void ViewGeneratorFrontier::generateViews()
{
  generated_poses.clear();
  
  if (cloud_occupied_ptr_->points.size() < 0)
  {
    std::cout << "[ViewpointGen::Frontier] No points in map. Rotating" << std::endl;
    
    Pose p;
    p.position.x = std::numeric_limits<double>::quiet_NaN(); //Set to NaN
    p.position.y = std::numeric_limits<double>::quiet_NaN();
    p.position.z = std::numeric_limits<double>::quiet_NaN();
    p.orientation = pose_conversion::getQuaternionFromYaw(res_yaw_); //Rotate 22.5 deg
    
    generated_poses.push_back(p);
  }
  else
  {
    std::cout << "[ViewpointGen::Frontier] Point in map. Generating viewpoint" << std::endl;
  }
}
