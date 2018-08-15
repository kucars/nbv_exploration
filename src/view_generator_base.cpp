#include <iostream>
#include <ros/ros.h>

#include <Eigen/Geometry>

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

#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/common.h"

ViewGeneratorBase::ViewGeneratorBase():
  vis_sphere_counter_(0),
  vis_marker_array_prev_size_(0)
{
	ros::NodeHandle n;
  pub_view_marker_array_ = n.advertise<visualization_msgs::MarkerArray>("generated_pose_marker_array", 10);
  pub_collision_marker_ = n.advertise<visualization_msgs::Marker>("collision_marker", 10);
  pub_intersections_marker_ = n.advertise<visualization_msgs::Marker>("intersections", 10);
  marker_id_ = 0;
  // Read parameters
  ros::param::param("~debug_view_generator", is_debug_, false);

  double obj_x_min, obj_x_max, obj_y_min, obj_y_max, obj_z_min, obj_z_max;
  ros::param::param("~object_bounds_x_min", obj_x_min,-1.0);
  ros::param::param("~object_bounds_x_max", obj_x_max, 1.0);
  ros::param::param("~object_bounds_y_min", obj_y_min,-1.0);
  ros::param::param("~object_bounds_y_max", obj_y_max, 1.0);
  ros::param::param("~object_bounds_z_min", obj_z_min, 0.0);
  ros::param::param("~object_bounds_z_max", obj_z_max, 1.0);

  std::string object_mesh_f;
  ros::param::param<std::string>("~object_mesh_file", object_mesh_f, "etihad.obj");
  ros::param::param("~object_collision_check", collision_check_mesh_, true);
  ros::param::param<int>("~view_generator_frontier_nearest_count", nearest_frontiers_count_, 1);

  double nav_x_min, nav_x_max, nav_y_min, nav_y_max, nav_z_min, nav_z_max;
  ros::param::param("~nav_bounds_x_min", nav_x_min,-5.0);
  ros::param::param("~nav_bounds_x_max", nav_x_max, 5.0);
  ros::param::param("~nav_bounds_y_min", nav_y_min,-5.0);
  ros::param::param("~nav_bounds_y_max", nav_y_max, 5.0);
  ros::param::param("~nav_bounds_z_min", nav_z_min, 1.0);
  ros::param::param("~nav_bounds_z_max", nav_z_max, 5.0);

  double collision_radius;
  ros::param::param("~uav_collision_radius", collision_radius, 1.0);

  setObjectBounds(obj_x_min, obj_x_max, obj_y_min, obj_y_max, obj_z_min, obj_z_max);
  setNavigationBounds(nav_x_min, nav_x_max, nav_y_min, nav_y_max, nav_z_min, nav_z_max);
  setCollisionRadius(collision_radius);

  if(collision_check_mesh_)
  {
      std::cout<<"********************************************* collision_check ********************************"<<std::endl;
      std::string mesh_name = ros::package::getPath("nbv_exploration")+"/models/"+object_mesh_f;
      loadOBJFile(mesh_name.c_str(), model_points_, triangles_);
      std::cout<<"*************triangles: "<<triangles_.size()<<std::endl;
      cgal_tree_ = new Tree1(triangles_.begin(),triangles_.end());
  }
}

std::string ViewGeneratorBase::getMethodName()
{
  return "Base";
}

bool ViewGeneratorBase::isRecentPose(geometry_msgs::Pose p)
{
  bool is_recent = false;

  int num_poses = 50;
  if (nbv_history_->selected_poses.size() < num_poses)
    num_poses = nbv_history_->selected_poses.size();

  int end = nbv_history_->selected_poses.size()-1;
  double yaw = pose_conversion::getYawFromQuaternion(p.orientation);
  for (int i=0; i<num_poses & !is_recent; i++)
  {
    geometry_msgs::Pose p2 = nbv_history_->selected_poses[end-i];
    double yaw2 = pose_conversion::getYawFromQuaternion(p2.orientation);

    is_recent |=  fabs(p2.position.x - p.position.x) < 0.05 &&
                  fabs(p2.position.y - p.position.y) < 0.05 &&
                  fabs(p2.position.z - p.position.z) < 0.05 &&
                  fabs(yaw - yaw2) < 0.1;
  }

  return is_recent;
}

bool ViewGeneratorBase::isCollidingWithOctree(geometry_msgs::Pose p)
{
  /* Collision detection based on octomap
   * 
   * Source: https://github.com/kuri-kustar/laser_collision_detection/blob/master/src/laser_obstacle_detect.cpp#L135-L228
   */
  
  // Create UAV collision object
  fcl::Sphere Shpere0(collision_radius_);
  std::shared_ptr<fcl::CollisionGeometry > cgeomSphere_(&Shpere0);
  //boost::shared_ptr<fcl::Sphere>
  
  fcl::Transform3f tf0;
  tf0.setIdentity();
  tf0.setTranslation(fcl::Vec3f(p.position.x, p.position.y, p.position.z));


  fcl::CollisionObject* co0 = new fcl::CollisionObject(cgeomSphere_, tf0);

  // Visualize
  visualizeDrawSphere(p, collision_radius_);
  
  bool collisionDetected = false;

  for(size_t i = 0; i < collision_boxes_.size(); ++i)
  {
    fcl::CollisionObject* box =  collision_boxes_[i];
    static const int num_max_contacts = std::numeric_limits<int>::max();
    static const bool enable_contact = true ;

    fcl::CollisionResult result;
    fcl::CollisionRequest request(num_max_contacts, enable_contact);
    fcl::collide(co0, box, request, result);

    if ( result.isCollision() )
    {
      collisionDetected = true;
      break;
    }
  }
  
  return collisionDetected;
}

bool ViewGeneratorBase::isInFreeSpace(geometry_msgs::Pose p)
{
  // Create a cloud with the point to check

  octomap::point3d pt (
        p.position.x,
        p.position.y,
        p.position.z);

  // Find key corresponding to this position
  octomap::OcTreeKey key;
  if ( !tree_->coordToKeyChecked(pt, key) )
    return false;

  // Get node
  octomap::OcTreeNode* node = tree_->search(pt);

  // Node not found in tree
  if (!node)
    return false;

  // Is it free?
  if (node->getOccupancy() > 1-tree_->getOccupancyThres())
    return false;

  return true;
}

bool ViewGeneratorBase::isInsideBounds(geometry_msgs::Pose p)
{
  if (p.position.x < nav_bounds_x_min_ || p.position.x > nav_bounds_x_max_ ||
      p.position.y < nav_bounds_y_min_ || p.position.y > nav_bounds_y_max_ ||
      p.position.z < nav_bounds_z_min_ || p.position.z > nav_bounds_z_max_)
  {
    return false;
  }

  return true;
}

//taken from coverage_path_planning_heuristic.cpp of sscpp / asscpp
bool ViewGeneratorBase::isConnectionConditionSatisfied(geometry_msgs::Pose pt)
{
    //collision check
//    std::cout<<" collision_check "<<std::endl;
    int intersectionsCount=0;
    //parent
    Point a(current_pose_.position.x, current_pose_.position.y ,current_pose_.position.z );
//    std::cout<<cc.magenta<<"[VIEWGENERATORBASE] CURRENT POSITION ("<<a.x()<<", "<<a.y()<<", "<<a.z()<<cc.reset<<std::endl;

    //child
    Point b(pt.position.x, pt.position.y, pt.position.z);
    Segment seg_query(a,b);
    intersectionsCount = cgal_tree_->number_of_intersected_primitives(seg_query);
//    std::cout<<" intersections count:  "<<intersectionsCount<<std::endl;

    //visualize
    std::vector<geometry_msgs::Point> pts;
    geometry_msgs::Point pt1,pt2;
    pt1.x=current_pose_.position.x;pt1.y=current_pose_.position.y; pt1.z=current_pose_.position.z;
    pt2.x=pt.position.x;pt2.y=pt.position.y;pt2.z=pt.position.z;
    pts.push_back(pt1);
    pts.push_back(pt2);
    visualization_msgs::Marker intersections_lines = drawLines(pts,marker_id_++,2,10000,0.3);
    pub_intersections_marker_.publish(intersections_lines);

    if(intersectionsCount==0)
        return true;
    else
        return false;
}

//taken from aircraft inspection package filtering.cpp
bool ViewGeneratorBase::isInsideModel(octomap::point3d pt)
{
    //collision check
    int intersectionsCount=0;

    Point a(pt.x(), pt.y() ,pt.z());
    // Some Random point in arbitrary orientation
    Point b(100.0, 10.0, 56.0);
    Ray ray_query(a,b);
    // counts #intersections

    intersectionsCount = cgal_tree_->number_of_intersected_primitives(ray_query);

    if(intersectionsCount%2 != 1)
        return false;
    else
        return true;
}

bool ViewGeneratorBase::isConnectionConditionSatisfied(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    //collision check
//    std::cout<<" collision_check "<<std::endl;
    int intersectionsCount=0;
    //parent
    Point a(p1.position.x, p1.position.y ,p1.position.z );

    //child
    Point b(p2.position.x, p2.position.y, p2.position.z);
    Segment seg_query(a,b);
    intersectionsCount = cgal_tree_->number_of_intersected_primitives(seg_query);
//    std::cout<<" intersections count:  "<<intersectionsCount<<std::endl;

    //visualize
    std::vector<geometry_msgs::Point> pts;
    geometry_msgs::Point pt1,pt2;
    pt1.x=p1.position.x;pt1.y=p1.position.y; pt1.z=p1.position.z;
    pt2.x=p2.position.x;pt2.y=p2.position.y;pt2.z=p2.position.z;
    pts.push_back(pt1);
    pts.push_back(pt2);
    visualization_msgs::Marker intersections_lines = drawLines(pts,marker_id_++,2,10000,0.3);
    pub_intersections_marker_.publish(intersections_lines);

    if(intersectionsCount==0)
        return true;
    else
        return false;
}


bool ViewGeneratorBase::isValidViewpoint(geometry_msgs::Pose p)
{
  if (!isInsideBounds(p) )
    return false;

  if (!isInFreeSpace(p) )
    return false;
    
  if (isCollidingWithOctree(p) )
    return false;

  if (isRecentPose(p))
    return false;

  if(collision_check_mesh_)
  {
      if(!(getMethodName()=="Frontier"))
      {
          if(!isConnectionConditionSatisfied(p))
              return false;
      }
  }

  return true;
}

void ViewGeneratorBase::setCollisionRadius(double r)
{
  collision_radius_ = r;
}

void ViewGeneratorBase::setCurrentPose(geometry_msgs::Pose p)
{
  current_pose_ = p;
}

void ViewGeneratorBase::setDebug(bool b)
{
  is_debug_ = b;
}

void ViewGeneratorBase::setHistory(NBVHistory* h)
{
  nbv_history_ = h;
}

void ViewGeneratorBase::setNavigationBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  nav_bounds_x_min_ = x_min;
  nav_bounds_x_max_ = x_max;
  nav_bounds_y_min_ = y_min;
  nav_bounds_y_max_ = y_max;
  nav_bounds_z_min_ = z_min;
  nav_bounds_z_max_ = z_max;
}

void ViewGeneratorBase::setMappingModule(MappingModule* m)
{
  mapping_module_ = m;

  // Set cloud
  cloud_occupied_ptr_ = mapping_module_->getPointCloud();

  // Set octree
  tree_ = m->getOctomap();
  updateCollisionBoxesFromOctomap();

  // Set prediction
  tree_prediction_ = m->getOctomapPredicted();
}

void ViewGeneratorBase::setObjectBounds(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  obj_bounds_x_min_ = x_min;
  obj_bounds_x_max_ = x_max;
  obj_bounds_y_min_ = y_min;
  obj_bounds_y_max_ = y_max;
  obj_bounds_z_min_ = z_min;
  obj_bounds_z_max_ = z_max;
}

void ViewGeneratorBase::updateCollisionBoxesFromOctomap()
{
  // convert the octomap::octree to fcl::octree fcl_octree object
  fcl::OcTree* tree2 = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_));
  std::vector<std::array<fcl::FCL_REAL, 6> > boxes = tree2->toBoxes();

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
    fcl::CollisionObject* obj = new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box), fcl::Transform3f(fcl::Vec3f(x, y, z)));
    collision_boxes_.push_back(obj);
  }
}
visualization_msgs::Marker ViewGeneratorBase::drawLines(std::vector<geometry_msgs::Point> links, int id, int inColor, int duration, double scale)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="world";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns = "link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = scale;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(duration);
    std_msgs::ColorRGBA color;
    if(inColor == 1)
    {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else if(inColor == 2)
    {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else if(inColor == 3)
    {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
    }
    else
    {
        color.r = 0.9;
        color.g = 0.9;
        color.b = 0.0;
        color.a = 1.0;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    return linksMarkerMsg;
}
void ViewGeneratorBase::visualize(std::vector<geometry_msgs::Pose> valid_poses, std::vector<geometry_msgs::Pose> invalid_poses)
{
  if (pub_view_marker_array_.getNumSubscribers() == 0)
    return;

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
    visualization_msgs::Marker pose_marker = visualizeCreateArrowMarker(i, valid_poses[i], true, max_z, min_z);
    pose_array.markers.push_back(pose_marker);
  }
  
  // Invalid markers
  int offset = valid_poses.size();
  for (int i=0; i<invalid_poses.size(); i++)
  {
    visualization_msgs::Marker pose_marker = visualizeCreateArrowMarker(i+offset, invalid_poses[i], false);
    pose_array.markers.push_back(pose_marker);
  }
  
  // Delete old markers
  for (int i=valid_poses.size() + invalid_poses.size(); i<vis_marker_array_prev_size_; i++)
  {
    visualization_msgs::Marker pose_marker = visualizeDeleteArrowMarker(i);
    pose_array.markers.push_back(pose_marker);
  }
  
  vis_marker_array_prev_size_ = valid_poses.size() + invalid_poses.size();
  pub_view_marker_array_.publish(pose_array);
  
  if (is_debug_)
  {
    std::cout << "Press ENTER to continue\n";
    std::cin.get();
  }
}

visualization_msgs::Marker ViewGeneratorBase::visualizeDeleteArrowMarker(int id)
{
  visualization_msgs::Marker pose_marker;

  pose_marker.header.frame_id = "world";
  pose_marker.header.stamp = ros::Time::now();
  pose_marker.id = id;

  //pose_marker.action = visualization_msgs::Marker::DELETE;
  pose_marker.type = visualization_msgs::Marker::ARROW;
  pose_marker.pose = geometry_msgs::Pose();
  return pose_marker;
}

visualization_msgs::Marker ViewGeneratorBase::visualizeCreateArrowMarker(int id, geometry_msgs::Pose pose, bool valid, double max_z, double min_z)
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

void ViewGeneratorBase::visualizeDrawSphere(geometry_msgs::Pose p, double r)
{
  if (pub_collision_marker_.getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.id = vis_sphere_counter_;
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
  pub_collision_marker_.publish( marker );

  vis_sphere_counter_++;
}

//taken from coverage_path_planning_heuristic.cpp of sscpp / asscpp
void ViewGeneratorBase::loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::list<CGALTriangle>& triangles)
{
    FILE* file = fopen(filename, "rb");
    if(!file)
    {
        std::cerr << "file not exist" << std::endl;
        return;
    }

    bool has_normal = false;
    bool has_texture = false;
    char line_buffer[2000];
    while(fgets(line_buffer, 2000, file))
    {
        char* first_token = strtok(line_buffer, "\r\n\t ");
        if(!first_token || first_token[0] == '#' || first_token[0] == 0)
            continue;

        switch(first_token[0])
        {
        case 'v':
        {
            if(first_token[1] == 'n')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_normal = true;
            }
            else if(first_token[1] == 't')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_texture = true;
            }
            else
            {
                fcl::FCL_REAL x = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::FCL_REAL y = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::FCL_REAL z = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::Vec3f p(x, y, z);
                points.push_back(p);
            }
        }
            break;
        case 'f':
        {
            CGALTriangle tri;
            char* data[30];
            int n = 0;
            while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
            {
                if(strlen(data[n]))
                    n++;
            }

            for(int t = 0; t < (n - 2); ++t)
            {
                if((!has_texture) && (!has_normal))
                {
                    Point p1(points[atoi(data[0]) - 1][0],points[atoi(data[0]) - 1][1],points[atoi(data[0]) - 1][2]);
                    Point p2(points[atoi(data[1]) - 1][0],points[atoi(data[1]) - 1][1],points[atoi(data[1]) - 1][2]);
                    Point p3(points[atoi(data[2]) - 1][0],points[atoi(data[2]) - 1][1],points[atoi(data[2]) - 1][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
                else
                {
                    const char *v1;
                    uint indxs[3];
                    for(int i = 0; i < 3; i++)
                    {
                        // vertex ID
                        if(i == 0)
                            v1 = data[0];
                        else
                            v1 = data[t + i];

                        indxs[i] = atoi(v1) - 1;
                    }
                    Point p1(points[indxs[0]][0],points[indxs[0]][1],points[indxs[0]][2]);
                    Point p2(points[indxs[1]][0],points[indxs[1]][1],points[indxs[1]][2]);
                    Point p3(points[indxs[2]][0],points[indxs[2]][1],points[indxs[2]][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
            }
        }
        }
    }
}
