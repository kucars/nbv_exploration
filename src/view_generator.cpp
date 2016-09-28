#include <iostream>
#include <ros/ros.h>

#include <Eigen/Geometry>

#include <nbv_exploration/view_generator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nbv_exploration/pose_conversion.h>

// ==============
// Base
// ==============
ros::Publisher marker_array_pub;
int marker_array_prev_size = 0;

ViewGeneratorBase::ViewGeneratorBase()
{
	ros::NodeHandle n;
  marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("generated_pose_marker_array", 10);
}
  

void ViewGeneratorBase::visualize()
{
  visualization_msgs::MarkerArray pose_array;
  
  double max_z = -1/.0;
  double min_z =  1/.0;
  for (int i=0; i<generated_poses.size(); i++)
  {
    if (generated_poses[i].position.z > max_z)
      max_z = generated_poses[i].position.z;
    else if (generated_poses[i].position.z < min_z)
      min_z = generated_poses[i].position.z;
  }

  for (int i=0; i<generated_poses.size(); i++)
  {
    visualization_msgs::Marker pose_marker;
    
    pose_marker.header.frame_id = "world";
    pose_marker.header.stamp = ros::Time::now();
    //pose_marker.action = visualization_msgs::Marker::ADD;
  
    pose_marker.id = i;
    pose_marker.type = visualization_msgs::Marker::ARROW;
    pose_marker.pose = generated_poses[i];
    
    // Style
    double curr_z = generated_poses[i].position.z;
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
    
    pose_marker.scale.x = 0.5;
    pose_marker.scale.y = 0.1;
    pose_marker.scale.z = 0.1;
    
    pose_array.markers.push_back(pose_marker);
  }
  
  // Delete old markers
  for (int i=generated_poses.size(); i<marker_array_prev_size; i++)
  {
    visualization_msgs::Marker pose_marker;
    
    pose_marker.header.frame_id = "world";
    pose_marker.header.stamp = ros::Time::now();
    pose_marker.action = visualization_msgs::Marker::DELETE;
    pose_marker.id = i;
    
    pose_array.markers.push_back(pose_marker);
  }
  
  marker_array_prev_size = generated_poses.size();
  marker_array_pub.publish(pose_array);
  
  std::cout << "Press ENTER to continue\n";
  std::cin.get();
}


// ==============
// Nearest Neighbor
// ==============
void ViewGeneratorNN::generateViews()
{
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
      for (int i_y=-1; i_y<=0; i_y++)
      {
        for (int i_z=-1; i_z<=1; i_z++)
        {
          for (int i_yaw=-1; i_yaw<=1; i_yaw++)
          {
            if (i_x==0 && i_y==0 && i_z==0 && i_yaw==0)
              continue;
            
            Pose p;
            p.position.x = currX + res_x_*i_x*cos(currYaw) + res_y_*i_y*sin(currYaw);
            p.position.y = currY - res_x_*i_x*sin(currYaw) + res_y_*i_y*cos(currYaw);
            if (currZ + res_z_*i_z > 1) //Don't collide with the floor
            {
              p.position.z = currZ + res_z_*i_z;
            }
            else
            {
              continue;
            }
            p.orientation = pose_conversion::getQuaternionFromYaw(currYaw + res_yaw_*i_yaw);
            generated_poses.push_back(p);
          }
        }
      }
    }
    
    if (is_debug_)
    {
      std::cout << "[ViewpointGen::NN] Generated " << generated_poses.size() << " poses" << std::endl;
      visualize();
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
    p.position.y = std::numeric_limits<double>::quiet_NaN();;
    p.position.z = std::numeric_limits<double>::quiet_NaN();;
    p.orientation = pose_conversion::getQuaternionFromYaw(res_yaw_); //Rotate 22.5 deg
    
    generated_poses.push_back(p);
  }
  else
  {
    std::cout << "[ViewpointGen::Frontier] Point in map. Generating viewpoint" << std::endl;
  }
}
