#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include "nbv_exploration/view_generator_nn.h"
#include "nbv_exploration/common.h"

ViewGeneratorNN::ViewGeneratorNN():
  ViewGeneratorBase() //Call base class constructor
{
  ros::param::param("~view_generator_nn_pos_res_x", res_x_, 1.0);
  ros::param::param("~view_generator_nn_pos_res_y", res_y_, 1.0);
  ros::param::param("~view_generator_nn_pos_res_z", res_z_, 1.0);
  ros::param::param("~view_generator_nn_pos_res_yaw", res_yaw_, M_PI_4);
}

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
    std::cout << "[ViewGeneratorNN] No points in map. Rotating" << std::endl;
    
    Pose p;
    p.position.x = std::numeric_limits<double>::quiet_NaN(); //Set to NaN
    p.position.y = std::numeric_limits<double>::quiet_NaN();;
    p.position.z = std::numeric_limits<double>::quiet_NaN();;
    p.orientation = pose_conversion::getQuaternionFromYaw(res_yaw_); //Rotate 22.5 deg
    
    generated_poses.push_back(p);
  }
  else
  {
    if (is_debug_)
      std::cout << "[ViewGeneratorNN] Generating 4-D state lattice" << std::endl;
    
    for (int i_x=-1; i_x<=1; i_x++)
    {
      for (int i_y=-1; i_y<=1; i_y++)
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
    
    std::cout << "[ViewGeneratorNN] Generated " << generated_poses.size() << " poses (" << rejected_poses.size() << " rejected)" << std::endl;
    visualize(generated_poses, rejected_poses);
  }
}

std::string ViewGeneratorNN::getMethodName()
{
  return "NN";
}
