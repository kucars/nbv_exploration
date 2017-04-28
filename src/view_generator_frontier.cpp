#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include "nbv_exploration/view_generator_frontier.h"
#include "nbv_exploration/common.h"

ViewGeneratorFrontier::ViewGeneratorFrontier():
  ViewGeneratorBase() //Call base class constructor
{

}

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
