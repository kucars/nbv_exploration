#include <iostream>
#include <nbv_exploration/view_generator.h>
#include <nbv_exploration/view_selecter.h>

#include <tf_conversions/tf_eigen.h>

#include <culling/occlusion_culling.h>


// =======
// Occlusion culling
// =======




// =======
// Base
// =======
double ViewSelecterBase::calculateIG(Pose p)
{
  OcclusionCulling occlusionCulling(cloud_ptr_);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud <pcl::PointXYZ>);
  *tempCloud = occlusionCulling.extractVisibleSurface(p);
  
  return tempCloud->points.size();
}

double ViewSelecterBase::calculateDistance(Pose p)
{
  return sqrt( (p.position.x-current_pose_.position.x)*(p.position.x-current_pose_.position.x) + 
         (p.position.y-current_pose_.position.y)*(p.position.y-current_pose_.position.y) +
         (p.position.z-current_pose_.position.z)*(p.position.z-current_pose_.position.z) );
}

double ViewSelecterBase::calculateAngularDistance(Pose p)
{
  double yaw1 = getYawFromQuaternion(current_pose_.orientation);
  double yaw2 = getYawFromQuaternion(p.orientation);
  
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
  double IG = -calculateIG(p);
  double effort = calculateDistance(p) + calculateAngularDistance(p)/M_PI;
  
  return IG;// /effort;
}

void ViewSelecterBase::evaluate()
{
  // Update curernt pose and map
  cloud_ptr_    = view_gen_->cloud_ptr_;
  current_pose_ = view_gen_->current_pose_;
  
  double maxUtility = -1/.0; //-inf
  
  
  for (int i=0; i<view_gen_->generated_poses.size(); i++)
  {
    Pose p = view_gen_->generated_poses[i];
    
    double utility = calculateUtility(p);
    std::cout << "[ViewSelecterBase::evaluate()] Utility of pose[" << i << "]: " << utility << "\n";
    
    if (utility > maxUtility)
    {
      
      maxUtility = utility;
      selected_pose_ = p;
    }
    
    //std::cout << "[ViewSelecterBase::evaluate] Looking at pose[" << i << "]:\nx = " << p.position.x << "\ty = "  << p.position.y << "\tz = "  << p.position.z << "\n";
  }
  
}
