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
  OcclusionCulling occlusionCulling(_cloudPtr);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud <pcl::PointXYZ>);
  *tempCloud = occlusionCulling.extractVisibleSurface(p);
  
  return tempCloud->points.size();
}

double ViewSelecterBase::calculateDistance(Pose p)
{
  return sqrt( (p.position.x-_currentPose.position.x)*(p.position.x-_currentPose.position.x) + 
         (p.position.y-_currentPose.position.y)*(p.position.y-_currentPose.position.y) +
         (p.position.z-_currentPose.position.z)*(p.position.z-_currentPose.position.z) );
}

double ViewSelecterBase::calculateAngularDistance(Pose p)
{
  double yaw1 = getYawFromQuaternion(_currentPose.orientation);
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
  _cloudPtr    = _viewGen->cloudPtr;
  _currentPose = _viewGen->currentPose;
  
  double maxUtility = -1/.0; //-inf
  
  
  for (int i=0; i<_viewGen->generated_poses.size(); i++)
  {
    Pose p = _viewGen->generated_poses[i];
    
    double utility = calculateUtility(p);
    std::cout << "[ViewSelecterBase::evaluate()] Utility of pose[" << i << "]: " << utility << "\n";
    
    if (utility > maxUtility)
    {
      
      maxUtility = utility;
      selected_pose = p;
    }
    
    //std::cout << "[ViewSelecterBase::evaluate] Looking at pose[" << i << "]:\nx = " << p.position.x << "\ty = "  << p.position.y << "\tz = "  << p.position.z << "\n";
  }
  
}
