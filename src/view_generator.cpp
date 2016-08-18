#include <iostream>
#include <nbv_exploration/view_generator.h>


// ==============
// Nearest Neighbor
// ==============
void ViewGeneratorNN::generateViews()
{
  generated_poses.clear();
  
  double currX = currentPose.position.x;
  double currY = currentPose.position.y;
  double currZ = currentPose.position.z;
  double currYaw = getYawFromQuaternion(currentPose.orientation);
  
  //@TODO: Must check if viewpoints area reachable
  
  if (cloudPtr->points.size() < 0)
  {
    std::cout << "[ViewpointGen::NN] No points in map. Rotating" << std::endl;
    
    Pose p;
    p.position.x = std::numeric_limits<double>::quiet_NaN(); //Set to NaN
    p.position.y = std::numeric_limits<double>::quiet_NaN();;
    p.position.z = std::numeric_limits<double>::quiet_NaN();;
    p.orientation = getQuaternionFromYaw(res_yaw); //Rotate 22.5 deg
    
    generated_poses.push_back(p);
  }
  else
  {
    std::cout << "[ViewpointGen::NN] Generating 4-D state lattice" << std::endl;
    
    for (int i=-1; i<=1; i+=2)
    {
      Pose p;
      p.position.x = currX + res_x*i;
      p.position.y = currY;
      p.position.z = currZ;
      p.orientation = currentPose.orientation;
      generated_poses.push_back(p);
    }
    
    for (int i=-1; i<=1; i+=2)
    {
      Pose p;
      p.position.x = currX;
      p.position.y = currY + res_y*i;
      p.position.z = currZ;
      p.orientation = currentPose.orientation;
      generated_poses.push_back(p);
    }
    
    for (int i=-1; i<=1; i+=2)
    {
      Pose p;
      p.position.x = currX;
      p.position.y = currY;
      p.position.z = currZ + res_z*i;
      p.orientation = currentPose.orientation;
      generated_poses.push_back(p);
    }
    
    for (int i=-1; i<=1; i+=2)
    {
      Pose p;
      p.position.x = currX;
      p.position.y = currY;
      p.position.z = currZ;
      p.orientation = getQuaternionFromYaw(currYaw + res_yaw*i);
      generated_poses.push_back(p);
    }
    
    std::cout << "[ViewpointGen::NN] Generated " << generated_poses.size() << " poses" << std::endl;
  }
}

// ==============
// Frontier class
// ==============
void ViewGeneratorFrontier::generateViews()
{
  generated_poses.clear();
  
  if (cloudPtr->points.size() < 0)
  {
    std::cout << "[ViewpointGen::Frontier] No points in map. Rotating" << std::endl;
    
    Pose p;
    p.position.x = std::numeric_limits<double>::quiet_NaN(); //Set to NaN
    p.position.y = std::numeric_limits<double>::quiet_NaN();;
    p.position.z = std::numeric_limits<double>::quiet_NaN();;
    p.orientation = getQuaternionFromYaw(res_yaw); //Rotate 22.5 deg
    
    generated_poses.push_back(p);
  }
  else
  {
    std::cout << "[ViewpointGen::Frontier] Point in map. Generating viewpoint" << std::endl;
  }
}
