#include <iostream>
#include <nbv_exploration/view_generator.h>


// ==============
// Nearest Neighbor
// ==============
void ViewGeneratorNN::generateViews()
{
  generated_poses.clear();
  
  double currX = current_pose_.position.x;
  double currY = current_pose_.position.y;
  double currZ = current_pose_.position.z;
  double currYaw = getYawFromQuaternion(current_pose_.orientation);
  
  //@TODO: Must check if viewpoints area reachable
  
  if (cloud_occupied_ptr_->points.size() < 0)
  {
    std::cout << "[ViewpointGen::NN] No points in map. Rotating" << std::endl;
    
    Pose p;
    p.position.x = std::numeric_limits<double>::quiet_NaN(); //Set to NaN
    p.position.y = std::numeric_limits<double>::quiet_NaN();;
    p.position.z = std::numeric_limits<double>::quiet_NaN();;
    p.orientation = getQuaternionFromYaw(res_yaw_); //Rotate 22.5 deg
    
    generated_poses.push_back(p);
  }
  else
  {
    std::cout << "[ViewpointGen::NN] Generating 4-D state lattice" << std::endl;
    
    for (int i_x=-1; i_x<=1; i_x+=2)
    {
      for (int i_y=-1; i_y<=1; i_y+=2)
      {
        for (int i_z=-1; i_z<=1; i_z+=2)
        {
          for (int i_yaw=-1; i_yaw<=1; i_yaw+=2)
          {
            Pose p;
            p.position.x = currX + res_x_*i_x;
            p.position.y = currY + res_y_*i_y;
            if (currZ + res_z_*i_z > 1) //Don't collide with the floor
            {
              p.position.z = currZ + res_z_*i_z;
            }
            else
            {
              continue;
            }
            p.orientation = getQuaternionFromYaw(currYaw + res_yaw_*i_yaw);
            generated_poses.push_back(p);
          }
        }
      }
    }
    
    /*
    for (int i=-1; i<=1; i+=2)
    {
      Pose p;
      p.position.x = currX + res_x_*i;
      p.position.y = currY;
      p.position.z = currZ;
      p.orientation = current_pose_.orientation;
      generated_poses.push_back(p);
    }
    
    for (int i=-1; i<=1; i+=2)
    {
      Pose p;
      p.position.x = currX;
      p.position.y = currY + res_y_*i;
      p.position.z = currZ;
      p.orientation = current_pose_.orientation;
      generated_poses.push_back(p);
    }
    
    for (int i=-1; i<=1; i+=2)
    {
      Pose p;
      p.position.x = currX;
      p.position.y = currY;
      p.position.z = currZ + res_z_*i;
      p.orientation = current_pose_.orientation;
      generated_poses.push_back(p);
    }
    
    for (int i=-1; i<=1; i+=2)
    {
      Pose p;
      p.position.x = currX;
      p.position.y = currY;
      p.position.z = currZ;
      p.orientation = getQuaternionFromYaw(currYaw + res_yaw_*i);
      generated_poses.push_back(p);
    }
    */
    
    std::cout << "[ViewpointGen::NN] Generated " << generated_poses.size() << " poses" << std::endl;
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
    p.orientation = getQuaternionFromYaw(res_yaw_); //Rotate 22.5 deg
    
    generated_poses.push_back(p);
  }
  else
  {
    std::cout << "[ViewpointGen::Frontier] Point in map. Generating viewpoint" << std::endl;
  }
}
