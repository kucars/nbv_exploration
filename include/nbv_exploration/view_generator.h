#ifndef NBV_EXPLORATION_VIEW_GENERATOR_H
#define NBV_EXPLORATION_VIEW_GENERATOR_H

#include <iostream>

#include <geometry_msgs/Pose.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf_conversions/tf_eigen.h>

//typedef pcl::PointXYZRGBA PointT;
typedef geometry_msgs::Pose Pose;



double getYawFromQuaternion(geometry_msgs::Quaternion gm_q)
{
  double roll, pitch, yaw;

  tf::Quaternion quat (gm_q.x, gm_q.y, gm_q.z, gm_q.w);
    
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  
  return yaw;
}

geometry_msgs::Quaternion getQuaternionFromYaw(double yaw)
{
  geometry_msgs::Quaternion quat;
  
  tf::Quaternion tf_q;
  tf_q = tf::createQuaternionFromYaw(yaw); //Rotate 22.5 deg
  
  quat.x = tf_q.getX();
  quat.y = tf_q.getY();
  quat.z = tf_q.getZ();
  quat.w = tf_q.getW();
  
  return quat;
}




class ViewGeneratorBase
{
protected:
  double res_x;
  double res_y;
  double res_z;
  double res_yaw = M_PI_4;
  
public:
  ViewGeneratorBase(){}
  ~ViewGeneratorBase(){}

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
  Pose currentPose;
  std::vector<Pose, Eigen::aligned_allocator<Pose> > generated_poses;
  

  void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_cloud)
  {
    cloudPtr = in_cloud;
  }
  
  void setCurrentPose(Pose p)
  {
    currentPose = p;
  }
  
  virtual void generateViews()
  {
    std::cout << "[WARNING] Call to ViewGeneratorBase::generateViews(). Impliment function in derived class. No poses generated." << std::endl;
  }
  
  // Not used by all derived classes
  virtual void setResolution(double x, double y, double z, double yaw)
  {
    res_x = x;
    res_y = y;
    res_z = z;
    res_yaw = yaw;
  }
};


// Nearest neighbor
class ViewGeneratorNN : public ViewGeneratorBase
{
public:
  ViewGeneratorNN(){}
  ~ViewGeneratorNN(){}

  void generateViews();
  
};

// Frontier
class ViewGeneratorFrontier : public ViewGeneratorBase
{
public:
  ViewGeneratorFrontier(){}
  ~ViewGeneratorFrontier(){}

  void generateViews();
};


#endif
