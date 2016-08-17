#pragma once

#include <iostream>

#include <geometry_msgs/Pose.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//typedef pcl::PointXYZRGBA PointT;
typedef geometry_msgs::Pose Pose;

class ViewGenerator_Base
{
protected:
    double res_x;
    double res_y;
    double res_z;
    double res_yaw = M_PI_4;
    
public:
    ViewGenerator_Base(){}
    ~ViewGenerator_Base(){}

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
    Pose currentPose;
    std::vector<Pose, Eigen::aligned_allocator<Pose> > generated_poses;
    

    void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_cloud)
    {
        cloudPtr = in_cloud;
    }
    
    void setCurrentPose(Pose p){
        currentPose = p;
    }
    
    virtual void generate(){
        std::cout << "[WARNING] Call to ViewGenerator_Base::generate(). Impliment function in derived class. No poses generated." << std::endl;
    }
    
    // Not used by all derived classes
    virtual void setResolution(double x, double y, double z, double yaw){
        res_x = x;
        res_y = y;
        res_z = z;
        res_yaw = yaw;
    }
};


// Nearest neighbor
class ViewGenerator_NN : public ViewGenerator_Base
{
public:
    ViewGenerator_NN(){}
    ~ViewGenerator_NN(){}

    void generate();
    
};

// Frontier
class ViewGenerator_Frontier : public ViewGenerator_Base
{
public:
    ViewGenerator_Frontier(){}
    ~ViewGenerator_Frontier(){}

    void generate();
};
