#include <iostream>

#include <geometry_msgs/Pose.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//typedef pcl::PointXYZRGBA PointT;
typedef geometry_msgs::Pose Pose;

class ViewGenerator_Base
{
public:
    ViewGenerator_Base();
    ~ViewGenerator_Base();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
    std::vector<Pose, Eigen::aligned_allocator<Pose> > poses;


    virtual void generate();

    void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_cloud)
    {
        cloudPtr = in_cloud;
    }
};

class ViewGenerator_Frontier : public ViewGenerator_Base
{
public:
    ViewGenerator_Frontier();
    ~ViewGenerator_Frontier();

    void generate();
};
