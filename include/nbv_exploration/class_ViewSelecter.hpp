#pragma once

#include <iostream>

#include <geometry_msgs/Pose.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nbv_exploration/class_ViewGenerator.hpp>

//typedef pcl::PointXYZRGBA PointT;
typedef geometry_msgs::Pose Pose;


class ViewSelecter_Base
{
protected:
	ViewGenerator_Base* _viewGen;
	Pose _currentPose;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloudPtr;
	
public:
    ViewSelecter_Base(){}
    ~ViewSelecter_Base(){}
    
    Pose selectedPose;
    
    
    Pose getTargetPose(){
		return selectedPose;
	}
    void setViewGenerator(ViewGenerator_Base* v){
		_viewGen     = v;
		_cloudPtr    = v->cloudPtr;
		_currentPose = v->currentPose;
	}

	void evaluate();
	double calculateIG(Pose p);
	double calculateDistance(Pose p);
	double calculateAngularDistance(Pose p);
	
	virtual double calculateUtility(Pose p);
};
