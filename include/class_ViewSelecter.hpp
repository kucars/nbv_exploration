#pragma once

#include <iostream>

#include <geometry_msgs/Pose.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <class_ViewGenerator.hpp>

//typedef pcl::PointXYZRGBA PointT;
typedef geometry_msgs::Pose Pose;


class ViewSelecter_Base
{
protected:
	ViewGenerator_Base* _viewGen;
public:
    ViewSelecter_Base(){}
    ~ViewSelecter_Base(){}
    
    Pose selectedPose;
    
    
    Pose getTargetPose(){
		return selectedPose;
	}
    void setViewGenerator(ViewGenerator_Base* v){
		_viewGen = v;
	}

	double calculateIG();
	double calculateDistance(Pose p);
	void evaluate();
	
	virtual double calculateUtility(){};
};
