#include <iostream>
#include <class_ViewGenerator.hpp>
#include <class_ViewSelecter.hpp>

#include <tf_conversions/tf_eigen.h>


double ViewSelecter_Base::calculateIG()
{
	return 1;
}

double ViewSelecter_Base::calculateDistance(Pose p){
	return 1;
}

void ViewSelecter_Base::evaluate(){
	selectedPose = _viewGen->generated_poses[0];
	
	std::cout << "[ViewSelecter_Base::evaluate] We're in." << std::endl;
	std::cout << "[ViewSelecter_Base::evaluate] Selected pose:\nx = " << selectedPose.position.x << "\ty = "  << selectedPose.position.y << "\tz = "  << selectedPose.position.z << "\n";
}
