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
double ViewSelecterBase::getNodeOccupancy(octomap::OcTreeNode* node)
{
	double p;
	
	if( node==NULL )
	{
		p = 0.5;
	}
	else
	{
		p = node->getOccupancy();
	}
	return p;
}


double ViewSelecterBase::getNodeEntropy(octomap::OcTreeNode* node)
{
	double p = getNodeOccupancy(node);
	
	if (p==0 || p==1)
		return 0;
		
	return - p*log(p) - (1-p)*log(1-p);
}

double ViewSelecterBase::computeRelativeRays()
{
	ray_directions_.clear();
	
	double deg2rad = M_PI/180;
	double min_x = -range_max_ * tan(fov_horizontal_ * deg2rad);
	double min_y = -range_max_ * tan(fov_vertical_ * deg2rad);
	double max_x =  range_max_ * tan(fov_horizontal_ * deg2rad);
	double max_y =  range_max_ * tan(fov_vertical_ * deg2rad);
	
	double x_step = tree_resolution_;
	double y_step = tree_resolution_;
	
	std::cout << "min_x: " << min_x << "\n";
	std::cout << "step: " << x_step << "\n";
	
	for( double x = min_x; x<=max_x; x+=x_step )
	{
		for( double y = min_y; y<=max_y; y+=y_step )
		{
			Eigen::Vector3d ray_dir(x, y, range_max_);
			ray_dir.normalize();
			ray_directions_.push_back(ray_dir);
		}
  }
}

void ViewSelecterBase::computeOrientationMatrix(Pose p)
{
	Eigen::Quaterniond q(p.orientation.x,
											 p.orientation.y,
											 p.orientation.z,
											 p.orientation.w); 
	
	rotation_mtx_ = q.toRotationMatrix();
}

octomap::point3d ViewSelecterBase::getGlobalRayDirection(Eigen::Vector3d ray_dir)
{
	Eigen::Vector3d temp = rotation_mtx_*ray_dir;
	octomap::point3d p (temp[0], temp[1], temp[2]);
	
	return p;
}



double ViewSelecterBase::calculateIG(Pose p)
{
	// Source: Borrowed partially from
	// https://github.com/uzh-rpg/rpg_ig_active_reconstruction/blob/master/ig_active_reconstruction_octomap/src/code_base/octomap_basic_ray_ig_calculator.inl
	double t_start, t_end;
  t_start = ros::Time::now().toSec();
  
	octomap::point3d origin (p.position.x, p.position.y, p.position.z);
	
	double ig_total = 0;
	int nodes_traversed = 0;
	
	//std::set<> nodes;
	
	for (int i=0; i<ray_directions_.size(); i++)
	{
		double ig_ray = 0;
		
		octomap::point3d dir = getGlobalRayDirection(ray_directions_[i]);
		octomap::point3d endpoint;
		
		// Cast through unknown cells as well as free cells
		bool found_endpoint = tree_->castRay( origin, dir, endpoint, true, range_max_ );
		
		if (!found_endpoint)
		{
			endpoint = origin + dir * range_max_;
		}
		
		octomap::KeyRay ray;
		tree_->computeRayKeys( origin, endpoint, ray );
		
		for( octomap::KeyRay::iterator it = ray.begin() ; it!=ray.end(); ++it )
		{
			octomap::point3d coord = tree_->keyToCoord(*it);
			octomap::OcTreeNode* node = tree_->search(*it);
			nodes_traversed++;
			
			ig_ray += getNodeEntropy(node);
		}
			
		octomap::OcTreeKey end_key;
		
		if( tree_->coordToKeyChecked(endpoint, end_key) )
		{
			octomap::OcTreeNode* node = tree_->search(end_key);
			nodes_traversed++;
			
			ig_ray += getNodeEntropy(node);
		}
		
		ig_total += ig_ray;
		
		//if (i%1000 == 0)
		//	std::cout << "\tRay: " << i << "\tIG: " << ig_ray << "\n";
	}
	
	t_end = ros::Time::now().toSec();
  std::cout << "Time: " << t_end-t_start << " sec\tNodes: " << nodes_traversed << " (" << 1000*(t_end-t_start)/nodes_traversed << " ms/node)\n";
	std::cout << "\tRays: " << ray_directions_.size() << "\n";
	return ig_total;
}

double ViewSelecterBase::calculateDistance(Pose p)
{
  return sqrt(
				(p.position.x-current_pose_.position.x)*(p.position.x-current_pose_.position.x) + 
        (p.position.y-current_pose_.position.y)*(p.position.y-current_pose_.position.y) +
        (p.position.z-current_pose_.position.z)*(p.position.z-current_pose_.position.z)
        );
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
  double IG = calculateIG(p);
  double effort = calculateDistance(p) + calculateAngularDistance(p)/M_PI;
  
  return IG;// /effort;
}

void ViewSelecterBase::evaluate()
{
	update();
	
  // Update curernt pose and map
  cloud_occupied_ptr_ = view_gen_->cloud_occupied_ptr_;
  current_pose_ = view_gen_->current_pose_;
  
  double maxUtility = -1/.0; //-inf
  
  
  for (int i=0; i<view_gen_->generated_poses.size(); i++)
  {
    Pose p = view_gen_->generated_poses[i];
    computeOrientationMatrix(p);
    
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
