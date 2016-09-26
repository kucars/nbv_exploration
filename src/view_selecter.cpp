#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

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
ros::Publisher marker_pub;
ros::Publisher pose_pub;

ViewSelecterBase::ViewSelecterBase()
{
	ros::NodeHandle n;
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("visualization_marker_pose", 10);
	
	last_max_utility_ = 1/.0;
}

void ViewSelecterBase::addToRayMarkers(octomap::point3d origin, octomap::point3d endpoint)
{
	geometry_msgs::Point p;
	
	// Start
	p.x = origin.x();
	p.y = origin.y();
	p.z = origin.z();
	line_list.points.push_back(p);

	// End
	p.x = endpoint.x();
	p.y = endpoint.y();
	p.z = endpoint.z();
	line_list.points.push_back(p);
}


void ViewSelecterBase::clearRayMarkers()
{
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.01;
	
	// Blue lines
	line_list.color.b = 1.0;
  line_list.color.a = 0.1;
	
	line_list.points.clear();

}

void ViewSelecterBase::publishRayMarkers()
{
	line_list.header.frame_id = "world";
  line_list.header.stamp = ros::Time::now();
  
  marker_pub.publish(line_list);
}

void ViewSelecterBase::publishPose(Pose p)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  
  msg.pose = p;
  
  pose_pub.publish(msg);
}




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
	double min_x = -range_max_ * tan(fov_horizontal_/2 * deg2rad);
	double min_y = -range_max_ * tan(fov_vertical_/2 * deg2rad);
	double max_x =  range_max_ * tan(fov_horizontal_/2 * deg2rad);
	double max_y =  range_max_ * tan(fov_vertical_/2 * deg2rad);
	
	double x_step = tree_resolution_;
	double y_step = tree_resolution_;
	
	for( double x = min_x; x<=max_x; x+=x_step )
	{
		for( double y = min_y; y<=max_y; y+=y_step )
		{
			Eigen::Vector3d ray_dir(y, x, range_max_);
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
	
	/* Additional rotation to correct alignment of rays */
	Eigen::Matrix3d r;
	r << 0, 0, 1,
			 0, 1, 0,
			 1, 0, 0;
	
	rotation_mtx_ = r*q.toRotationMatrix();
}

octomap::point3d ViewSelecterBase::getGlobalRayDirection(Eigen::Vector3d ray_dir)
{
	Eigen::Vector3d temp = rotation_mtx_*ray_dir;
	octomap::point3d p (temp[0], temp[1], temp[2]);
	
	return p;
}

struct octomapKeyCompare {
  bool operator() (const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const
  {
		size_t h1 = size_t(lhs.k[0]) + 1447*size_t(lhs.k[1]) + 345637*size_t(lhs.k[2]);
		size_t h2 = size_t(rhs.k[0]) + 1447*size_t(rhs.k[1]) + 345637*size_t(rhs.k[2]);
		return h1< h2;
	}
};

double ViewSelecterBase::calculateIG(Pose p)
{
	// Source: Borrowed partially from
	// https://github.com/uzh-rpg/rpg_ig_active_reconstruction/blob/master/ig_active_reconstruction_octomap/src/code_base/octomap_basic_ray_ig_calculator.inl
	double t_start, t_end;
  t_start = ros::Time::now().toSec();
  
	octomap::point3d origin (p.position.x, p.position.y, p.position.z);
	
	int nodes_traversed = 0;
	double observedOccupied = false;
	double min_height = 0.5;
	
	std::set<octomap::OcTreeKey, octomapKeyCompare> nodes; //all nodes in a set are UNIQUE
	
	clearRayMarkers();
	
	for (int i=0; i<ray_directions_.size(); i++)
	{
		double ig_ray = 0;
		
		octomap::point3d dir = getGlobalRayDirection(ray_directions_[i]);
		octomap::point3d endpoint;
		
		// Cast through unknown cells as well as free cells
		bool found_endpoint = tree_->castRay( origin, dir, endpoint, true, range_max_ );
		if (found_endpoint)
		{
			observedOccupied = true;
		}
		else
		{
			endpoint = origin + dir * range_max_;
			
			if (endpoint.z() < min_height)
			{
				double k = (min_height - origin.z()) / dir.z();
				endpoint = origin + dir * k;
			}
		}
		
		addToRayMarkers(origin, endpoint);
		
		octomap::KeyRay ray;
		tree_->computeRayKeys( origin, endpoint, ray );
		for( octomap::KeyRay::iterator it = ray.begin() ; it!=ray.end(); ++it )
		{
			nodes.insert(*it);
			nodes_traversed++;
		}
			
		octomap::OcTreeKey end_key;
		if( tree_->coordToKeyChecked(endpoint, end_key) )
		{
			nodes.insert(end_key);
			nodes_traversed++;
		}
	}
	
	publishRayMarkers();
	publishPose(p);
	
	int nodes_processed = nodes.size();
	double ig_total = 0;
	
	if (observedOccupied) //Only compute IG if we've seen at least one valid occupied voxel
	{
		for (std::set<octomap::OcTreeKey>::iterator it=nodes.begin(); it!=nodes.end(); ++it)
		{
			octomap::OcTreeNode* node = tree_->search(*it);	
			ig_total += getNodeEntropy(node);
		}
	}
	
	t_end = ros::Time::now().toSec();
  std::cout << "Time: " << t_end-t_start << " sec\tNodes: " << nodes_processed << "/" << nodes_traversed<< " (" << 1000*(t_end-t_start)/nodes_processed << " ms/node)\n";
  //std::cout << "\tAverage nodes per ray: " << nodes_traversed/ray_directions_.size() << "\n";
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
  
  ros::Duration sleep_duration(1);
  for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
  {
    Pose p = view_gen_->generated_poses[i];
    computeOrientationMatrix(p);
    
    double utility = calculateUtility(p);
    std::cout << "Utility of pose[" << i << "]: " << utility << "\n";
    
    if (utility > maxUtility)
    {
      
      maxUtility = utility;
      selected_pose_ = p;
    }
    
    //sleep_duration.sleep();
    //std::cout << "[ViewSelecterBase::evaluate] Looking at pose[" << i << "]:\nx = " << p.position.x << "\ty = "  << p.position.y << "\tz = "  << p.position.z << "\n";
  }
  
  last_max_utility_ = maxUtility;
  
}

bool ViewSelecterBase::isEntropyLow()
{
	if (last_max_utility_ == 0)
		return true;
	
	return false;
}
