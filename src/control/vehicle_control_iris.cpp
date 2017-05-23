#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "control/vehicle_control_iris.h"
#include "nbv_exploration/common.h"

VehicleControlIris::VehicleControlIris():
  VehicleControlBase() // Call super class constructor
{
  ros::NodeHandle ros_node;

  // Parameters
  std::string topic_odometry;
  ros::param::param("~topic_odometry", topic_odometry, std::string("/iris/ground_truth/odometry") );
  ros::param::param("~nav_bounds_z_min", uav_height_min_, 0.1 );
  ros::param::param("~nav_bounds_z_max", uav_height_max_, 10.0);

  // Callbacks
  sub_odom = ros_node.subscribe(topic_odometry, 1, &VehicleControlIris::callbackOdometry, this);
  pub_setpoint = ros_node.advertise<geometry_msgs::PoseStamped>("/iris/mavros/setpoint_position/local", 10);

  getPose();
  std::cout << cc.yellow << "Warning: 'uav_height_max_' monitoring not implimented in the VehicleControlIris class\n" << cc.reset;
}

// Update global position of UGV
void VehicleControlIris::callbackOdometry(const nav_msgs::Odometry& odom_msg)
{
  // Save pose and velocities
  vehicle_current_pose_ = odom_msg.pose.pose;
  vehicle_current_twist_= odom_msg.twist.twist;

  if (std::isnan(vehicle_current_pose_.position.x) || std::isnan(vehicle_current_pose_.orientation.x))
  {
    std::cout << cc.red << "Current vehicle position not found..." << cc.reset;
    is_ready_ = false;
  }
  else
  {
    is_ready_ = true;
  }
}


bool VehicleControlIris::isReady()
{
  return is_ready_;
}


bool VehicleControlIris::isStationary(double threshold_sensitivity)
{
  double max_speed = linear_speed_threshold_*threshold_sensitivity;
  double max_rate = angular_speed_threshold_*threshold_sensitivity;

  if (vehicle_current_twist_.linear.x < max_speed &&
      vehicle_current_twist_.linear.y < max_speed &&
      vehicle_current_twist_.linear.z < max_speed &&
      vehicle_current_twist_.angular.x < max_rate &&
      vehicle_current_twist_.angular.y < max_rate &&
      vehicle_current_twist_.angular.z < max_rate)
  {
    return true;
  }

  return false;
}


void VehicleControlIris::moveVehicle(double threshold_sensitivity)
{
  // Create stamped pose
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "base_footprint";
  ps.header.stamp = ros::Time::now();
  ps.pose = setpoint_;

  pub_setpoint.publish(ps);


  // Convert setpoint to world frame
  geometry_msgs::Pose setpoint_world;
  setpoint_world = transformSetpoint2Global(setpoint_);

  // Wait till we've reached the waypoint
  ros::Rate rate(30);
  while(ros::ok() && (!isNear(setpoint_world, vehicle_current_pose_, threshold_sensitivity) || !isStationary(threshold_sensitivity) ) )
  {
    ros::spinOnce();
    rate.sleep();
  }
}


void VehicleControlIris::setSpeed(double speed)
{
  std::cout << cc.yellow << "Warning: setSpeed(double) not implimented in the VehicleControlIris class\n" << cc.reset;

  if (speed < 0)
    return; //Ignore invalid speeds
}


void VehicleControlIris::setWaypoint(double x, double y, double z, double yaw)
{
  geometry_msgs::Pose setpoint_world;

  // Position
  setpoint_world.position.x = x;
  setpoint_world.position.y = y;
  setpoint_world.position.z = z;

  // Orientation
  setpoint_world.orientation = pose_conversion::getQuaternionFromYaw(yaw);

  // Transform to setpoint frame
  setpoint_ = transformGlobal2Setpoint(setpoint_world);
}


void VehicleControlIris::setWaypoint(geometry_msgs::Pose p)
{
  // Transform to setpoint frame
  setpoint_ = transformGlobal2Setpoint(p);
}


void VehicleControlIris::setWaypointIncrement(double x, double y, double z, double yaw)
{
  geometry_msgs::Pose setpoint_world;

  // Position
  setpoint_world.position.x = vehicle_current_pose_.position.x + x;
  setpoint_world.position.y = vehicle_current_pose_.position.y + y;
  setpoint_world.position.z = vehicle_current_pose_.position.z + z;

  // Orientation
  double yaw_current =  pose_conversion::getYawFromQuaternion(vehicle_current_pose_.orientation);
  setpoint_world.orientation =  pose_conversion::getQuaternionFromYaw(yaw_current + yaw);

  // Transform to setpoint frame
  setpoint_ = transformGlobal2Setpoint(setpoint_world);
}


void VehicleControlIris::start()
{
  ros::Rate rate(10);
  while(ros::ok() && !is_ready_)
  {
    std::cout << cc.yellow << "Iris not ready!\n" << cc.reset;

    ros::spinOnce();
    rate.sleep();
  }

  // Take off
  std::cout << cc.green << "Taking off\n" << cc.reset;

  if (vehicle_current_pose_.position.z < uav_height_min_)
  {
    setWaypointIncrement(0, 0, uav_height_min_, 0);
    moveVehicle(0.3);
  }

  std::cout << cc.green << "Done taking off\n" << cc.reset;
  is_ready_ = true;
}


geometry_msgs::Pose VehicleControlIris::transformSetpoint2Global (const geometry_msgs::Pose p_set)
{
  geometry_msgs::Pose p_global;

  // Apply a 90 degree clockwise rotation on the z-axis
  p_global.position.x = p_set.position.y;
  p_global.position.y =-p_set.position.x;
  p_global.position.z = p_set.position.z;

  // Rotate orientation
  double yaw = pose_conversion::getYawFromQuaternion(p_set.orientation);
  yaw -= M_PI_2;
  p_global.orientation = pose_conversion::getQuaternionFromYaw(yaw);

  return p_global;
}


geometry_msgs::Pose VehicleControlIris::transformGlobal2Setpoint (const geometry_msgs::Pose p_global)
{
  geometry_msgs::Pose p_set;

  // Apply a 90 degree anti-clockwise rotation on the z-axis
  p_set.position.x =-p_global.position.y;
  p_set.position.y = p_global.position.x;
  p_set.position.z = p_global.position.z;

  // Rotate orientation
  double yaw = pose_conversion::getYawFromQuaternion(p_global.orientation);
  yaw += M_PI_2;
  p_set.orientation = pose_conversion::getQuaternionFromYaw(yaw);

  return p_set;
}
