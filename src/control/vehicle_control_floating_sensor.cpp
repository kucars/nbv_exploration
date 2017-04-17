#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "control/vehicle_control_floating_sensor.h"
#include "nbv_exploration/common.h"


VehicleControlFloatingSensor::VehicleControlFloatingSensor():
  VehicleControlBase() // Call super class constructor
{
  ros::NodeHandle ros_node;

  // Callbacks
  sub_odom = ros_node.subscribe("/floating_sensor/pose", 1, &VehicleControlFloatingSensor::callbackPose, this);
  pub_pose = ros_node.advertise<geometry_msgs::Pose>("/floating_sensor/set_pose", 10);

  getPose();
}


// Update global position of UGV
void VehicleControlFloatingSensor::callbackPose(const geometry_msgs::Pose& msg)
{
  // Save pose and velocities
  vehicle_current_pose_ = msg;

  if (isnan(vehicle_current_pose_.position.x) || isnan(vehicle_current_pose_.orientation.x))
  {
    std::cout << cc.red << "Current vehicle position not found..." << cc.reset;
    is_ready_ = false;
  }
  else
  {
    is_ready_ = true;
  }
}


bool VehicleControlFloatingSensor::isReady()
{
  return is_ready_;
}


void VehicleControlFloatingSensor::moveVehicle(double threshold_sensitivity)
{
  // Publish pose
  setpoint_.header.frame_id = "base_footprint";
  setpoint_.header.stamp = ros::Time::now();
  pub_setpoint.publish(setpoint_);

  // Wait till we've reached the waypoint
  ros::Rate rate(30);
  while(ros::ok())
  {
    /*
    if (is_debug && is_debug_callbacks)
    {
      std::cout << cc.green << "Moving to destination. " <<
        "Distance to target: " << getDistance(setpoint_world, vehicle_current_pose_) <<
        "\tAngle to target: " << getAngularDistance(setpoint_world, vehicle_current_pose_) << "\n" << cc.reset;
    }
    */

    ros::spinOnce();
    rate.sleep();

    break;
  }
}


void VehicleControlFloatingSensor::setSpeed(double speed)
{
  std::cout << cc.yellow << "Warning: setSpeed(double) not implimented in the VehicleControlFloatingSensor class\n" << cc.reset;
}


void VehicleControlFloatingSensor::setWaypoint(double x, double y, double z, double yaw)
{
  // Position
  setpoint_.position.x = x;
  setpoint_.position.y = y;
  setpoint_.position.z = z;

  // Orientation
  setpoint_.orientation = pose_conversion::getQuaternionFromYaw(yaw);
}


void VehicleControlFloatingSensor::setWaypoint(geometry_msgs::Pose p)
{
  setpoint_.pose = p;
}


void VehicleControlFloatingSensor::setWaypointIncrement(double x, double y, double z, double yaw)
{
  // Position
  setpoint_.position.x = vehicle_current_pose_.position.x + x;
  setpoint_.position.y = vehicle_current_pose_.position.y + y;
  setpoint_.position.z = vehicle_current_pose_.position.z + z;

  // Orientation
  double yaw_current =  pose_conversion::getYawFromQuaternion(vehicle_current_pose_.orientation);
  setpoint_.orientation =  pose_conversion::getQuaternionFromYaw(yaw_current + yaw);
}


void VehicleControlFloatingSensor::start()
{
  ros::Rate rate(10);
  while(ros::ok() && !is_ready_)
  {
    std::cout << cc.yellow << "Sensor not ready!\n" << cc.reset;

    ros::spinOnce();
    rate.sleep();
  }

  is_ready_ = true;
}
