#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "control/vehicle_control_floating_sensor.h"
#include "nbv_exploration/common.h"


VehicleControlFloatingSensor::VehicleControlFloatingSensor():
  VehicleControlBase() // Call super class constructor
{
  ros::NodeHandle ros_node;

  // Callbacks
  sub_pose  = ros_node.subscribe("/floating_sensor/pose", 1, &VehicleControlFloatingSensor::callbackPose, this);
  pub_twist = ros_node.advertise<geometry_msgs::Twist>("/floating_sensor/set_twist", 10);
  pub_pose  = ros_node.advertise<geometry_msgs::Pose>("/floating_sensor/set_pose", 10);

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

bool VehicleControlFloatingSensor::isSationary(double threshold_sensitivity)
{
  // Doesn't check if we're stationary since the sensor teleports
  // Instead it checks if the sensor is in the desired location

  return isNear(vehicle_current_pose_, setpoint_, threshold_sensitivity );
}

void VehicleControlFloatingSensor::moveVehicle(double threshold_sensitivity)
{
  if (speed_ < 0)
  {
    // Teleport sensor instantly
    pub_pose.publish(setpoint_);

    // Wait for sensor to reach destination
    distance_threshold_ = 0.1;
    while (ros::ok() && !isNear(setpoint_, vehicle_current_pose_, 1) )
    {
      ros::spinOnce();
    }

    return;
  }

  updateTwist();

  // Set up variables for motion
  geometry_msgs::Pose p = vehicle_current_pose_;
  double dt = 1.0/30;
  ros::Rate rate(1/dt);
  double calibration_constant = 11; //scale speed to match reality

  double time_elapsed = 0;

  // Continue moving util we've reached the setpoint
  while(ros::ok() && time_elapsed < time_to_target_)
  {
    p.position.x += twist_.linear.x * dt * calibration_constant;
    p.position.y += twist_.linear.y * dt * calibration_constant;
    p.position.z += twist_.linear.z * dt * calibration_constant;
    time_elapsed += dt;

    // Publish pose
    pub_pose.publish(p);

    ros::spinOnce();
    rate.sleep();
  }

  // Done, publish setpoint to make sure we're in target location
  pub_pose.publish(setpoint_);
}


void VehicleControlFloatingSensor::setSpeed(double speed)
{
  speed_ = speed;
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
  setpoint_ = p;
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
    std::cout << cc.yellow << "Vehicle (floating sensor) not ready!\n" << cc.reset;

    ros::spinOnce();
    rate.sleep();
  }

  is_ready_ = true;
}


void VehicleControlFloatingSensor::updateTwist()
{
  // Find direction vector (end point - start point)
  twist_.linear.x = setpoint_.position.x - vehicle_current_pose_.position.x;
  twist_.linear.y = setpoint_.position.y - vehicle_current_pose_.position.y;
  twist_.linear.z = setpoint_.position.z - vehicle_current_pose_.position.z;

  // Normalize and set speed accordingly
  double norm = sqrt(twist_.linear.x*twist_.linear.x + twist_.linear.y*twist_.linear.y + twist_.linear.z*twist_.linear.z);
  time_to_target_ = norm / speed_;

  twist_.linear.x = twist_.linear.x / time_to_target_;
  twist_.linear.y = twist_.linear.y / time_to_target_;
  twist_.linear.z = twist_.linear.z / time_to_target_;

  printf("Update Twist z: %lf, norm: %lf\n", twist_.linear.z, norm);
}
