#ifndef VEHICLE_CONTROL_FLOATING_SENSOR_H
#define VEHICLE_CONTROL_FLOATING_SENSOR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "control/vehicle_control_base.h"
#include "nbv_exploration/common.h"

class VehicleControlFloatingSensor : public VehicleControlBase
{
private:
  ros::Subscriber sub_pose;
  ros::Publisher  pub_pose;
  ros::Publisher  pub_twist;

  double speed_;
  double time_to_target_;
  geometry_msgs::Twist twist_;

public:
  VehicleControlFloatingSensor();

  void callbackPose(const geometry_msgs::Pose& pose_msg);

  bool isReady();
  bool isSationary(double threshold_sensitivity = 1);

  void moveVehicle(double threshold_sensitivity = 1);
  void setSpeed(double speed);
  void setWaypoint(double x, double y, double z, double yaw);
  void setWaypoint(geometry_msgs::Pose p);
  void setWaypointIncrement(double x, double y, double z, double yaw);
  void start();
  void updateTwist();
};

#endif // VEHICLECONTROLIRIS_H
