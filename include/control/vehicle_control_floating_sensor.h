#ifndef VEHICLECONTROLIRIS_H
#define VEHICLECONTROLIRIS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "control/vehicle_control_base.h"
#include "nbv_exploration/common.h"

class VehicleControlFloatingSensor : public VehicleControlBase
{
private:
  ros::Subscriber sub_pose;
  ros::Publisher  pub_pose;

public:
  VehicleControlFloatingSensor();

  void callbackOdometry(const nav_msgs::Odometry& odom_msg);

  bool isReady();

  void moveVehicle(double threshold_sensitivity = 1);
  void setSpeed(double speed);
  void setWaypoint(double x, double y, double z, double yaw);
  void setWaypoint(geometry_msgs::Pose p);
  void setWaypointIncrement(double x, double y, double z, double yaw);
  void start();
};

#endif // VEHICLECONTROLIRIS_H
