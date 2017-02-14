#ifndef VEHICLECONTROLBASE_H
#define VEHICLECONTROLBASE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include "nbv_exploration/common.h"

class VehicleControlBase
{
protected:
  bool is_ready_;

public:
  double distance_threshold_;
  double angular_threshold_;
  double linear_speed_threshold_;
  double angular_speed_threshold_;

  geometry_msgs::Pose  vehicle_current_pose_;
  geometry_msgs::Twist vehicle_current_twist_;
  geometry_msgs::PoseStamped setpoint_;

  VehicleControlBase();
  virtual void initialize(){};
  virtual void start(){};
  virtual bool isReady(){};
  virtual void setSpeed(double speed){};
  virtual void setWaypoint(double x, double y, double z, double yaw){};
  virtual void setWaypoint(geometry_msgs::Pose p){};
  virtual void setWaypointIncrement(double x, double y, double z, double yaw){};
  virtual void moveVehicle(double threshold_sensitivity = 1){};

  geometry_msgs::Pose  getPose();
  geometry_msgs::Twist getTwist();
  geometry_msgs::Point getPosition();
  geometry_msgs::Quaternion getOrientation();
  double getYaw();

  double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
  double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
};

#endif // VEHICLECONTROLBASE_H
