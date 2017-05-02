#include <ros/ros.h>

#include "nbv_exploration/model_profiler_circular_adaptive.h"
#include "nbv_exploration/common.h"

ModelProfilerCircularAdaptive::ModelProfilerCircularAdaptive():
  ModelProfilerBase(), //call base class constructor
  profile_angle_(0),
  is_sensor_rising_(false)
{
  ros::param::param("~uav_obstacle_distance_min_", uav_obstacle_distance_min_, 1.0);
  ros::param::param("~nav_bounds_z_min", uav_height_min_, 0.1);
  ros::param::param("~nav_bounds_z_max", uav_height_max_, 10.0);
  ros::param::param("~profiling_circular_waypoints", waypoint_count_, 5);

  angle_inc_ = 2*M_PI/waypoint_count_;
}


bool ModelProfilerCircularAdaptive::run(PointCloudXYZ::Ptr profile_cloud_ptr)
{
  profile_angle_ += angle_inc_;

  if (profile_angle_ > 2*M_PI + angle_inc_)
  {
    std::cout << cc.green << "Profiling complete\n" << cc.reset;

    mapping_module_->commandProfilingStop();
    if ( mapping_module_->commandProfileSave() )
      std::cout << "Successfully saved profile\n";

    else
      std::cout << cc.red << "Failed to save profile\n";

    return true; // Done profiling
  }


  is_sensor_rising_ = !is_sensor_rising_;
  scan();

  // Calculate centroid
  if (profile_cloud_ptr->points.size() == 0){
    std::cout << cc.red << "Error: No points detected in profile cloud. Make sure mapping node is running\n" << cc.reset;
    return false;
  }

  double x, y;

  for (int i=0; i<profile_cloud_ptr->points.size(); i++)
  {
    x += profile_cloud_ptr->points[i].x;
    y += profile_cloud_ptr->points[i].y;
  }
  x /= profile_cloud_ptr->points.size();
  y /= profile_cloud_ptr->points.size();

  // Calculate radius (top n point to remove outliers)
  double r = 0;


  for (int i=0; i<profile_cloud_ptr->points.size(); i++)
  {
    double r_temp = (profile_cloud_ptr->points[i].x - x)*(profile_cloud_ptr->points[i].x - x)
                  + (profile_cloud_ptr->points[i].y - y)*(profile_cloud_ptr->points[i].y - y);

    if (r_temp > r)
      r = r_temp;
  }

  r = sqrt(r);
  r += uav_obstacle_distance_min_; // add a safety margin (to avoid collision and see free spaces close to structure

  std::cout << cc.magenta << "x = " << x << "\ty = " << y << "\tr = " << r << "\n" << cc.reset;

  // Move back on the circle
  geometry_msgs::Point current_position = vehicle_->getPosition();

  double theta  = atan2(current_position.y - y, current_position.x - x);
  double x_move = x+r*cos(theta);
  double y_move = y+r*sin(theta);
  double yaw_move = theta-M_PI; //towards the center of the circle

  std::cout << cc.magenta << "Moving back\n" << cc.reset;
  vehicle_->setSpeed(0.2);
  vehicle_->setSpeed(-1); //Allows floating sensor to move instantly, ignored by real vehicles
  vehicle_->setWaypoint(x_move, y_move, current_position.z, yaw_move);
  vehicle_->moveVehicle();

  // Move along the circle
  double arc_length = r * angle_inc_;
  double step_length = 7.0;
  int steps = arc_length/step_length;

  double z_move = current_position.z;

  if (steps < 2)
  {
    theta  += angle_inc_;
    x_move = x+r*cos(theta);
    y_move = y+r*sin(theta);
    yaw_move = theta-M_PI; //towards the center of the circle

    std::cout << cc.magenta << "Moving along\n" << cc.reset;
    vehicle_->setWaypoint(x_move, y_move, z_move, yaw_move);
    vehicle_->moveVehicle(1.5);
  }
  else
  {
    double step_angle = angle_inc_ / steps;
    for (int i=0; i<steps; i++)
    {
      theta  += step_angle;
      x_move = x+r*cos(theta);
      y_move = y+r*sin(theta);
      yaw_move = theta-M_PI; //towards the center of the circle

      std::cout << cc.magenta << "Moving along\n" << cc.reset;
      vehicle_->setWaypoint(x_move, y_move, z_move, yaw_move);
      vehicle_->moveVehicle(1.5); // Be less picky about position, just move around quickly
    }
  }

  return false; //Not done profiling
}





void ModelProfilerCircularAdaptive::scan()
{
  //Start scanning
  mapping_module_->commandScanningStart();

  // Scan
  if (is_sensor_rising_)
  {
    std::cout << cc.magenta << "Profiling move up\n" << cc.reset;

    while (ros::ok() && vehicle_->getPosition().z < uav_height_max_)
    {
      vehicle_->setSpeed(scan_speed_);
      vehicle_->setWaypointIncrement(0, 0, uav_height_max_ - vehicle_->getPosition().z, 0);
      vehicle_->moveVehicle(0.25);
      ros::spinOnce();
    }
  }
  else
  {
    std::cout << cc.magenta << "Profiling move down\n" << cc.reset;

    while (ros::ok() && vehicle_->getPosition().z > uav_height_min_)
    {
      vehicle_->setSpeed(scan_speed_);
      vehicle_->setWaypointIncrement(0, 0, uav_height_min_ - vehicle_->getPosition().z, 0);
      vehicle_->moveVehicle(0.25);
      ros::spinOnce();
    }
  }

  // Stop scanning
  mapping_module_->commandScanningStop();
}
