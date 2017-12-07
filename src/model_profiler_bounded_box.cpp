#include <ros/ros.h>

#include "nbv_exploration/model_profiler_bounded_box.h"
#include "nbv_exploration/common.h"

ModelProfilerBoundedBox::ModelProfilerBoundedBox():
  ModelProfilerBase(), //call base class constructor
  is_sensor_rising_(false),
  waypoints_exist_(false)
{
}

ModelProfilerBoundedBox::UAVWaypoint ModelProfilerBoundedBox::createSingleWaypoint(double x, double y, double z, double x_center, double y_center)
{
  UAVWaypoint w;
  w.x = x;
  w.y = y;
  w.z = z;
  w.yaw = atan2(w.y - y_center, w.x - x_center) - M_PI; //Point toward center
  return w;
}

void ModelProfilerBoundedBox::createWaypoints()
{
  std::vector<UAVWaypoint> waypoints_temp;

  // Compute center
  double x_center = (bounds.x_min + bounds.x_max)/2;
  double y_center = (bounds.y_min + bounds.y_max)/2;

  // Create 8 waypoints
  UAVWaypoint w;

  w = createSingleWaypoint(bounds.x_min, bounds.y_min, bounds.z_max, x_center, y_center);
  waypoints_temp.push_back(w);

  w = createSingleWaypoint(x_center, bounds.y_min, bounds.z_max, x_center, y_center);
  waypoints_temp.push_back(w);

  w = createSingleWaypoint(bounds.x_max, bounds.y_min, bounds.z_max, x_center, y_center);
  waypoints_temp.push_back(w);

  w = createSingleWaypoint(bounds.x_max, y_center, bounds.z_max, x_center, y_center);
  waypoints_temp.push_back(w);

  w = createSingleWaypoint(bounds.x_max, bounds.y_max, bounds.z_max, x_center, y_center);
  waypoints_temp.push_back(w);

  w = createSingleWaypoint(x_center, bounds.y_max, bounds.z_max, x_center, y_center);
  waypoints_temp.push_back(w);

  w = createSingleWaypoint(bounds.x_min, bounds.y_max, bounds.z_max, x_center, y_center);
  waypoints_temp.push_back(w);

  w = createSingleWaypoint(bounds.x_min, y_center, bounds.z_max, x_center, y_center);
  waypoints_temp.push_back(w);

  // Find closest corner
  geometry_msgs::Point current_position = vehicle_->getPosition();

  int min_idx = -1;
  double min_dist = std::numeric_limits<double>::max();
  for (int i_way=0; i_way<waypoints_temp.size(); i_way++)
  {
    double dx = current_position.x - waypoints_temp[i_way].x;
    double dy = current_position.y - waypoints_temp[i_way].y;
    double dist = dx*dx + dy*dy;

    if (dist < min_dist)
    {
      min_dist = dist;
      min_idx  = i_way;
    }
  }


  // Rearrange waypoints so closest waypoint is first
  for (int i_way=min_idx; i_way<waypoints_temp.size(); i_way++)
    waypoints.push_back ( waypoints_temp[i_way] );

  for (int i_way=0; i_way<min_idx; i_way++)
    waypoints.push_back ( waypoints_temp[i_way] );

  waypoints_exist_ = true;
  waypoint_counter_= 0;
  is_sensor_rising_ = false; //Scan downwards
}


bool ModelProfilerBoundedBox::run(PointCloudXYZ::Ptr profile_cloud_ptr)
{
  if (!waypoints_exist_)
    createWaypoints();

  printf("Waypoint count: %d/%lu\n", waypoint_counter_, waypoints.size());

  // Display current position
  geometry_msgs::Point current_position = vehicle_->getPosition();
  printf("Current position: [%lf, %lf, %lf]\n", current_position.x, current_position.y, current_position.z);

  // Move to waypoint
  UAVWaypoint w = waypoints[waypoint_counter_];
  printf("Target position: [%lf, %lf, %lf]\n", w.x, w.y, w.z);

  vehicle_->setWaypoint(w.x, w.y, w.z, w.yaw);
  vehicle_->setSpeed(1);
  vehicle_->setSpeed(-1); //Allow floating sensor to teleport, ignored by other vehicles
  vehicle_->moveVehicle(0.25);

  current_position = vehicle_->getPosition();
  printf("Done moving: [%lf, %lf, %lf]\n", current_position.x, current_position.y, current_position.z);
  printf("Stating scan\n");

  // Perform scan
  scan();
  waypoint_counter_++;


  // Check if all waypoints are complete
  if (waypoint_counter_ >= waypoints.size())
  {
    std::cout << cc.green << "Profiling complete\n" << cc.reset;

    mapping_module_->commandProfilingStop();

    if ( mapping_module_->commandProfileSave() )
      std::cout << "Successfully saved profile\n";

    else
      std::cout << cc.red << "Failed to save profile\n";

    return true; // Done profiling
  }

  return false; //Not done profiling
}

void ModelProfilerBoundedBox::scan()
{
  // Make sure sensor is stationary/at target location
  // Helps with floating sensor when it's teleporting
  ros::Rate loopRate(30);
  while (ros::ok() && !vehicle_->isStationary(1) )
  {
    ROS_INFO_THROTTLE(1,"Stabalizing Vehicle");
    ros::spinOnce();
    loopRate.sleep();
  }

  // Stop scanning
  mapping_module_->commandScanningStart();

  // Scan
  if (is_sensor_rising_)
  {
    std::cout << cc.magenta << "Profiling move up\n" << cc.reset;

    while (ros::ok() && vehicle_->getPosition().z < bounds.z_max)
    {
      vehicle_->setSpeed(scan_speed_);
      vehicle_->setWaypointIncrement(0, 0, bounds.z_max - vehicle_->getPosition().z, 0);
      vehicle_->moveVehicle(0.25);
      ros::spinOnce();
      loopRate.sleep();
    }
  }
  else
  {
    std::cout << cc.magenta << "Profiling move down\n" << cc.reset;

    while (ros::ok() && vehicle_->getPosition().z > bounds.z_min)
    {
      vehicle_->setSpeed(scan_speed_);
      vehicle_->setWaypointIncrement(0, 0, bounds.z_min - vehicle_->getPosition().z, 0);
      vehicle_->moveVehicle(0.25);
      ros::spinOnce();
      loopRate.sleep();
    }
  }

  // Stop scanning
  mapping_module_->commandScanningStop();
}
