#ifndef MODELPROFILERBOUNDEDBOX_H
#define MODELPROFILERBOUNDEDBOX_H

#include "nbv_exploration/common.h"
#include "nbv_exploration/model_profiler_base.h"

class ModelProfilerBoundedBox : public ModelProfilerBase
{
public:
  struct Bounds{
    double x_min, x_max, y_min, y_max, z_min, z_max;
  };

  struct UAVWaypoint{
    double x, y, z, yaw;
  };

  ModelProfilerBoundedBox();
  bool run(PointCloudXYZ::Ptr profile_cloud_ptr);
  void scan();

protected:
  std::vector<UAVWaypoint> waypoints;
  bool waypoints_exist_;
  int waypoint_counter_;

  bool is_sensor_rising_;
  Bounds bounds;

  void createWaypoints();
  UAVWaypoint createSingleWaypoint(double x, double y, double z, double x_center, double y_center);
};

#endif // MODELPROFILERBOUNDEDBOX_H
