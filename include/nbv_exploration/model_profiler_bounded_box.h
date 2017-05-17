#ifndef MODEL_PROFILER_BOUNDED_BOX_H
#define MODEL_PROFILER_BOUNDED_BOX_H

#include "nbv_exploration/common.h"
#include "nbv_exploration/model_profiler_base.h"

class ModelProfilerBoundedBox : public ModelProfilerBase
{
public:
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

  void createWaypoints();
  UAVWaypoint createSingleWaypoint(double x, double y, double z, double x_center, double y_center);
};

#endif // MODELPROFILERBOUNDEDBOX_H
