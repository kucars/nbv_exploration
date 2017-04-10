#ifndef MODELPROFILERCIRCULARADAPTIVE_H
#define MODELPROFILERCIRCULARADAPTIVE_H

#include "nbv_exploration/common.h"
#include "nbv_exploration/model_profiler_base.h"

class ModelProfilerCircularAdaptive : public ModelProfilerBase
{
protected:
  double profile_angle_;
  int waypoint_count_;
  double angle_inc_;

  double uav_obstacle_distance_min_;
  double uav_height_max_;
  double uav_height_min_;

  bool is_sensor_rising_;

public:
  ModelProfilerCircularAdaptive();
  bool run(pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cloud_ptr);
  void scan();
};

#endif // MODELPROFILERCIRCULARADAPTIVE_H
