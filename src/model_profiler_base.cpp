#include <ros/ros.h>

#include "nbv_exploration/model_profiler_base.h"
#include "nbv_exploration/common.h"

ModelProfilerBase::ModelProfilerBase():
  mapping_module_(NULL),
  vehicle_(NULL),
  scan_speed_(0.1)
{
  ros::param::param("~nav_bounds_x_min", bounds.x_min, 0.1);
  ros::param::param("~nav_bounds_x_max", bounds.x_max, 10.0);
  ros::param::param("~nav_bounds_y_min", bounds.y_min, 0.1);
  ros::param::param("~nav_bounds_y_max", bounds.y_max, 10.0);
  ros::param::param("~nav_bounds_z_min", bounds.z_min, 0.1);
  ros::param::param("~nav_bounds_z_max", bounds.z_max, 10.0);
}

void ModelProfilerBase::setMappingModule(MappingModule* m)
{
  mapping_module_ = m;
}

void ModelProfilerBase::setVehicle(VehicleControlBase* v)
{
  vehicle_ = v;
}

void ModelProfilerBase::setScanSpeed(double speed)
{
  scan_speed_ = speed;
}

bool ModelProfilerBase::skipProfiling(bool skip_map_loading)
{
  if (skip_map_loading)
  {
    std::cout << "Skipping profiling (empty map)\n";

    /* Take a single reading */
    mapping_module_->commandProfilingStart();
    mapping_module_->commandScanningStart();
    ros::Duration(1).sleep();
    mapping_module_->commandScanningStop();
    mapping_module_->commandProfilingStop();
    mapping_module_->commandGetCameraData();
  }
  else
  {
    std::cout << "Skipping profiling (load map)\n";

    bool success = mapping_module_->commandProfileLoad();
    if (!success)
    {
      std::cout << cc.red << "Failed to load profile\n" << cc.reset;
      return false;
    }
  }

  return true;
}

bool ModelProfilerBase::startProfiling()
{
  bool success = mapping_module_->commandProfilingStart();
  if (!success)
    std::cout << cc.red << "Failed to start profiler\n" << cc.reset;

  return success;
}
