#include <ros/ros.h>

#include "nbv_exploration/model_profiler_base.h"
#include "nbv_exploration/common.h"

ModelProfilerBase::ModelProfilerBase():
  mapping_module_(NULL),
  vehicle_(NULL),
  scan_speed_(0.1)
{
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


bool ModelProfilerBase::skipProfiling(bool load_map)
{
  if (load_map)
  {
    std::cout << "Skipping profiling (load map)\n";

    bool success = mapping_module_->commandProfileLoad();
    if (!success)
    {
      std::cout << cc.red << "Failed to load profile\n" << cc.reset;
      return false;
    }
  }

  else
  {
    std::cout << "Skipping profiling (empty map)\n";

    /* Create an empty octomap */
    mapping_module_->commandProfilingStart();
    bool success = mapping_module_->commandProfilingStop();
    if (!success)
    {
      std::cout << cc.red << "Failed to start profiler\n" << cc.reset;
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
