#include <ros/ros.h>

#include "nbv_exploration/model_profiler_base.h"
#include "nbv_exploration/common.h"

ModelProfilerBase::ModelProfilerBase()
{
  ros::NodeHandle ros_node;
  srvclient_mapping = ros_node.serviceClient<nbv_exploration::MappingSrv>("nbv_exploration/mapping_command");
}

bool ModelProfilerBase::callMappingService(int command)
{
  nbv_exploration::MappingSrv srv;
  srv.request.data = command;

  bool success = srvclient_mapping.call(srv);
  bool error = false;

  if (!success)
  {
    ROS_ERROR("Failed to call service \"mapping_command\"");
    return false; //failure
  }
  else
  {
    if (srv.response.data != nbv_exploration::MappingSrv::Response::ERROR)
      return true; //success
  }

  // Error encountered
  std::string cmd_name;
  switch (command)
  {
    case nbv_exploration::MappingSrv::Request::START_SCANNING:
      cmd_name = "START_SCANNING";
      break;
    case nbv_exploration::MappingSrv::Request::STOP_SCANNING:
      cmd_name = "STOP_SCANNING";
      break;
    case nbv_exploration::MappingSrv::Request::START_PROFILING:
      cmd_name = "START_PROFILING";
      break;
    case nbv_exploration::MappingSrv::Request::STOP_PROFILING:
      cmd_name = "STOP_PROFILING";
      break;
    case nbv_exploration::MappingSrv::Request::SAVE_MAP:
      cmd_name = "SAVE_MAP";
      break;
    case nbv_exploration::MappingSrv::Request::LOAD_MAP:
      cmd_name = "LOAD_MAP";
      break;
    case nbv_exploration::MappingSrv::Request::GET_CAMERA_DATA:
      cmd_name = "GET_CAMERA_DATA";
      break;
    default:
      cmd_name = "UNKNOWN";
      break;
  }

  std::cout << cc.red << "Error after sending " <<  cmd_name << "(ID: " << command << ") to service \"mapping_commands\" \n";
  std::cout << cc.red << "\t" << srv.response.error << "\n" << cc.reset;
  return false;
}


void ModelProfilerBase::setVehicle(VehicleControlBase* v)
{
  vehicle_ = v;
}


bool ModelProfilerBase::skipProfiling(bool load_map)
{
  if (load_map)
  {
    std::cout << "Skipping profiling (load map)\n";

    bool success = callMappingService(nbv_exploration::MappingSrv::Request::LOAD_MAP);
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
    bool success = callMappingService(nbv_exploration::MappingSrv::Request::START_PROFILING);
    success = callMappingService(nbv_exploration::MappingSrv::Request::STOP_PROFILING);
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
  bool success = callMappingService(nbv_exploration::MappingSrv::Request::START_PROFILING);
  if (!success)
    std::cout << cc.red << "Failed to start profiler\n" << cc.reset;

  return success;
}
