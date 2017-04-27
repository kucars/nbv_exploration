#ifndef MODELPROFILERBASE_H
#define MODELPROFILERBASE_H

#include <pcl_conversions/pcl_conversions.h>
#include "control/vehicle_control_base.h"
#include "nbv_exploration/sensing_and_mapping.h"

#include "nbv_exploration/MappingSrv.h"

class ModelProfilerBase
{
protected:
  VehicleControlBase * vehicle_;
  MappingModule * mapping_module_;
  ros::ServiceClient srvclient_mapping;

public:
  ModelProfilerBase();

  virtual bool run(pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cloud_ptr){};
  virtual void scan(){};

  bool callMappingService(int command);
  void setMappingModule(MappingModule* v);
  void setVehicle(VehicleControlBase* v);
  bool skipProfiling(bool load_map);
  bool startProfiling();
};

#endif // MODELPROFILERBASE_H
