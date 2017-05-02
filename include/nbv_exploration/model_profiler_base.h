#ifndef MODELPROFILERBASE_H
#define MODELPROFILERBASE_H

#include <pcl_conversions/pcl_conversions.h>
#include "control/vehicle_control_base.h"
#include "nbv_exploration/mapping_module.h"

class ModelProfilerBase
{
protected:
  double scan_speed_;
  VehicleControlBase * vehicle_;
  MappingModule * mapping_module_;

public:
  ModelProfilerBase();

  virtual bool run(PointCloudXYZ::Ptr profile_cloud_ptr){};
  virtual void scan(){};

  void setScanSpeed(double speed);
  void setMappingModule(MappingModule* v);
  void setVehicle(VehicleControlBase* v);
  bool skipProfiling(bool load_map);
  bool startProfiling();
};

#endif // MODELPROFILERBASE_H
