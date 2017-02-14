#ifndef MODELPROFILERBASE_H
#define MODELPROFILERBASE_H

#include <pcl_conversions/pcl_conversions.h>

#include "nbv_exploration/MappingSrv.h"

class ModelProfilerBase
{
public:
  ros::ServiceClient srvclient_mapping;

  ModelProfilerBase();

  virtual bool run(pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cloud_ptr){};
  virtual void scan(){};

  bool callMappingService(int command);
  bool skipProfiling(bool load_map);
  bool startProfiling();
};

#endif // MODELPROFILERBASE_H
