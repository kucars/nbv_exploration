#ifndef NBV_EXPLORATION_COMMON_H
#define NBV_EXPLORATION_COMMON_H

#include <ros/ros.h>
#include <pcl/point_types.h>

// include input and output archivers for serialization
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include "utilities/console_utility.h"
#include "utilities/pose_conversion.h"
#include "utilities/time_profiler.h"


extern TimeProfiler timer;
static ConsoleUtility cc;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<PointN> PointCloudN;

#endif // NBV_EXPLORATION_COMMON_H
