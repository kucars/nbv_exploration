#ifndef NBVEXPLORATIONCOMMON_H
#define NBVEXPLORATIONCOMMON_H

#include <ros/ros.h>

/*
#include "nbv_exploration/view_generator.h"
#include "nbv_exploration/termination_check_base.h"
#include "nbv_exploration/view_selecter.h"
#include "nbv_exploration/model_profiler_base.h"
#include "nbv_exploration/model_profiler_circular_adaptive.h"
#include "nbv_exploration/nbv_loop.h"

#include "control/vehicle_control_base.h"
#include "control/vehicle_control_iris.h"
*/

#include <pcl/point_types.h>

#include "utilities/console_utility.h"
#include "utilities/pose_conversion.h"

static ConsoleUtility cc;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<PointN> PointCloudN;

#endif // NBVEXPLORATIONCOMMON_H
