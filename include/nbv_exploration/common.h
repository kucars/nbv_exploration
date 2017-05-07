#ifndef NBV_EXPLORATION_COMMON_H
#define NBV_EXPLORATION_COMMON_H

#include <ros/ros.h>
#include <pcl/point_types.h>

#include "utilities/console_utility.h"
#include "utilities/pose_conversion.h"

static ConsoleUtility cc;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<PointN> PointCloudN;

#endif // NBV_EXPLORATION_COMMON_H
