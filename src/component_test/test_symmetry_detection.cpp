#include <iostream>

#include <ros/ros.h>

#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../lib/MeanShift/MeanShift.h"
#include "nbv_exploration/symmetry_detector.h"

int main (int argc, char** argv)
{
  // ===================
  // Read config parameters
  // ===================
  ros::init(argc, argv, "test_symmetry_detection");

  std::string filename_pcd;
  ros::param::param<std::string>("~filename_pcd", filename_pcd, "-1");

  if (filename_pcd == "-1")
  {
    printf("Could not read rosparam 'filename_pcd' from config files.\n");
    return -1;
  }

  // ====================
  // Run symmetry detection
  // ====================
  SymmetryDetector* sym_det = new SymmetryDetector();
  if (sym_det->setInputCloudFromFile(filename_pcd) == -1)
    return (-1);
  sym_det->run();
  sym_det->visualize();


  return (0);
}
