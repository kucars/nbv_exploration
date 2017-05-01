#ifndef NBV_EXPLORATION_SYMMETRY_DETECTOR_H
#define NBV_EXPLORATION_SYMMETRY_DETECTOR_H

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

#include "nbv_exploration/common.h"


// =========
// Structures
// =========
struct PlaneTransform{
  double a, b, c, d;
  double distance;
};

struct KeypointFeature{
  PointN point;
  pcl::FPFHSignature33 feature;
};

class SymmetryDetector
{
public:
  // =========
  // Methods
  // =========
  SymmetryDetector();
  void getOutputCloud(PointCloudXYZ::Ptr& cloud_out);
  void run();
  void setInputCloud(PointCloudXYZ::Ptr cloud_in);
  int  setInputCloudFromFile(std::string filename);
  void visualize();

private:
  // =========
  // Methods
  // =========
  std::vector<std::vector<double> > convertPlanesToVectors(std::vector<PlaneTransform> planes);
  std::vector<PlaneTransform>       convertVectorsToPlanes(std::vector<std::vector<double> > planes);
  float getFeatureDistance(pcl::FPFHSignature33 p1, pcl::FPFHSignature33 p2);
  std::vector<KeypointFeature> getKeypointFeatures(PointCloudN::Ptr cloud_normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeatures);

  // =========
  // Variables
  // =========
  PointCloudXYZ::Ptr cloud_in_;
  PointCloudN::Ptr cloud_mirrored_;
  PointCloudN::Ptr cloud_mirrored_corrected_;
  PointCloudN::Ptr cloud_pairs_;
  PointCloudXYZ::Ptr cloud_plane_;


  // Symmetry Detection Parameters
  bool use_exhaustive_pairing = false;
  bool use_prefilter = false;
  bool use_sift_points = false;
  bool icp_with_normals = false;


  double mean_shift_kernel_bandwidth;
  int mean_shift_num_points;

  double pairing_subset_percent;
  double pairing_threshold;
  double prefilter_leaf_size;
  double sift_min_contrast;
  double sift_min_scale;
  int sift_num_octaves;
  int sift_num_scales_per_octave;
  int tree_K; //Number of neighbors in KD trees
  double subtraction_search_radius;
};

#endif
