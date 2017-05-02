#include <iostream>

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

//convenient typedefs
typedef pcl::PointXYZ PointXYZ;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<PointN> PointCloudN;

int matchesForCloud1InCloud2(PointCloudXYZ::Ptr& cloud_match, PointCloudXYZ::Ptr& cloud_ref, float dist_thresh)
{
  float dist_sqr_threshold = dist_thresh*dist_thresh;
  int points_corresponding = 0;

  pcl::KdTreeFLANN<PointXYZ> kdtree;
  kdtree.setInputCloud (cloud_ref);
  int K = 1;

  for (int i=0; i<cloud_match->points.size(); i++)
  {
    PointXYZ searchPoint = cloud_match->points[i];

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      // Find closest point in original cloud
      float dist = pointNKNSquaredDistance[0];
      if (dist > dist_sqr_threshold)
        continue;

      points_corresponding++;
    }
  }

  return points_corresponding;
}

void readCloudFromFile(std::string filename, PointCloudXYZ::Ptr& cloud_ptr, double scale=1)
{
  if (pcl::io::loadPCDFile<PointXYZ> (filename, *cloud_ptr) == -1)
  {
    std::cout << "ERROR: Could not read PCD file: " << filename << "\n";
    return;
  }

  // ===================
  // Copy input file point by point, otherwise the z-dimension is squashed in visualization
  // ===================
  PointCloudXYZ::Ptr cloud_scale (new PointCloudXYZ);
  for (int i=0; i<cloud_ptr->points.size(); i++)
  {
    PointXYZ p = cloud_ptr->points[i];

    if (scale != 1)
    {
      p.x *= scale;
      p.y *= scale;
      p.z *= scale;
    }

    cloud_scale->points.push_back(p);
  }

  cloud_ptr = cloud_scale;
}

int main (int argc, char** argv)
{
  std::string filename_reference = "/home/abdullah/catkin_ws/src/nbv_exploration/models/pcd/etihad/etihad.pcd";
  std::string filename_final = "/home/abdullah/.ros/final_cloud.pcd";

  PointCloudXYZ::Ptr cloud_ref (new PointCloudXYZ);
  PointCloudXYZ::Ptr cloud_final (new PointCloudXYZ);

  // Read input file
  readCloudFromFile(filename_reference, cloud_ref, 0.5);
  readCloudFromFile(filename_final, cloud_final);

  // ================
  // Compute accuracy
  // ================
  int true_positives = matchesForCloud1InCloud2(cloud_final, cloud_ref, 0.05);
  int false_positives = cloud_final->points.size() - true_positives;

  printf("Number of TP: %d/%lu\n", true_positives, cloud_final->points.size());
  printf("Number of FP: %d/%lu\n", false_positives, cloud_final->points.size());


  // ================
  // Compute coverage
  // ================
  int matched = matchesForCloud1InCloud2(cloud_ref, cloud_final, 0.05);
  float coverage = float(matched)/cloud_ref->points.size();

  printf("Coverage: %f%%\n", coverage*100);

  // =========
  // Visualize
  // =========
  int vp_1;
  pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer ("Coverage");
  p->createViewPort (0.0, 0.0, 1.0, 1.0, vp_1);
  p->setBackgroundColor(255, 255, 255);
  p->setCameraClipDistances(65.3496, 129.337);
  p->setCameraPosition(35.1704, -68.6111, 63.8152,       0.0329489, 0.689653, 0.72339, 1);//, 0.0427789, -0.185814, 0.0496169, -0.0956887, -0.992963, 0.0697719);
  p->setCameraFieldOfView(0.523599);
  p->setSize(1366, 700);
  p->setPosition(0, 300);

  pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_ref_handler (cloud_ref, 0, 255, 0);
  p->addPointCloud (cloud_ref, cloud_ref_handler, "ref_cloud", vp_1);

  pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_final_handler (cloud_final, 255, 0, 0);
  p->addPointCloud (cloud_final, cloud_final_handler, "final_cloud", vp_1);

  p->spin();

  return 0;
}
