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

#include "component_test/voxel_grid_occlusion_estimation.h"

//convenient typedefs
typedef pcl::PointXYZ PointXYZ;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<PointN> PointCloudN;

int matchesForGrid1InGrid2(pcl::VoxelGridOcclusionEstimationT g1, pcl::VoxelGridOcclusionEstimationT g2)
{
  Eigen::Vector3i min_b = g1.getMinBoxCoordinates ();
  Eigen::Vector3i max_b = g1.getMaxBoxCoordinates ();

  int MatchedVoxels=0 ;

      //iterate through the entire coverage grid to check the number of matched voxel between the original and the covered ones
  for (int kk = min_b.z (); kk <= max_b.z (); ++kk)
  {
    for (int jj = min_b.y (); jj <= max_b.y (); ++jj)
    {
      for (int ii = min_b.x (); ii <= max_b.x (); ++ii)
      {
        Eigen::Vector3i ijk (ii, jj, kk);
        int index1 = g1.getCentroidIndexAt (ijk);
        if(index1!=-1)
        {
          Eigen::Vector4f centroid = g1.getCentroidCoordinate (ijk);
          Eigen::Vector3i ijk_in_Original= g2.getGridCoordinates(centroid[0],centroid[1],centroid[2]) ;

          int index = g2.getCentroidIndexAt (ijk_in_Original);

          if(index!=-1)
            MatchedVoxels++;
        }
      }
    }
  }

  return MatchedVoxels;
}

 int createVoxelGrid(pcl::VoxelGridOcclusionEstimationT& g1, PointCloudXYZ::Ptr& cloud_ptr, double voxelRes)
{
  int OriginalVoxelsSize=0;

  g1.setInputCloud (cloud_ptr);
  g1.setLeafSize (voxelRes, voxelRes, voxelRes);
  g1.initializeVoxelGrid();
  Eigen::Vector3i min_b1 = g1.getMinBoxCoordinates ();
  Eigen::Vector3i max_b1 = g1.getMaxBoxCoordinates ();
  for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
  {
     for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
     {
         for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
         {
           Eigen::Vector3i ijk1 (ii, jj, kk);
           int index1 = g1.getCentroidIndexAt (ijk1);
           if(index1!=-1)
           {
               OriginalVoxelsSize++;
           }
         }
     }
  }

  return OriginalVoxelsSize;
}

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

void readCloudFromFile(std::string filename, PointCloudXYZ::Ptr& cloud_ptr)
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
    cloud_scale->points.push_back(p);
  }

  cloud_ptr = cloud_scale;
}

void transformCloud(PointCloudXYZ::Ptr& cloud_ptr, double scale=1, double x_shift=0, double y_shift=0, double z_shift=0)
{
  PointCloudXYZ::Ptr cloud_scale (new PointCloudXYZ);
  for (int i=0; i<cloud_ptr->points.size(); i++)
  {
    PointXYZ p = cloud_ptr->points[i];

    p.x = p.x*scale + x_shift;
    p.y = p.y*scale + y_shift;
    p.z = p.z*scale + z_shift;

    cloud_scale->points.push_back(p);
  }

  cloud_ptr = cloud_scale;
}

int main (int argc, char** argv)
{
  std::string filename_reference = "/home/abdullah/catkin_ws/src/nbv_exploration/models/pcd/etihad/etihad_clean.pcd";
  std::string filename_final = "/home/abdullah/.ros/final_cloud.pcd";

  PointCloudXYZ::Ptr cloud_ref (new PointCloudXYZ);
  PointCloudXYZ::Ptr cloud_final (new PointCloudXYZ);

  // ===============
  // Read input file
  // ===============
  readCloudFromFile(filename_reference, cloud_ref);
  transformCloud(cloud_ref, 0.5, 0, 0, 1.5);

  readCloudFromFile(filename_final, cloud_final);

  // ===============
  // Create voxel grids
  // ===============
  std::vector<double> voxelRes;
  if (argc == 1)
    voxelRes.push_back(0.05);
  else
  {
    for (int i=1; i<argc; i++)
    {
      voxelRes.push_back( atof(argv[i]) );
    }
  }

  for (int i=0; i<voxelRes.size(); i++)
  {
    int grid_size_ref, grid_size_final;
    pcl::VoxelGridOcclusionEstimationT grid_ref, grid_final;

    grid_size_ref = createVoxelGrid(grid_ref, cloud_ref, voxelRes[i]);
    grid_size_final = createVoxelGrid(grid_final, cloud_final, voxelRes[i]);

    // ================
    // Compute coverage
    // ================
    int matched = matchesForGrid1InGrid2(grid_ref, grid_final);
    float coverage = float(matched)/grid_size_ref;

    printf("Resolution: %f, Coverage: %f%%\n", voxelRes[i], coverage*100);
  }

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
