#include <iostream>

#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lib/MeanShift_cpp/MeanShift.h"

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointN> PointCloudN;

// Structures
struct PlaneTransform{
  double a, b, c, d;
  double distance;
};

// PCL Visualization
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2, vp_3;

// segmentation variables
int num_of_samples_ = 2000;

std::vector<std::vector<double> > convertPlanesToVectors(std::vector<PlaneTransform> planes)
{
  std::vector<std::vector<double> > final;

  for (int i=0; i<planes.size(); i++)
  {
    std::vector<double> v;
    v.push_back(planes[i].a);
    v.push_back(planes[i].b);
    v.push_back(planes[i].c);
    v.push_back(planes[i].d);
    //v.push_back(planes[i].distance);

    final.push_back(v);
  }

  return final;
}

std::vector<PlaneTransform> convertVectorsToPlanes(std::vector<std::vector<double> > planes)
{
  std::vector<PlaneTransform> final;

  for (int i=0; i<planes.size(); i++)
  {
    PlaneTransform pt;
    pt.a = planes[i][0];
    pt.b = planes[i][1];
    pt.c = planes[i][2];
    pt.d = planes[i][3];
    //pt.distance = planes[i][4];

    final.push_back(pt);
  }

  return final;
}

void findSymmetry(PointCloud::Ptr cloud_in)
{
  /* Based on a simplified method proposed by N. J. Mitra (2003) in
   *  "Approximate Symmetry Detection and Symmetrization"
   *
   * This method does the following:
   * 1- Randomly sample two points
   * 2- Find line connecting the points and midpoint
   * 3- Construct perpendicular plane at midpoint
   * 4- Record plane parameters (phi) and distance of points to plane (d)
   * 5- Repeat for N random pairs
   * 6- Perform clustering on transform space to find planes of symmetry
   */

  pcl::StopWatch timer; //start timer

  // Estimate points normals (parallelized)
  PointCloudN::Ptr cloud_normals (new PointCloudN);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimationOMP<PointT, PointN> norm_est;

  norm_est.setSearchMethod (tree);
  norm_est.setInputCloud (cloud_in);
  norm_est.setKSearch (50);
  norm_est.compute (*cloud_normals);

  // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
  for(size_t i = 0; i<cloud_normals->points.size(); ++i)
  {
    cloud_normals->points[i].x = cloud_in->points[i].x;
    cloud_normals->points[i].y = cloud_in->points[i].y;
    cloud_normals->points[i].z = cloud_in->points[i].z;
  }

  printf("[TIME] Normal estimation: %5.2lf ms\n", timer.getTime());
  timer.reset();

  // ==========
  // Determine keypoints
  // =========
  // Parameters for sift computation
  const float min_scale = 0.1f;
  const int n_octaves = 3;
  const int n_scales_per_octave = 4;
  const float min_contrast = 0.0001f;

  // Estimate the sift interest points using normals values from xyz as the Intensity variants
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointNormal> sift;
  pcl::PointCloud<pcl::PointNormal> result;

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree_sift(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree_sift);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_normals);
  sift.compute(result);

  printf("[TIME] SIFT keypoints: %5.2lf ms\n", timer.getTime());
  timer.reset();

  // Copying the pointnormal to pointxyz so as to visualize the cloud
  PointCloud::Ptr cloud_keypoint (new PointCloud);
  copyPointCloud(result, *cloud_keypoint);

  printf("Cloud: %lu, Keypoints: %lu\n", cloud_in->points.size(), cloud_keypoint->points.size());


  // ==========
  // Find nearest corresponding point for each keypoint in original cloud and copy its curvature
  // ==========
  // K nearest neighbor search

  pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
  kdtree.setInputCloud (cloud_normals);
  int K = 1;

  for (int i_keypoint=0; i_keypoint<result.points.size(); i_keypoint++)
  {
    pcl::PointNormal searchPoint = result.points[i_keypoint];

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      // Copy curvature to result
      int normal_idx = pointIdxNKNSearch[0];
      result.points[i_keypoint].curvature = cloud_normals->points[normal_idx].curvature;
    }
    else
    {
      printf("NO CORRESPONDING POINT FOR KEYPOINT %d\n", i_keypoint);
    }
  }


  // ==========
  // Pair keypoints based on curvature
  // =========
  double pairing_threshold = 0.0001;
  PointCloudN::Ptr cloud_pairs (new PointCloudN);

  int num_points_rem = result.points.size();
  for (; result.points.size()>0; )
  {
    //Get last point in result cloud
    PointN p1 = result.points.back();
    bool found_pair = false;

    //Find first point with similar curvature
    for (int i=0; i<result.points.size()-1; i++)
    {
      PointN p2 = result.points[i];
      //printf("num_points_rem: %d - diff: %f\n", num_points_rem, fabs(p1.curvature - p2.curvature));
      if (fabs(p1.curvature - p2.curvature) <= pairing_threshold)
      {
        // Assign the two as a pair and delete them from the cloud
        cloud_pairs->points.push_back(p1);
        cloud_pairs->points.push_back(p2);

        result.points.erase (result.points.end());
        result.points.erase (result.points.begin()+i);

        found_pair = true;

        break;
      }
    }

    //If no pair was found, delete the last point in the cloud anyway
    if (!found_pair)
    {
      result.points.erase (result.points.end());
    }
  }

  printf("Found %lu pairs\n", cloud_pairs->points.size()/2);

  printf("[TIME] Pairing: %5.2lf ms\n", timer.getTime());
  timer.reset();

  // Copying the pointnormal to pointxyz so as to visualize the cloud
  PointCloud::Ptr cloud_pairs_xyz (new PointCloud);
  copyPointCloud(*cloud_pairs, *cloud_pairs_xyz);



  // ==========
  // Random sampling
  // =========
  /*
  PointCloud::Ptr cloud_pairs (new PointCloud);

  pcl::RandomSample<PointT> rand_sample;
  rand_sample.setInputCloud(cloud_in);
  rand_sample.setSample(num_of_samples_);
  rand_sample.filter(*cloud_pairs);


  printf("[TIME] Random sampling: %5.2lf ms\n", timer.getTime());
  timer.reset();
  */

  // ==========
  // Fit plane for every pair of points
  // =========
  // @todo: handle all normals in one octant

  std::vector<PlaneTransform> plane_transforms;

  for (int i=0; i<cloud_pairs->points.size(); i+=2)
  {
    PointN p1 = cloud_pairs->points[i];
    PointN p2 = cloud_pairs->points[i+1];

    // Reorder points to keep normals in one half of R^3
    if (p1.x < p2.x)
    {
      PointN temp = p1;
      p1 = p2;
      p2 = temp;
    }

    // Midpoint
    double x_mid, y_mid, z_mid;
    x_mid = (p1.x + p2.x)/2;
    y_mid = (p1.y + p2.y)/2;
    z_mid = (p1.z + p2.z)/2;


    // Normal to the plane & Distance to midpoint
    double nx, ny, nz;
    nx = x_mid - p1.x;
    ny = y_mid - p1.y;
    nz = z_mid - p1.z;

    // Normalize
    double dist;
    dist = sqrt(nx*nx + ny*ny + nz*nz);
    nx /= dist;
    ny /= dist;
    nz /= dist;

    // Final parameters
    PlaneTransform pt;

    pt.a = nx;
    pt.b = ny;
    pt.c = nz;
    pt.d = -(nx*x_mid + ny*y_mid + nz*z_mid);
    pt.distance = dist;

    // Only append in the plane is not NaN
    if (!isnan(pt.a))
    {
      plane_transforms.push_back(pt);

      //printf("pair %d: curvature: %f, eqn=[%2.2f, %2.2f, %2.2f, %2.2f]\n", i/2, fabs(p1.curvature - p2.curvature), pt.a, pt.b, pt.c, pt.d);
    }

  }

  printf("[TIME] Plane fitting: %5.2lf ms\n", timer.getTime());
  timer.reset();

  // ==========
  // Mean Shift Clustering
  // ==========

  // Convert from my struct (PlaneTransfrom) to a vector of features
  std::vector<std::vector<double> > features = convertPlanesToVectors(plane_transforms);

  // Perform clustering
  MeanShift *msp = new MeanShift();
  double kernel_bandwidth = 10;

  int num_of_cluster_samples_ = cloud_pairs->points.size()/2;
  std::vector<Cluster> clusters = msp->cluster(features, kernel_bandwidth, num_of_cluster_samples_);

  printf("[TIME] Clustering: %5.2lf ms\n", timer.getTime());
  timer.reset();

  printf("Found %lu clusters\n", clusters.size());


  // ==========
  // Verify planes
  // ==========
  // @todo

  printf("[TIME] Verification: %5.2lf ms\n", timer.getTime());
  timer.reset();


  // ==========
  // Mirror points in the input cloud
  // ==========

  // Move the point p=[px, py, pz] into the plane along the normal with the paramter 't':
  //   [x, y, z] = [px, py, pz] + t*[a, b, c]
  //   ax + by + cz + d = 0
  //
  // t = -(a*px + b*py + c*pz + d)/(a^2 + b^2 + c^2)
  // t = -(a*px + b*py + c*pz + d)/den
  //
  // The mirrored point lies at 2t:
  //   mx = px + 2*t*a
  //   my = py + 2*t*b
  //   mz = pz + 2*t*c

  // Copy cloud
  PointCloud::Ptr cloud_mirrored (new PointCloud);
  for (int i=0; i<cloud_in->points.size(); i++)
  {
    cloud_mirrored->points.push_back(cloud_in->points[i]);
  }

  for (int c = 0; c<clusters.size(); c++)
  {
    // Get plane equation
    std::vector<double> n = clusters[c].shifted_points[0];

    // PLANE REPRESENTATION
    //    a*x +    b*y +    c*z + d    = 0
    // n[0]*x + n[1]*y + n[2]*z + n[3] = 0

    double den = n[0]*n[0] + n[1]*n[1] + n[2]*n[2];

    // Mirror each point
    for(size_t i = 0; i<cloud_in->points.size(); ++i)
    {
      PointT p = cloud_in->points[i];
      // Find the parameter 't'
      double t = -(n[0]*p.x + n[1]*p.y + n[2]*p.x + n[3])/den;

      // Find mirrored point
      PointT m;
      m.x = p.x + 2*t*n[0];
      m.y = p.y + 2*t*n[1];
      m.z = p.z + 2*t*n[2];

      // Push the point into the cloud
      cloud_mirrored->points.push_back(m);
    }
  }


  // Voxel grid filtering to remove close points
  /*
  float leaf_size = 0.005f;

  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud_mirrored);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (*cloud_mirrored);
  */


  printf("[TIME] Model Mirroring: %5.2lf ms\n", timer.getTime());
  timer.reset();

  // ===========
  // Create point cloud representing plane obtained from clustering
  // ===========
  PointCloud::Ptr cloud_plane (new PointCloud);

  for(int cluster = 0; cluster < clusters.size(); cluster++) {
    // Get cluster mean which represents a plane
    std::vector<double> n = clusters[cluster].shifted_points[0];

    // PLANE REPRESENTATION
    //    a*x +    b*y +    c*z + d    = 0
    // n[0]*x + n[1]*y + n[2]*z + n[3] = 0


    // Get plane perpendicular
    std::vector<double> u, v;
    {

      // find smallest component
      int min=0;
      int i;
      for (i=1; i<3; ++i)
        if (abs(n[min])>abs(n[i]))
          min=i;

      // get the other two indices
      int a=(i+1)%3;
      int b=(i+2)%3;

      u.push_back(0);
      u.push_back( n[b] );
      u.push_back( -n[a] );

      // Normalize
      double u_mag = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
      u[0] /= u_mag;
      u[1] /= u_mag;
      u[2] /= u_mag;
    }

    // Get cross product
    v.push_back(n[1]*u[2] - n[2]*u[1]);
    v.push_back(n[2]*u[0] - n[0]*u[2]);
    v.push_back(n[0]*u[1] - n[1]*u[0]);

    // Normalize
    double v_mag = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    v[0] /= v_mag;
    v[1] /= v_mag;
    v[2] /= v_mag;

    // Find point on plane closest to origin
    // This point is the center of the visual
    double k, x_plane, y_plane, z_plane;

    k = -n[3]/(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
    x_plane = k*n[0];
    y_plane = k*n[1];
    z_plane = k*n[2];

    // Draw circles around the selected point on the plane
    for (double r=1; r<30; r+=1)
    {
      double angle_inc = M_PI_4/(2*r);
      for (double theta=0; theta<2*M_PI; theta+=angle_inc)
      {
        double x_circle = r*cos(theta);
        double y_circle = r*sin(theta);

        PointT p;
        p.x = x_plane + x_circle*u[0] + y_circle*v[0];
        p.y = y_plane + x_circle*u[1] + y_circle*v[1];
        p.z = z_plane + x_circle*u[2] + y_circle*v[2];

        cloud_plane->points.push_back(p);
      }
    }
  }

  printf("[TIME] Plane visualization: %5.2lf ms\n", timer.getTime());
  timer.reset();

  // ==========
  // Visualize
  // ==========

  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer ("Object symmetry");
  p->createViewPort (0.00, 0, 0.33, 1.0, vp_1);
  p->createViewPort (0.33, 0, 0.66, 1.0, vp_2);
  p->createViewPort (0.66, 0, 1.00, 1.0, vp_3);

  p->setBackgroundColor(255, 255, 255);
  p->setCameraClipDistances(65.3496, 129.337);
  p->setCameraPosition(35.1704, -68.6111, 63.8152,       0.0329489, 0.689653, 0.72339, 1);//, 0.0427789, -0.185814, 0.0496169, -0.0956887, -0.992963, 0.0697719);
  p->setCameraFieldOfView(0.523599);
  p->setSize(960, 540);
  p->setPosition(0, 500);

  //65.3496,129.337/6.66135,-0.846776,0.509762/35.1704,-68.6111,63.8152/0.0329489,0.689653,0.72339/0.523599/960,540/65,52


  // Viewport 1
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler (cloud_in, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> keypoints_color_handler (cloud_keypoint, 0, 255, 0);
  p->addPointCloud (cloud_in, cloud_color_handler, "input_cloud", vp_1);
  p->addPointCloud (cloud_keypoint, keypoints_color_handler, "cloud_keypoint", vp_1);
  p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "cloud_keypoint");

  // Viewport 2
  //pcl::visualization::PointCloudColorHandlerCustom<PointT> handler2 (cloud_pairs_xyz, 255, 0, 0);
  //p->addPointCloud (cloud_pairs_xyz, handler2, "paired_cloud", vp_2);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> handler2 (cloud_in, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> handler3 (cloud_plane, 0, 255, 0);
  p->addPointCloud (cloud_in, handler2, "paired_cloud", vp_2);
  p->addPointCloud (cloud_plane, handler3, "plane", vp_2);


  // Viewport 3
  pcl::visualization::PointCloudColorHandlerCustom<PointT> mirrored_color_handler (cloud_mirrored, 255, 0, 0);
  p->addPointCloud (cloud_mirrored, mirrored_color_handler, "mirrored_cloud", vp_3);

  p->spin();

}


int main (int argc, char** argv)
{
  // ===================
  // Read input arguments
  // ===================
  if (argc < 2)
  {
    printf("Not enough input arguments. Specify the filename of the .pcd file\n");
    return (-1);
  }
  std::string file_in (argv[1]);

  if (argc > 2)
    num_of_samples_ = atoi(argv[2]); //convert string to int


  // ===================
  // Read input file
  // ===================
  PointCloud::Ptr cloud (new PointCloud);

  if (pcl::io::loadPCDFile<PointT> (file_in, *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read PCD file\n");
    return (-1);
  }

  // ===================
  // Copy input file point by point, otherwise the z-dimension is squashed in visualization
  // ===================
  PointCloud::Ptr cloud_scale (new PointCloud);
  for (int i=0; i<cloud->points.size(); i++)
  {
    PointT p = cloud->points[i];
    cloud_scale->points.push_back(p);
  }


  // Run segmentation
  findSymmetry(cloud_scale);


  return (0);
}
