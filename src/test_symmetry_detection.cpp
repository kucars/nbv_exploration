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

#include "lib/MeanShift/MeanShift.h"

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

struct KeypointFeature{
  PointN point;
  pcl::FPFHSignature33 feature;
};

// PCL Visualization
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2, vp_3, vp_4, vp_5;


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

std::string filename_pcd;




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

float getFeatureDistance(pcl::FPFHSignature33 p1, pcl::FPFHSignature33 p2)
{
  // Euclidian distance
  float sum = 0;

  for (int i=0; i<33; i++)
    sum += pow(p1.histogram[i] - p2.histogram[i], 2);

  return sqrt(sum);
}




std::vector<KeypointFeature> getKeypointFeatures(PointCloudN::Ptr cloud_normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeatures)
{
  pcl::StopWatch timer; //start timer

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
  sift.setScales(sift_min_scale, sift_num_octaves, sift_num_scales_per_octave);
  sift.setMinimumContrast(sift_min_contrast);
  sift.setInputCloud(cloud_normals);
  sift.compute(result);

  printf("[TIME] SIFT keypoints: %5.0lf ms\n", timer.getTime());
  timer.reset();

  // Copying the pointnormal to pointxyz so as to visualize the cloud
  PointCloud::Ptr cloud_keypoint (new PointCloud);
  copyPointCloud(result, *cloud_keypoint);

  printf("Cloud: %lu, Keypoints: %lu\n", cloud_normals->points.size(), cloud_keypoint->points.size());


  // ==========
  // Find nearest corresponding point for each keypoint in original cloud and its associated feature vector
  //
  // SIFT keypoints are computed by downscaling the cloud multiple times.
  // When returning to the original scale, sometimes the keypoint doesn't
  // coincide with a point in the original cloud. Here, we find the nearest
  // point in the original cloud and assign the keypoint to that location
  // ==========

  // K nearest neighbor search
  std::vector<KeypointFeature> keypoint_features;

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
      // Find closest point in original cloud
      int idx = pointIdxNKNSearch[0];

      // Add feature to vector
      KeypointFeature f;
      f.point   = cloud_normals->points[idx]; //searchPoint;
      f.feature = fpfhFeatures->points[idx];

      keypoint_features.push_back(f);
    }
    else
    {
      printf("NO CORRESPONDING POINT FOR KEYPOINT %d\n", i_keypoint);
    }
  }

  return keypoint_features;
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

  // ==========
  // Filter input
  // ==========
  if (use_prefilter)
  {
    PointCloud::Ptr cloud_temp (new PointCloud);

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud_in);
    sor.setLeafSize (prefilter_leaf_size, prefilter_leaf_size, prefilter_leaf_size);
    sor.filter (*cloud_temp);

    cloud_in = cloud_temp;

    printf("[TIME] Voxel Grid Filter: %5.0lf ms\n", timer.getTime());
    printf("Points: %lu\n", cloud_in->points.size() );
    timer.reset();
  }

  // ==========
  // Estimate points normals (parallelized)
  // ==========
  PointCloudN::Ptr cloud_normals (new PointCloudN);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimationOMP<PointT, PointN> norm_est;

  norm_est.setSearchMethod (tree);
  norm_est.setInputCloud (cloud_in);
  norm_est.setKSearch (tree_K);
  norm_est.compute (*cloud_normals);

  // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
  for(size_t i = 0; i<cloud_normals->points.size(); ++i)
  {
    cloud_normals->points[i].x = cloud_in->points[i].x;
    cloud_normals->points[i].y = cloud_in->points[i].y;
    cloud_normals->points[i].z = cloud_in->points[i].z;
  }

  printf("[TIME] Normal estimation: %5.0lf ms\n", timer.getTime());
  timer.reset();

  // ==========
  // Compute FPFH features
  // ==========
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::FPFHEstimation<PointT, PointN, pcl::FPFHSignature33> fpfhEstimation;
  fpfhEstimation.setInputCloud  (cloud_in);
  fpfhEstimation.setInputNormals(cloud_normals);

  // Use the same KdTree from the normal estimation
  fpfhEstimation.setSearchMethod (tree);
  fpfhEstimation.setKSearch (tree_K);

  // Compute features
  fpfhEstimation.compute (*fpfhFeatures);

  printf("[TIME] Feature calculation: %5.0lf ms\n", timer.getTime());
  timer.reset();

  // ==========
  // Get features of keypoints (if applicable)
  // ==========
  std::vector<KeypointFeature> feature_vec;
  if (use_sift_points)
  {
    // Get the features of only a few keypoints
    printf("Using SIFT features\n");
    feature_vec = getKeypointFeatures(cloud_normals, fpfhFeatures);
  }
  else
  {
    // Get the features of a subset of all points
    printf("Using all points (subset)\n");

    //int total_features = pairing_subset_percent*fpfhFeatures->points.size();
    double increment = 1/pairing_subset_percent;

    for (double i_point=0; i_point < fpfhFeatures->points.size(); i_point += increment)
    {
      KeypointFeature f;
      int idx = floor(i_point);
      f.point   = cloud_normals->points[idx]; //searchPoint;
      f.feature = fpfhFeatures->points[idx];

      feature_vec.push_back(f);
    }
  }


  // ==========
  // Pair keypoints based on feature similarity
  //
  // Do an exhaustive search, only eliminating a point from consideration
  // after considering how well it matches with all other keypoints
  // =========
  PointCloudN::Ptr cloud_pairs (new PointCloudN);

  long total_features = feature_vec.size();
  for (; feature_vec.size()>0; )
  {
    printf("\r Points remaining for pairing: %ld/%ld [%3.1lf\%]       ", feature_vec.size(), total_features, (1-1.0*feature_vec.size()/total_features)*100.0 );
    //Get last point in result cloud
    KeypointFeature p1 = feature_vec.back();

    //Find keypoints with similar features
    for (int i=0; i<feature_vec.size(); i++)
    {
      KeypointFeature p2 = feature_vec[i];

      // Don't pair a point with itself
      if (p1.point.x == p2.point.x &&
          p1.point.y == p2.point.y &&
          p1.point.z == p2.point.z)
        continue;

      float feat_dist = getFeatureDistance(p1.feature, p2.feature);

      if (feat_dist <= pairing_threshold)
      {
        // Assign the two as a pair
        cloud_pairs->points.push_back(p1.point);
        cloud_pairs->points.push_back(p2.point);

        if (!use_exhaustive_pairing)
        {
          // Not exhaustive, remove matched point and run next point
          feature_vec.erase (feature_vec.begin()+i);
          break;
        }
      }
    }

    // Remove the considered keypoint from the list and continue
    feature_vec.erase (feature_vec.end());
  }

  printf("\nFound %lu pairs\n", cloud_pairs->points.size()/2);

  printf("[TIME] Pairing: %5.0lf ms\n", timer.getTime());
  timer.reset();

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
      plane_transforms.push_back(pt);

  }

  printf("[TIME] Plane fitting: %5.0lf ms\n", timer.getTime());
  timer.reset();

  // ==========
  // Mean Shift Clustering
  // ==========

  // Convert from my struct (PlaneTransfrom) to a vector of features
  std::vector<std::vector<double> > features = convertPlanesToVectors(plane_transforms);

  // Perform clustering
  MeanShift *msp = new MeanShift();

  //int num_of_cluster_samples_ = cloud_pairs->points.size()/2;
  //std::vector<Cluster> clusters = msp->cluster(features, kernel_bandwidth, num_of_cluster_samples_);
  std::vector<Cluster> clusters = msp->run(features, mean_shift_kernel_bandwidth, mean_shift_num_points);

  printf("[TIME] Clustering: %5.0lf ms\n", timer.getTime());
  timer.reset();

  printf("Found %lu clusters\n", clusters.size());



  // ==========
  // Verify planes of symmetry
  // ==========
  // @todo

  //printf("[TIME] Verification: %5.0lf ms\n", timer.getTime());
  //timer.reset();


  // ==========
  // Mirror points in the input cloud
  // ==========

  // Notation:
  //   plane            n  = [ a,  b,  c] -> ax + by + cz + d = 0
  //   point            p  = [px, py, pz]
  //   normals          r  = [rx, ry, rz]
  //   mirrored point   P  = [Px, Py, Pz]
  //   mirrored normals R  = [Rx, Ry, Rz]
  //
  // Move the point 'p' into the plane along the normal with the paramter 't':
  //           P = p + t*n
  //   [x, y, z] = [px, py, pz] + t*[a, b, c]
  //
  // t = -(dot(n,p)+d) / ||n||^2
  // t = -(a*px + b*py + c*pz + d)/(a^2 + b^2 + c^2)
  // t = -(a*px + b*py + c*pz + d)/norm_sqr
  //
  // The mirrored point lies at 2t:
  //   P = p + 2*t*n
  //
  // The normals are vectors and are mirrored as follows:
  //   k = -dot(n,r) / ||n||^2
  //   R = r + 2*n*k

  // Copy cloud
  PointCloudN::Ptr cloud_mirrored (new PointCloudN);

  // Do for each plane of symmetry
  for (int c = 0; c<clusters.size(); c++)
  {
    // Get plane equation
    std::vector<double> n = clusters[c].shifted_points[0];

    // PLANE REPRESENTATION
    //    a*x +    b*y +    c*z + d    = 0
    // n[0]*x + n[1]*y + n[2]*z + n[3] = 0

    double norm_sqr = n[0]*n[0] + n[1]*n[1] + n[2]*n[2];

    // Mirror each point
    for(size_t i = 0; i<cloud_in->points.size(); ++i)
    {
      PointN p = cloud_normals->points[i];
      // Find the parameter 't'
      double t = -(n[0]*p.x + n[1]*p.y + n[2]*p.x + n[3])/norm_sqr;

      // Find mirrored point
      PointN m;
      m.x = p.x + 2*t*n[0];
      m.y = p.y + 2*t*n[1];
      m.z = p.z + 2*t*n[2];

      // Find parameter 'k'
      double k = -(n[0]*p.normal_x + n[1]*p.normal_y + n[2]*p.normal_z) / norm_sqr;

      // Find mirrored normals
      m.normal_x = p.normal_x + 2*n[0]*k;
      m.normal_y = p.normal_y + 2*n[1]*k;
      m.normal_z = p.normal_z + 2*n[2]*k;

      // Push the point into the cloud
      cloud_mirrored->points.push_back(m);
    }
  }



  printf("[TIME] Model Mirroring: %5.0lf ms\n", timer.getTime());
  timer.reset();


  // ==========
  // Correct the plane of symmetry
  // ==========
  PointCloudN::Ptr cloud_mirrored_corrected (new PointCloudN);

  pcl::IterativeClosestPoint<PointN, PointN>* icp;

  if (icp_with_normals)
    icp = new pcl::IterativeClosestPointWithNormals<PointN, PointN>;
  else
    icp = new pcl::IterativeClosestPoint<PointN, PointN>;

  /*
  pcl::registration::CorrespondenceEstimationNormalShooting<PointN, PointN, PointN>::Ptr ce (new pcl::registration::CorrespondenceEstimationNormalShooting<PointN, PointN, PointN>);
  ce->setInputSource (cloud_mirrored);
  ce->setInputTarget (cloud_normals);
  ce->setSourceNormals (cloud_mirrored);
  //ce->setTargetNormals (cloud_normals);
  ce->setKSearch (5);
  icp->setCorrespondenceEstimation (ce);
  */

  icp->setInputSource(cloud_mirrored);
  icp->setInputTarget(cloud_normals);

  /*
  double max_euclid, max_transform, max_distance;
  ros::param::param<double>("~icp_max_euclid", max_euclid, 0.1);
  ros::param::param<double>("~icp_max_transform", max_transform, 10);
  ros::param::param<double>("~icp_max_distance", max_distance, 1);

  icp->setMaxCorrespondenceDistance (max_distance);
  icp->setEuclideanFitnessEpsilon (max_euclid);
  icp->setTransformationEpsilon (max_transform);
  */

  icp->align(*cloud_mirrored_corrected);
  std::cout << "has converged:" << icp->hasConverged() << " score: " <<
  icp->getFitnessScore() << std::endl;
  std::cout << icp->getFinalTransformation() << std::endl;

  printf("[TIME] Realignment: %5.0lf ms\n", timer.getTime());
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

  printf("[TIME] Plane visualization: %5.0lf ms\n", timer.getTime());
  timer.reset();

  // ==========
  // Get Final Clouds for visualization
  // ==========
  // Copying the pointnormal to pointxyz so as to visualize the cloud
  PointCloud::Ptr cloud_pairs_xyz (new PointCloud);
  copyPointCloud(*cloud_pairs, *cloud_pairs_xyz);

  // Copy PointN to PointT before ICP
  PointCloud::Ptr cloud_mirrored_xyz (new PointCloud);
  copyPointCloud(*cloud_mirrored, *cloud_mirrored_xyz);

  // Copy PointN to PointT after ICP
  PointCloud::Ptr cloud_mirrored_corrected_xyz (new PointCloud);
  copyPointCloud(*cloud_mirrored_corrected, *cloud_mirrored_corrected_xyz);

  // Final result with original and corrected mirror points
  PointCloud::Ptr cloud_final (new PointCloud);
  *cloud_final = *cloud_in + *cloud_mirrored_corrected_xyz;

  // Determine what new points were added to the input cloud due to the method
  // Go through each point in the output and determine if any input points are near it
  PointCloud::Ptr cloud_fill (new PointCloud);

  pcl::KdTreeFLANN<PointT> tree_filled_pts;
  tree_filled_pts.setInputCloud (cloud_in);

  for (int i_pt = 0; i_pt < cloud_mirrored_corrected->points.size(); i_pt++)
  {
    PointT p = cloud_mirrored_corrected_xyz->points[i_pt];

    std::vector<int>   in_indices;
    std::vector<float> sqr_distances;

    // No original points found, so this point was added newly
    if ( tree_filled_pts.radiusSearch (p, subtraction_search_radius, in_indices, sqr_distances) == 0 )
      cloud_fill->points.push_back( p );
  }

  printf("[TIME] Subtract clouds: %5.0lf ms\n", timer.getTime());
  timer.reset();

  // ==========
  // Visualize
  // ==========

  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer ("Object symmetry");
  p->createViewPort (0.0, 0, 0.2, 1.0, vp_1);
  p->createViewPort (0.2, 0, 0.4, 1.0, vp_2);
  p->createViewPort (0.4, 0, 0.6, 1.0, vp_3);
  p->createViewPort (0.6, 0, 0.8, 1.0, vp_4);
  p->createViewPort (0.8, 0, 1.0, 1.0, vp_5);

  p->setBackgroundColor(255, 255, 255);
  p->setCameraClipDistances(65.3496, 129.337);
  p->setCameraPosition(35.1704, -68.6111, 63.8152,       0.0329489, 0.689653, 0.72339, 1);//, 0.0427789, -0.185814, 0.0496169, -0.0956887, -0.992963, 0.0697719);
  p->setCameraFieldOfView(0.523599);
  p->setSize(1366, 700);
  p->setPosition(0, 300);

  // Viewport 1 - Original Cloud
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler (cloud_in, 255, 0, 0);
  p->addPointCloud (cloud_in, cloud_color_handler, "input_cloud", vp_1);

  // Viewport 2 - Line of Symmetry
  pcl::visualization::PointCloudColorHandlerCustom<PointT> handler2 (cloud_in, 255, 0, 0);
  p->addPointCloud (cloud_in, handler2, "paired_cloud", vp_2);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> handler3 (cloud_plane, 0, 255, 0);
  p->addPointCloud (cloud_plane, handler3, "plane", vp_2);


  // Viewport 3 - Mirrored without correction
  pcl::visualization::PointCloudColorHandlerCustom<PointT> mirrored_color_handler (cloud_mirrored_xyz, 0, 255, 0);
  p->addPointCloud (cloud_mirrored_xyz, mirrored_color_handler, "mirrored_cloud", vp_3);

  p->addPointCloud (cloud_in, cloud_color_handler, "input_cloud_vp3", vp_3);

  // Viewport 4 - Final Result
  pcl::visualization::PointCloudColorHandlerCustom<PointT> final_color_handler (cloud_final, 255, 0, 0);
  p->addPointCloud (cloud_final, final_color_handler, "final_cloud", vp_4);


  // Viewport 5 - What was added to original
  pcl::visualization::PointCloudColorHandlerCustom<PointT> fill_color_handler (cloud_fill, 255, 0, 0);
  p->addPointCloud (cloud_fill, fill_color_handler, "fill_cloud", vp_5);


  p->spin();
}


void readRosParams ()
{
  ros::param::param<std::string>("~filename_pcd", filename_pcd, "-1");

  ros::param::param<bool>("~use_prefilter", use_prefilter, false);
  ros::param::param("~prefilter_leaf_size", prefilter_leaf_size, 0.1);

  ros::param::param<bool>("~use_sift_points", use_sift_points, false);
  ros::param::param("~sift_min_contrast", sift_min_contrast, 0.0001);
  ros::param::param("~sift_min_scale", sift_min_scale, 0.1);
  ros::param::param("~sift_num_octaves", sift_num_octaves, 3);
  ros::param::param("~sift_num_scales_per_octave", sift_num_scales_per_octave, 4);

  ros::param::param<bool>("~use_exhaustive_pairing", use_exhaustive_pairing, false);
  ros::param::param("~mean_shift_kernel_bandwidth", mean_shift_kernel_bandwidth, 10.0);
  ros::param::param<int>("~mean_shift_num_points", mean_shift_num_points, -1);
  ros::param::param("~pairing_subset_percent", pairing_subset_percent, 0.2);
  ros::param::param("~pairing_threshold", pairing_threshold, 20.0);
  ros::param::param("~tree_K", tree_K, 50);


  ros::param::param("~icp_with_normals", icp_with_normals, false);
  ros::param::param("~subtraction_search_radius", subtraction_search_radius, 0.05);
}


int main (int argc, char** argv)
{
  // ===================
  // Read config parameters
  // ===================
  ros::init(argc, argv, "test_symmetry_detection");
  readRosParams();

  if (filename_pcd == "-1")
  {
    printf("Could not read rosparam 'filename_pcd' from config files.\n");
    return -1;
  }

  // ===================
  // Read input file
  // ===================
  PointCloud::Ptr cloud (new PointCloud);

  if (pcl::io::loadPCDFile<PointT> (filename_pcd, *cloud) == -1)
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
