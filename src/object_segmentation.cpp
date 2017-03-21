#include <iostream>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointN> PointCloudN;

// PCL Visualization
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;

// segmentation variables
int seg_min_cluster_size_ = 50;
int seg_max_cluster_size_ = 100000;
int seg_num_neighbors_ = 80;
double seg_smoothness_threshold_ = DEG2RAD(4.5);
double seg_curvature_threshold_ = 1.0;

void segmentObject(PointCloud::Ptr cloud_in)
{
  PointCloud::Ptr cloud_filtered (new PointCloud);

  // Estimate points normals
  PointCloudN::Ptr cloud_normals (new PointCloudN);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, PointN> norm_est;

  norm_est.setSearchMethod (tree);
  norm_est.setInputCloud (cloud_in);
  norm_est.setKSearch (50);
  norm_est.compute (*cloud_normals);

  /*
  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<PointT, PointN> seg;

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.5);
  seg.setRadiusLimits (0, 20);
  seg.setInputCloud (cloud_in);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Extract the cylindrical inliers from the input cloud
  pcl::ExtractIndices<PointT> extract;

  extract.setInputCloud (cloud_in);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*cloud_filtered);
  */

  // Run Region growing segmentation based on curvature
  pcl::RegionGrowing<PointT, PointN> reg;
  reg.setMinClusterSize (seg_min_cluster_size_);
  reg.setMaxClusterSize (seg_max_cluster_size_);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (seg_num_neighbors_);
  reg.setInputCloud (cloud_in);
  reg.setInputNormals (cloud_normals);
  reg.setSmoothnessThreshold (seg_smoothness_threshold_);
  reg.setCurvatureThreshold (seg_curvature_threshold_);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  printf("Found %lu clusters\n", clusters.size());


  std::vector<PointCloud::Ptr> output_clouds;
  // Extract the clusters from input point cloud
  for (int i=0; i<clusters.size(); i++)
  {
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    *indices = clusters[i];


    PointCloud::Ptr pc (new PointCloud);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_in);
    extract.setIndices (indices);
    extract.setNegative (false);
    extract.filter (*pc);

    output_clouds.push_back(pc);
  }

  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer ("Object segmentation");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  p->setBackgroundColor(255, 255, 255);
  p->setCameraClipDistances(65.3496, 129.337);
  p->setCameraPosition(35.1704, -68.6111, 63.8152,       0.0329489, 0.689653, 0.72339, 1);//, 0.0427789, -0.185814, 0.0496169, -0.0956887, -0.992963, 0.0697719);
  p->setCameraFieldOfView(0.523599);
  p->setSize(960, 540);
  p->setPosition(0, 500);

  //65.3496,129.337/6.66135,-0.846776,0.509762/35.1704,-68.6111,63.8152/0.0329489,0.689653,0.72339/0.523599/960,540/65,52

  pcl::visualization::PointCloudColorHandlerCustom<PointT> handler1 (cloud_in, 255, 0, 0);
  p->addPointCloud (cloud_in, handler1, "input_cloud", vp_1);


  //const int color_count = 26;
  //int colors[color_count][3] = {{2,63,165},{125,135,185},{190,193,212},{214,188,192},{187,119,132},{142,6,59},{74,111,227},{133,149,225},{181,187,227},{230,175,185},{224,123,145},{211,63,106},{17,198,56},{141,213,147},{198,222,199},{234,211,198},{240,185,141},{239,151,8},{15,207,192},{156,222,214},{213,234,231},{243,225,235},{246,196,225},{247,156,212}};

  // 26 colors designed for white background
  //int colors[color_count][3] = {{240,163,255},{0,117,220},{153,63,0},{76,0,92},{25,25,25},{0,92,49},{43,206,72},{255,204,153},{128,128,128},{148,255,181},{143,124,0},{157,204,0},{194,0,136},{0,51,128},{255,164,5},{255,168,187},{66,102,0},{255,0,16},{94,241,242},{0,153,143},{224,255,102},{116,10,255},{153,0,0},{255,255,128},{255,255,0},{255,80,5}};

  // 64 colors
  const int color_count = 64;
  int colors[color_count][3] = {
    {0  , 0  , 0},
    {1  , 0  , 103},
    {213, 255, 0},
    {255, 0  , 86},
    {158, 0  , 142},
    {14 , 76 , 161},
    {255, 229, 2},
    {0  , 95 , 57},
    {0  , 255, 0},
    {149, 0  , 58},
    {255, 147, 126},
    {164, 36 , 0},
    {0  , 21 , 68},
    {145, 208, 203},
    {98 , 14 , 0},
    {107, 104, 130},
    {0  , 0  , 255},
    {0  , 125, 181},
    {106, 130, 108},
    {0  , 174, 126},
    {194, 140, 159},
    {190, 153, 112},
    {0  , 143, 156},
    {95 , 173, 78},
    {255, 0  , 0},
    {255, 0  , 246},
    {255, 2  , 157},
    {104, 61 , 59},
    {255, 116, 163},
    {150, 138, 232},
    {152, 255, 82},
    {167, 87 , 64},
    {1  , 255, 254},
    {255, 238, 232},
    {254, 137, 0},
    {189, 198, 255},
    {1  , 208, 255},
    {187, 136, 0},
    {117, 68 , 177},
    {165, 255, 210},
    {255, 166, 254},
    {119, 77 , 0},
    {122, 71 , 130},
    {38 , 52 , 0},
    {0  , 71 , 84},
    {67 , 0  , 44},
    {181, 0  , 255},
    {255, 177, 103},
    {255, 219, 102},
    {144, 251, 146},
    {126, 45 , 210},
    {189, 211, 147},
    {229, 111, 254},
    {222, 255, 116},
    {0  , 255, 120},
    {0  , 155, 255},
    {0  , 100, 1},
    {0  , 118, 255},
    {133, 169, 0},
    {0  , 185, 23},
    {120, 130, 49},
    {0  , 255, 198},
    {255, 110, 65},
    {232, 94 , 190}};


  for (int i=0; i<output_clouds.size(); i++)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> handler2 (output_clouds[i], colors[i%color_count][0], colors[i%color_count][1], colors[i%color_count][2]);

    char name[16];
    sprintf(name, "%d",i);
    p->addPointCloud (output_clouds[i], handler2, name, vp_2);
  }

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

  if (argc>2)
  {
    seg_min_cluster_size_ = atoi(argv[2]);
  }

  if (argc>3)
  {
    seg_max_cluster_size_ = atoi(argv[3]);
  }

  if (argc>4)
  {
    seg_num_neighbors_ = atoi(argv[4]);
  }

  if (argc>5)
  {
    seg_smoothness_threshold_ = DEG2RAD( atof(argv[5]) );
  }

  if (argc>6)
  {
    seg_curvature_threshold_ = atof(argv[6]);
  }

  // ===================
  // Read input file
  // ===================
  PointCloud::Ptr cloud (new PointCloud);

  if (pcl::io::loadPCDFile<PointT> (file_in, *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file etihad.pcd \n");
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
  segmentObject(cloud_scale);

  return (0);
}
