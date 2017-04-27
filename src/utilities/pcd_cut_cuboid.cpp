#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// PCL Visualization
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;

int main (int argc, char** argv)
{
  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr cloud_filtered (new PointCloud);


  // ===================
  // Read input arguments
  // ===================
  if (argc < 8)
  {
    printf("Not enough input arguments. Syntax is: \n%s input x_min x_max y_min y_max z_min z_max\n", "pcd_cut_cuboid");
    return (-1);
  }

  std::string file_in (argv[1]);
  double bounds[6];

  for (int i=0; i<6; i++)
  {
    if (sscanf(argv[2+i], "%lf", &bounds[i]) != 1)
    {
      printf("Error in input %d (%s). Expected float.\n", i+2, argv[2+i]);
      return (-1);
    }
  }

  // ===================
  // Read input file
  // ===================
  if (pcl::io::loadPCDFile<PointT> (file_in, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file etihad.pcd \n");
    return (-1);
  }


  // ===================
  // Perform filtering
  // ===================
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (bounds[0], bounds[1]);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (bounds[2], bounds[3]);
  pass.filter (*cloud_filtered);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits (bounds[4], bounds[5]);
  pass.filter (*cloud_filtered);


  // ===================
  // Output results
  // ===================
  pcl::io::savePCDFileASCII ("output_cut.pcd", *cloud_filtered);

  printf("Cloud before filtering: %lu\n", cloud->points.size ());
  printf("Cloud after filtering: %lu\n", cloud_filtered->points.size ());

  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  // Show before and after
  pcl::visualization::PointCloudColorHandlerCustom<PointT> before_h (cloud, 0, 255, 0);
  p->addPointCloud (cloud, before_h, "vp1", vp_1);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> after_h (cloud_filtered, 0, 255, 0);
  p->addPointCloud (cloud_filtered, after_h, "vp2", vp_2);

  p->spin();

  return (0);
}
