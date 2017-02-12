/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <signal.h>

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
//#include <gazebo_msgs/ModelStates.h> //Used for absolute positioning

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

//#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>


#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/registration/icp.h>

#include "utilities/console_utility.h"
ConsoleUtility cc;


// ===================
// === Variables  ====
// ===================
bool is_debug = true; //Set to true to see debug text
bool is_debug_states = true;
bool is_debug_continuous_states = !true;

bool isScanning = false;
std::vector<octomap::point3d> pose_vec;
std::vector<octomap::Pointcloud> scan_vec;
double max_range;

geometry_msgs::Pose mobile_base_pose;
octomap::OcTree tree(0.3);

// == Consts
std::string scan_topic = "scan_in";
std::string scan_command_topic = "/nbv_exploration/scan_command";
std::string tree_topic = "output_tree";
std::string position_topic = "/iris/ground_truth/pose";


// == Publishers
ros::Subscriber sub_pose;
ros::Subscriber sub_scan;
ros::Subscriber sub_scan_command;

ros::Publisher pub_tree;

tf::TransformListener *tf_listener;




// ======================
// Function prototypes
// ======================
void scanCallback(const sensor_msgs::LaserScan& laser_msg);
void scanCommandCallback(const std_msgs::UInt8& msg);

void processScans();

void print_query_info(octomap::point3d query, octomap::OcTreeNode* node) {
  if (node != NULL) {
    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
  }
  else 
    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;    
}

double randomDouble(double min, double max)
{
  return ((double) random()/RAND_MAX)*(max-min) + min;
}


void addToTree(octomap::Pointcloud cloud, octomap::point3d origin, double r_max)
{
  /*
  octomap::KeySet free_cells, occupied_cells;
  tree.computeUpdate(cloud, origin, free_cells, occupied_cells, r_max);

  // insert data into tree  -----------------------
  bool lazy_eval = false;
  
  double bad_range = 0.2;
  
  for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
    octomap::point3d coord = tree.keyToCoord(*it);
    double r = sqrt( pow(origin.x() - coord.x(), 2) + pow(origin.y() - coord.y(), 2) + pow(origin.z() - coord.z(), 2) );
    double prob = 1/(r + bad_range);
    prob = octomap::logodds(prob);
    
    tree.updateNode(*it, (float) prob, lazy_eval);
  }
  for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
    octomap::point3d coord = tree.keyToCoord(*it);
    double r = sqrt( pow(origin.x() - coord.x(), 2) + pow(origin.y() - coord.y(), 2) + pow(origin.z() - coord.z(), 2) );
    double prob = r/(r + bad_range);
    prob = octomap::logodds(prob);
    
    tree.updateNode(*it, (float) prob, lazy_eval);
  }
    */
    
  //tree.insertPointCloud(ocCloud, sensor_origin, max_range);
}

Eigen::Matrix4d convertStampedTransform2Matrix4d(tf::StampedTransform t){
  Eigen::Matrix4d tf_eigen;
        
  Eigen::Vector3d T1(
      t.getOrigin().x(),
      t.getOrigin().y(),
      t.getOrigin().z()
  );
  
  Eigen::Matrix3d R;
  tf::Quaternion qt = t.getRotation();
  tf::Matrix3x3 R1(qt);
  tf::matrixTFToEigen(R1,R);
  
  // Set
  tf_eigen.setZero ();
  tf_eigen.block (0, 0, 3, 3) = R;
  tf_eigen.block (0, 3, 3, 1) = T1;
  tf_eigen (3, 3) = 1;
  
  return tf_eigen;
}

int main(int argc, char **argv)
{
  // >>>>>>>>>>>>>>>>>
  // Initialize ROS
  // >>>>>>>>>>>>>>>>>

  /* Override SIGINT handler */
  ros::init(argc, argv, "test_octomap");
  ros::NodeHandle ros_node;

  // >>>>>>>>>>>>>>>>>
  // Topic handlers
  // >>>>>>>>>>>>>>>>>
  sub_scan         = ros_node.subscribe(scan_topic, 1, scanCallback);
  sub_scan_command = ros_node.subscribe(scan_command_topic, 1, scanCommandCallback);
  
  pub_tree = ros_node.advertise<octomap_msgs::Octomap>(tree_topic, 10);
  
  tf_listener = new tf::TransformListener();
  
  
  
  // >>>>>>>>>>>>>>>>>
  // Main function
  // >>>>>>>>>>>>>>>>>
  std::cout << cc.magenta << "\nStarted\n" << cc.reset;
  std::cout << "Listening for the following topics: \n";
  std::cout << "\t" << scan_topic << "\n";
  std::cout << "\t" << position_topic << "\n";
  std::cout << "\n";

  ros::spin();
  return 0;
}

void scanCommandCallback(const std_msgs::UInt8& msg)
{  
  switch (msg.data)
  {
    case 0:
      isScanning = false;
      std::cout << cc.green << "Processing " << scan_vec.size() << " scans\n" << cc.reset;
      processScans();
      break;
    case 1:
      isScanning = true;
      std::cout << cc.green << "Starting scan\n" << cc.reset;
      break;
    case 2:
      isScanning = false;
      std::cout << cc.green << "Done profiling\n" << cc.reset;
      ros::shutdown();
      break;
  }
}

void scanCallback(const sensor_msgs::LaserScan& laser_msg)
{
  if (!isScanning)
    return;
    
  //std::cout << cc.green << "Got scan\n" << cc.reset;
  
  // == Get transform
  tf::StampedTransform transform;
  try{
    // Listen for transform
    tf_listener->lookupTransform("world", "/iris/hokuyo_laser_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
    return;
  }
  
  // == Reject scan if it's too close to the last one
  /*
  if (pose_vec.size() > 0)
  {
    double z_prev = pose_vec[pose_vec.size()-1].z();
    double z_curr = transform.getOrigin().z();
    
    if ( fabs(z_prev - z_curr) < 0.03)
      return;
  }
  */
  
  max_range = laser_msg.range_max;
  //if (max_range > 25)
  //  max_range = 25;
  
  // == Build point cloud from scan
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  int steps = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment;
  float step_size = (laser_msg.angle_max - laser_msg.angle_min)/steps;
  
  for (int i=0; i<steps; i++)
  {
    float r = laser_msg.ranges[i];
    float angle = laser_msg.angle_min + step_size*i;
    
    // Discard points that are too close to sensor
    if (r < laser_msg.range_min)
    {
      continue;
    }
    
    // If nothing is seen on that ray, assume it is free
    if (r > laser_msg.range_max)
    {
      r = laser_msg.range_max+1;
    }

    // Add scan point to temporary point cloud
    pcl::PointXYZRGB p;
    
    p.x = r*cos(angle);
    p.y = r*sin(angle);
    p.z = 0;
    
    scan_ptr->push_back(p);
    
  }
  
  // == Transform
  {
    // == Convert tf:Transform to Eigen::Matrix4d
    Eigen::Matrix4d tf_eigen = convertStampedTransform2Matrix4d(transform);
    
    // == Transform point cloud
    pcl::transformPointCloud(*scan_ptr, *scan_ptr, tf_eigen);
    
    // == Discard points close to the ground
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_ptr_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    octomap::Pointcloud ocCloud;
    
    for (int i=0; i<scan_ptr->points.size(); i++)
    {
      if (scan_ptr->points[i].z > 0.5)
      {
        ocCloud.push_back(scan_ptr->points[i].x,
                          scan_ptr->points[i].y,
                          scan_ptr->points[i].z);
        // == Populate an octomap::Pointcloud
        //scan_ptr_filtered->push_back(scan_ptr->points[i]);
        
      }
    }
    
    // Add to vectors for later processing
    octomap::point3d sensor_origin (transform.getOrigin().x(),
                                    transform.getOrigin().y(),
                                    transform.getOrigin().z());
                                    
    pose_vec.push_back(sensor_origin);
    scan_vec.push_back(ocCloud);
  }
}

void processScans()
{
  double t_start, t_end;
  t_start = ros::Time::now().toSec();
  
  int count =0;
  for (int i=scan_vec.size()-1; i>0; i--)
  {
    octomap::point3d sensor_origin = pose_vec[i];
    octomap::Pointcloud ocCloud = scan_vec[i];
    
    tree.insertPointCloud(ocCloud, sensor_origin, max_range);
    //addToTree(ocCloud, sensor_origin, max_range);
    
    pose_vec.pop_back();
    scan_vec.pop_back();
    count++;
  }
  
  // == Publish
  octomap_msgs::Octomap msg;
  octomap_msgs::fullMapToMsg (tree, msg);
  
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  pub_tree.publish(msg);
  
  t_end = ros::Time::now().toSec();
  std::cout << cc.green << "Done processing.\n" << cc.reset;
  std::cout << "   Total time: " << t_end-t_start << " sec\tTotal scan: " << count << "\t(" << (t_end-t_start)/count << " sec/scan)\n";
}
