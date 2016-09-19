/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>

//PCL
//#include <pcl/filters.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>

// =========================
// Colors for console window
// =========================
const std::string cc_black("\033[0;30m");
const std::string cc_red("\033[0;31m");
const std::string cc_green("\033[1;32m");
const std::string cc_yellow("\033[1;33m");
const std::string cc_blue("\033[1;34m");
const std::string cc_magenta("\033[0;35m");
const std::string cc_cyan("\033[0;36m");
const std::string cc_white("\033[0;37m");

const std::string cc_bold("\033[1m");
const std::string cc_darken("\033[2m");
const std::string cc_underline("\033[4m");
const std::string cc_background("\033[7m");
const std::string cc_strike("\033[9m");

const std::string cc_erase_line("\033[2K");
const std::string cc_reset("\033[0m");


// ===================
// === Variables  ====
// ===================
bool isDebug = !true; //Set to true to see debug text
bool isDebugContinuousStates = !true;

// Profiling --------
bool isScanning = false;
std::vector<octomap::point3d> pose_vec;
std::vector< pcl::PointCloud<pcl::PointXYZRGB> > scan_vec;
//std::vector<octomap::Pointcloud> scan_vec;
double laser_range = -1;

// == Consts
std::string depth_topic    = "/iris/xtion_sensor/iris/xtion_sensor_camera/depth/points";
std::string position_topic = "/iris/ground_truth/pose";
std::string scan_in_topic  = "scan_in";
std::string scan_command_topic = "/nbv_exploration/scan_command";

std::string map_topic      = "/global_cloud";
std::string scan_out_topic = "/global_scan_cloud";
std::string profile_out_topic = "/global_profile_cloud";
std::string tree_topic = "/nbv_exploration/output_tree";

// == Navigation variables
geometry_msgs::Pose mobile_base_pose;

// == Publishers
ros::Publisher pub_global_cloud;
ros::Publisher pub_scan_cloud;
ros::Publisher pub_profile_cloud;
ros::Publisher pub_tree;

// == Subsctiptions
ros::Subscriber sub_kinect;
ros::Subscriber sub_pose;
ros::Subscriber sub_scan;
ros::Subscriber sub_scan_command;

tf::TransformListener *tf_listener;


// == Point clouds and octrees
float grid_res = 0.1f; //Voxel grid resolution

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sensed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_ptr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_projected_cloud_ptr;

octomap::OcTree tree(0.3);

// ======================
// Function prototypes (@todo: move to header file)
// ======================

void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void positionCallback(const geometry_msgs::PoseStamped& pose_msg);
void scanCallback(const sensor_msgs::LaserScan& laser_msg);
void scanCommandCallback(const std_msgs::UInt8& msg);
void processScans();

void addToGlobalCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, bool should_filter = true);
Eigen::Matrix4d convertStampedTransform2Matrix4d(tf::StampedTransform t);

int main(int argc, char **argv)
{
    // >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    std::cout << cc_red << "Begin Sensing\n" << cc_reset;

    ros::init(argc, argv, "sensing_and_mapping");
    ros::NodeHandle ros_node;

    // >>>>>>>>>>>>>>>>>
    // Subscribers
    // >>>>>>>>>>>>>>>>>
    
    // Sensor data
    sub_kinect = ros_node.subscribe(depth_topic, 1, depthCallback);
    sub_pose   = ros_node.subscribe(position_topic, 1, positionCallback);
    sub_scan   = ros_node.subscribe(scan_in_topic, 1, scanCallback);
    sub_scan_command = ros_node.subscribe(scan_command_topic, 1, scanCommandCallback);
    
    tf_listener = new tf::TransformListener();
    
    // >>>>>>>>>>>>>>>>>
    // Publishers
    // >>>>>>>>>>>>>>>>>
    pub_global_cloud  = ros_node.advertise<sensor_msgs::PointCloud2>(map_topic, 10);
    pub_scan_cloud    = ros_node.advertise<sensor_msgs::PointCloud2>(scan_out_topic, 10);
    pub_profile_cloud = ros_node.advertise<sensor_msgs::PointCloud2>(profile_out_topic, 10);
    pub_tree          = ros_node.advertise<octomap_msgs::Octomap>(tree_topic, 10);
    
    
    // >>>>>>>>>>>>>>>>>
    // Main function
    // >>>>>>>>>>>>>>>>>
    std::cout << cc_magenta << "\nStarted\n" << cc_reset;
    std::cout << "Listening for the following topics: \n";
    std::cout << "\t" << depth_topic << "\n";
    std::cout << "\t" << position_topic << "\n";
    std::cout << "\t" << scan_in_topic << "\n";
    std::cout << "\t" << scan_command_topic << "\n";
    std::cout << "\n";
    
    ros::Rate rate(5);
    while(ros::ok())
    {
      // == Publish
      // Cloud
      if (profile_cloud_ptr)
      {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*profile_cloud_ptr, cloud_msg); 	//cloud of original (white) using original cloud
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        pub_scan_cloud.publish(cloud_msg);
      }
      
      // 2D profile
      if (profile_projected_cloud_ptr)
      {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*profile_projected_cloud_ptr, cloud_msg); 	//cloud of original (white) using original cloud
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        pub_profile_cloud.publish(cloud_msg);
      }
    
      // Octomap
      octomap_msgs::Octomap msg;
      octomap_msgs::fullMapToMsg (tree, msg);
      
      msg.header.frame_id = "world";
      msg.header.stamp = ros::Time::now();
      pub_tree.publish(msg);
    
      // Sleep
      ros::spinOnce();
      rate.sleep();
    }
    
    //ros::spin();
    return 0;
}


void scanCallback(const sensor_msgs::LaserScan& laser_msg){
  if (isDebug && isDebugContinuousStates){
    std::cout << cc_magenta << "Scan readings\n" << cc_reset;
  }
  
  if (!isScanning)
    return;
  
  // == Get transform
  tf::StampedTransform transform;
  Eigen::Matrix4d tf_eigen;
  
  try{
    tf_listener->lookupTransform("world", "/iris/hokuyo_laser_link", ros::Time(0), transform);
    tf_eigen = convertStampedTransform2Matrix4d(transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
    return;
  }
  
  // == Add scan to point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_far_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  int steps = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment;
  float step_size = (laser_msg.angle_max - laser_msg.angle_min)/steps;
  
  laser_range = laser_msg.range_max;
  
  // == Discard points outside range
  for (int i=0; i<steps; i++)
  {
    float r = laser_msg.ranges[i];
    float angle = laser_msg.angle_min + step_size*i;
    
    if (r < laser_msg.range_min)
    {
      // Discard points that are too close
      continue;
    }
    
    // Add scan point to temporary point cloud
    pcl::PointXYZRGB p;
    
    p.x = r*cos(angle);
    p.y = r*sin(angle);
    p.z = 0;
    
    if (r > laser_msg.range_max)
      scan_far_ptr->push_back(p);
    else
      scan_ptr->push_back(p);
  }
  
  // == Transform scan point cloud
  pcl::transformPointCloud(*scan_ptr, *scan_ptr, tf_eigen);
  pcl::transformPointCloud(*scan_far_ptr, *scan_far_ptr, tf_eigen);
  
  // == Discard points close to the ground
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_ptr_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  for (int i=0; i<scan_ptr->points.size(); i++)
  {
    if (scan_ptr->points[i].z > 0.2)
    {
      scan_ptr_filtered->push_back(scan_ptr->points[i]);
    }
  }
  
  // == Add to global map
  addToGlobalCloud(scan_ptr_filtered, profile_cloud_ptr, true);
  
  // == Add to vectors for later octomap processing
  octomap::point3d sensor_origin (transform.getOrigin().x(),
                                  transform.getOrigin().y(),
                                  transform.getOrigin().z());
                                  
  pose_vec.push_back(sensor_origin);
  scan_vec.push_back(*scan_ptr + *scan_far_ptr);
  
  // == Get profile (remove Z coordinate)
  profile_projected_cloud_ptr = profile_cloud_ptr->makeShared();
  
  for (int i=0; i<profile_projected_cloud_ptr->points.size(); i++)
  {
    profile_projected_cloud_ptr->points[i].z = 0;
  }
}


void findLargestGap()
{
  
}


void scanCommandCallback(const std_msgs::UInt8& msg)
{  
  switch (msg.data)
  {
    case 0:
      isScanning = false;
      std::cout << cc_green << "Processing " << scan_vec.size() << " scans\n" << cc_reset;
      processScans();
      break;
    case 1:
      isScanning = true;
      std::cout << cc_green << "Starting scan\n" << cc_reset;
      break;
    case 2:
      isScanning = false;
      std::cout << cc_green << "Done profiling\n" << cc_reset;
      //ros::shutdown();
      break;
  }
}

void processScans()
{
  double t_start, t_end;
  t_start = ros::Time::now().toSec();
  
  int count =0;
  for (int i=scan_vec.size()-1; i>=0; i--)
  {
    octomap::point3d sensor_origin = pose_vec[i];
    pcl::PointCloud<pcl::PointXYZRGB> scan = scan_vec[i];
    
    octomap::Pointcloud ocCloud;
    for (int j=0; j<scan.points.size(); j++)
    {
      ocCloud.push_back(scan.points[j].x,
                        scan.points[j].y,
                        scan.points[j].z);
    }
    
    // == Insert laser scan
    //tree.insertPointCloud(ocCloud, sensor_origin, laser_range);
    
    octomap::KeySet free_cells, occupied_cells;
    tree.computeUpdate(ocCloud, sensor_origin, free_cells, occupied_cells, laser_range);

    // insert data into tree  -----------------------
    for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
    {
      tree.updateNode(*it, false);
    }
    for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
    {
      octomap::point3d p = tree.keyToCoord(*it);
      if (p.z() <= 0.2)
        tree.updateNode(*it, false);
      else
        tree.updateNode(*it, true);
    }
    
    // == Pop
    pose_vec.pop_back();
    scan_vec.pop_back();
    count++;
  }
  
  // == Publish
  /*
  octomap_msgs::Octomap msg;
  octomap_msgs::fullMapToMsg (tree, msg);
  
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  pub_tree.publish(msg);
  */
  
  // == Timing
  t_end = ros::Time::now().toSec();
  std::cout << cc_green << "Done processing.\n" << cc_reset;
  std::cout << "   Total time: " << t_end-t_start << " sec\tTotal scan: " << count << "\t(" << (t_end-t_start)/count << " sec/scan)\n";
}

// Update global position of UGV
void positionCallback(const geometry_msgs::PoseStamped& pose_msg)
{
    if (isDebug && isDebugContinuousStates){
        std::cout << cc_magenta << "Grabbing location\n" << cc_reset;
    }
    
    // Save UGV pose
    mobile_base_pose = pose_msg.pose;
}

void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    if (isDebug && isDebugContinuousStates){
        std::cout << cc_green << "SENSING\n" << cc_reset;
    }
    
    
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr;

    // == Convert to pcl pointcloud
    pcl::fromROSMsg (*cloud_msg, cloud);
    cloud_ptr = cloud.makeShared();

    //std::cout << cc_blue << "Number of points detected: " << cloud_ptr->points.size() << "\n" << cc_reset;
    
    // == Remove NAN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_ptr,*cloud_ptr, indices);
    //std::cout << cc_blue << "Number of points remaining: " << cloud_ptr->points.size() << "\n" << cc_reset;
    
    
    
    // == Perform voxelgrid filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_ptr);
    sor.setLeafSize (grid_res, grid_res, grid_res);
    sor.filter (*cloud_filtered);
    
    //std::cout << cc_blue << "Number of points remaining: " << cloud_filtered->points.size() << "\n" << cc_reset;
    
    // == Transform
    try{
        // Listen for transform
        tf::StampedTransform transform;
        tf_listener->lookupTransform("world", "iris/xtion_sensor/camera_depth_optical_frame", ros::Time(0), transform);
        
        // == Convert tf:Transform to Eigen::Matrix4d
        Eigen::Matrix4d tf_eigen = convertStampedTransform2Matrix4d(transform);
        
        // == Transform point cloud
        pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, tf_eigen);
        
        // == Add filtered to global
        addToGlobalCloud(cloud_filtered, global_cloud_ptr);
        
        // == Publish
        sensor_msgs::PointCloud2 cloud_msg;
        
        pcl::toROSMsg(*global_cloud_ptr, cloud_msg); 	//cloud of original (white) using original cloud
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        
        pub_global_cloud.publish(cloud_msg);
        
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void addToGlobalCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, bool should_filter) {
    if (isDebug && isDebugContinuousStates){
        std::cout << cc_green << "MAPPING\n" << cc_reset;
    }
    
    // Initialize global cloud if not already done so
    if (!cloud_out){
        cloud_out = cloud_in;
        return;
    }

    *cloud_out += *cloud_in;
    
    // Perform voxelgrid filtering
    if (should_filter)
    {
      // == Voxel grid filter
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

      pcl::VoxelGrid<pcl::PointXYZRGB> vox_sor;
      vox_sor.setInputCloud (cloud_out);
      vox_sor.setLeafSize (grid_res, grid_res, grid_res);
      vox_sor.filter (*voxel_filtered);


      // == Statistical outlier removal
      /*
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> stat_sor;
      stat_sor.setInputCloud (voxel_filtered);
      stat_sor.setMeanK (10);
      stat_sor.setStddevMulThresh (0.5);
      stat_sor.filter (*stat_filtered);

      cloud_out = stat_filtered;
      */
    }
    
    if (isDebug && isDebugContinuousStates){
        std::cout << cc_blue << "Number of points in global map: " << cloud_out->points.size() << "\n" << cc_reset;
    }
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
