/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
//#include <gazebo_msgs/ModelStates.h> //Used for absolute positioning

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

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

// == Consts
std::string depth_topic    = "/iris/xtion_sensor/iris/xtion_sensor_camera/depth/points";
std::string position_topic = "/iris/ground_truth/pose";
std::string scan_in_topic  = "scan_in";

std::string map_topic      = "/global_cloud";
std::string scan_out_topic = "/global_scan_cloud";
std::string profile_out_topic = "/global_profile_cloud";

// == Navigation variables
geometry_msgs::Pose mobile_base_pose;

// == Publishers
ros::Publisher pub_global_cloud;
ros::Publisher pub_setpoint;
ros::Publisher pub_scan_cloud;
ros::Publisher pub_profile_cloud;

// == Subsctiptions
tf::TransformListener *tf_listener;


// == Point clouds
float grid_res = 0.1f; //Voxel grid resolution

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sensed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_ptr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_projected_cloud_ptr;


// ======================
// Function prototypes (@todo: move to header file)
// ======================

void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void positionCallback(const geometry_msgs::PoseStamped& pose_msg);
void scanCallback(const sensor_msgs::LaserScan& laser_msg);

void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out);
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
    ros::Subscriber sub_kinect = ros_node.subscribe(depth_topic, 1, depthCallback);
    ros::Subscriber sub_pose   = ros_node.subscribe(position_topic, 1, positionCallback);
    ros::Subscriber sub_scan   = ros_node.subscribe(scan_in_topic, 1, scanCallback);
    
    tf_listener = new tf::TransformListener();
    
    // >>>>>>>>>>>>>>>>>
    // Publishers
    // >>>>>>>>>>>>>>>>>
    pub_global_cloud = ros_node.advertise<sensor_msgs::PointCloud2>(map_topic, 10);
    pub_scan_cloud   = ros_node.advertise<sensor_msgs::PointCloud2>(scan_out_topic, 10);
    pub_profile_cloud   = ros_node.advertise<sensor_msgs::PointCloud2>(profile_out_topic, 10);

    ros::spin();

    return 0;
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

void scanCallback(const sensor_msgs::LaserScan& laser_msg){
  if (isDebug && isDebugContinuousStates){
    std::cout << cc_magenta << "Scan readings\n" << cc_reset;
  }
  
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  int steps = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment;
  float step_size = (laser_msg.angle_max - laser_msg.angle_min)/steps;
  float range_buffer = 0.5;
  
  
  int discarded = 0;
  for (int i=0; i<steps; i++)
  {
    float r = laser_msg.ranges[i];
    float angle = laser_msg.angle_min + step_size*i;
    
    if (r > laser_msg.range_max - range_buffer  ||  r < laser_msg.range_min + range_buffer)
    {
      // Discard points that are too close or too far to sensor
      discarded++;
      continue;
    }
    else
    {
      // Add scan point to temporary point cloud
      pcl::PointXYZRGB p;
      
      p.x = r*cos(angle);
      p.y = r*sin(angle);
      p.z = 0;
      
      scan_ptr->push_back(p);
    }
    
  }
  
  // == Transform
  try{
      // Listen for transform
      tf::StampedTransform transform;
      tf_listener->lookupTransform("world", "/iris/hokuyo_laser_link", ros::Time(0), transform);
      
      // == Convert tf:Transform to Eigen::Matrix4d
      Eigen::Matrix4d tf_eigen = convertStampedTransform2Matrix4d(transform);
      
      // == Transform point cloud
      pcl::transformPointCloud(*scan_ptr, *scan_ptr, tf_eigen);
      
      // == Discard points close to the ground
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_ptr_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      
      for (int i=0; i<scan_ptr->points.size(); i++)
      {
        if (scan_ptr->points[i].z > 0.5)
        {
          scan_ptr_filtered->push_back(scan_ptr->points[i]);
        }
      }
      
      // == Add to global
      addToGlobalCloud(scan_ptr_filtered, profile_cloud_ptr);
      
      
      // == Filtering stage (to remove outliers)
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud (scan_ptr_filtered);
      sor.setMeanK (10);
      sor.setStddevMulThresh (0.5);
      sor.filter (*scan_ptr_filtered);
      
      
      // == Publish
      sensor_msgs::PointCloud2 cloud_msg;
    
      pcl::toROSMsg(*profile_cloud_ptr, cloud_msg); 	//cloud of original (white) using original cloud
      cloud_msg.header.frame_id = "world";
      cloud_msg.header.stamp = ros::Time::now();
      
      pub_scan_cloud.publish(cloud_msg);
      
      
      
      
      // == Get profile (remove Z coordinate)
      profile_projected_cloud_ptr = profile_cloud_ptr->makeShared();
      
      for (int i=0; i<profile_projected_cloud_ptr->points.size(); i++)
      {
        profile_projected_cloud_ptr->points[i].z = 0;
      }
      
      
      
      // == Publish
      pcl::toROSMsg(*profile_projected_cloud_ptr, cloud_msg); 	//cloud of original (white) using original cloud
      cloud_msg.header.frame_id = "world";
      cloud_msg.header.stamp = ros::Time::now();
      
      pub_profile_cloud.publish(cloud_msg);
        
      
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }
}

void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out) {
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_out);
    sor.setLeafSize (grid_res, grid_res, grid_res);
    sor.filter (*cloud_filtered);

    cloud_out = cloud_filtered;
    
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
