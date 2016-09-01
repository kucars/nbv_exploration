/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <signal.h>

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

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
bool is_debug = true; //Set to true to see debug text
bool is_debug_states = true;
bool is_debug_continuous_states = !true;


geometry_msgs::Pose mobile_base_pose;
octomap::OcTree tree(0.3);

// == Consts
std::string scan_topic = "scan_in";
std::string tree_topic = "output_tree";
std::string position_topic = "/iris/ground_truth/pose";


// == Publishers
ros::Subscriber sub_pose;
ros::Subscriber sub_scan;

ros::Publisher pub_tree;

tf::TransformListener *tf_listener;




// ======================
// Function prototypes
// ======================
void scanCallback(const sensor_msgs::LaserScan& laser_msg);

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


void addToTree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out) {
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
  sub_scan = ros_node.subscribe(scan_topic, 1, scanCallback);
  
  pub_tree = ros_node.advertise<octomap_msgs::Octomap>(tree_topic, 10);
  
  tf_listener = new tf::TransformListener();
  
  
  
  // >>>>>>>>>>>>>>>>>
  // Main function
  // >>>>>>>>>>>>>>>>>
  std::cout << cc_magenta << "\nStarted\n" << cc_reset;
  std::cout << "Listening for the following topics: \n";
  std::cout << "\t" << scan_topic << "\n";
  std::cout << "\t" << position_topic << "\n";
  std::cout << "\n";
  
  
  /*
  //octomap::OcTree tree (0.1);  // create empty tree with resolution 0.1
  // insert some measurements of occupied cells

  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        octomap::point3d endpoint ((float) x*0.5f, (float) y*0.5f, (float) z*0.5f);
        tree.updateNode(endpoint, octomap::logodds(1.0f)); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

  for (int x=-15; x<15; x++) {
    for (int y=-15; y<15; y++) {
      for (int z=-15; z<15; z++) {
        octomap::point3d endpoint ((float) x*1.0f, (float) y*1.0f, (float) z*1.0f);
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }
  
  
  // insert some random cell

  for (int i=1; i<100; i++) {
    octomap::point3d endpoint (randomDouble(-10,10)*0.1f, randomDouble(-10,10)*0.1f, randomDouble(-10,10)*0.1f);
    tree.updateNode(endpoint, octomap::logodds(randomDouble(0,1)) );
  }
  
  std::cout << std::endl;
  std::cout << "performing some queries:" << std::endl;
  
  octomap::point3d query (0., 0., 0.);
  octomap::OcTreeNode* result = tree.search (query);
  print_query_info(query, result);
  //std::cout << result->getOccupancy() << "\n";

  query = octomap::point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);
  //std::cout << result->getOccupancy() << "\n";

  query = octomap::point3d(1.,1.,1.);
  result = tree.search (query);
  print_query_info(query, result);
  //std::cout << result->getOccupancy() << "\n";

  std::cout << std::endl;
  tree.writeBinary("simple_tree.bt");
  std::cout << "wrote example file simple_tree.bt" << std::endl << std::endl;
  std::cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << std::endl;
  std::cout << "Hint: hit 'F'-key in viewer to see the freespace" << std::endl << std::endl;  
  */
  
  
  
  
  
  
  
  ros::spin();
  return 0;
}

void scanCallback(const sensor_msgs::LaserScan& laser_msg)
{
  std::cout << cc_green << "Got scan\n" << cc_reset;
  
  
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
  try{
    // Listen for transform
    tf::StampedTransform transform;
    tf_listener->lookupTransform("world", "/iris/hokuyo_laser_link", ros::Time(0), transform);
    
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
    
   
    
    // == Add to tree
    octomap::point3d sensor_origin (transform.getOrigin().x(),
                                    transform.getOrigin().y(),
                                    transform.getOrigin().z());
    
    double max_range = laser_msg.range_max;
    //if (max_range > 25)
    //  max_range = 25;
      
    tree.insertPointCloud(ocCloud, sensor_origin, max_range);
    
    //addToTree(scan_ptr_filtered);
    
    // == Publish
    octomap_msgs::Octomap msg;
    octomap_msgs::fullMapToMsg (tree, msg);
    
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    pub_tree.publish(msg);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}



/*
bool isNear(double p1, double p2)
{
  if (fabs(p1-p2)< distance_threshold)
  {
    return true;
  }
    
  return false;
}

bool isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current){
  if (
    getDistance(p_target, p_current) < distance_threshold &&
    fabs(getAngularDistance(p_target, p_current)) < angular_threshold)
    {
    return true;
  }
    
  return false;
}

double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return sqrt(
    (p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) +
    (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) +
    (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z) );
}

double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  double roll1, pitch1, yaw1;
  double roll2, pitch2, yaw2;
  
  tf::Quaternion q1 (
    p1.orientation.x,
    p1.orientation.y,
    p1.orientation.z,
    p1.orientation.w
    );
    
  tf::Quaternion q2 (
    p2.orientation.x,
    p2.orientation.y,
    p2.orientation.z,
    p2.orientation.w
    );
  
  tf::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
  tf::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);
  
  // Set differnce from -pi to pi
  double yaw_diff = fmod(yaw1 - yaw2, 2*M_PI);
  if (yaw_diff > M_PI)
  {
    //std::cout << cc_green << "Decreased by 360: \n";
    yaw_diff = yaw_diff - 2*M_PI;
  }
  else if (yaw_diff < -M_PI)
  {
    //std::cout << cc_green << "Increased by 360: \n";
    yaw_diff = yaw_diff + 2*M_PI;
  }
    
  //std::cout << cc_green << "Yaw1: " << yaw1*180/M_PI << "\tYaw2: " << yaw2*180/M_PI << "\n" << cc_reset;
  //std::cout << cc_green << "Yaw difference: " << yaw_diff*180/M_PI << "\n" << cc_reset;
  
  return yaw_diff;
}

void transformSetpoint2Global (const geometry_msgs::Pose & p_set, geometry_msgs::Pose& p_global)
{
  // Apply a 90 degree clockwise rotation on the z-axis
  p_global.position.x = p_set.position.y;
  p_global.position.y =-p_set.position.x;
  p_global.position.z = p_set.position.z;
  
  
  
  double roll, pitch, yaw;
  
  tf::Quaternion q1 (
    p_set.orientation.x,
    p_set.orientation.y,
    p_set.orientation.z,
    p_set.orientation.w
    );
    
  tf::Matrix3x3(q1).getRPY(roll, pitch, yaw);
  
  yaw -= M_PI_2; //Rotate
  
  tf::Quaternion qt = tf::createQuaternionFromRPY(roll,pitch,yaw);
  
  p_global.orientation.x = qt.getX();
  p_global.orientation.y = qt.getY();
  p_global.orientation.z = qt.getZ();
  p_global.orientation.w = qt.getW();
}

void transformGlobal2Setpoint (const geometry_msgs::Pose & p_global, geometry_msgs::Pose& p_set)
{
  // Apply a 90 degree anti-clockwise rotation on the z-axis
  p_set.position.x =-p_global.position.y;
  p_set.position.y = p_global.position.x;
  p_set.position.z = p_global.position.z;
  
  
  double roll, pitch, yaw;
  
  tf::Quaternion q1 (
    p_global.orientation.x,
    p_global.orientation.y,
    p_global.orientation.z,
    p_global.orientation.w
    );
    
  tf::Matrix3x3(q1).getRPY(roll, pitch, yaw);
  
  yaw += M_PI_2; //Rotate
  
  tf::Quaternion qt = tf::createQuaternionFromRPY(roll,pitch,yaw);
  
  p_set.orientation.x = qt.getX();
  p_set.orientation.y = qt.getY();
  p_set.orientation.z = qt.getZ();
  p_set.orientation.w = qt.getW();
}
*/
