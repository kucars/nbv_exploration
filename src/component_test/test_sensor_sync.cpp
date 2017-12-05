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
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "control/vehicle_control_floating_sensor.h"
#include "nbv_exploration/mapping_module.h"
#include "utilities/time_profiler.h"

TimeProfiler timer;

/*
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
//#include <gazebo_msgs/ModelStates.h> //Used for absolute positioning


#include <eigen_conversions/eigen_msg.h>

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
*/

// == Publishers
ros::Subscriber sub_pose;
ros::Publisher  pub_viz_pose;
ros::Publisher  pub_viz_tf;

tf::TransformListener *tf_listener;

MappingModule*        mapping_module_;
VehicleControlBase*   vehicle_;
geometry_msgs::Pose   last_pose;

void visualize(bool newline)
{
  ros::Time tf_time = ros::Time::now();

  tf::StampedTransform transform;

  try{
    tf_listener->waitForTransform("world", "floating_sensor/camera_frame", tf_time, ros::Duration(1.0), ros::Duration(0.01));
    tf_listener->lookupTransform ("world", "floating_sensor/camera_frame", tf_time, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  tf::Vector3 vec;
  vec = transform.getOrigin();

  if (newline)
    printf("\n\n");
  printf("TF  : (%3.3f, %3.3f, %3.3f)\n", vec.getX(), vec.getY(), vec.getZ() );
  //printf("Pose: (%3.3f, %3.3f, %3.3f)\n", last_pose.position.x, last_pose.position.y, last_pose.position.z);
}


void callbackPose(const geometry_msgs::Pose& msg){
  last_pose = msg;
}

int main(int argc, char **argv)
{
  // >>>>>>>>>>>>>>>>>
  // Initialize ROS
  // >>>>>>>>>>>>>>>>>

  ros::init(argc, argv, "test_sensor_sync");

  ros::NodeHandle nh, nh_private("~");
  vehicle_ = new VehicleControlFloatingSensor();
  vehicle_->start();

  mapping_module_ = new MappingModule(nh,nh_private);
  mapping_module_->commandProfilingStart();
  mapping_module_->commandProfilingStop();
  boost::thread mapping_thread_(&MappingModule::run, mapping_module_);

  ros::Duration(2.0).sleep();
  
  // >>>>>>>>>>>>>>>>>
  // Publishers
  // >>>>>>>>>>>>>>>>>
  ros::NodeHandle n;

  //sub_pose     = n.subscribe("/floating_sensor/pose", 10, &callbackPose);
  pub_viz_pose = n.advertise<visualization_msgs::Marker>("sync_test/pose", 10);
  pub_viz_tf   = n.advertise<visualization_msgs::Marker>("sync_test/tf", 10);

  tf_listener = new tf::TransformListener();

  // >>>>>>>>>>>>>>>>>
  // Main function
  // >>>>>>>>>>>>>>>>>

  double x, y, z, yaw;
  ros::param::param<double>("~uav_pose_x_1", x, 0);
  ros::param::param<double>("~uav_pose_y_1", y, 0);
  ros::param::param<double>("~uav_pose_z_1", z, 10);
  ros::param::param<double>("~uav_pose_yaw_1", yaw, 0);

  double x_inc, y_inc, z_inc, yaw_inc;
  ros::param::param<double>("~uav_pose_x_inc", x_inc, 0);
  ros::param::param<double>("~uav_pose_y_inc", y_inc, 1);
  ros::param::param<double>("~uav_pose_z_inc", z_inc, 0);
  ros::param::param<double>("~uav_pose_yaw_inc", yaw_inc, 0);

  int iterations;
  ros::param::param<int>("~uav_pose_iterations", iterations, 10);

  for (int i=0; i<iterations; i++)
  {
    printf("\n=============\nIteration: %d\n", i);

    vehicle_->setWaypoint(x, y, z, yaw);
    vehicle_->setSpeed(-1);

    vehicle_->moveVehicle(0.25); //Make sure we go to the exact position

    //ros::Duration(0.3).sleep(); // Sleep momentarily to allow tf to catch up for teleporting sensor
    //ros::Duration(2.0).sleep(); // 2 cameras
    visualize(false);

    printf("Getting data...\n");
    mapping_module_->commandGetCameraData();
    printf("Done\n");

    visualize(true);

    x   += x_inc;
    y   += y_inc;
    z   += z_inc;
    yaw += yaw_inc;
  }

  ros::spin();
  return 0;
}
