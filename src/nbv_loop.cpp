/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/Pointcloud.h>
//#include <gazebo_msgs/ModelStates.h> //Used for absolute positioning

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

//#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
//#include <tf/transform_listener.h>

//PCL
//#include <pcl/filters.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/registration/icp.h>

// Custom classes
#include <nbv_exploration/nbv_loop.h>
#include <nbv_exploration/model_profiler_base.h>
#include <nbv_exploration/model_profiler_circular_adaptive.h>
#include <nbv_exploration/view_generator.h>
#include <nbv_exploration/view_selecter.h>
#include <nbv_exploration/termination_check_base.h>

#include "control/vehicle_control_base.h"
#include "control/vehicle_control_iris.h"


// ===================
// === Variables  ====
// ===================
bool is_debug = true; //Set to true to see debug text
bool is_debug_states = true;
bool is_debug_callbacks = !true;

bool is_done_profiling = false;
bool is_scan_empty = false;
bool is_flying_up = false;

bool waiting_for_profile_cloud = false;
bool waiting_for_profile_octomap = false;


double profile_angle = 0;
double uav_height_min, uav_height_max, uav_obstacle_distance_min;


// == Publishers / Clients
ros::Publisher pub_global_cloud;
ros::Publisher pub_scan_command;


// == Point clouds
float grid_res = 0.1f; //Voxel grid resolution

pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_projected_cloud_ptr  (new pcl::PointCloud<pcl::PointXYZRGB>);
octomap::OcTree* global_octomap;

// ================
// Functions
// ================

NBVLoop::NBVLoop()
{
  // >>>>>>>>>>>>>>>>>
  // Initialize ROS
  // >>>>>>>>>>>>>>>>>
  state = NBVState::INITIALIZING;
  
  ROS_INFO("nbv_loop: BEGIN NBV LOOP");
  ros::NodeHandle ros_node;
  
  // >>>>>>>>>>>>>>>>>
  // Initialize parameters
  // >>>>>>>>>>>>>>>>>
  initParameters();
  

  // >>>>>>>>>>>>>>>>>
  // Topic handlers
  // >>>>>>>>>>>>>>>>>
  
  // Sensor data
  ros::Subscriber sub_scan      = ros_node.subscribe(topic_scan_cloud, 1, &NBVLoop::callbackScan, this);
  ros::Subscriber sub_octomap   = ros_node.subscribe(topic_octree, 1, &NBVLoop::callbackOctomap, this);
  
  
  // >>>>>>>>>>>>>>>>
  // Initialize modules
  // >>>>>>>>>>>>>>>>
  initViewGenerator();
  initViewSelecter();
  initVehicle();
  initModelProfiler();
  termination_check_module_ = new TerminationCheckBase();


  // >>>>>>>>>>>>>>>>>
  // Parse Inputs
  // >>>>>>>>>>>>>>>>>
  bool success = true;
  if (skip_profiling)
  {
    is_done_profiling = true;
    success = model_profiler_->skipProfiling(skip_profiling_load_map);
  }
  else
  {
    success = model_profiler_->startProfiling();
  }

  if (!success)
  {
    ros::shutdown();
    return;
  }

  // >>>>>>>>>>>>>>>>>
  // Start the FSM
  // >>>>>>>>>>>>>>>>>
  NBVLoop::runStateMachine();

  // >>>>>>>>>>>>>>>>>
  // Clean up
  // >>>>>>>>>>>>>>>>>
  std::cout << cc.yellow << "Shutting down\n" << cc.reset;
  ros::shutdown();
}

void NBVLoop::callbackOctomap(const octomap_msgs::Octomap& octomap_msg)
{
  if (is_debug && is_debug_callbacks)
  {
    std::cout << cc.magenta << "Grabbing octree\n" << cc.reset;
  }

  global_octomap = static_cast<octomap::OcTree* >( octomap_msgs::fullMsgToMap(octomap_msg) );


  if (global_octomap)
  {
    // Correct the information that was lost in transmission
    //global_octomap->setClampingThresMin( 1-global_octomap->getClampingThresMax() );

    waiting_for_profile_octomap = false;
  }
}

void NBVLoop::callbackScan(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  if (is_debug && is_debug_callbacks)
  {
    std::cout << cc.magenta << "Grabbing profile point cloud\n" << cc.reset;
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  // == Convert to pcl pointcloud
  pcl::fromROSMsg (*cloud_msg, cloud);
  profile_cloud_ptr = cloud.makeShared();

  waiting_for_profile_cloud = false;
}

void NBVLoop::generateViewpoints()
{
  if (state != NBVState::VIEWPOINT_GENERATION)
  {
    std::cout << cc.red << "ERROR: Attempt to generate viewpoints out of order\n" << cc.reset;
    return;
  }
  if (is_debug && is_debug_states)
  {
    std::cout << cc.green << "Generating viewpoints\n" << cc.reset;
  }

  waiting_for_profile_cloud = true;
  waiting_for_profile_octomap = true;

  ros::Rate rate(30);
  while (ros::ok() && (!profile_cloud_ptr || waiting_for_profile_cloud) )
  {
    ROS_INFO_ONCE("Waiting for point cloud profile");

    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok() && (!global_octomap || waiting_for_profile_octomap) )
  {
    ROS_INFO_ONCE("Waiting for octree profile");

    ros::spinOnce();
    rate.sleep();
  }

  view_generator_->setCloud(profile_cloud_ptr);
  view_generator_->setMap(global_octomap);
  view_generator_->setCurrentPose(vehicle_->getPose());
  view_generator_->generateViews();

  state = NBVState::VIEWPOINT_GENERATION_COMPLETE;
}

void NBVLoop::evaluateViewpoints()
{
  if (state != NBVState::VIEWPOINT_EVALUATION)
  {
    std::cout << cc.red << "ERROR: Attempt to evaluate viewpoints out of order\n" << cc.reset;
    return;
  }
  if (is_debug && is_debug_states)
  {
    std::cout << cc.green << "Evaluating viewpoints\n" << cc.reset;
  }

  view_selecter_->evaluate();
  geometry_msgs::Pose p = view_selecter_->getTargetPose();
  vehicle_->setWaypoint(p);

  state = NBVState::VIEWPOINT_EVALUATION_COMPLETE;
}

void NBVLoop::initModelProfiler()
{
  model_profiler_ = new ModelProfilerCircularAdaptive();
  model_profiler_->setVehicle(vehicle_);
}

void NBVLoop::initParameters()
{
  // >>>>>>>>>>>>>>>>>
  // Fixed values
  // >>>>>>>>>>>>>>>>>
  is_terminating = false;
  iteration_count = 0;

  grid_res = 0.1f;


  // >>>>>>>>>>>>>>>>>
  // Read params
  // >>>>>>>>>>>>>>>>>
  // STATES
  ros::param::param("~skip_profiling", skip_profiling, false);
  ros::param::param("~skip_profiling_load_map", skip_profiling_load_map, true);

  // NAVIGATION
  ros::param::param("~nav_bounds_z_min", uav_height_min, 0.05);
  ros::param::param("~nav_bounds_z_max", uav_height_max, 10.0);
  ros::param::param("~uav_obstacle_distance_min", uav_obstacle_distance_min, 1.0);

  // TOPIC NAMES
  ros::param::param("~topic_depth", topic_depth, std::string("/iris/xtion_sensor/iris/xtion_sensor_camera/depth/points") );
  ros::param::param("~topic_octree", topic_octree, std::string("/nbv_exploration/output_tree") );
  ros::param::param("~topic_scan_cloud", topic_scan_cloud, std::string("/nbv_exploration/scan_cloud") );
  ros::param::param("~topic_profile_cloud", topic_profile_cloud, std::string("/nbv_exploration/profile_cloud") );
}

void NBVLoop::initViewGenerator()
{
  //view_generator_ = new ViewGenerator_Frontier();
  view_generator_ = new ViewGeneratorNN();
  view_generator_->setDebug(true);
}

void NBVLoop::initViewSelecter()
{
  view_selecter_ = new ViewSelecterBase();
  view_selecter_->setViewGenerator(view_generator_);
  view_selecter_->setDebug(false);
}

void NBVLoop::initVehicle()
{
   vehicle_ = new VehicleControlIris();
}

void NBVLoop::profilingProcessing(){
  if (state != NBVState::PROFILING_PROCESSING)
  {
    std::cout << cc.red << "ERROR: Attempt to start profiling out of order\n" << cc.reset;
    return;
  }
  if (is_debug && is_debug_states)
  {
    std::cout << cc.green << "Processing profile\n" << cc.reset;
  }

  bool done_profiling;
  done_profiling = model_profiler_->run(profile_cloud_ptr);

  if (done_profiling)
  {
    state = NBVState::PROFILING_COMPLETE;
  }
}

void NBVLoop::runStateMachine()
{
  ROS_INFO("nbv_loop: Ready to take off. Waiting for current position information.");
  state = NBVState::STARTING_ROBOT;

  ros::Rate loop_rate(30);

  while (ros::ok() && !is_terminating)
  {
    switch(state)
    {
      case NBVState::IDLE:
        state = NBVState::MOVING_COMPLETE;
      break;

      case NBVState::STARTING_ROBOT:
        if( vehicle_->isReady() )
        {
          state = NBVState::STARTING_ROBOT_COMPLETE;
          break;
        }

        vehicle_->start();
        break;

      case NBVState::STARTING_ROBOT_COMPLETE:
        if (!is_done_profiling)
        {
          state = NBVState::PROFILING_PROCESSING;
          profilingProcessing();
        }
        else
        {
          state = NBVState::MOVING_COMPLETE;
        }
        break;

      case NBVState::PROFILING_PROCESSING:
        profilingProcessing();
        break;

      case NBVState::MOVING_COMPLETE:
        std::cout << "[" << ros::Time::now().toSec() << "]" << cc.magenta << "Requesting camera data\n" << cc.reset;
        model_profiler_->callMappingService(nbv_exploration::MappingSrv::Request::GET_CAMERA_DATA);

        state = NBVState::TERMINATION_CHECK;
        terminationCheck();

        break;

      case NBVState::TERMINATION_MET:
        is_terminating = true;
        std::cout << cc.yellow << "Termination condition met\n" << cc.reset;
        break;

      case NBVState::PROFILING_COMPLETE:
      case NBVState::TERMINATION_NOT_MET:
        state = NBVState::VIEWPOINT_GENERATION;
        generateViewpoints();
        break;

      case NBVState::VIEWPOINT_GENERATION_COMPLETE:
        state = NBVState::VIEWPOINT_EVALUATION;
        evaluateViewpoints();
        break;

      case NBVState::VIEWPOINT_EVALUATION_COMPLETE:
        state = NBVState::MOVING;
        vehicle_->moveVehicle(0.25); //Make sure we go to the exact position
        break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return;
}

void NBVLoop::terminationCheck()
{
  if (state != NBVState::TERMINATION_CHECK)
  {
    std::cout << cc.red << "ERROR: Attempt to check termination out of order\n" << cc.reset;
    return;
  }
  if (is_debug && is_debug_states)
  {
    std::cout << cc.green << "Checking termination condition\n" << cc.reset;
  }


  termination_check_module_->update();

  if (termination_check_module_->isTerminated())
    state = NBVState::TERMINATION_MET;
  else
    state = NBVState::TERMINATION_NOT_MET;
}
