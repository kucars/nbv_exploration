/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <iostream>
#include <boost/thread/thread.hpp>

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

#include "nbv_exploration/nbv_loop.h"


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
  
  // >>>>>>>>>>>>>>>>
  // Initialize modules
  // >>>>>>>>>>>>>>>>
  history_ = new NBVHistory();
  initMappingModule();
  initViewGenerator();
  initViewSelecter();
  initVehicle();
  initModelProfiler();
  initTerminationChecker();


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
  std::cout << "[NBVLoop] " << cc.yellow << "Shutting down\n" << cc.reset;
  ros::shutdown();
}

void NBVLoop::generateViewpoints()
{
  if (state != NBVState::VIEWPOINT_GENERATION)
  {
    std::cout << "[NBVLoop] " << cc.red << "ERROR: Attempt to generate viewpoints out of order\n" << cc.reset;
    return;
  }
  if (is_debug_states)
  {
    std::cout << "[NBVLoop] " << cc.green << "Generating viewpoints\n" << cc.reset;
  }

  view_generator_->setCloud(mapping_module_->getPointCloud());
  view_generator_->setMap(mapping_module_->getOctomap());
  view_generator_->setMapPrediction(mapping_module_->getOctomapPredicted());
  view_generator_->setCurrentPose(vehicle_->getPose());
  view_generator_->generateViews();

  if (view_generator_->generated_poses.size() == 0)
  {
    std::cout << "[NBVLoop] " << cc.red << "View generator created no poses. Terminating.\n" << cc.reset;
    state = NBVState::TERMINATION_MET;
  }
  else
  {
    state = NBVState::VIEWPOINT_GENERATION_COMPLETE;
  }
}

void NBVLoop::evaluateViewpoints()
{
  if (state != NBVState::VIEWPOINT_EVALUATION)
  {
    std::cout << "[NBVLoop] " << cc.red << "ERROR: Attempt to evaluate viewpoints out of order\n" << cc.reset;
    return;
  }
  if (is_debug_states)
  {
    std::cout << "[NBVLoop] " << cc.green << "Evaluating viewpoints\n" << cc.reset;
  }

  // Evaluate viewpoints
  view_selecter_->evaluate();
  //view_selecter_comparison_->evaluate();

  // Update history
  updateHistory();


  // Move to next best view
  geometry_msgs::Pose p = view_selecter_->getTargetPose();
  if ( isnan(p.position.x) )
  {
    std::cout << "[NBVLoop] " << cc.red << "View selecter determined all poses are invalid. Terminating.\n" << cc.reset;
    state = NBVState::TERMINATION_MET;
    return;
  }

  vehicle_->setWaypoint(p);

  std::cout << "[NBVLoop] " << cc.green << "Done evaluating viewpoints\n" << cc.reset;

  state = NBVState::VIEWPOINT_EVALUATION_COMPLETE;
}

void NBVLoop::initMappingModule()
{
  mapping_module_ = new MappingModule();

  // Run in a new thread
  boost::thread mapping_thread_(&MappingModule::run, mapping_module_);
}

void NBVLoop::initModelProfiler()
{
  int profiling_method;
  double profiling_speed;

  ros::param::param("~profiling_method", profiling_method, 0);
  ros::param::param("~profiling_scan_speed", profiling_speed, 0.1);

  switch(profiling_method)
  {
  default:
  case 0:
    model_profiler_ = new ModelProfilerCircularAdaptive();
    break;
  case 1:
    model_profiler_ = new ModelProfilerBoundedBox();
    break;
  }

  model_profiler_->setScanSpeed(profiling_speed);
  model_profiler_->setVehicle(vehicle_);
  model_profiler_->setMappingModule(mapping_module_);
}

void NBVLoop::initParameters()
{
  // >>>>>>>>>>>>>>>>>
  // Fixed values
  // >>>>>>>>>>>>>>>>>
  is_terminating = false;
  iteration_count = 0;

  is_done_profiling  = false;

  // >>>>>>>>>>>>>>>>>
  // Read params
  // >>>>>>>>>>>>>>>>>
  // DEBUG
  ros::param::param("~debug_nbv_main_states", is_debug_states, false);


  // STATES
  ros::param::param("~profiling_skip", skip_profiling, false);
  ros::param::param("~profiling_skip_load_map", skip_profiling_load_map, true);

  // NAVIGATION
  ros::param::param("~nav_bounds_z_min", uav_height_min, 0.05);
  ros::param::param("~nav_bounds_z_max", uav_height_max, 10.0);
  ros::param::param("~uav_obstacle_distance_min", uav_obstacle_distance_min, 1.0);
}

void NBVLoop::initTerminationChecker()
{
  int termination_method;
  ros::param::param("~termination_type", termination_method, 0);

  switch(termination_method)
  {
  default:
  case 0:
    termination_check_module_ = new TerminationCheckMaxIterations();
    break;
  case 1:
    termination_check_module_ = new TerminationCheckGlobalEntropyPercentageDifference();
    break;
  case 2:
    termination_check_module_ = new TerminationCheckLocalEntropyPerVoxel();
    break;
  }

  termination_check_module_->setHistory(history_);
}

void NBVLoop::initVehicle()
{
  int vehicle_type;
  ros::param::param("~vehicle_type", vehicle_type, 0);

  switch(vehicle_type)
  {
  default:
  case 0:
    vehicle_ = new VehicleControlFloatingSensor();
    break;
  case 1:
    vehicle_ = new VehicleControlIris();
    break;
  }
}

void NBVLoop::initViewGenerator()
{
  int view_generator_method;
  ros::param::param("~view_generator_type", view_generator_method, 0);

  switch(view_generator_method)
  {
  default:
  case 0:
    view_generator_ = new ViewGeneratorNN();
    break;
  case 1:
    view_generator_ = new ViewGeneratorNNAdaptive();
    break;
  case 2:
    view_generator_ = new ViewGeneratorFrontier();
    break;
  case 3:
    view_generator_ = new ViewGeneratorNNFrontier();
    break;
  }

  view_generator_->setHistory(history_);

}

void NBVLoop::initViewSelecter()
{
  int view_selecter_method;
  ros::param::param("~view_selecter_type", view_selecter_method, 0);

  switch(view_selecter_method)
  {
  default:
  case 0:
    view_selecter_ = new ViewSelecterIg();
    break;
  case 1:
    view_selecter_ = new ViewSelecterIgExpDistance();
    break;
  case 2:
    view_selecter_ = new ViewSelecterSymmetryPrediction();
    break;
  case 3:
    view_selecter_ = new ViewSelecterPointDensity();
    break;
  }

  view_selecter_->setViewGenerator(view_generator_);


  // Another selecter to compare with
  if (view_selecter_method != 0)
    view_selecter_comparison_ = new ViewSelecterIg();
  else
    view_selecter_comparison_ = new ViewSelecterIgExpDistance();

  view_selecter_comparison_->setViewGenerator(view_generator_);

}

void NBVLoop::positionVehicleAfterProfiling()
{
  // Move vehicle up to the model after profiling
  if (is_debug_states)
  {
    std::cout << "[NBVLoop] " << cc.green << "Moving vehicle after profiling\n" << cc.reset;
  }
  // @todo
  std::cout << "[NBVLoop] " << cc.yellow << "Note: Moving vehicle after profiling uses a fixed waypoint defined in config settings. Create adaptive method.\n" << cc.reset;

  double x, y, z, yaw;
  ros::param::param<double>("~profiling_complete_pose_x", x, 0);
  ros::param::param<double>("~profiling_complete_pose_y", y, 0);
  ros::param::param<double>("~profiling_complete_pose_z", z, 10);
  ros::param::param<double>("~profiling_complete_pose_yaw", yaw, 0);

  vehicle_->setWaypoint(x, y, z, yaw);
  vehicle_->setSpeed(1.0);
  vehicle_->setSpeed(-1); //Allow instant teleportation if using the floating sensor. Ignored by other vehicles
  vehicle_->moveVehicle();

  // Do not process the termination condition, move to view generation
  state = NBVState::TERMINATION_NOT_MET;
  //state = NBVState::MOVING_COMPLETE;
}

void NBVLoop::profilingProcessing(){
  if (state != NBVState::PROFILING_PROCESSING)
  {
    std::cout << "[NBVLoop] " << cc.red << "ERROR: Attempt to start profiling out of order\n" << cc.reset;
    return;
  }
  if (is_debug_states)
  {
    std::cout << "[NBVLoop] " << cc.green << "Processing profile\n" << cc.reset;
  }

  bool done_profiling;
  done_profiling = model_profiler_->run(mapping_module_->getPointCloud());

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
          state = NBVState::PROFILING_COMPLETE;
        }
        break;

      case NBVState::PROFILING_PROCESSING:
        profilingProcessing();
        break;

      case NBVState::MOVING_COMPLETE:
        std::cout << "[NBVLoop] " << cc.magenta << "Requesting camera data\n" << cc.reset;

        ros::Duration(0.2).sleep(); // Sleep momentarily to allow tf to catch up for teleporting sensor
        mapping_module_->commandGetCameraData();

        state = NBVState::TERMINATION_CHECK;
        terminationCheck();

        break;

      case NBVState::TERMINATION_MET:
        is_terminating = true;
        std::cout << "[NBVLoop] " << cc.yellow << "Termination condition met\n" << cc.reset;

        // Save final maps
        mapping_module_->commandFinalMapSave();
        break;

      case NBVState::PROFILING_COMPLETE:
        positionVehicleAfterProfiling();
        break;

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
        state = NBVState::MOVING_COMPLETE;
        break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Node achieved termination condition, spin to continue publishing visualization data
  if (is_terminating)
  {
    bool should_spin;
    ros::param::param("~termination_ros_spin_afterwards", should_spin, true);

    if (should_spin)
      ros::spin();
  }

  return;
}

void NBVLoop::terminationCheck()
{
  if (state != NBVState::TERMINATION_CHECK)
  {
    std::cout << "[NBVLoop] " << cc.red << "ERROR: Attempt to check termination out of order\n" << cc.reset;
    return;
  }
  if (is_debug_states)
  {
    std::cout << "[NBVLoop] " << cc.green << "Checking termination condition\n" << cc.reset;
  }


  termination_check_module_->update();

  if (termination_check_module_->isTerminated())
    state = NBVState::TERMINATION_MET;
  else
    state = NBVState::TERMINATION_NOT_MET;
}

void NBVLoop::updateHistory()
{
  history_->selected_poses.push_back(view_selecter_->getTargetPose());
  history_->selected_utility.push_back(view_selecter_->info_utility_max_);
  history_->total_entropy.push_back(view_selecter_->info_entropy_total_);
  history_->update();
}
