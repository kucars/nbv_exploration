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
  pub_iteration_info = ros_node.advertise<nbv_exploration::IterationInfo>("nbv_exploration/iteration_info", 10);
}

ViewSelecterBase* NBVLoop::createViewSelecter(int type)
{
  ViewSelecterBase* v;
  switch(type)
  {
  default:
  case 0:
    v = new ViewSelecterIg();
    break;
  case 1:
    v = new ViewSelecterIgExpDistance();
    break;
  case 2:
    v = new ViewSelecterPointDensity();
    break;

  case 10:
    v = new ViewSelecterProposed();
    break;
  case 11:
    v = new ViewSelecterProposedRayLength();
    break;
  }

  return v;
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
  timer.start("[NBVLoop]Evaluate");
  view_selecter_->evaluate();
  timer.stop("[NBVLoop]Evaluate");

  if (is_view_selecter_compare)
  {
    timer.start("[NBVLoop]EvaluateComparison");
    view_selecter_comparison_->evaluate();
    timer.stop("[NBVLoop]EvaluateComparison");
  }

  // Set the next best view
  geometry_msgs::Pose p = view_selecter_->getTargetPose();
  if ( std::isnan(p.position.x) )
  {
    std::cout << "[NBVLoop] " << cc.red << "View selecter determined all poses are invalid. Terminating.\n" << cc.reset;
    state = NBVState::TERMINATION_MET;
    return;
  }
  vehicle_->setWaypoint(p);


  std::cout << "[NBVLoop] " << cc.green << "Done evaluating viewpoints\n" << cc.reset;
  state = NBVState::VIEWPOINT_EVALUATION_COMPLETE;
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

  view_generator_->setMappingModule(mapping_module_);
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

void NBVLoop::initAllModules(bool loaded_state)
{
  is_debug_load_state = loaded_state;

  // >>>>>>>>>>>>>>>>
  // Initialize modules
  // >>>>>>>>>>>>>>>>
  if (!loaded_state)
  {
    history_ = new NBVHistory();
    initMappingModule();
  }

  initViewGenerator();
  initViewSelecter();
  initVehicle();
  initModelProfiler();
  initTerminationChecker();

  if (loaded_state)
  {
    // Move to last recorded position
    geometry_msgs::Pose p = history_->selected_poses.back();
    vehicle_->setWaypoint(p);
    vehicle_->setSpeed(-1);
    vehicle_->moveVehicle();
  }

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
}

void NBVLoop::initMappingModule()
{
  mapping_module_ = new MappingModule();
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
  ros::param::param("~view_selecter_compare", is_view_selecter_compare, false);


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
  case 3:
    termination_check_module_ = new TerminationCheckUtilityThreshold();
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

  // Set starting position
  ros::Duration(2.0).sleep();

  int pose_number;
  std::string pose_number_str;
  ros::param::param<int>("~uav_start_pose", pose_number, 1);
  pose_number_str = std::to_string(pose_number);

  double x, y, z, yaw;
  ros::param::param<double>("~uav_pose_x_" + pose_number_str, x, 0);
  ros::param::param<double>("~uav_pose_y_" + pose_number_str, y, 0);
  ros::param::param<double>("~uav_pose_z_" + pose_number_str, z, 10);
  ros::param::param<double>("~uav_pose_yaw_" + pose_number_str, yaw, 0);

  vehicle_->setWaypoint(x, y, z, yaw);
  vehicle_->setSpeed(1.0);
  vehicle_->setSpeed(-1); //Allow instant teleportation if using the floating sensor. Ignored by other vehicles
  vehicle_->moveVehicle();
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
  int view_selecter_method, view_selecter_compare_method;
  ros::param::param("~view_selecter_type", view_selecter_method, 0);
  ros::param::param("~view_selecter_compare_type", view_selecter_compare_method, 0);

  view_selecter_ = createViewSelecter(view_selecter_method);
  view_selecter_->setViewGenerator(view_generator_);
  view_selecter_->setMappingModule(mapping_module_);

  // Another selecter to compare with
  view_selecter_comparison_ = createViewSelecter(view_selecter_compare_method);
  view_selecter_comparison_->setViewGenerator(view_generator_);
}

void NBVLoop::positionVehicleAfterProfiling()
{
  // Move vehicle up to the model after profiling
  if (is_debug_states)
  {
    std::cout << "[NBVLoop] " << cc.green << "Moving vehicle after profiling\n" << cc.reset;
  }
  std::cout << "[NBVLoop] " << cc.yellow << "Note: Moving vehicle after profiling uses a fixed waypoint defined in config settings. Create adaptive method.\n" << cc.reset;

  if (is_debug_load_state)
  {
    // Move to last recorded position
    geometry_msgs::Pose p = history_->selected_poses.back();
    vehicle_->setWaypoint(p);
  }
  else
  {
    int pose_number;
    std::string pose_number_str;
    ros::param::param<int>("~profiling_complete_pose_number", pose_number, 1);
    pose_number_str = std::to_string(pose_number);

    double x, y, z, yaw;
    ros::param::param<double>("~uav_pose_x_" + pose_number_str, x, 0);
    ros::param::param<double>("~uav_pose_y_" + pose_number_str, y, 0);
    ros::param::param<double>("~uav_pose_z_" + pose_number_str, z, 10);
    ros::param::param<double>("~uav_pose_yaw_" + pose_number_str, yaw, 0);

    vehicle_->setWaypoint(x, y, z, yaw);
  }

  vehicle_->setSpeed(1.0);
  vehicle_->setSpeed(-1); //Allow instant teleportation if using the floating sensor. Ignored by other vehicles
  vehicle_->moveVehicle();


  // Process the termination condition
  terminationCheck();
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

  // >>>>>>>>>>>>
  // Run mapping module in a new thread
  // >>>>>>>>>>>>
  boost::thread mapping_thread_(&MappingModule::run, mapping_module_);

  // >>>>>>>>>>>>
  // Main NBV Loop
  // >>>>>>>>>>>>
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
        // Update history
        updateHistory();

        timer.start("[NBVLoop]TerminationCheck");
        state = NBVState::TERMINATION_CHECK;
        terminationCheck();
        timer.stop("[NBVLoop]TerminationCheck");
        timer.stop("[NBVLoop]-Iteration");
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
        timer.start("[NBVLoop]-Iteration");
        state = NBVState::VIEWPOINT_GENERATION;
        timer.start("[NBVLoop]Generator");
        generateViewpoints();
        timer.stop("[NBVLoop]Generator");

        break;

      case NBVState::VIEWPOINT_GENERATION_COMPLETE:
        state = NBVState::VIEWPOINT_EVALUATION;
        evaluateViewpoints();

        break;

      case NBVState::VIEWPOINT_EVALUATION_COMPLETE:
        state = NBVState::MOVING;

        timer.start("[NBVLoop]Moving");
        vehicle_->moveVehicle(0.25); //Make sure we go to the exact position
        timer.start("[NBVLoop]Moving");


        timer.start("[NBVLoop]commandGetCameraData");
        std::cout << "[NBVLoop] " << cc.magenta << "Requesting camera data\n" << cc.reset;
        ros::Duration(0.15).sleep(); // Sleep momentarily to allow tf to catch up for teleporting sensor
        mapping_module_->commandGetCameraData();
        timer.stop("[NBVLoop]commandGetCameraData");

        state = NBVState::MOVING_COMPLETE;
        break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // >>>>>>>>>>>>>>>>>
  // Node achieved termination condition, spin to continue publishing visualization data
  // >>>>>>>>>>>>>>>>>
  if (is_terminating)
  {
    bool should_spin;
    ros::param::param("~termination_ros_spin_afterwards", should_spin, true);

    if (should_spin)
      ros::spin();
  }


  // >>>>>>>>>>>>>>>>>
  // Dump time data
  // >>>>>>>>>>>>>>>>>
  timer.dump();

  // >>>>>>>>>>>>>>>>>
  // Clean up
  // >>>>>>>>>>>>>>>>>
  std::cout << "[NBVLoop] " << cc.yellow << "Shutting down\n" << cc.reset;
  ros::shutdown();

  return;
}

void NBVLoop::terminationCheck()
{
  if (is_debug_states)
  {
    std::cout << "[NBVLoop] " << cc.green << "Checking termination condition for iteration " << history_->iteration << "\n" << cc.reset;
  }

  if (termination_check_module_->isTerminated())
    state = NBVState::TERMINATION_MET;
  else
    state = NBVState::TERMINATION_NOT_MET;
}

void NBVLoop::updateHistory()
{
  history_->selected_poses.push_back(view_selecter_->getTargetPose());
  history_->selected_utility.push_back(view_selecter_->info_selected_utility_);
  history_->selected_utility_entropy.push_back(view_selecter_->info_selected_utility_);
  history_->selected_utility_density.push_back(view_selecter_->info_selected_utility_density_);
  history_->selected_utility_prediction.push_back(view_selecter_->info_selected_utility_prediction_);
  history_->selected_utility_occupied_voxels.push_back(view_selecter_->info_selected_occupied_voxels_);

  history_->total_entropy.push_back(view_selecter_->info_entropy_total_);
  history_->avg_point_density.push_back(mapping_module_->getAveragePointDensity());
  history_->update();

  //printf("Time Total: %lf ms, Gen: %lf, Select: %lf, Mapping: %lf, Terminate: %lf\n", time_total_, time_view_generation_, time_view_selection_, time_mapping_, time_termination_);

  // Publish information about this iteration
  if (pub_iteration_info.getNumSubscribers() > 0)
  {
    nbv_exploration::IterationInfo iteration_msg;
    iteration_msg.iteration        = view_selecter_->info_iteration_;
    iteration_msg.distance_total   = view_selecter_->info_distance_total_;
    iteration_msg.entropy_total    = view_selecter_->info_entropy_total_;
    iteration_msg.point_density_avg= history_->avg_point_density.back();
    iteration_msg.method_generation= view_generator_->getMethodName();
    iteration_msg.method_selection = view_selecter_->getMethodName();
    iteration_msg.selected_pose    = view_selecter_->getTargetPose();
    iteration_msg.selected_utility                 = view_selecter_->info_selected_utility_;
    iteration_msg.selected_utility_density         = view_selecter_->info_selected_utility_density_;
    iteration_msg.selected_utility_distance        = view_selecter_->info_selected_utility_distance_;
    iteration_msg.selected_utility_entropy         = view_selecter_->info_selected_utility_entropy_;
    iteration_msg.selected_utility_prediction      = view_selecter_->info_selected_utility_prediction_;
    iteration_msg.selected_utility_occupied_voxels = view_selecter_->info_selected_occupied_voxels_;

    iteration_msg.time_iteration   = timer.getLatestTime("[NBVLoop]-Iteration");
    iteration_msg.time_generation  = timer.getLatestTime("[NBVLoop]Generator");
    iteration_msg.time_selection   = timer.getLatestTime("[NBVLoop]Evaluate");
    iteration_msg.time_mapping     = timer.getLatestTime("[NBVLoop]commandGetCameraData");
    iteration_msg.time_termination = timer.getLatestTime("[NBVLoop]commandGetCameraData");

    iteration_msg.utilities        = view_selecter_->info_utilities_;

    pub_iteration_info.publish(iteration_msg);

    if (is_view_selecter_compare)
    {
      iteration_msg.iteration        = view_selecter_comparison_->info_iteration_;
      iteration_msg.distance_total   = view_selecter_comparison_->info_distance_total_;
      iteration_msg.entropy_total    = view_selecter_comparison_->info_entropy_total_;
      iteration_msg.method_generation= view_selecter_comparison_->getMethodName();
      iteration_msg.method_selection = view_selecter_comparison_->getMethodName();
      iteration_msg.selected_pose    = view_selecter_comparison_->getTargetPose();
      iteration_msg.selected_utility                 = view_selecter_comparison_->info_selected_utility_;
      iteration_msg.selected_utility_density         = view_selecter_comparison_->info_selected_utility_density_;
      iteration_msg.selected_utility_distance        = view_selecter_comparison_->info_selected_utility_distance_;
      iteration_msg.selected_utility_entropy         = view_selecter_comparison_->info_selected_utility_entropy_;
      iteration_msg.selected_utility_prediction      = view_selecter_comparison_->info_selected_utility_prediction_;
      iteration_msg.selected_utility_occupied_voxels = view_selecter_comparison_->info_selected_occupied_voxels_;
      iteration_msg.utilities        = view_selecter_comparison_->info_utilities_;
      pub_iteration_info.publish(iteration_msg);
    }
  }
}
