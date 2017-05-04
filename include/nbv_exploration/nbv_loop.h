#ifndef NBV_EXPLORATION_MAIN_LOOP_H
#define NBV_EXPLORATION_MAIN_LOOP_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <octomap_msgs/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include "nbv_exploration/common.h"

#include "nbv_exploration/mapping_module.h"
#include "nbv_exploration/model_profiler_base.h"
#include "nbv_exploration/model_profiler_bounded_box.h"
#include "nbv_exploration/model_profiler_circular_adaptive.h"
#include "nbv_exploration/nbv_history.h"
#include "nbv_exploration/termination_check_base.h"
#include "nbv_exploration/termination_check_global_entropy_percent_diff.h"
#include "nbv_exploration/termination_check_local_entropy_per_voxel.h"
#include "nbv_exploration/termination_check_max_iterations.h"
#include "nbv_exploration/view_generator_base.h"
#include "nbv_exploration/view_generator_frontier.h"
#include "nbv_exploration/view_generator_adaptive_nn.h"
#include "nbv_exploration/view_generator_nn.h"
#include "nbv_exploration/view_selecter_base.h"
#include "nbv_exploration/view_selecter_ig.h"
#include "nbv_exploration/view_selecter_ig_exp_distance.h"
#include "nbv_exploration/view_selecter_symmetry_prediction.h"

#include "control/vehicle_control_base.h"
#include "control/vehicle_control_floating_sensor.h"
#include "control/vehicle_control_iris.h"

namespace NBVState
{
  enum State {
    INITIALIZING,
    IDLE,
    STARTING_ROBOT, STARTING_ROBOT_COMPLETE,
    PROFILING_PROCESSING, PROFILING_COMPLETE,
    TERMINATION_CHECK, TERMINATION_MET, TERMINATION_NOT_MET,
    VIEWPOINT_GENERATION, VIEWPOINT_GENERATION_COMPLETE,
    VIEWPOINT_EVALUATION, VIEWPOINT_EVALUATION_COMPLETE,
    PATH_PLANNING,
    MOVING, MOVING_COMPLETE
    };
}

class NBVLoop
{
public:
  /* =========
   * VARIABLES
   * ========= */
  // MODULES
  MappingModule*        mapping_module_;
  ModelProfilerBase*    model_profiler_;
  NBVHistory*           history_;
  TerminationCheckBase* termination_check_module_;
  VehicleControlBase*   vehicle_;
  ViewGeneratorBase*    view_generator_;
  ViewSelecterBase*     view_selecter_;
  ViewSelecterBase*     view_selecter_comparison_;

  // DEBUG
  bool is_debug_states;

  // STATE VARIABLES
  NBVState::State state;

  bool is_done_profiling;
  bool is_scan_empty;
  bool is_flying_up;

  bool skip_profiling;
  bool skip_profiling_load_map;

  // TERMINATION VARIABLES
  bool is_terminating;
  int iteration_count;
  int max_iterations;

  // NAVIGATION VARIABLES
  float distance_threshold;
  float angular_threshold;
  float linear_speed_threshold;
  float angular_speed_threshold;

  // FLIGHT VARIABLES
  double profile_angle;
  double uav_height_min, uav_height_max, uav_obstacle_distance_min;

  /* =========
   * METHODS
   * ========= */
  NBVLoop();
  void initParameters();

  void initMappingModule();
  void initModelProfiler();
  void initTerminationChecker();
  void initVehicle();
  void initViewGenerator();
  void initViewSelecter();
  void runStateMachine();
  void sigIntHandler(int sig);

  // FSM functions
  void terminationCheck();
  void generateViewpoints();
  void evaluateViewpoints();
  void positionVehicleAfterProfiling();
  void profilingProcessing();
  void updateHistory();

  double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
  double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
};

#endif // end include
