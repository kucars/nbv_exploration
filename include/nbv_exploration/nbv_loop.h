#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <octomap_msgs/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include "nbv_exploration/model_profiler_base.h"
#include "nbv_exploration/model_profiler_bounded_box.h"
#include "nbv_exploration/model_profiler_circular_adaptive.h"
#include "nbv_exploration/view_generator.h"
#include "nbv_exploration/view_selecter.h"
#include "nbv_exploration/termination_check_base.h"

#include "control/vehicle_control_base.h"
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
  ModelProfilerBase * model_profiler_;
  TerminationCheckBase * termination_check_module_;
  ViewGeneratorBase* view_generator_;
  ViewSelecterBase* view_selecter_;
  VehicleControlBase * vehicle_;

  // DEBUG
  bool is_debug;
  bool is_debug_states;
  bool is_debug_callbacks;

  // STATE VARIABLES
  NBVState::State state;

  bool is_done_profiling;
  bool is_scan_empty;
  bool is_flying_up;

  bool waiting_for_profile_cloud;
  bool waiting_for_profile_octomap;

  bool skip_profiling;
  bool skip_profiling_load_map;

  // CALLBACK VARIABLES
  std::string topic_depth;
  std::string topic_octree;
  std::string topic_scan_cloud;
  std::string topic_profile_cloud;

  ros::Publisher pub_global_cloud;
  ros::Publisher pub_scan_command;
  ros::ServiceClient srvclient_mapping;

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

  // MAP VARIABLES
  float grid_res; //Voxel grid resolution

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_projected_cloud_ptr;
  octomap::OcTree* global_octomap;



  /* =========
   * METHODS
   * ========= */
  NBVLoop();
  void initParameters();

  void initModelProfiler();
  void initViewGenerator();
  void initViewSelecter();
  void initVehicle();
  void runStateMachine();
  void sigIntHandler(int sig);

  // CALLBACKS
  void callbackScan(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
  void callbackOctomap(const octomap_msgs::Octomap& octomap_msg);

  // FSM functions
  void terminationCheck();
  void generateViewpoints();
  void evaluateViewpoints();
  void profilingProcessing();

  double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
  double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
};

