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
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <nbv_exploration/MappingSrv.h>
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
#include <nbv_exploration/view_generator.h>
#include <nbv_exploration/view_selecter.h>

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
bool is_debug_callbacks = !true;

bool is_done_profiling = false;
bool is_scan_empty = false;
bool is_flying_up = false;
double profile_angle = 0;

// == Consts
std::string topic_depth         = "/iris/xtion_sensor/iris/xtion_sensor_camera/depth/points";
std::string topic_position      = "/iris/ground_truth/pose";
std::string topic_octree        = "/nbv_exploration/output_tree";
std::string topic_scan_cloud    = "/nbv_exploration/scan_cloud";
std::string topic_profile_cloud = "/nbv_exploration/profile_cloud";

// == Termination variables
bool is_terminating = false;
int iteration_count = 0;
int max_iterations = 40;

// == Navigation variables
float distance_threshold = 0.5f;
float angular_threshold = 10.0 * M_PI/180.0;//Degrees to radians

geometry_msgs::Pose mobile_base_pose;
geometry_msgs::PoseStamped setpoint;


// == Viewpoint variables
ViewGeneratorBase* viewGen;
ViewSelecterBase* viewSel;


// == Publishers / Clients
ros::Publisher pub_global_cloud;
ros::Publisher pub_setpoint;
ros::Publisher pub_scan_command;
ros::ServiceClient srvclient_mapping;


// == Point clouds
float grid_res = 0.1f; //Voxel grid resolution

pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_projected_cloud_ptr  (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::VoxelGrid<pcl::PointXYZRGB> occGrid;
//octomap::OcTree* global_octomap;
//octomap::OcTreeBase<octomap::OcTreeNode>* global_octomap;
octomap::OcTree* global_octomap;

// ======================
// Function prototypes (@todo: move to header file)
// ======================
// Callbacks
//void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void callbackScan(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
//void callbackProfile(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void callbackPosition(const geometry_msgs::PoseStamped& pose_msg);
void callbackOctomap(const octomap_msgs::Octomap& octomap_msg);

bool callMappingService(int command);

// FSM functions
void profilingProcessing();
void profileMove(bool is_rising);

void terminationCheck();
void generateViewpoints();
void evaluateViewpoints();
void setWaypoint(double x, double y, double z, double yaw, bool is_relative);
void setWaypoint(geometry_msgs::Pose p);
void moveVehicle(double threshold_sensitivity = 1);
void takeoff();

double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
bool isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current, double threshold_sensitivity = 1);
bool isNear(double p1, double p2, double threshold_sensitivity = 1);

void transformSetpoint2Global (const geometry_msgs::Pose & p_set, geometry_msgs::Pose & p_global);
void transformGlobal2Setpoint (const geometry_msgs::Pose & p_global, geometry_msgs::Pose & p_set);

// ======================
// Create a state machine
// ======================
namespace NBVState
{
  enum State {
    NOT_STARTED,
    INITIALIZING,
    IDLE,
    TAKING_OFF,
    PROFILING_PROCESSING, PROFILING_DONE,
    TERMINATION_CHECK, TERMINATION_MET, TERMINATION_NOT_MET,
    VIEWPOINT_GENERATION, VIEWPOINT_GENERATION_COMPLETE, 
    VIEWPOINT_EVALUATION, VIEWPOINT_EVALUATION_COMPLETE,
    PATH_PLANNING,
    MOVING, MOVING_COMPLETE
    };
}
NBVState::State state = NBVState::NOT_STARTED;

// ================
// Functions
// ================

double randomDouble(double min, double max)
{
  return ((double) random()/RAND_MAX)*(max-min) + min;
}

void sigIntHandler(int sig)
{
  std::cout << cc_yellow << "Handling SIGINT exception\n" << cc_reset;
  
  // Forces ros::Rate::sleep() to end if it's stuck (for example, Gazebo isn't running)
  ros::shutdown();
}

int main(int argc, char **argv)
{
  // >>>>>>>>>>>>>>>>>
  // Initialize ROS
  // >>>>>>>>>>>>>>>>>
  state = NBVState::INITIALIZING;
  
  ROS_INFO("nbv_loop: BEGIN NBV LOOP");

  /* Override SIGINT handler */
  ros::init(argc, argv, "nbv_loop", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);
  
  ros::NodeHandle ros_node;


  // >>>>>>>>>>>>>>>>>
  // Subscribers
  // >>>>>>>>>>>>>>>>>
  
  // Sensor data
  ros::Subscriber sub_pose      = ros_node.subscribe(topic_position, 1, callbackPosition);
  ros::Subscriber sub_scan      = ros_node.subscribe(topic_scan_cloud, 1, callbackScan);
  //ros::Subscriber sub_profile   = ros_node.subscribe(topic_profile_cloud, 1, callbackProfile);
  ros::Subscriber sub_octomap   = ros_node.subscribe(topic_octree, 1, callbackOctomap);
  
  // >>>>>>>>>>>>>>>>>
  // Publishers / Clients
  // >>>>>>>>>>>>>>>>>
  
  // Drone setpoints
  pub_setpoint      = ros_node.advertise<geometry_msgs::PoseStamped>("/iris/mavros/setpoint_position/local", 10);
  srvclient_mapping = ros_node.serviceClient<nbv_exploration::MappingSrv>("nbv_exploration/mapping_command");
  
  

  // >>>>>>>>>>>>>>>>>
  // Parse Inputs
  // >>>>>>>>>>>>>>>>>  
  for (int n = 1; n < argc; n++)
  {
    if (std::string(argv[n]) == "-s") //Skip profiling
    {
      is_done_profiling = true;
      std::cout << "Skipping profiling\n";
      
      bool success = callMappingService(nbv_exploration::MappingSrv::Request::LOAD_MAP);
      if (!success)
      {
        std::cout << cc_red << "Failed to load profile\n" << cc_reset;
        ros::shutdown();
        return 1;
      }
    }
  }
  
  if (argc == 1)
  {
    bool success = callMappingService(nbv_exploration::MappingSrv::Request::START_PROFILING);
    
    if (!success)
    {
      std::cout << cc_red << "Failed to start profiler\n" << cc_reset;
      ros::shutdown();
      return 1;
    }
  }
  
  // >>>>>>>>>>>>>>>>
  // Set up viewpoint generator
  // >>>>>>>>>>>>>>>>
  //viewGen = new ViewGenerator_Frontier();
  viewGen = new ViewGeneratorNN();
  viewGen->setResolution(1.0, 1.0, 1.0, M_PI_4);

  viewSel = new ViewSelecterBase();
  viewSel->setViewGenerator(viewGen);

  // >>>>>>>>>>>>>>>>>
  // Start the FSM
  // >>>>>>>>>>>>>>>>>
  ROS_INFO("nbv_loop: Ready to take off. Waiting for current position information.");
  state = NBVState::TAKING_OFF;
  
  ros::Rate loop_rate(10);
  
  while (ros::ok() && !is_terminating)
  {
    switch(state)
    {
      case NBVState::TAKING_OFF:
        // Wait till we have current location to properly set waypoint for takeoff
        if( mobile_base_pose.orientation.x == 0 &&
          mobile_base_pose.orientation.y == 0 &&
          mobile_base_pose.orientation.z == 0 &&
          mobile_base_pose.orientation.w == 0)
        {
          break;
        }
          
        takeoff();
        break;
        
      case NBVState::IDLE:
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
        state = NBVState::TERMINATION_CHECK;
      
        iteration_count++;
        ROS_INFO("nbv_loop: Iteration %d", iteration_count);
        //ROS_INFO_STREAM("nbv_loop: " << console::COLOR_YELLOW << "Iteration " << iteration_count);
         
        terminationCheck();
  
        break;
        
      case NBVState::TERMINATION_MET:
        is_terminating = true;
        std::cout << cc_yellow << "Termination condition met\n" << cc_reset;
        break;
        
      case NBVState::PROFILING_DONE:
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
        moveVehicle();
        break;
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << cc_yellow << "Shutting down\n" << cc_reset;
  ros::shutdown();
  return 0;
}


// Update global position of UGV
void callbackPosition(const geometry_msgs::PoseStamped& pose_msg)
{
  if (is_debug && is_debug_callbacks)
  {
    std::cout << cc_magenta << "Grabbing location\n" << cc_reset;
  }
  
  // Save UGV pose
  mobile_base_pose = pose_msg.pose;
}


void callbackOctomap(const octomap_msgs::Octomap& octomap_msg)
{
  if (is_debug && is_debug_callbacks)
  {
    std::cout << cc_magenta << "Grabbing octree\n" << cc_reset;
  }
  
  global_octomap = static_cast<octomap::OcTree* >( octomap_msgs::fullMsgToMap(octomap_msg) );
}


void callbackScan(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  if (is_debug && is_debug_callbacks)
  {
    std::cout << cc_magenta << "Grabbing profile point cloud\n" << cc_reset;
  }
  
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  // == Convert to pcl pointcloud
  pcl::fromROSMsg (*cloud_msg, cloud);
  profile_cloud_ptr = cloud.makeShared();
}

bool callMappingService(int command)
{
  nbv_exploration::MappingSrv srv;
  srv.request.data = command;
  
  bool success = srvclient_mapping.call(srv);
  bool error = false;
  
  if (success)
  {
    if (srv.response.data == nbv_exploration::MappingSrv::Response::ERROR)
    {
      error = true;
    }
    else
    {
      return true; //success
    }
  }
  else
  {
    ROS_ERROR("Failed to call service \"mapping_command\"");
    return false; //failure
  }
  
  if (error)
  {
    std::string cmd_name;
    switch (command)
    {
      case nbv_exploration::MappingSrv::Request::START_SCANNING:
        cmd_name = "START_SCANNING";
        break;
      case nbv_exploration::MappingSrv::Request::STOP_SCANNING:
        cmd_name = "STOP_SCANNING";
        break;
      case nbv_exploration::MappingSrv::Request::START_PROFILING:
        cmd_name = "START_PROFILING";
        break;
      case nbv_exploration::MappingSrv::Request::STOP_PROFILING:
        cmd_name = "STOP_PROFILING";
        break;
      case nbv_exploration::MappingSrv::Request::SAVE_MAP:
        cmd_name = "SAVE_MAP";
        break;
      case nbv_exploration::MappingSrv::Request::LOAD_MAP:
        cmd_name = "LOAD_MAP";
        break;
      default:
        cmd_name = "UNKNOWN";
        break;
    }
    
    std::cout << cc_red << "Error after sending " <<  cmd_name << "(ID: " << command << ") to service \"mapping_commands\" \n";
    std::cout << cc_red << "\t" << srv.response.error << "\n" << cc_reset;
    return false;
  }
  
  return true;//So the compiler doesn't complain
}



void profilingProcessing(){
  if (state != NBVState::PROFILING_PROCESSING)
  {
    std::cout << cc_red << "ERROR: Attempt to start profiling out of order\n" << cc_reset;
    return;
  }
  if (is_debug && is_debug_states)
  {
    std::cout << cc_green << "Processing profile\n" << cc_reset;
  }
    
  double angle_inc = (2*M_PI)/5;
  
  profile_angle += angle_inc;
  
  if (profile_angle >= 2*M_PI + angle_inc){
    state = NBVState::PROFILING_DONE;
    std::cout << cc_green << "Profiling complete\n" << cc_reset;
    
    callMappingService(nbv_exploration::MappingSrv::Request::STOP_PROFILING);
    
    return;
  }
  
  is_flying_up = !is_flying_up;
  if (is_flying_up)
  {
    profileMove(true);
  }
  else
  {
    profileMove(false);
  }
  
  // Calculate centroid
  if (profile_cloud_ptr->points.size() == 0){
    std::cout << cc_red << "Error: No points detected in profile cloud. Make sure mapping node is running\n" << cc_reset;
    return;
  }
  
  double x, y;
  
  for (int i=0; i<profile_cloud_ptr->points.size(); i++)
  {
    x += profile_cloud_ptr->points[i].x;
    y += profile_cloud_ptr->points[i].y;
  }
  x /= profile_cloud_ptr->points.size();
  y /= profile_cloud_ptr->points.size();
  
  // Calculate radius (top n point to remove outliers)
  double r = 0;
    
  
  for (int i=0; i<profile_cloud_ptr->points.size(); i++)
  {
    double r_temp = (profile_cloud_ptr->points[i].x - x)*(profile_cloud_ptr->points[i].x - x)
                  + (profile_cloud_ptr->points[i].y - y)*(profile_cloud_ptr->points[i].y - y);
    
    if (r_temp > r)
      r = r_temp;
  }
  
  r = sqrt(r);
  r += 2; // add a safety margin
  
  std::cout << cc_magenta << "x = " << x << "\ty = " << y << "\tr = " << r << "\n" << cc_reset;
  
  // Move back on the circle
  double theta  = atan2(mobile_base_pose.position.y - y, mobile_base_pose.position.x - x);
  double x_move = x+r*cos(theta);
  double y_move = y+r*sin(theta);
  double yaw_move = theta-M_PI; //towards the center of the circle
  
  std::cout << cc_magenta << "Moving back\n" << cc_reset;
  setWaypoint(x_move, y_move, mobile_base_pose.position.z, yaw_move, false);
  moveVehicle();
  
  // Move along the circle
  double arc_length = r * angle_inc;
  double step_length = 7.0;
  int steps = arc_length/step_length;
  
  double z_move = mobile_base_pose.position.z;
  
  if (steps < 2)
  {
    theta  += angle_inc;
    x_move = x+r*cos(theta);
    y_move = y+r*sin(theta);
    yaw_move = theta-M_PI; //towards the center of the circle
    
    std::cout << cc_magenta << "Moving along\n" << cc_reset;
    setWaypoint(x_move, y_move, z_move, yaw_move, false);
    moveVehicle(1.5);
  }
  else
  {
    double step_angle = angle_inc / steps;
    for (int i=0; i<steps; i++)
    {
      theta  += step_angle;
      x_move = x+r*cos(theta);
      y_move = y+r*sin(theta);
      yaw_move = theta-M_PI; //towards the center of the circle
      
      std::cout << cc_magenta << "Moving along\n" << cc_reset;
      setWaypoint(x_move, y_move, z_move, yaw_move, false);
      moveVehicle(1.5); // Be less picky about position, just move around quickly
    }
  }
  
}


void profileMove(bool is_rising)
{
  // ==Request START_SCANNING
  callMappingService(nbv_exploration::MappingSrv::Request::START_SCANNING);
  
  
  // ==Scan
  if (is_rising)
  {
    std::cout << cc_magenta << "Profiling move up\n" << cc_reset;
    
    //while (ros::ok() && !is_scan_empty)
    while (ros::ok() && mobile_base_pose.position.z < 10)
    {
      setWaypoint(0, 0, 0.3, 0, true);
      moveVehicle(0.25); // Scan slowly
      ros::spinOnce();
    }
  }
  else
  {
    std::cout << cc_magenta << "Profiling move down\n" << cc_reset;
    
    while (ros::ok() && mobile_base_pose.position.z > 0.7)
    {
      setWaypoint(0, 0, -0.3, 0, true);
      moveVehicle(0.25);
      ros::spinOnce();
    }
  }
  
  // ==Request STOP_SCANNING
  callMappingService(nbv_exploration::MappingSrv::Request::STOP_SCANNING);
  if (callMappingService(nbv_exploration::MappingSrv::Request::SAVE_MAP))
  {
    std::cout << "Successfully saved profile\n";
  }
  else
  {
    std::cout << cc_red << "Failed to save profile\n";
  }
}


void terminationCheck()
{
  if (state != NBVState::TERMINATION_CHECK)
  {
    std::cout << cc_red << "ERROR: Attempt to check termination out of order\n" << cc_reset;
    return;
  }
  if (is_debug && is_debug_states)
  {
    std::cout << cc_green << "Checking termination condition\n" << cc_reset;
  }
  
  
  if (iteration_count > max_iterations)
  {
    state = NBVState::TERMINATION_MET;
  }
  else
  {
    state = NBVState::TERMINATION_NOT_MET;
  }
}


void generateViewpoints()
{
  if (state != NBVState::VIEWPOINT_GENERATION)
  {
    std::cout << cc_red << "ERROR: Attempt to generate viewpoints out of order\n" << cc_reset;
    return;
  }
  if (is_debug && is_debug_states)
  {
    std::cout << cc_green << "Generating viewpoints\n" << cc_reset;
  }
  
  ros::Rate rate(10);
  while (ros::ok() && !profile_cloud_ptr)
  {
    ROS_INFO_ONCE("Waiting for point cloud profile");
    
    ros::spinOnce();
    rate.sleep();
  }
  
  while (ros::ok() && !global_octomap)
  {
    ROS_INFO_ONCE("Waiting for octree profile");
    
    ros::spinOnce();
    rate.sleep();
  }
  
  viewGen->setCloud(profile_cloud_ptr);
  viewGen->setMap(global_octomap);
  viewGen->setCurrentPose(mobile_base_pose);
  viewGen->generateViews();
  
  state = NBVState::VIEWPOINT_GENERATION_COMPLETE;
}


void evaluateViewpoints()
{
  if (state != NBVState::VIEWPOINT_EVALUATION)
  {
    std::cout << cc_red << "ERROR: Attempt to evaluate viewpoints out of order\n" << cc_reset;
    return;
  }
  if (is_debug && is_debug_states)
  {
    std::cout << cc_green << "Evaluating viewpoints\n" << cc_reset;
  }
  
  viewSel->evaluate();
  geometry_msgs::Pose p = viewSel->getTargetPose();
  setWaypoint(p);
  
  state = NBVState::VIEWPOINT_EVALUATION_COMPLETE;
}

void setWaypoint(double x, double y, double z, double yaw, bool is_relative=false){
  geometry_msgs::Pose setpoint_world;
  
  if (is_relative)
  {
    // Position
    setpoint_world.position.x = mobile_base_pose.position.x + x;
    setpoint_world.position.y = mobile_base_pose.position.y + y;
    setpoint_world.position.z = mobile_base_pose.position.z + z;
    
    // Orientation
    double yaw_current = getYawFromQuaternion(mobile_base_pose.orientation);
    setpoint_world.orientation = getQuaternionFromYaw(yaw_current + yaw);
  }
  else
  {
    // Position
    setpoint_world.position.x = x;
    setpoint_world.position.y = y;
    setpoint_world.position.z = z;
    
    // Orientation
    setpoint_world.orientation = getQuaternionFromYaw(yaw);
  }
  
  // Transform to setpoint frame
  transformGlobal2Setpoint(setpoint_world, setpoint.pose);
}

void setWaypoint(geometry_msgs::Pose p)
{
  // Transform to setpoint frame
  transformGlobal2Setpoint(p, setpoint.pose);
}

void moveVehicle(double threshold_sensitivity)
{
  if (state != NBVState::MOVING
      && state != NBVState::TAKING_OFF
      && state != NBVState::PROFILING_PROCESSING)
  {
    std::cout << cc_red << "ERROR: Attempt to move vehicle out of order\n" << cc_reset;
    return;
  }
  if (is_debug && is_debug_states)
  {
    std::cout << cc_green << "Moving (setting waypoints)\n" << cc_reset;
  }
  
  
  // Publish pose (http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)
  setpoint.header.frame_id = "base_footprint";
  setpoint.header.stamp = ros::Time::now();
  pub_setpoint.publish(setpoint);
  
  
  // Convert setpoint to world frame
  geometry_msgs::Pose setpoint_world;
  transformSetpoint2Global (setpoint.pose, setpoint_world);
  
  // Wait till we've reached the waypoint
  ros::Rate rate(10);
  while(ros::ok() && !isNear(setpoint_world, mobile_base_pose, threshold_sensitivity))
  {
    if (is_debug && is_debug_callbacks)
    {
      std::cout << cc_green << "Moving to destination. " <<
        "Distance to target: " << getDistance(setpoint_world, mobile_base_pose) <<
        "\tAngle to target: " << getAngularDistance(setpoint_world, mobile_base_pose) << "\n" << cc_reset;
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  
  if (state == NBVState::MOVING)
  {
    std::cout << cc_green << "Done moving\n" << cc_reset;
    state = NBVState::MOVING_COMPLETE;
  }
}


void takeoff()
{
  if (state != NBVState::TAKING_OFF)
  {
    std::cout << cc_red << "ERROR: Attempt to take off out of order\n" << cc_reset;
    return;
  }
  
  std::cout << cc_green << "Taking off\n" << cc_reset;
  
  if (mobile_base_pose.position.z < 0.5)
  {
    setWaypoint(0, 0, 1, 0, true);
    moveVehicle();
  }
  
  std::cout << cc_green << "Done taking off\n" << cc_reset;
  
  state = NBVState::IDLE;
}

bool isNear(double p1, double p2, double threshold_sensitivity )
{
  if (fabs(p1-p2)< distance_threshold*threshold_sensitivity)
  {
    return true;
  }
    
  return false;
}

bool isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current, double threshold_sensitivity){
  if (
    getDistance(p_target, p_current) < distance_threshold*threshold_sensitivity &&
    fabs(getAngularDistance(p_target, p_current)) < angular_threshold*threshold_sensitivity )
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
