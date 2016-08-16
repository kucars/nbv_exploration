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
#include <gazebo_msgs/ModelStates.h> //Used for absolute positioning

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


#include <octomap/octomap.h>
#include <octomap/OcTree.h>

/*
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>
*/

/*
#include <math.h>
#include <cmath>
#include <deque>
#include <cstdlib>



#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/PointCloud.h>

#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>

#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



//#include <voxel_grid_occlusion_estimation.h>
#include <component_test/occlusion_culling.h>
#include "fcl_utility.h"
using namespace fcl;


geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);
double rad2deg (double rad);
double getDistanceXY(pcl::PointXYZ p1, pcl::PointXYZ p2);

std::string modelPath;
int maxIterations;
float initial_x;
float initial_y;
float initial_z;
float initial_yaw;
float voxelRes;
*/

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
bool isDebug = true; //Set to true to see debug text
bool isDebugStates = true;
bool isDebugContinuousStates = !true;

// == Consts
std::string depth_topic   = "/iris/xtion_sensor/iris/xtion_sensor_camera/depth/points";
std::string position_topic = "/iris/ground_truth/pose";

// == Termination variables
bool isTerminating = false;
int iteration_count = 0;
int max_iterations = 40;

// == Navigation variables
float distance_threshold = 0.5f;
float angular_threshold = 10.0 * M_PI/180.0;//Degrees to radians

geometry_msgs::Pose mobile_base_pose;
//geometry_msgs::Pose mobile_base_pose_prev;
geometry_msgs::PoseStamped setpoint;

// == Publishers
ros::Publisher pub_global_cloud;
ros::Publisher pub_setpoint;

// == Subsctiptions
tf::TransformListener *listener;


// == Point clouds
float grid_res = 0.1f; //Voxel grid resolution

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sensed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloudPtr;
//pcl::VoxelGrid<pcl::PointXYZRGB> occGrid;

// Create octree
octomap::OcTree tree (grid_res); // create empty tree with resolution 0.1



// ======================
// Function prototypes (@todo: move to header file)
// ======================

void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void positionCallback(const geometry_msgs::PoseStamped& pose_msg);
void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
void termination_check();
void generate_viewpoints();
void evaluate_viewpoints();
void set_waypoint(double x, double y, double z, double yaw);
void move_vehicle();
void takeoff();

double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
bool isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current);
bool isNear(double p1, double p2);

void transformSetpoint2Global (const geometry_msgs::Pose & p_set, geometry_msgs::Pose & p_global);
void transformGlobal2Setpoint (const geometry_msgs::Pose & p_global, geometry_msgs::Pose & p_set);

// ======================
// Create a state machine
// ======================
enum NBV_STATE {
    NBV_STATE_NOT_STARTED,
    NBV_STATE_INITIALIZING,
    NBV_STATE_IDLE,
    NBV_STATE_TAKING_OFF,
    NBV_STATE_TERMINATION_CHECK, NBV_STATE_TERMINATION_MET, NBV_STATE_TERMINATION_NOT_MET,
    NBV_STATE_VIEWPOINT_GENERATION, NBV_STATE_DONE_VIEWPOINT_GENERATION, 
    NBV_STATE_VIEWPOINT_EVALUATION, NBV_STATE_DONE_VIEWPOINT_EVALUATION,
    NBV_STATE_PATH_PLANNING,
    NBV_STATE_MOVING, NBV_STATE_DONE_MOVING
    };
NBV_STATE state = NBV_STATE_NOT_STARTED;

// ================
// Functions
// ================

double randomDouble(double min, double max) {
    return ((double) random()/RAND_MAX)*(max-min) + min;
}

/*
double rad2deg (double rad) {
    return rad*180/M_PI;
}

double getDistanceXY(pcl::PointXYZ p1, pcl::PointXYZ p2){
	return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );
}
*/


int main(int argc, char **argv)
{
    // >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    state = NBV_STATE_INITIALIZING;
    
    std::cout << cc_red << "BEGIN NBV LOOP\n" << cc_reset;

    ros::init(argc, argv, "nbv_loop");
    ros::NodeHandle ros_node;

    // >>>>>>>>>>>>>>>>>
    // Subscribers
    // >>>>>>>>>>>>>>>>>
    
    // Sensor data
    ros::Subscriber sub_kinect = ros_node.subscribe(depth_topic, 1, depthCallback);
    ros::Subscriber sub_pose = ros_node.subscribe(position_topic, 1, positionCallback);
    
    listener = new tf::TransformListener();
    
    // >>>>>>>>>>>>>>>>>
    // Publishers
    // >>>>>>>>>>>>>>>>>
    
    pub_global_cloud = ros_node.advertise<sensor_msgs::PointCloud2>("/global_cloud", 10);
    //pub = ros_node.advertise<sensor_msgs::PointCloud2> ("/voxgrid", 1);
    //pub_pose = ros_node.advertise<geometry_msgs::PoseStamped> ("/voxgrid/pose", 1);

    // Drone setpoints
    pub_setpoint = ros_node.advertise<geometry_msgs::PoseStamped>("/iris/mavros/setpoint_position/local", 10);

    // >>>>>>>>>>>>>>>>>
    // Start the FSM
    // >>>>>>>>>>>>>>>>>
    state = NBV_STATE_TAKING_OFF;
    
    ros::Rate loop_rate(10);
    while (ros::ok() && !isTerminating)
    {
        switch(state){
            case NBV_STATE_TAKING_OFF:
                // Wait till we have current location to properly set waypoint for takeoff
                if(mobile_base_pose.orientation.x == 0 &&
                    mobile_base_pose.orientation.y == 0 &&
                    mobile_base_pose.orientation.z == 0 &&
                    mobile_base_pose.orientation.w == 0){
                        break;
                    }
                takeoff();
                break;
                
            case NBV_STATE_IDLE:
            case NBV_STATE_DONE_MOVING:
                state = NBV_STATE_TERMINATION_CHECK;
                
                iteration_count++;
                std::cout << cc_yellow << "Iteration: " << iteration_count << "\n" << cc_reset;
                
                termination_check();
                break;
                
            case NBV_STATE_TERMINATION_MET:
                isTerminating = true;
                std::cout << cc_yellow << "Termination condition met\n" << cc_reset;
                break;
                
            case NBV_STATE_TERMINATION_NOT_MET:
                state = NBV_STATE_VIEWPOINT_GENERATION;
                generate_viewpoints();
                break;
                
            case NBV_STATE_DONE_VIEWPOINT_GENERATION:
                state = NBV_STATE_VIEWPOINT_EVALUATION;
                evaluate_viewpoints();
                break;
            
            case NBV_STATE_DONE_VIEWPOINT_EVALUATION:
                state = NBV_STATE_MOVING;
                move_vehicle();
                break;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

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
    
    /*
    // Save UGV pose
    mobile_base_pose_prev = mobile_base_pose;
    mobile_base_pose = pose_msg.pose[index];

    // Publish
    geometry_msgs::PoseStamped ps;
    ps.pose = mobile_base_pose;
    ps.header.frame_id = "base_link";
    ps.header.stamp = ros::Time::now();

    pub_pose.publish(ps);
    */
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
        listener->lookupTransform("world", "iris/xtion_sensor/camera_depth_optical_frame", ros::Time(0), transform);
        
        // == Convert tf:Transform to Eigen::Matrix4d
        Eigen::Matrix4d tf_eigen;
        
        Eigen::Vector3d T1(
            transform.getOrigin().x(),
            transform.getOrigin().y(),
            transform.getOrigin().z()
        );
        
        Eigen::Matrix3d R;
        tf::Quaternion qt = transform.getRotation();
        tf::Matrix3x3 R1(qt);
        tf::matrixTFToEigen(R1,R);
        
        // Set
        tf_eigen.setZero ();
        tf_eigen.block (0, 0, 3, 3) = R;
        tf_eigen.block (0, 3, 3, 1) = T1;
        tf_eigen (3, 3) = 1;
        
        // == Transform point cloud
        pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, tf_eigen);
        
        // == Add filtered to global
        addToGlobalCloud(cloud_filtered);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}


void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
    if (isDebug && isDebugContinuousStates){
        std::cout << cc_green << "MAPPING\n" << cc_reset;
    }
    
    // Initialize global cloud if not already done so
    if (!globalCloudPtr){
        globalCloudPtr = cloud_in;
        return;
    }

    *globalCloudPtr += *cloud_in;
    
    

    // Perform voxelgrid filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (globalCloudPtr);
    sor.setLeafSize (grid_res, grid_res, grid_res);
    sor.filter (*cloud_filtered);

    globalCloudPtr = cloud_filtered;
    
    if (isDebug && isDebugContinuousStates){
        std::cout << cc_blue << "Number of points in global map: " << globalCloudPtr->points.size() << "\n" << cc_reset;
    }
    
    
    // Publish
    sensor_msgs::PointCloud2 cloud_msg;
    
    pcl::toROSMsg(*globalCloudPtr, cloud_msg); 	//cloud of original (white) using original cloud
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    
    pub_global_cloud.publish(cloud_msg);
}


void termination_check(){
    if (state != NBV_STATE_TERMINATION_CHECK){
        std::cout << cc_red << "ERROR: Attempt to check termination out of order\n" << cc_reset;
        return;
    }
    if (isDebug && isDebugStates){
        std::cout << cc_green << "Checking termination condition\n" << cc_reset;
    }
    
    
    if (iteration_count > max_iterations){
        state = NBV_STATE_TERMINATION_MET;
    }
    else{
        state = NBV_STATE_TERMINATION_NOT_MET;
    }
}


void generate_viewpoints(){
    if (state != NBV_STATE_VIEWPOINT_GENERATION){
        std::cout << cc_red << "ERROR: Attempt to generate viewpoints out of order\n" << cc_reset;
        return;
    }
    if (isDebug && isDebugStates){
        std::cout << cc_green << "Generating viewpoints\n" << cc_reset;
    }
    
    state = NBV_STATE_DONE_VIEWPOINT_GENERATION;
}


void evaluate_viewpoints(){
    if (state != NBV_STATE_VIEWPOINT_EVALUATION){
        std::cout << cc_red << "ERROR: Attempt to evaluate viewpoints out of order\n" << cc_reset;
        return;
    }
    if (isDebug && isDebugStates){
        std::cout << cc_green << "Evaluating viewpoints\n" << cc_reset;
    }
    
    
    double yaw = randomDouble(M_PI-M_PI_4, M_PI+M_PI_4);
    set_waypoint(
        5,      //x
        mobile_base_pose.position.y + 1, //y
        4,      //z
        yaw    //yaw
    );
    
    
    state = NBV_STATE_DONE_VIEWPOINT_EVALUATION;
}

void set_waypoint(double x, double y, double z, double yaw){
    geometry_msgs::Pose setpoint_world;
    
    // Position
    setpoint_world.position.x = x;
    setpoint_world.position.y = y;
    setpoint_world.position.z = z;
    
    // Orientation
    tf::Quaternion tf_q;
    tf_q = tf::createQuaternionFromYaw(yaw);
    
    setpoint_world.orientation.x = tf_q.getX();
    setpoint_world.orientation.y = tf_q.getY();
    setpoint_world.orientation.z = tf_q.getZ();
    setpoint_world.orientation.w = tf_q.getW();
    
    // Transform to setpoint frame
    transformGlobal2Setpoint(setpoint_world, setpoint.pose);
}

void move_vehicle(){
    if (state != NBV_STATE_MOVING && state != NBV_STATE_TAKING_OFF){
        std::cout << cc_red << "ERROR: Attempt to move vehicle out of order\n" << cc_reset;
        return;
    }
    if (isDebug && isDebugStates){
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
    while(ros::ok() && !isNear(setpoint_world, mobile_base_pose)){
        if (isDebug && isDebugContinuousStates){
            std::cout << cc_green << "Moving to destination. " <<
                "Distance to target: " << getDistance(setpoint_world, mobile_base_pose) <<
                "\tAngle to target: " << getAngularDistance(setpoint_world, mobile_base_pose) << "\n" << cc_reset;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    if (state == NBV_STATE_MOVING){
        std::cout << cc_green << "Done moving\n" << cc_reset;
        state = NBV_STATE_DONE_MOVING;
    }
}


void takeoff(){
    if (state != NBV_STATE_TAKING_OFF){
        std::cout << cc_red << "ERROR: Attempt to take off out of order\n" << cc_reset;
        return;
    }
    
    double x = mobile_base_pose.position.x;
    double y = mobile_base_pose.position.y;
    double yaw = M_PI;
    
    // Simulate smooth takeoff at 4 meters
    int target_height = 4;
    
    if ( isNear(mobile_base_pose.position.z, target_height) )
    {
        std::cout << cc_green << "Skipping takeoff\n" << cc_reset;
    }
    else{
        std::cout << cc_green << "Taking off\n" << cc_reset;
        
        for (int i=1; i<=target_height; i++){
            set_waypoint(x, y, i, yaw);
            move_vehicle();
        }
        
        std::cout << cc_green << "Done taking off\n" << cc_reset;
    }
    
    state = NBV_STATE_IDLE;
}

bool isNear(double p1, double p2){
    if (fabs(p1-p2)< distance_threshold){
        return true;
    }
        
    return false;
}

bool isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current){
    if (
        getDistance(p_target, p_current) < distance_threshold &&
        fabs(getAngularDistance(p_target, p_current)) < angular_threshold){
        return true;
    }
        
    return false;
}

double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
	return sqrt(
        (p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) +
        (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) +
        (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z) );
}

double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
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
    if (yaw_diff > M_PI){
        //std::cout << cc_green << "Decreased by 360: \n";
        yaw_diff = yaw_diff - 2*M_PI;
    }
    else if (yaw_diff < -M_PI){
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
