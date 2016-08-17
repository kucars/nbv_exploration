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
std::string map_topic      = "/global_cloud";

// == Navigation variables
geometry_msgs::Pose mobile_base_pose;

// == Publishers
ros::Publisher pub_global_cloud;
ros::Publisher pub_setpoint;

// == Subsctiptions
tf::TransformListener *listener;


// == Point clouds
float grid_res = 0.1f; //Voxel grid resolution

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sensed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloudPtr;



// ======================
// Function prototypes (@todo: move to header file)
// ======================

void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void positionCallback(const geometry_msgs::PoseStamped& pose_msg);
void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);

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
    ros::Subscriber sub_pose = ros_node.subscribe(position_topic, 1, positionCallback);
    
    listener = new tf::TransformListener();
    
    // >>>>>>>>>>>>>>>>>
    // Publishers
    // >>>>>>>>>>>>>>>>>
    pub_global_cloud = ros_node.advertise<sensor_msgs::PointCloud2>(map_topic, 10);

    ros::spin();

    /*
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    */

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
