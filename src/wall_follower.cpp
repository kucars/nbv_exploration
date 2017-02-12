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

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/registration/icp.h>

#include "utilities/console_utility.h"
ConsoleUtility cc;

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

/*
double randomDouble(double min, double max) {
    return ((double) random()/RAND_MAX)*(max-min) + min;
}

double rad2deg (double rad) {
    return rad*180/M_PI;
}

double getDistanceXY(pcl::PointXYZ p1, pcl::PointXYZ p2){
	return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );
}
*/

ros::Publisher pub;
ros::Publisher pub_pose;
geometry_msgs::Pose mobile_base_pose;
geometry_msgs::Pose mobile_base_pose_prev;


pcl::VoxelGrid<pcl::PointXYZRGB> occGrid;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloudPtr;

float res = 0.1f; //Voxel grid resolution


// Prototype
void depthCallback(const sensor_msgs::PointCloud2& cloud_msg);
void positionCallback(const geometry_msgs::PoseStamped& pose_msg);
void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
void transformCam2Robot (const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);




int main(int argc, char **argv)
{
    std::string depth_topic   = "/iris/xtion_sensor/iris/xtion_sensor_camera/depth/points";
    std::string position_topic = "/iris/ground_truth/pose";
    
    // >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    std::cout << cc.red << "Begin wall_follower\n" << cc.reset;

    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle ros_node;

    ros::Subscriber sub_kinect = ros_node.subscribe(depth_topic, 1, depthCallback);
    ros::Subscriber sub_pose = ros_node.subscribe(position_topic, 1, positionCallback);

    pub = ros_node.advertise<sensor_msgs::PointCloud2> ("/voxgrid", 1);
    pub_pose = ros_node.advertise<geometry_msgs::PoseStamped> ("/voxgrid/pose", 1);

    ros::spin();

    return 0;
}


// Update global position of UGV
void positionCallback(const geometry_msgs::PoseStamped& pose_msg)
{
    // Save UGV pose
    mobile_base_pose_prev = mobile_base_pose;
    mobile_base_pose = gazebo_msg.pose[index];

    // Publish
    geometry_msgs::PoseStamped ps;
    ps.pose = mobile_base_pose;
    ps.header.frame_id = "base_link";
    ps.header.stamp = ros::Time::now();

    pub_pose.publish(ps);
}

int count = 0;
void depthCallback(const sensor_msgs::PointCloud2& cloud_msg)
{
    /*
    geometry_msgs::Point pt1 = mobile_base_pose.position;
    geometry_msgs::Point pt2 = mobile_base_pose_prev.position;
    
    geometry_msgs::Quaternion ori1 = mobile_base_pose_prev.orientation;
    geometry_msgs::Quaternion ori2 = mobile_base_pose.orientation;
    
    double dist = (pt1.x-pt2.x)*(pt1.x-pt2.x) + 
                  (pt1.y-pt2.y)*(pt1.y-pt2.y) + 
                  (pt1.z-pt2.z)*(pt1.z-pt2.z);
    
    double roll1, pitch1, yaw1, roll2, pitch2, yaw2;
    tf::Quaternion q1(ori1.x, ori1.y, ori1.z, ori1.w);
    tf::Quaternion q2(ori2.x, ori2.y, ori2.z, ori2.w);
    
    tf::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
    tf::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);
    
    if (dist > 0.1 || fabs(yaw1-yaw2)>20*180/M_PI){
        count = 0;
        return;
    }
    
    if (count<20){
        count++;
        return;
    }
    */
    
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Convert to pcl pointcloud
    pcl::fromROSMsg (cloud_msg, cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = cloud.makeShared();

    // Remove NAN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudPtr,*cloudPtr, indices);


    // Perform voxelgrid filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (res, res, res);
    sor.filter (*cloud_filtered);


    // Add filtered to global
    addToGlobalCloud(cloud_filtered);


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*globalCloudPtr, output);
    //pcl::toROSMsg(*cloudPtr, output);

    output.header.frame_id = cloud_msg.header.frame_id;
    output.header.stamp = ros::Time::now();

    std::cout << "Input: " << cloudPtr->points.size() << "\tOutput: " << globalCloudPtr->points.size() << "\n";

    // Publish the data
    pub.publish (output);
}


void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
    /*
    // Initialize global cloud if not already done so
    if (!globalCloudPtr){
        globalCloudPtr = cloud_in;
        return;
    }
    */

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    transformCam2Robot (*cloud_in, *transformed_cloud);

    // Initialize global cloud if not already done so
    if (!globalCloudPtr) {
        globalCloudPtr = transformed_cloud;
        return;
    }

    /*
    // Preform ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputCloud(transformed_cloud);
    icp.setInputTarget(globalCloudPtr);
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    *globalCloudPtr += Final;
    */

    //globalCloudPtr = transformed_cloud;
    //return;

    *globalCloudPtr += *transformed_cloud;

    // Perform voxelgrid filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (globalCloudPtr);
    sor.setLeafSize (res, res, res);
    sor.filter (*cloud_filtered);

    globalCloudPtr = cloud_filtered;
}


void transformCam2Robot (const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
    if (&cloud_in != &cloud_out) {
        cloud_out = cloud_in;
    }

    // Sensor settings
    Eigen::Vector3d rpy(0, 0.18, 3.14);
    Eigen::Vector4d xyz(mobile_base_pose.position.x, mobile_base_pose.position.y, mobile_base_pose.position.z, 1);

    Eigen::Matrix3d R;
    tf::Quaternion qt = tf::createQuaternionFromRPY(-rpy[1],-rpy[0],-rpy[2]);

    tf::Matrix3x3 R1(qt);
    tf::matrixTFToEigen(R1,R);

    Eigen::Matrix4d tf_matrix;
    tf_matrix.setZero();
    tf_matrix.block (0, 0, 3, 3) = R;
    tf_matrix (3, 3) = 1;

    Eigen::Matrix4d tf_matrix_swap;
    tf_matrix_swap <<   0, 1, 0, 0,
                        0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 0, 0, 1;


    double roll, pitch, yaw;
    tf::Quaternion q(
        mobile_base_pose.orientation.x,
        mobile_base_pose.orientation.y,
        mobile_base_pose.orientation.z,
        mobile_base_pose.orientation.w
    );
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf::Quaternion qt2 = tf::createQuaternionFromRPY(-rpy[1], -yaw, M_PI);

    tf::Matrix3x3 R2(qt2);
    tf::matrixTFToEigen(R2,R);

    Eigen::Matrix4d tf_final;
    tf_final.setZero();
    tf_final.block (0, 0, 3, 3) << tf_matrix.block (0, 0, 3, 3) * R;
    //tf_matrix.block (0, 0, 3, 3) = R;//.transpose();
    tf_final.block (0, 3, 4, 1) << tf_matrix * tf_matrix_swap * xyz;
    //tf_matrix (3, 3) = 1;

    //tf_final = tf_matrix * tf_matrix_swap * tf_final;

    // Perfrom transformation
    for (size_t i = 0; i < cloud_in.points.size (); i++)
    {
        Eigen::Vector4d pt(cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z, 1);
        Eigen::Vector4d pt_tf = tf_final*pt;

        cloud_out.points[i].x = pt_tf[0];
        cloud_out.points[i].y = pt_tf[1];
        cloud_out.points[i].z = pt_tf[2];
    }
}
