/* Author: Abdullah Abduldayem
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/registration/icp.h>

// ===================
// === Variables  ====
// ===================
pcl::PointXYZRGB* cloud_max_range;

// == Parameters
double range_adjustment_max;
double range_adjustment_min;

double depth_range_max;
double depth_range_min;
int width_px;
int height_px;
double fov_vertical;
double fov_horizontal;


// == Consts
std::string topic_scan_in   = "scan_in";
std::string topic_scan_out  = "scan_out";
std::string topic_depth_in  = "depth_in";
std::string topic_depth_out = "depth_out";

// == Publishers
ros::Subscriber sub_scan;
ros::Subscriber sub_depth;
ros::Publisher  pub_scan;
ros::Publisher  pub_depth;


// ======================
// Function prototype
// ======================
void callbackScan(const sensor_msgs::LaserScan& input_msg);
void callbackDepth(const sensor_msgs::PointCloud2& input_msg);
void createMaxRangeCloud();

int main(int argc, char **argv)
{
    // >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    ros::init(argc, argv, "correct_laser_scan_and_depth");
    ros::NodeHandle ros_node;

    // >>>>>>>>>>>>>>>>>
    // Get input parameters
    // >>>>>>>>>>>>>>>>>
    ros_node.param("range_adjustment_max", range_adjustment_max, 0.5);
    ros_node.param("range_adjustment_min", range_adjustment_min, 0.5);
    
    ros_node.param("depth_range_max", depth_range_max, 8.0);
    ros_node.param("depth_range_min", depth_range_min, 0.5);
    ros_node.param("width_px", width_px, 640);
    ros_node.param("height_px", height_px, 480);
    ros_node.param("fov_vertical", fov_vertical, 45.0);
    ros_node.param("fov_horizontal", fov_horizontal, 60.0);

    // >>>>>>>>>>>>>>>>>
    // Topic Handlers
    // >>>>>>>>>>>>>>>>>
    sub_scan  = ros_node.subscribe(topic_scan_in, 40, callbackScan);
    sub_depth = ros_node.subscribe(topic_depth_in, 30, callbackDepth);
    
    pub_scan  = ros_node.advertise<sensor_msgs::LaserScan>(topic_scan_out, 40);
    pub_depth = ros_node.advertise<sensor_msgs::PointCloud2>(topic_depth_out, 10);
    
    createMaxRangeCloud();
           
    ros::spin();
    return 0;
}

void callbackScan(const sensor_msgs::LaserScan& input_msg){
  sensor_msgs::LaserScan output_msg;
  
  output_msg.header = input_msg.header;
  output_msg.angle_min = input_msg.angle_min;
  output_msg.angle_max = input_msg.angle_max;
  output_msg.angle_increment = input_msg.angle_increment;
  output_msg.time_increment = input_msg.time_increment;
  output_msg.scan_time = input_msg.scan_time;
  
  output_msg.range_min = input_msg.range_min;
  output_msg.range_max  = input_msg.range_max;
  
  output_msg.ranges = input_msg.ranges;
  output_msg.intensities = input_msg.intensities;
  
  
  /*
   * Points near the max and min range are pushed outside the range
   * This way, they can be ignored
   */
  float inf = output_msg.range_max + range_adjustment_max + 1;
  int steps = (output_msg.angle_max - output_msg.angle_min)/output_msg.angle_increment;
  
  for (int i=0; i<=steps; i++)
  {
    if (output_msg.ranges[i] + range_adjustment_max > output_msg.range_max)
      output_msg.ranges[i] = inf;
    else if (output_msg.ranges[i] - range_adjustment_min < output_msg.range_min)
      output_msg.ranges[i] = 0;
  }
  
  pub_scan.publish(output_msg);
}


void callbackDepth(const sensor_msgs::PointCloud2& input_msg)
{
  /*
   * Points near the max and min range are pushed outside the range
   * This way, they can be ignored
   * 
   * IMPORTANT: NANs are considered to be beyond the max range, and NOT before the earlier range
   * Violating this condition will result in occupied areas considered as "free"
   */
  
  // Convert to point cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg (input_msg, cloud);
  
  if (cloud.points.size() != width_px*height_px)
  {
    ROS_ERROR("Number of points in cloud (%d) do not match supplied inputs (%dx%d px)", (int) cloud.points.size(), width_px, height_px);
    return;
  }
  
  for (int i=0; i<=cloud.points.size(); i++)
  {
    if (std::isfinite(cloud.points[i].x) &&
        std::isfinite(cloud.points[i].y) &&
        std::isfinite(cloud.points[i].z) )
    {
      // Point is valid, continue
      continue;
    }
    
    int x_px = i%width_px;
    int y_px = i/width_px;
    //y_px*height_px + x_px
    
    //int x_px = i/height_px;
    //int y_px = i%height_px;
    cloud.points[i] = cloud_max_range[y_px*width_px + x_px];
  }
  
  
  
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(cloud, output_msg); 	//cloud of original (white) using original cloud
  
  output_msg.header = input_msg.header;
  output_msg.is_dense = true;
  
  //output_msg.header.frame_id = "world";
  //output_msg.header.stamp = ros::Time::now();
  
  pub_depth.publish(output_msg);
}



void createMaxRangeCloud()
{
  cloud_max_range = new pcl::PointXYZRGB[width_px*height_px];
  
  double r = depth_range_max + range_adjustment_max;
  
  double x_step = tan(fov_horizontal/2*M_PI/180)/(width_px/2);
  double y_step = tan(fov_vertical/2*M_PI/180)/(height_px/2);
  
  for (int j=0; j<height_px; j++)
  {
    for (int i=0; i<width_px; i++)
    {
      pcl::PointXYZRGB p;
      
      p.x = (i-width_px/2+0.5)*x_step*r; //x position, scaled outside valid range
      p.y = (j-height_px/2+0.5)*y_step*r;
      p.z = r;
      
      cloud_max_range[j*width_px + i] = p;
    }
  }
}
