/* Author: Abdullah Abduldayem
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

// ===================
// === Variables  ====
// ===================
// == Parameters
double range_adjustment_max;
double range_adjustment_min;

// == Consts
std::string scan_in_topic  = "scan_in";
std::string scan_out_topic = "scan_out";

// == Publishers
ros::Subscriber sub_scan;
ros::Publisher  pub_scan;

// ======================
// Function prototype
// ======================
void scanCallback(const sensor_msgs::LaserScan& input_msg);

int main(int argc, char **argv)
{
    // >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    ros::init(argc, argv, "correct_laser_scan");
    ros::NodeHandle ros_node;

    // >>>>>>>>>>>>>>>>>
    // Get input parameters
    // >>>>>>>>>>>>>>>>>
    ros_node.param("range_adjustment_max", range_adjustment_max, 0.5);
    ros_node.param("range_adjustment_min", range_adjustment_min, 0.5);

    // >>>>>>>>>>>>>>>>>
    // Topic Handlers
    // >>>>>>>>>>>>>>>>>
    sub_scan = ros_node.subscribe(scan_in_topic, 40, scanCallback);
    pub_scan = ros_node.advertise<sensor_msgs::LaserScan>(scan_out_topic, 40);
    
    ros::spin();
    return 0;
}

void scanCallback(const sensor_msgs::LaserScan& input_msg){
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
  float inf = 1/.0;
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
