#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1, const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
{
  ROS_INFO("Synched!");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cloud_sync");
  ros::NodeHandle nh;

  ROS_INFO("Testing Cloud Sync!");

  message_filters::Subscriber<sensor_msgs::PointCloud2> depth1_sub(nh, "/nbv_exploration/depth", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> depth2_sub(nh, "/nbv_exploration/depth2", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth1_sub, depth2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}
