#ifndef GAZEBO_FLOATING_SENSOR_POSITION_H
#define GAZEBO_FLOATING_SENSOR_POSITION_H

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>


#include <stdio.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <mutex>          // std::mutex

namespace gazebo
{

class FloatingSensorPosition : public ModelPlugin
{
public:
  FloatingSensorPosition();
  virtual ~FloatingSensorPosition();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();
  virtual void OnRosMsgPose(const geometry_msgs::PoseConstPtr &msg);
  virtual void OnRosMsgTwist(const geometry_msgs::TwistConstPtr &msg);
  virtual void QueueThread();
  virtual void GetCurrentPose();

private:
  std::mutex mtx_pose_;           // mutex for critical section

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::ModelPtr model_;

  std::string link_name_;
  std::string namespace_;

  event::ConnectionPtr update_connection_;

  // ROS transport
  /// \brief A node use for ROS transport
  ros::NodeHandle* rosNode;

  /// \brief A ROS subscriber
  ros::Subscriber rosSub;
  ros::Subscriber rosSub2;

  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;

  /// \brief Prefix to use for this model when communicating using ROS topics
  std::string topic_base_;

  ros::Publisher pub_pose_;
  geometry_msgs::Pose  current_pose_;
  geometry_msgs::Twist current_twist_;

  double current_pos_x_;
  double current_pos_y_;
  double current_pos_z_;
  double current_pos_roll_;
  double current_pos_pitch_;
  double current_pos_yaw_;
};

}

#endif
