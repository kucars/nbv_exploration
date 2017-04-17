#include <vehicle/floating_sensor_position_plugin.h>
#include <string>

#if GAZEBO_MAJOR_VERSION >= 6
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#endif

namespace gazebo
{

FloatingSensorPosition::FloatingSensorPosition()
{
}

FloatingSensorPosition::~FloatingSensorPosition()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void FloatingSensorPosition::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();
  namespace_.clear();

  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue())
    {
      link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
      link_ = _model->GetLink(link_name_);
    }

  if (!link)
  {
    ROS_FATAL("floating_sensor_position plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // Get current pose
  GetCurrentPose();

  // Get topic base for ROS messaging
  if (!_sdf->HasElement("topic_base"))
  {
    ROS_INFO("FloatingSensorPosition plugin missing <topic_base>, defaults to model name");
    topic_base_ = "/" + this->model_->GetName();
  }
  else
  {
    topic_base_ = "/" + _sdf->GetElement("topic_base")->Get<std::string>();
  }


  // Make sure the ROS node for Gazebo has already been initialized
  /*
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  */

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&FloatingSensorPosition::Update, this));



  // Initialize ros, if it has not already initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "floating_sensor_position_interface",
        ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  this->rosNode = new ros::NodeHandle("floating_sensor_position_interface");

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
        this->topic_base_ + "/set_pose",
        1,
        boost::bind(&FloatingSensorPosition::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);

  this->rosSub = this->rosNode->subscribe(so);

  // Create a publisher
  pub_pose_ = this->rosNode->advertise<geometry_msgs::Pose>(this->topic_base_ + "/pose", 10);

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&FloatingSensorPosition::QueueThread, this));
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void FloatingSensorPosition::Update()
{
}


////////////////////////////////////////////////////////////////////////////////
/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
void FloatingSensorPosition::OnRosMsg(const geometry_msgs::PoseConstPtr &msg)
{
  double x, y, z;

  x = msg->position.x;
  y = msg->position.y;
  z = msg->position.z;

  // Convert quaternion to Euler angles (w,x,y,z notation)
  math::Quaternion q = math::Quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  math::Vector3 rpy = q.GetAsEuler();

  double roll, pitch, yaw;
  roll  = rpy.x;
  pitch = rpy.y;
  yaw   = rpy.z;

  // Update model position
  model_->SetLinkWorldPose(math::Pose(x, y, z, roll, pitch, yaw), link_);

  // Save current pose
  //current_pose_ = *msg;
}



/////////////////////////////////////////////////////////////////////////////////
/// \brief ROS helper function that processes messages
void FloatingSensorPosition::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    // Publish current pose
    GetCurrentPose();
    pub_pose_.publish(current_pose_);

    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////////////////////////////////////
/// \brief Read the current pose in gazebo
void FloatingSensorPosition::GetCurrentPose()
{
  math::Pose p = model_->GetWorldPose();
  current_pose_.position.x = p.pos.x;
  current_pose_.position.y = p.pos.y;
  current_pose_.position.z = p.pos.z;
  current_pose_.orientation.x = p.rot.x;
  current_pose_.orientation.y = p.rot.y;
  current_pose_.orientation.z = p.rot.z;
  current_pose_.orientation.w = p.rot.w;
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void FloatingSensorPosition::Reset()
{
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(FloatingSensorPosition)

}  // namespace gazebo
