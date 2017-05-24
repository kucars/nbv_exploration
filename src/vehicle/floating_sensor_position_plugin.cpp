#include <vehicle/floating_sensor_position_plugin.h>
#include <string>
#include <ctime>

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
    topic_base_ = "/" + _sdf->GetElement("namespace")->Get<std::string>();
  }

  // Initialize variables
  GetCurrentPose();
  math::Quaternion q = math::Quaternion(
        current_pose_.orientation.w,
        current_pose_.orientation.x,
        current_pose_.orientation.y,
        current_pose_.orientation.z
      );
  math::Vector3 rpy = q.GetAsEuler();

  current_pos_x_    = current_pose_.position.x;
  current_pos_y_    = current_pose_.position.y;
  current_pos_z_    = current_pose_.position.z;
  current_pos_roll_ = rpy.x;
  current_pos_pitch_= rpy.y;
  current_pos_yaw_  = rpy.z;


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

  // Subscribe to "set_pose"
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
        this->topic_base_ + "/set_pose",
        1,
        boost::bind(&FloatingSensorPosition::OnRosMsgPose, this, _1),
        ros::VoidPtr(), &this->rosQueue);

  this->rosSub = this->rosNode->subscribe(so);

  // Subscribe to "set_twist"
  ros::SubscribeOptions so_twist =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
        this->topic_base_ + "/set_twist",
        1,
        boost::bind(&FloatingSensorPosition::OnRosMsgTwist, this, _1),
        ros::VoidPtr(), &this->rosQueue);

  this->rosSub2 = this->rosNode->subscribe(so_twist);

  // Create a publisher
  pub_pose_ = this->rosNode->advertise<geometry_msgs::Pose>(this->topic_base_ + "/pose", 10);

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&FloatingSensorPosition::QueueThread, this));

  // Start update
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&FloatingSensorPosition::Update, this));
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void FloatingSensorPosition::Update()
{
  static std::clock_t previous_time = std::clock();

  // Get time difference since last iteration
  std::clock_t current_time = std::clock();
  double duration_sec = ( current_time - previous_time ) / (double) CLOCKS_PER_SEC;
  previous_time = current_time;

  // Lock critical section
  mtx_pose_.lock();

  // Update position with set speed
  // pos += speed*dt
  current_pos_x_     += current_twist_.linear.x*duration_sec;
  current_pos_y_     += current_twist_.linear.y*duration_sec;
  current_pos_z_     += current_twist_.linear.z*duration_sec;
  current_pos_roll_  += current_twist_.angular.x*duration_sec;
  current_pos_pitch_ += current_twist_.angular.y*duration_sec;
  current_pos_yaw_   += current_twist_.angular.z*duration_sec;

  // Set angles between 0 to 2*PI
  current_pos_roll_  = fmod(current_pos_roll_,  2*M_PI);
  current_pos_pitch_ = fmod(current_pos_pitch_, 2*M_PI);
  current_pos_yaw_   = fmod(current_pos_yaw_,   2*M_PI);

  // Publish position
  UpdatePosition();

  // Unlock critical section
  mtx_pose_.unlock();
}


////////////////////////////////////////////////////////////////////////////////
/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
void FloatingSensorPosition::OnRosMsgPose(const geometry_msgs::PoseConstPtr &msg)
{
  // Lock critical section
  mtx_pose_.lock();

  // Set twist to zeros
  current_twist_.linear.x = 0;
  current_twist_.linear.y = 0;
  current_twist_.linear.z = 0;
  current_twist_.angular.x= 0;
  current_twist_.angular.y= 0;
  current_twist_.angular.z= 0;

  // Update positions
  current_pos_x_ = msg->position.x;
  current_pos_y_ = msg->position.y;
  current_pos_z_ = msg->position.z;

  // Convert quaternion to Euler angles (w,x,y,z notation)
  math::Quaternion q = math::Quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  math::Vector3 rpy = q.GetAsEuler();

  current_pos_roll_  = rpy.x;
  current_pos_pitch_ = rpy.y;
  current_pos_yaw_   = rpy.z;

  // Publish position
  UpdatePosition();

  // Unlock critical section
  mtx_pose_.unlock();
}

void FloatingSensorPosition::OnRosMsgTwist(const geometry_msgs::TwistConstPtr &msg)
{
  current_twist_ = *msg;
}



/////////////////////////////////////////////////////////////////////////////////
/// \brief ROS helper function that processes messages
void FloatingSensorPosition::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    // Check ros callbacks
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

void FloatingSensorPosition::UpdatePosition()
{
  // Update model position in gazebo
  model_->SetLinkWorldPose(
    math::Pose(current_pos_x_, current_pos_y_, current_pos_z_,
               current_pos_roll_, current_pos_pitch_, current_pos_yaw_)
    , link_);

  // Broadcast tf
  tf::Quaternion q;
  q.setRPY(current_pos_roll_, current_pos_pitch_, current_pos_yaw_);

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(current_pos_x_, current_pos_y_, current_pos_z_) );
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "floating_sensor/base_link"));

  // Publish current pose
  if (pub_pose_.getNumSubscribers() > 0)
  {
    GetCurrentPose();
    pub_pose_.publish(current_pose_);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void FloatingSensorPosition::Reset()
{
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(FloatingSensorPosition)

}  // namespace gazebo
