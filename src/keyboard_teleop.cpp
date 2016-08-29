/* Partially borrowed code from the universal_teleop package
 * http://wiki.ros.org/universal_teleop
*/

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <geometry_msgs/TwistStamped.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include "keyboard_teleop.h"
#include <keyboard/Key.h>

std::string vel_topic = "/iris/mavros/setpoint_velocity/cmd_vel";
std::string pose_topic= "/iris/mavros/local_position/pose";

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

sarKeyTeleop::sarKeyTeleop():
    ph_("~")
{
    ROS_INFO_STREAM ("Initialize Teleoperation UAV");
    std::cout<< "Read Parameters" << std::endl;

    ph_.param("scale_angular" , a_scale_, 1.0);
    ph_.param("scale_linear"  , l_scale_, 1.0);

    std::cout<< "scale_angular = " << a_scale_ << std::endl;
    std::cout<< "scale_linear  = " << l_scale_ << std::endl;
    
    /* load key mappings */
    //std::map<std::string, int> keys;
    key_map.insert ( std::pair<int,std::string>(KEY_w,   "forward") );
    key_map.insert ( std::pair<int,std::string>(KEY_s,   "backward") );
    key_map.insert ( std::pair<int,std::string>(KEY_a,   "left") );
    key_map.insert ( std::pair<int,std::string>(KEY_d,   "right") );
    key_map.insert ( std::pair<int,std::string>(Arrow_U, "up") );
    key_map.insert ( std::pair<int,std::string>(Arrow_D, "down") );
    key_map.insert ( std::pair<int,std::string>(Arrow_R, "yaw_right") );
    key_map.insert ( std::pair<int,std::string>(Arrow_L, "yaw_left") );

    //n.param("keys", keys, keys);
    
    // Subscribe and publish
    velocty_pub = ph_.advertise<geometry_msgs::TwistStamped>(vel_topic, 1000);
    pose_sub_ = ph_.subscribe(pose_topic ,1, &sarKeyTeleop::poseCallback, this );
    ros::Subscriber keyup_sub = ph_.subscribe("/keyboard/keyup", 1, &sarKeyTeleop::keyboard_up_event, this);
    ros::Subscriber keydown_sub = ph_.subscribe("/keyboard/keydown", 1, &sarKeyTeleop::keyboard_down_event, this);

    std::cout << "Subscribed to: " << "/keyboard/keyup" << "\n";
    std::cout << "Subscribed to: " << "/keyboard/keydown" << "\n";
    std::cout << "Subscribed to: " << pose_topic << "\n";
    std::cout << "Publishing to: " << vel_topic  << "\n";
}

void sarKeyTeleop::keyboard_up_event(const keyboard::Key::ConstPtr& key)
{
  ROS_INFO_STREAM("keyup: " << key->code);
  std:: string event;
  
  if (key_map.find(key->code) == key_map.end()) event = "unknown";
  else event = key_map[key->code];

  process_event(event, 0);
}

void sarKeyTeleop::keyboard_down_event(const keyboard::Key::ConstPtr& key)
{
  std::cout << "Got key: " << key->code << "\n";
  std:: string event;
  
  if (key_map.find(key->code) == key_map.end()) event = "unknown";
  else event = key_map[key->code];
  
  process_event(event, 1);
}

void sarKeyTeleop::process_event(std::string event, int state)
{
  int multiplier = state;
  std::cout << "Got key: " << event << "\n";
  
}

void sarKeyTeleop::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion q( msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    /*
    std::cout << "pitch " << pitch << std::endl ; 
    std::cout << "roll " << roll << std::endl ; 
    std::cout << "yaw " << yaw << std::endl ; 
    std::cout << "yaw in degrees: " << yaw * 180 / 3.14  << std::endl ; 
    */
    
}

void sarKeyTeleop::sendVel(geometry_msgs::TwistStamped & vel)
{
    vel.twist.linear.x  = l_scale_ * vel.twist.linear.x;
    vel.twist.linear.y  = l_scale_ * vel.twist.linear.y;
    vel.twist.linear.z  = l_scale_ * vel.twist.linear.z;
    vel.twist.angular.z = a_scale_ * vel.twist.angular.z;
    velocty_pub.publish(vel);
   // interface_.velCom(vel);
    return;
}

void sarKeyTeleop::keyLoop()
{
    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the UAV.");

    while (ros::ok())
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        ROS_DEBUG("value: 0x%02X\n", c);
        geometry_msgs::TwistStamped vel_msg;
        
        std::cout << c << "\n";
        
        switch(c)
        {
        case Arrow_U    : linear_x = 2.0 ; linear_y = 0.0; linear_z = 0.0; angular_z= 0.0 ; break;
        case Arrow_D    : linear_x =-1.0 ; linear_y = 0.0; linear_z = 0.0; angular_z= 0.0 ; break;
        case Arrow_L    : linear_x = 0.0 ; linear_y = 2.0; linear_z = 0.0; angular_z= 0.0 ; break;
        case Arrow_R    : linear_x = 0.0 ; linear_y =-2.0; linear_z = 0.0; angular_z= 0.0 ; break;	
        case KEY_w      : linear_x = 0.0 ; linear_y = 0.0; linear_z = 1.0; angular_z= 0.0 ; break;
        case KEY_s      : linear_x = 0.0 ; linear_y = 0.0; linear_z =-1.0; angular_z= 0.0 ; break;
       	case KEY_a      : linear_x = 0.0 ; linear_y = 0.0; linear_z = 0.0; angular_z= 1.57 ; break;
        case KEY_d      : linear_x = 0.0 ; linear_y = 0.0; linear_z = 0.0; angular_z=-1.57 ; break;
       }
        vel_msg.twist.linear.x = linear_x*cos(yaw ) - linear_y * sin(yaw) ; 
        vel_msg.twist.linear.y = linear_x*sin (yaw) + linear_y * cos(yaw) ;
        vel_msg.twist.linear.z = linear_z ;
        vel_msg.twist.angular.z = angular_z;

        boost::mutex::scoped_lock lock(publish_mutex_);
        if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) {
            first_publish_ = ros::Time::now();
        }
        last_publish_ = ros::Time::now();
        sendVel(vel_msg);
    }

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sar_teleop");


    sarKeyTeleop teleop_uav;
    ros::spin();

    return(0);
}
