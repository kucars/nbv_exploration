#ifndef TELEOP_UAV_H
#define TELEOP_UAV_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <keyboard/Key.h>

#define Arrow_R 0x43 // Right Arrow
#define Arrow_L 0x44 // Left Arrow
#define Arrow_U 0x41 // Up Arrow
#define Arrow_D 0x42 // Down Arrow
#define KEY_w   'w'
#define KEY_s   's'
#define KEY_d   'd'
#define KEY_a   'a'
#define KEY_f   'f'
#define KEY_l   'l'
#define KEY_e   'e'
#define KEY_t   't'
#define KEY_l   'l'
#define KEY_u   'u'
#define KEY_y   'y'
#define KEY_EMR ' '

int kfd = 0;
struct termios cooked, raw;

class sarKeyTeleop
{
public:
    sarKeyTeleop();
    void keyLoop();
    void watchdog();
    ros::Publisher velocty_pub  ; 
    ros::Subscriber pose_sub_;
private:
   // ArdroneInterface    interface_;
    ros::NodeHandle     ph_;
    ros::Time           first_publish_;
    ros::Time           last_publish_;
    boost::mutex        publish_mutex_;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    double roll, pitch, yaw;
    double robot_ox;
    double robot_oz;
    double robot_oy;
    double  l_scale_;
    double  a_scale_;
    double linear_x, linear_y, linear_z, angular_z ;
    std::map<uint16_t, std::string> key_map;
    void sendVel(geometry_msgs::TwistStamped & vel);
    void keyboard_up_event(const keyboard::Key::ConstPtr& key);
    void keyboard_down_event(const keyboard::Key::ConstPtr& key);
    void process_event(std::string event, int state);
};

#endif // TELEOP_UAV_H
