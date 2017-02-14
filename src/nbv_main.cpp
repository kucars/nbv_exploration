#include <iostream>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include "nbv_exploration/nbv_loop.h"
#include "nbv_exploration/common.h"
NBVLoop* n;

void sigIntHandler(int sig)
{
  std::cout << cc.yellow << "Handling SIGINT exception\n" << cc.reset;

  // Forces ros::Rate::sleep() to end if it's stuck (for example, Gazebo isn't running)
  ros::shutdown();
}

int main(int argc, char **argv)
{
  /* Override SIGINT handler */
  ros::init(argc, argv, "nbv_loop", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  n = new NBVLoop();
  return 0;
}
