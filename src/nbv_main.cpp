#include <iostream>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include "nbv_exploration/nbv_loop.h"
#include "nbv_exploration/common.h"
NBVLoop* n;
TimeProfiler timer;

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

  bool is_save_state, is_load_state;
  std::string serialization_file = "nbv_serialization.dat";
  ros::param::param("~debug_save_state", is_save_state, false);
  ros::param::param("~debug_load_state", is_load_state, false);

  // >>>>>>>>>
  // Create NBV class / Load previous state
  // >>>>>>>>>
  if (is_load_state)
  {
    try
    {
      // Create and open an archive for input
      std::ifstream ifs(serialization_file);
      boost::archive::text_iarchive ia(ifs);
      // read class state from archive
      ia >> n;
    }
    catch (...)
    {
      is_load_state = false;
      n = new NBVLoop();
    }
  }
  else
  {
    n = new NBVLoop();
  }

  // >>>>>>>>>
  // Run NBV loop
  // >>>>>>>>>
  n->initAllModules(is_load_state);

  PointCloudXYZ::Ptr cloud (new PointCloudXYZ);
  cloud = n->mapping_module_->getPointCloud();
  std::cout << "Point cloud size: " << cloud->points.size() << "\n\n";
  n->runStateMachine();

  // >>>>>>>>>
  // Save NBV state
  // >>>>>>>>>
  if (is_save_state)
  {
    // Save data to archive
    std::ofstream ofs(serialization_file);
    boost::archive::text_oarchive oa(ofs);
    oa << n;
  }

  return 0;
}
