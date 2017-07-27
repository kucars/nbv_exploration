# Next Best View Exploration
Next best view approach for 3D reconstruction


## Installing

Follow the steps below to install the simulation environment with all it's dependencies.

```
cd <catkin_ws>
wstool init src
wstool set -t src nbv_exploration https://github.com/kuri-kustar/nbv_exploration.git --git
wstool merge -t src https://raw.githubusercontent.com/kuri-kustar/nbv_exploration/master/nbv_exploration.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```


## Running
To run the main program, run the following commands in two separate terminals:

```
roslaunch nbv_exploration nbv_setup.launch
roslaunch nbv_exploration nbv_loop.launch
```

To launch the UAV simulation, run the following instead:

```
roslaunch nbv_exploration nbv_setup_uav.launch
roslaunch nbv_exploration nbv_loop_uav.launch
```

## Issues
For your first run, you may get the following message:
```
[pcl::PCDReader::readHeader] Could not find file 'profile_cloud.pcd'.
```

In that case, open the file `config/nbv_settings.yaml` and set:
  - `profiling_skip` to `false`
  - `profiling_skip_load_map` to `true`

This will create a profile from scratch. If you'd like to skip the profiling stage in future runs, set:
  - `profiling_skip` to `true`
  - `profiling_skip_load_map` to `false`


## Planned Updates
  - Path planning for UAV navigation to frontiers
  - 2 sensors for UAV
  - Improved frontier viewpoint generation scheme
