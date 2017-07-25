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
