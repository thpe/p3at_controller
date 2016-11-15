Install additional packages:
```
sudo aptitude install ros-kinetic-robot-pose-ekf
```
Setup the workspace:
```
mkdir -p ws/src
cd ws/src
source /opt/ros/kinetic/setup.zsh
catkin_init_workspace
git clone https://github.com/thpe/amr-ros-config
git clone https://github.com/thpe/p3at_controller
catkin_make
```

First source setup:
```
source ~/ws/devel/setup.zsh
```

Start the simulator:
```
roslaunch amr_robots_gazebo example-pioneer3at-world.launch
```

Then start localization:
```
roslaunch p3at_controller robot_pose_ekf.launch
```

start the controller:
```
rosrun p3at_controller controller-line.p
```
