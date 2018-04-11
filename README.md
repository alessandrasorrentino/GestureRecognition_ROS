# GestureRecognition_ROS
Gesture recognition module that allows to command the the direction a simulated robot should move


# Prerequisites 
  - Ubuntu 16.04.4 LTS
  - ROS Kinetic: http://wiki.ros.org/kinetic/Installation
  - Tensorflow 1.4.1+
  - Hri_software: https://bitbucket.org/iocchi/hri_software
  - Tf-pose-estimation ROS : https://github.com/ildoonet/tf-pose-estimation/blob/master/etcs/ros.md

# Installation
  a. Follow the installation online to install hri-software and tf-pose-estimation ROS in your catkin_ws. 
  
  b. Substitute the file start_simulation.py in $(CATKIN_WS)/src/stage_environments/scripts with the file stored here.
  
  c. Substitute the file demo_video.launch in $(CATKIN_WS)/src/tf-pose-estimation/launch with the file stored here.
  
  d. Substitute the file visualization.py in $(CATKIN_WS)/src/tf-pose-estimation/script with the file stored here.
    
# Running
1. Compile ROS packages 
```
$  cd catkin_ws
$  source devel/setup.bash
$  catkin_make
```
2. Run the Stage simulation launcher
```
$  source <path_to>/hri_software/catkin_ws/devel/setup.bash
$  roscd stage_environments/scripts
$ ./start_simulation.py
```
3. In the Simulation Launcher GUI select
```
Map: DISB1
Robot: diago  
Pose: (default one)
Number of robots: 1
Localization: srrg_localizer
Navigation: spqrel_planner
Demo: hri_pnp
```
and check off the box corresponding to Gestures
4. Perform the desired gesture
