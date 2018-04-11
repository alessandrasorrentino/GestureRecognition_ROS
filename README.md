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
1. Build the solution 
      $ lksfg;slfm
2. Copy the files present in the folder $(CHAI3DYOURPROJECTPATH)/bin/Debug in the vrep folder.
3. Connect the haptic device to the laptop and perform haptic diagnostic using the Diagnostic program. 
4. Open vrep and load the scene. Check on the terminal that no errors are reported. 
5. Run the scene. 
