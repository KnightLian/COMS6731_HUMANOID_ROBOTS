# COMS6731_HUMANOID_ROBOTS

#### Install assignment dependency
https://github.com/jingxixu/humanoid-robots-s19

#### Install NVIDIA and CUDA
https://docs.nvidia.com/deploy/cuda-compatibility/index.html

#### Install GraspIt Setup and ROS Setup for graspit_interface
https://github.com/graspit-simulator/graspit_interface  
add objects under .graspit/models/objects

#### Install DOPE for vision
https://github.com/NVlabs/Deep_Object_Pose  
download the weights and save them to the weights folder, i.e., ~/catkin_ws/src/dope/weights/

#### Install dependency for frontier_exploration in Ubuntu 14.04
sudo apt-get install ros-indigo-frontier-exploration ros-indigo-navigation-stage

#### Install follows in src
navigation_goals  
cse481wi18  
fetch_gazebo - git checkout gazebo7  
fetch_ros - git checkout indigo-devel  
robot_controllers - git checkout indigo-devel

#### Exploration Task in Ubuntu 14.04
groundnavigate.launch  
exploration.launch  

#### Navigation Task in Ubuntu 16.04
groundnavigate.launch  
navigation.launch  

#### Recognition and Motion Planning Tasks in Ubuntu 16.04
groundgrasp.launch  
dope.py  
vision.launch  
Try.py  
