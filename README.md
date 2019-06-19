This project provides a feasible solution for humanoid indoor service robot to navigation, recognition and grasping. Fetch robot has been selected as a platform to realize the goal. After building the environment and adding relevant models into the environment, the robot is expected to self explore the world map and perform real time navigation. When the robot reaches to the goal destination, vision will be performed for object recognition and pose estimation.Motion planning will be evaluated for grasping tasks which will be studied in the future. We describe the implementation details and give the results of our experiments for all stages in this section.  

Exploration - Youtube: https://youtu.be/2f2zy2ZkvsE  
Navigation - Youtube: https://youtu.be/qpAtfTZZJHI  
Vision Recognition - Youtube:  
Vision Motion Plan - Youtube:  

### ----------------------INSTALLATION GUIDE -----------------------------------  

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
fetch_gazebo - git checkout gazebo7  
cse481wi18 - https://github.com/jingxixu/cse481wi18.git  
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
