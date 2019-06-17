#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void send_goal(float input_x,float input_y,float input_w)
{
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

 
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = input_x;
  goal.target_pose.pose.position.y = input_y;
  goal.target_pose.pose.orientation.w = input_w;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

}


int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_goals");
  float input_x = std::stof(argv[1]);
  float input_y = std::stof(argv[2]);
  float input_w = std::stof(argv[3]);
  ros::Time::init();
  ros::Duration(10).sleep();
  send_goal(input_x,input_y,input_w);
  ros::Duration(60).sleep();
  return 0;
}

