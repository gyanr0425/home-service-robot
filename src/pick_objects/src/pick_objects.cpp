#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  // define a publisher to publish the reached_state
  ros::Publisher reached_pub = n.advertise<std_msgs::Int16>("reached_state", 1000);

  // reached_state-> 0: not reached, 1: reached to pickup zone, 2: reached to drop zone
  std_msgs::Int16 reached_state;
  reached_state.data = 0;
  reached_pub.publish(reached_state);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -0.5;  // target position for the pickup zone
  goal.target_pose.pose.position.y = 1.5;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is traveling to the pickup zone.");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    reached_state.data = 1;
    reached_pub.publish(reached_state);
    ROS_INFO("Robot picked up the virtual object.");
  }
  else {
    ROS_INFO("Robot failed to travel to the pickup zone.");
  }

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  goal.target_pose.pose.position.x = 3.5;
  goal.target_pose.pose.position.y = -0.5;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.7068252;
  goal.target_pose.pose.orientation.w = 0.7073883;

  ROS_INFO("Robot is traveling to the drop zone.");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    reached_state.data = 2;
    reached_pub.publish(reached_state);
    ROS_INFO("Robot droped the virtual object.");
  }
  else {
    ROS_INFO("Robot failed to travel to the drop zone.");
  }

  return 0;
}
