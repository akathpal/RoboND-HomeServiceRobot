#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;

  geometry_msgs::Pose msg;
  msg.position.x = -3.50;
  msg.position.y = 7.0;
  msg.position.z = 0.0;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;

  // Goal Publisher
  ros::Publisher goal_pub = n.advertise<geometry_msgs::Pose>("/goal_location",20);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickup_goal.target_pose.pose.position.x = -3.5;
  pickup_goal.target_pose.pose.position.y = 7.0;
  pickup_goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup location goal");
  ac.sendGoal(pickup_goal);

  
  


  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot Reached pick up location,waiting for 5 sec");
    ros::Duration(5.0).sleep();

    goal_pub.publish(msg);
    ROS_INFO("Publishing Pickup location goal x:%f, y:%f",msg.position.x,msg.position.y);
    ros::Duration(1.0).sleep();
  }
  else {
    ROS_INFO("Robot failed to reach to specified location");
    return 0;
  }
 
  move_base_msgs::MoveBaseGoal drop_goal;

  // set up the frame parameters
  drop_goal.target_pose.header.frame_id = "map";
  drop_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  drop_goal.target_pose.pose.position.x = -7.0;
  drop_goal.target_pose.pose.position.y = 0.0;
  drop_goal.target_pose.pose.orientation.w = 1.0;


   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal for drop off location");
  ac.sendGoal(drop_goal);

  
  

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Robot reached to the drop off location");
    msg.position.x = -7.0;
    msg.position.y = -0.0;
  
    goal_pub.publish(msg);
    ROS_INFO("Publishing drop off location x:%f, y:%f",msg.position.x,msg.position.y);
    ros::Duration(1.0).sleep();
  }
  else
    ROS_INFO("Robot failed to reach to the drop off location");

  ros::Duration(5.0).sleep();

  return 0;
}
