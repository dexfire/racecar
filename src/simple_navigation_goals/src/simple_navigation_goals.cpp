#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
using namespace std;


ros::Publisher pub;
double goal_fist_x,goal_fist_y,goal_second_x,goal_second_y;
double at_x, at_y;
double distance_goal;
bool first_goal, second_goal;

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

// Called once when the goal becomes active
//void activeCb()
//{
	//ROS_INFO("Goal Received");
//}
// Called every time feedback is received for the goal
//void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
//{
	//ROS_INFO("Got base_position of Feedback");
    //at_x = feedback->base_position.pose.position.x;
    //at_y = feedback->base_position.pose.position.y;
//}
  void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
    tf::StampedTransform transform;
    try
    {
       listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0) );
       listener.lookupTransform("map", "base_link", ros::Time(0), transform);
   }
    catch (tf::TransformException &ex)
   {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
   at_x = transform.getOrigin().x();
   at_y = transform.getOrigin().y();
   //cout<<"Car at x:"<<at_x<<" y:"<<at_y<<endl;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
  ros::Timer timer;
  
  pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  tf::TransformListener listener(ros::Duration(10));
  
  //we'll transform a point once every second
  timer = nh.createTimer(ros::Duration(0.2), boost::bind(&transformPoint, boost::ref(listener)));

  nh.param("goal_fist_x", goal_fist_x, 4.0);
  nh.param("goal_fist_y", goal_fist_y, -0.5);
  nh.param("goal_second_x", goal_second_x, 15.0);
  nh.param("goal_second_y", goal_second_y, 0.0);

  ros::Rate loop_rate(0.1);  
  //cout<<"Car at x:"<<at_x<<" y:"<<at_y<<endl;
  first_goal = true;
  second_goal = false ;
  //tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("move_base", true);
 
  //wait for the action server to come up
  //while(!ac.waitForServer(ros::Duration(5.0))){
    //ROS_INFO("Waiting for the move_base action server to come up");
  //}
  geometry_msgs::PoseStamped  point_goal;
  point_goal.header.frame_id = "map";
  point_goal.header.stamp = ros::Time::now();
  while(ros::ok()){
  //move_base_msgs::MoveBaseGoal goal;
  
  //we'll send a goal to the robot to move 1 meter forward
  //while(1){
  	 //tf::StampedTransform transform;
    //try
    //{
        //listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    //}
    //catch (tf::TransformException &ex)
    //{
        //ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
    //}
   //at_x = transform.getOrigin().x();
   //at_y = transform.getOrigin().y();


  if(first_goal){
  //goal.target_pose.header.frame_id = "map";
  //goal.target_pose.header.stamp = ros::Time::now();
  //goal.target_pose.pose.position.x = goal_fist_x;
  //goal.target_pose.pose.position.y = goal_fist_y;
  //goal.target_pose.pose.orientation.x =  0.0;
  //goal.target_pose.pose.orientation.y =  0.0;
  //goal.target_pose.pose.orientation.z =  0.0;
  //goal.target_pose.pose.orientation.w =  1.0;

  point_goal.header.frame_id = "map";
  point_goal.header.stamp = ros::Time::now();
  point_goal.pose.position.x = goal_fist_x;
  point_goal.pose.position.y = goal_fist_y;
  point_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);
 
  }
  
  distance_goal= (at_x - goal_fist_x)*(at_x - goal_fist_x) + (at_y - goal_fist_y)*(at_y - goal_fist_y);

  if (distance_goal<2)
  {
  first_goal = false;
  // goal.target_pose.header.frame_id = "map";
  // goal.target_pose.header.stamp = ros::Time::now();
  // goal.target_pose.pose.position.x = goal_second_x;
  // goal.target_pose.pose.position.y = goal_second_y;
  // goal.target_pose.pose.orientation.z = 1.0;
  // goal.target_pose.pose.orientation.w = 0.0;
  point_goal.header.frame_id = "/map";
  point_goal.header.stamp = ros::Time::now();
  point_goal.pose.position.x = goal_second_x;
  point_goal.pose.position.y = goal_second_y;
  point_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  }

  ROS_INFO("Sending goal");
  pub.publish(point_goal);
  //ac.sendGoal(goal);
  //ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
  cout<<"distance_goal:"<<distance_goal<<endl;
  //ac.waitForResult();
  
  
  //ac.waitForResult(ros::Duration(60));
 
  //if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //ROS_INFO("Hooray, the base moved 1 meter forward");
  //else
    //ROS_INFO("The base failed to move forward 1 meter for some reason");
  //}
  //ros::spin();
  ros::spinOnce();

  loop_rate.sleep();  
 }
  return 0;
}
