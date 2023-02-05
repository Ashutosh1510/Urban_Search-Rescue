/**
 * @file main.cpp
 * @authors Sri Sai Charan Velisetti (svellise@umd.edu), Mukundhan Rajendiran (mrajeni@umd.edu), Ashutosh Reddy Atimyala (atimyala@umd.edu)
 * @brief The aim of the project is Urban Search & Rescue by executing the individual tasks of navigating the explorer and follower robots in the given map. 
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h> //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h> //for publishing Transform Broadcaster
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "../header/data.h"


/**
 * @brief function used to find aruco marker data
 * 
 * @param msg 
 */
void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg);

/**
 * @brief Function used to collect data from camera frame.
 * 
 */
void broadcast()
{
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;  // This message is used by the tf package

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";

  transformStamped.transform.translation.x = 0.5;
  transformStamped.transform.translation.y = 0.5;
  transformStamped.transform.translation.z = 0.2;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;
  ROS_INFO("Broadcasting");
  br.sendTransform(transformStamped);
}

/**
 * @brief Function used to map follower path in aruco marker order
 * 
 * @param tfBuffer 
 * @param x //Current postion of robot in x-axis
 * @param y //Current postion of robot in y-axis
 */
void listen(tf2_ros::Buffer &tfBuffer, double x, double y)
{
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;

    if (fiducial_id == 0) //code for offesting cordinates
    {
      f_path[0].x_cord = ((trans_x + x) / 2);
      f_path[0].y_cord = ((trans_y + y) / 2);
    }
    else if (fiducial_id == 1)
    {
      f_path[1].x_cord = ((trans_x + x) / 2);
      f_path[1].y_cord = ((trans_y + y) / 2);
    }
    else if (fiducial_id == 2)
    {
      f_path[2].x_cord = ((trans_x + x) / 2);
      f_path[2].y_cord = ((trans_y + y) / 2);
    }
    else if (fiducial_id == 3)
    {
      f_path[3].x_cord = ((trans_x + x) / 2);
      f_path[3].y_cord = ((trans_y + y) / 2);
    }
    else
    {
      ROS_INFO_STREAM("No fiducial ID yet or all fiducial IDs have been transformed");
    }
    //Storing all goals for follower in the given order
    for (int i = 0; i < 4; i++)
    {
      follower_goal[i].target_pose.header.frame_id = "map";
      follower_goal[i].target_pose.header.stamp = ros::Time::now();
      follower_goal[i].target_pose.pose.position.x = f_path[i].x_cord; //
      follower_goal[i].target_pose.pose.position.y = f_path[i].y_cord; //
      follower_goal[i].target_pose.pose.orientation.w = 1.0;
    }
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char **argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;
  bool follower_start = false;
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh; // Manages an internal reference count to make starting and shutting down a node.
  //Array used to hold data pulled from the parameter server
  XmlRpc::XmlRpcValue my_list[5];
  //Settign home position of explorer
  my_list[4][0] = -4.000000;
  my_list[4][1] = 2.500000;

//Retrieving Parameters
  nh.getParam("/aruco_lookup_locations/target_1", my_list[0]);
  ROS_ASSERT(my_list[0].getType() == XmlRpc::XmlRpcValue::TypeArray);

  nh.getParam("/aruco_lookup_locations/target_2", my_list[1]);
  ROS_ASSERT(my_list[1].getType() == XmlRpc::XmlRpcValue::TypeArray);

  nh.getParam("/aruco_lookup_locations/target_3", my_list[2]);
  ROS_ASSERT(my_list[2].getType() == XmlRpc::XmlRpcValue::TypeArray);

  nh.getParam("/aruco_lookup_locations/target_4", my_list[3]);
  ROS_ASSERT(my_list[3].getType() == XmlRpc::XmlRpcValue::TypeArray);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }
// Sending velocity command to the mobile base.
  ros::Publisher rotate_pub = nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 2000);
  ros::Subscriber sub_fid = nh.subscribe("fiducial_transforms", 1000, &fiducial_callback);
  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.angular.z = 0.1;
  //Build goal for explorer

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(1);

  //Setting all goal location and home position
  for (int i = 0; i < 5; i++)
  {
    if (i == 4)
    {
      //Home Position for explorer
      explorer_goal[i].target_pose.header.frame_id = "map";
      explorer_goal[i].target_pose.header.stamp = ros::Time::now();
      explorer_goal[i].target_pose.pose.position.x = -4.000000; //
      explorer_goal[i].target_pose.pose.position.y = 2.500000;  //
      explorer_goal[i].target_pose.pose.orientation.w = 1.0;
      //Home Position for Follower
      follower_goal[i].target_pose.header.frame_id = "map";
      follower_goal[i].target_pose.header.stamp = ros::Time::now();
      follower_goal[i].target_pose.pose.position.x = -4.000000; //
      follower_goal[i].target_pose.pose.position.y = 3.500000;  //
      follower_goal[i].target_pose.pose.orientation.w = 1.0;
    }
    else
    {
      //Setting Goals
      explorer_goal[i].target_pose.header.frame_id = "map";
      explorer_goal[i].target_pose.header.stamp = ros::Time::now();
      explorer_goal[i].target_pose.pose.position.x = my_list[i][0]; //
      explorer_goal[i].target_pose.pose.position.y = my_list[i][1]; //
      explorer_goal[i].target_pose.pose.orientation.w = 1.0;
      e_path[i].x_cord = my_list[i][0];
      e_path[i].y_cord = my_list[i][1];
    }
  }

  int goal_count = 0;
  int follower_count = 0;

  while (ros::ok())
  {
    if (!explorer_goal_sent)
    {
      ROS_INFO("Sending goal for explorer");
      explorer_client.sendGoal(explorer_goal[goal_count]); //this should be sent only once
      explorer_goal_sent = true;
    }
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !follower_start)
    {
      ROS_INFO("Hooray,Explorer robot reached goal");
      if (goal_count < 4)
      {
        int temp = 0;
        //Rotating the robot till it detects the aruco marker
        while (temp < 20)
        {
          ROS_INFO("Robot rotating");
          rotate_pub.publish(msg);
          ros::spinOnce();
          listen(tfBuffer, explorer_goal[goal_count].target_pose.pose.position.x, //
                 explorer_goal[goal_count].target_pose.pose.position.y);
          loop_rate.sleep();
          temp++;
           if (in_view)  //Explorer stops rotating once the ArUco marker is detected.
           {
             ROS_INFO_STREAM("Aruco Found");

             break;
           }
        }

        goal_count++;
        explorer_goal_sent = false;
      }
      else
      {
        follower_start = true;
      }
    }
    if (follower_start)
    {

      if (!follower_goal_sent)
      {
        ROS_INFO("Sending goal for follower");
        follower_client.sendGoal(follower_goal[follower_count]); //this should be sent only once
        follower_goal_sent = true;
      }
      if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Hooray, follower robot reached goal");
        if (follower_count < 5)
        {
          follower_count++;
          follower_goal_sent = false;
        }
      }
    }
    ros::spinOnce();
  }
}

void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg)
{
  if (!msg->transforms.empty())
  { 
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame"; //name of the frame
    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;

    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;

    fiducial_id = msg->transforms[0].fiducial_id;
    f_path[fiducial_id].f_id = fiducial_id;
    in_view = true;
    br.sendTransform(transformStamped); //broadcast the transform on /tf Topic
  }
  else
    in_view = false;
}