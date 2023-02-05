#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h> //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
/**
 * @brief Creating a object of arrays that are used to store the x and y cordinates of the the follower and explorer
 * 
 */
class path
{
  public:
  double x_cord = 0;
  double y_cord = 0;
  int f_id = 0;
} f_path[5], e_path[5];

/**
 * @brief Global variables used to hold the fiducial id when robot is rotating at the goal
 * 
 */
int fiducial_id;

/**
 * @brief Condition used to break from rotating.
 * 
 */
bool in_view = false;

/**
 * @brief Defining movebase goal objects for follower and explorer
 * 
 */
move_base_msgs::MoveBaseGoal explorer_goal[5];
move_base_msgs::MoveBaseGoal follower_goal[5];