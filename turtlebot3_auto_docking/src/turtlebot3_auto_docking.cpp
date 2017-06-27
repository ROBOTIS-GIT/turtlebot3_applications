#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <math.h>

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI
#define SEARCH      0
#define MOVE_GOAL   1
#define FINISH_GOAL 1

ros::Publisher     cmd_vel_pub;
ros::Publisher     finish_goal_state_pub;
ros::Publisher     searching_dock_vel_pub;
ros::Subscriber    searching_dock_vel_sub;
ros::Subscriber    move_goal_state_sub;

geometry_msgs::Twist                vel_msg;
geometry_msgs::Twist                searching_dock_vel_msg;
std_msgs::Int8                      move_goal_state_msg;
std_msgs::Int8                      finish_goal_state_msg;

double x_position, y_position;
double dist, theta;
int robot_state = 1;

void get_dist_theta(void);
void go_goal_position(void);

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr &searching_dock_vel_sub_msg)
{
  searching_dock_vel_msg.angular.z = searching_dock_vel_sub_msg->angular.z;
  searching_dock_vel_msg.linear.x  = searching_dock_vel_sub_msg->linear.x;
}

void move_goal_state_Callback(const std_msgs::Int8::ConstPtr &move_goal_state_sub_msg)
{
  move_goal_state_msg.data = move_goal_state_sub_msg->data;
}

void get_dist_theta()
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
   {
     listener.waitForTransform("base_footprint", "goal_poisition", ros::Time(0), ros::Duration(3.0));
     listener.lookupTransform("base_footprint", "goal_poisition", ros::Time(0), transform);
   }
  catch (tf::TransformException &ex)
   {
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
   }
   x_position = transform.getOrigin().x();
   y_position = transform.getOrigin().y();

   theta =  atan2(y_position, x_position);
   dist  =  sqrt(pow(x_position, 2) + pow(y_position, 2));
}

void go_goal_position()
{
  if(move_goal_state_msg.data == MOVE_GOAL)
  {
    get_dist_theta();
    switch (robot_state)
    {
      case 1:
        if(theta > DEG2RAD(3) || theta < DEG2RAD(-3))
        {
          vel_msg.angular.z = 0.2;
          vel_msg.linear.x  = 0.0;
        }
        else
        {
          robot_state = 2;
        }
      break;

      case 2:
        if(dist > 0.05)
        {
          vel_msg.angular.z = 0.0;
          vel_msg.linear.x  = 0.05;
        }
        else
        {
          vel_msg.angular.z = 0.0;
          vel_msg.linear.x  = 0.0;
          finish_goal_state_msg.data = FINISH_GOAL;
        }
      break;
    }
    cmd_vel_pub.publish(vel_msg);
    finish_goal_state_pub.publish(finish_goal_state_msg);
    ROS_INFO("vel_msg = %f, ang_msg = %f", vel_msg.linear.x ,vel_msg.angular.z);
  }
  else
  {
    searching_dock_vel_pub.publish(searching_dock_vel_msg);
    ROS_INFO("searching_dock_vel = %f, searching_dock_ang = %f", searching_dock_vel_msg.linear.x ,searching_dock_vel_msg.angular.z);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_auto_docking");
  ros::NodeHandle node;

  cmd_vel_pub            = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  finish_goal_state_pub  = node.advertise<std_msgs::Int8>("finish_goal_state", 10);
  searching_dock_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  searching_dock_vel_sub = node.subscribe("/searching_dock_vel", 10, &cmd_vel_Callback);
  move_goal_state_sub    = node.subscribe("/move_goal_state", 10, &move_goal_state_Callback);

  ros::Rate rate(125);
  while(node.ok())
  {
    go_goal_position();
    ros::spinOnce();
    rate.sleep();
  }
    return 0;
}
