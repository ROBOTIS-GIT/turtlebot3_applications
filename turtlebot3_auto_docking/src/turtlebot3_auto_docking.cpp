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

geometry_msgs::Twist vel_msg;
geometry_msgs::Twist searching_dock_vel_msg;
std_msgs::Int8       move_goal_state_msg;
std_msgs::Int8       finish_goal_state_msg;

ros::Publisher  cmd_vel_pub;
ros::Publisher  finish_goal_state_pub;
ros::Publisher  searching_dock_vel_pub;
ros::Subscriber searching_dock_vel_sub;
ros::Subscriber move_goal_state_sub;

double x, y;
double dist, theta;

void go_goal_position(void);
void robot_present_state(void);

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr &searching_dock_vel_sub_msg)
{
  searching_dock_vel_msg.angular.z = searching_dock_vel_sub_msg->angular.z;
  searching_dock_vel_msg.linear.x  = searching_dock_vel_sub_msg->linear.x;
}

void move_goal_state_Callback(const std_msgs::Int8::ConstPtr &move_goal_state_sub_msg)
{
  move_goal_state_msg.data = move_goal_state_sub_msg->data;
}

void go_goal_position()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;

    if(move_goal_state_msg.data == MOVE_GOAL)
    {
     try
      {
        listener.lookupTransform("base_footprint", "goal_poisition", ros::Time::now(), transform);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      x = transform.getOrigin().x();
      y = transform.getOrigin().y();

      theta =  atan2(y, x);
      dist  =  sqrt(pow(x, 2) + pow(y, 2));

      vel_msg.angular.z = theta;
      vel_msg.linear.x  = dist;

      if(dist < 0.05)
      {
        vel_msg.angular.z = 0.0;
        vel_msg.linear.x  = 0.0;
        finish_goal_state_msg.data = FINISH_GOAL;
      }
      cmd_vel_pub.publish(vel_msg);
      finish_goal_state_pub.publish(finish_goal_state_msg);
      ROS_INFO("vel = %f, ang = %f", vel_msg.linear.x ,vel_msg.angular.z);
    }
    else
    {
      searching_dock_vel_pub.publish(searching_dock_vel_msg);
      ROS_INFO("searching_dock_vel_msg = %f, searching_dock_vel_msg = %f", searching_dock_vel_msg.linear.x ,searching_dock_vel_msg.angular.z);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_auto_docking");
  ros::NodeHandle node;

  cmd_vel_pub            = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  finish_goal_state_pub  = node.advertise<std_msgs::Int8>("finish_goal_state", 10);
  searching_dock_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  searching_dock_vel_sub = node.subscribe<geometry_msgs::Twist>("/searching_dock_vel", 10, &cmd_vel_Callback);
  move_goal_state_sub    = node.subscribe<std_msgs::Int8>("/move_goal_state", 10, &move_goal_state_Callback);

  ros::Rate rate(125);
  while(node.ok())
  {
    go_goal_position();
    ros::spinOnce();
    rate.sleep();
  }
    return 0;
}
