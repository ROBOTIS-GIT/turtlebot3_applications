#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <math.h>

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

double x, y;
double dist, theta;
int state = 0;

void go_position(double x, double y);

geometry_msgs::Twist vel_msg;
geometry_msgs::Twist searching_dock_vel_msg;
std_msgs::Int8 robot_state;

ros::Publisher cmd_vel_pub;
ros::Publisher searching_dock_vel_pub;
ros::Subscriber searching_dock_vel_sub;
ros::Subscriber robot_state_sub;

void go_position(double x, double y)
{
  theta =  atan2(y, x);
  dist  =  sqrt(pow(x, 2) + pow(y, 2));
  
  vel_msg.angular.z = theta;
  vel_msg.linear.x  = dist;
  state = 0;

  if(dist < 0.05)
  {
    vel_msg.angular.z = 0.0;
    vel_msg.linear.x  = 0.0;
    state = 1;
  }
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &searching_dock_vel_sub_msg)
{
  searching_dock_vel_msg.angular.z = searching_dock_vel_sub_msg->angular.z;
  searching_dock_vel_msg.linear.x = searching_dock_vel_sub_msg->linear.x;
}

void robot_state_callback(const std_msgs::Int8::ConstPtr &robot_state_sub)
{
  robot_state.data = robot_state_sub->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_auto_docking");
  ros::NodeHandle node;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  searching_dock_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  searching_dock_vel_sub = node.subscribe<geometry_msgs::Twist>("/searching_docking_station", 10, &cmd_vel_callback);
  robot_state_sub = node.subscribe<std_msgs::Int8>("/robot_state", 10, &robot_state_callback);

  ros::Rate rate(125);

  while (node.ok())
  {
    try
    {
      listener.lookupTransform("base_footprint", "final_goal_point", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();

    go_position(x , y);

    if(state == 0)
    {
      cmd_vel_pub.publish(vel_msg);
      ROS_INFO("vel = %f, ang = %f", vel_msg.linear.x ,vel_msg.angular.z);
    }

    else if(state == 1)
    {
      searching_dock_vel_pub.publish(searching_dock_vel_msg);
      ROS_INFO("searching_dock_vel_msg = %f, searching_dock_vel_msg = %f", searching_dock_vel_msg.linear.x ,searching_dock_vel_msg.angular.z);
    }

    rate.sleep();
  }
  return 0;
}
