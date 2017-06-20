#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <math.h>

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

double x, y;
double dist, theta;

geometry_msgs::Twist vel_msg;
std_msgs::Int8 state;
tf::TransformListener listener;
tf::StampedTransform transform;
tf::Quaternion q;

ros::Publisher cmd_vel_pub;
ros::Publisher robot_state_pub;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_goal_position");
  ros::NodeHandle node;

  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  robot_state_pub = node.advertise<std_msgs::Int8>("/robot_state", 10);

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

    theta =  atan2(y,x);
    dist  =  sqrt(pow(x, 2) + pow(y, 2));

    vel_msg.angular.z = theta;
    vel_msg.linear.x  = dist;

    if(dist < 0.05)
    {
      vel_msg.angular.z = 0.0;
      vel_msg.linear.x  = 0.0;
      state.data = 1;
      robot_state_pub.publish(state);
    }

    ROS_INFO("vel = %f, ang = %f", vel_msg.linear.x ,vel_msg.angular.z);
    cmd_vel_pub.publish(vel_msg);
    rate.sleep();
  }
  return 0;
}
