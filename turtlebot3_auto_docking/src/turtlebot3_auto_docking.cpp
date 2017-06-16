#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

double x, y;
double dist, theta;

ros::Publisher cmd_vel_pub;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_goal_position");
  ros::NodeHandle node;

  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  geometry_msgs::Twist vel_msg;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::Quaternion q;

  ros::Rate rate(1000);

  while (node.ok())
  {
    try
    {
      listener.lookupTransform("odom", "final_goal_point", ros::Time(0), transform);
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

    if(dist < 0.05)
    {
      vel_msg.angular.z = theta;
      vel_msg.linear.x  = 0.0;
    }

    else if(theta < DEG2RAD(2) && theta > DEG2RAD(-2))

    vel_msg.angular.z = theta;
    vel_msg.linear.x  = dist;

    cmd_vel_pub.publish(vel_msg);
    rate.sleep();
  }
  return 0;
}
