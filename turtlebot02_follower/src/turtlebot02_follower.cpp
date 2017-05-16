#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/turtlebotB/cmd_vel", 10);
  tf::TransformListener listener;

  ros::Rate rate(250);

  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      //ros::Time past = ros::Time::now() - ros::Duration(1.5);
      //listener.waitForTransform("/turtlebotB", "/turtlebotA", past, ros::Duration(1.0));
      listener.lookupTransform("/turtlebotB/base_footprint", "/turtlebotA",  ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;

    double x;
    double y;
    double theta;
    double dist;

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();

    theta = atan2(y,x);
    dist =  sqrt(pow(x, 2) + pow(y, 2));

    if(dist < 0.10)
    {
      vel_msg.angular.z = 0.0;
      vel_msg.linear.x = 0.0;
      //turtle_vel.publish(vel_msg);
    }
    else
    {
      vel_msg.angular.z =  0.1 * theta;
      vel_msg.linear.x  =  0.5 * dist;
      turtle_vel.publish(vel_msg);
      theta = 0;
      dist = 0;
    }

     ROS_INFO("x = %f y = %f", transform.getOrigin().x(), transform.getOrigin().y());
     ROS_INFO("dist = %f theta = %f", dist, theta * 180/M_PI);
     // ROS_INFO("angular.z = %f",theta);
     // ROS_INFO("linear = %f angular = %f", vel_msg.linear.x, vel_msg.angular.z);

    rate.sleep();
  }
  return 0;
}
