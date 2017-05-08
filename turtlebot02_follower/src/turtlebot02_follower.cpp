#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/turtlebotB/cmd_vel", 10);
  tf::TransformListener listener;

  float x;
  float y;

  ros::Rate rate(10.0);

  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("turtlebotB/base_footprint", "turtlebotA/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;

     x = transform.getOrigin().x();
     y = transform.getOrigin().y();

     ROS_INFO("x = %f y = %f", transform.getOrigin().x(), transform.getOrigin().y());
     ROS_INFO("angular.z = %f",5.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x()));

     if(x < 0.00001 && x > -0.00001)
     {
       x = 0;
     }


     if(y < 0.00001 && y > -0.00001)
     {
       y = 0;
     }

    vel_msg.angular.z = 5.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
    vel_msg.linear.x = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
}
