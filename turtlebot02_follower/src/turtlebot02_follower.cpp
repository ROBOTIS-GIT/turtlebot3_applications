#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  ros::Publisher turtlebotB_vel = node.advertise<geometry_msgs::Twist>("/turtlebotB/cmd_vel", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);

    while (node.ok())
    {
      geometry_msgs::TransformStamped transformStamped;
      geometry_msgs::Twist vel_msg;

      try
      {
        transformStamped = tfBuffer.lookupTransform("turtlebotB/base_footprint", "turtlebotA/base_footprint", ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      // float x;
      // float y;

      vel_msg.angular.z = 5.0 * atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
      vel_msg.linear.x = sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));
      turtlebotB_vel.publish(vel_msg);
      // x = transform.getOrigin().x();
      // y = transform.getOrigin().y();
      //
      // ROS_INFO("x = %f y = %f", transform.getOrigin().x(), transform.getOrigin().y());
      // ROS_INFO("angular.z = %f",5.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x()));
      //
      // if(x < 0.00001 && x > -0.00001)
      // {
      //   x = 0;
      // }
      //
      // if(y < 0.00001 && y > -0.00001)
      // {
      //   y = 0;
      // }

    //  vel_msg.angular.z = 5.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
    //  vel_msg.linear.x = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    //  turtle_vel.publish(vel_msg);

     rate.sleep();
   }
   return 0;
}
