#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <turtlesim/Spawn.h>

std::string turtle_name;
std::string turtle_name_2;
float btw_dist;
float btw_dist_A;

void poseCallback(const nav_msgs::Odometry& odom)
{
  static tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  tf::Quaternion quaternion;
  tf::Quaternion q(odom.pose.pose.orientation.x,
                   odom.pose.pose.orientation.y,
                   odom.pose.pose.orientation.z,
                   odom.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  transform.setOrigin( tf::Vector3(odom.pose.pose.position.x + btw_dist , odom.pose.pose.position.y , 0.0) );
  quaternion.setRPY(0, 0, yaw);
  transform.setRotation(quaternion);
  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtlebotC"));

  // transform.setOrigin( tf::Vector3(odom.pose.pose.position.x + btw_dist - 0.2*sin(1.57-yaw) , odom.pose.pose.position.y - 0.2*cos(1.57-yaw) , 0.0) );
  // quaternion.setRPY(0, 0, yaw);
  // transform.setRotation(quaternion);
  // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name_2));
  //ROS_INFO("origin.x = %f origin.y = %f", odom.pose.pose.position.x, odom.pose.pose.position.y);
  //ROS_INFO("yaw = %f", yaw * 180/M_PI);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TurtleBot3_tf_broadcaster");

  ros::NodeHandle nh_priv("~");
  ros::NodeHandle node;
  nh_priv.getParam("turtle_name", turtle_name);
  nh_priv.getParam("turtle_name_2", turtle_name_2);
  nh_priv.getParam("btw_dist", btw_dist);
  nh_priv.getParam("btw_dist_A", btw_dist_A);

  ros::Subscriber sub = node.subscribe( "/turtlebotC/odom", 10, &poseCallback);

  ROS_INFO("Startoda!!!");
  //ROS_INFO("turtle_name = %s", turtle_name.c_str());

  ros::spin();
  return 0;
}
