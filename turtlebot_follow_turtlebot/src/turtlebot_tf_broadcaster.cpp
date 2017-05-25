#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

std::string turtlebot;
std::string turtlebot_behind;
float btw_dist;

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
  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtlebot));

  transform.setOrigin( tf::Vector3(odom.pose.pose.position.x + btw_dist - 0.3 * cos(yaw) , odom.pose.pose.position.y - 0.3 * sin(yaw) , 0.0) );
  quaternion.setRPY(0, 0, yaw);
  transform.setRotation(quaternion);
  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtlebot_behind));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TurtleBot3_tf_broadcaster");

  ros::NodeHandle nh_priv("~");
  ros::NodeHandle node;
  nh_priv.getParam("turtlebot", turtlebot);
  nh_priv.getParam("turtlebot_behind", turtlebot_behind);
  nh_priv.getParam("btw_dist", btw_dist);

  ros::Subscriber sub = node.subscribe( turtlebot+"/odom", 10, &poseCallback);
  ros::spin();
  return 0;
}
