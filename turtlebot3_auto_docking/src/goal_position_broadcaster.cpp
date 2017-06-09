#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>

ros::Subscriber cloud_sub;

void poseCallback(const geometry_msgs::Point32::ConstPtr& point)
{
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  tf::Quaternion quaternion;

  float x = point->x;
  float y = point->y;
  ROS_INFO("%f, %f ", x, y);

  transform.setOrigin( tf::Vector3(x, y, 0.0) );
  quaternion.setRPY(0, 0, 1);
  transform.setRotation(quaternion);
  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time(0), "odom", "position"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_position_broadcaster");
  ros::NodeHandle node;

  cloud_sub = node.subscribe<geometry_msgs::Point32>( "/goal_point", 10, &poseCallback);

  ros::spin();
  return 0;
}
