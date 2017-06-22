#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>
#include <math.h>

ros::Subscriber goal_position_sub;

float goal_position_x = 0.0, goal_position_y = 0.0, goal_position_theta = 0.0;

void goal_position_Callback(const geometry_msgs::Point32::ConstPtr& goal_position_msg)
{
  goal_position_x     = goal_position_msg->x;
  goal_position_y     = goal_position_msg->y;
  goal_position_theta = atan2(goal_position_y, goal_position_x);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_position_broadcaster");
  ros::NodeHandle node;

  goal_position_sub = node.subscribe<geometry_msgs::Point32>( "/goal_poisition", 10, &goal_position_Callback);

  ros::Rate rate(125);
  while (node.ok())
  {
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion quaternion;

    transform.setOrigin( tf::Vector3(goal_position_x - 0.05 * cos(goal_position_theta), goal_position_y - 0.05 * sin(goal_position_theta), 0.0) );
    quaternion.setRPY(0, 0, goal_position_theta);
    transform.setRotation(quaternion);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "goal_poisition"));

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
